#include "render_depth.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <limits>
#include <omp.h>

void show_single_camera(uint8_t *cam_buffer, uint32_t cam_size, int mode, const char *win_name)
{

    if (!cam_buffer || cam_size <= 0 || !win_name) {
        std::cerr << "[Camera] Invalid input for " << win_name << "!" << std::endl;
        return;
    }

    if (mode != 1 && mode != 2) {
        std::cerr << "[Camera] Unsupported mode " << mode << " for " << win_name << std::endl;
        return;
    }

    int src_w, src_h;
    if (mode == 1) {
        src_w = 1280;
        src_h = 1088;
    } else { // mode == 2
        src_w = 1088;
        src_h = 1280;
    }

    uint32_t expected_size = src_w * src_h * 3 / 2;
    if (cam_size != expected_size) {
        std::cerr << "[Camera] " << win_name << " size mismatch! Expected " 
                  << expected_size << " bytes, got " << cam_size << std::endl;
        return;
    }

    cv::Mat nv12_mat(src_h * 3 / 2, src_w, CV_8UC1, cam_buffer);
    cv::Mat bgr_mat;
    cv::cvtColor(nv12_mat, bgr_mat, cv::COLOR_YUV2BGR_NV12);


    if (mode == 2) {
        cv::rotate(bgr_mat, bgr_mat, cv::ROTATE_90_CLOCKWISE);
    }

    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
   
    int display_w = std::min(1280, bgr_mat.cols);
    int display_h = bgr_mat.rows * display_w / bgr_mat.cols;
    cv::resizeWindow(win_name, display_w, display_h);
    cv::imshow(win_name, bgr_mat);
    cv::waitKey(1);
}

void show_depth_map(uint16_t *depth_buffer, size_t depth_size, int width, int height) {
    // 输入有效性检查
    if (!depth_buffer || depth_size <= 0 || width <= 0 || height <= 0) {
        std::cerr << "Invalid depth buffer or size/width/height!" << std::endl;
        return;
    }
    // 检查数据大小是否匹配（每个像素2字节）
    size_t expected_size = width * height * sizeof(uint16_t);
    if (depth_size != expected_size) {
        std::cerr << "Depth size mismatch! Expected " << expected_size << " bytes, got " << depth_size << std::endl;
        return;
    }

    // 转换为OpenCV Mat（16位单通道，单位：毫米）
    cv::Mat depth_raw(height, width, CV_16UC1, depth_buffer);

    // ====================== 1. 原始深度图（归一化到8位显示）======================
    cv::Mat raw_depth_3ch;
    {
        // 提取有效深度（排除0值）
        cv::Mat valid_depth;
        depth_raw.copyTo(valid_depth, depth_raw > 0);
        
        if (valid_depth.empty()) {
            // 全无效时显示黑色
            raw_depth_3ch = cv::Mat::zeros(height, width, CV_8UC3);
        } else {
            // 归一化到[0,255]
            double min_depth, max_depth;
            cv::minMaxLoc(valid_depth, &min_depth, &max_depth);
            cv::Mat depth_normalized;
            valid_depth.convertTo(depth_normalized, CV_8UC1, 
                                255.0 / (max_depth - min_depth), 
                                -min_depth * 255.0 / (max_depth - min_depth));
            // 无效区域（0值）设为黑色
            depth_normalized.setTo(0, depth_raw == 0);
            // 转为3通道（方便拼接）
            cv::cvtColor(depth_normalized, raw_depth_3ch, cv::COLOR_GRAY2BGR);
        }
    }

    // ====================== 2. 可视化深度图（outdoor模式，通用像素处理）======================
    cv::Mat visual_depth;
    {
        // 转换深度为浮点数（米），并反转（模拟视差特性：近大远小）
        cv::Mat depth_m;
        depth_raw.convertTo(depth_m, CV_32F, 1.0 / 1000.0); // 毫米→米
        cv::Mat inv_depth = 1.0f / (depth_m + 1e-6f); // 反转深度
        inv_depth.setTo(0, depth_raw == 0); // 无效区域置0

        // 提取有效反转深度值，计算分位数（outdoor模式核心逻辑）
        std::vector<float> depth_vals;
        depth_vals.reserve(height * width);
        float min_val = std::numeric_limits<float>::max();
        float max_val = std::numeric_limits<float>::lowest();

#pragma omp parallel
        {
            std::vector<float> local_vals;
            local_vals.reserve(height * width / 4);
            float local_min = std::numeric_limits<float>::max();
            float local_max = std::numeric_limits<float>::lowest();
#pragma omp for nowait
            for (int r = 0; r < height; ++r) {
                const float *row_ptr = inv_depth.ptr<float>(r);
                for (int c = 0; c < width; ++c) {
                    float v = row_ptr[c];
                    if (v > 0) {
                        local_vals.push_back(v);
                        if (v < local_min) local_min = v;
                        if (v > local_max) local_max = v;
                    }
                }
            }
#pragma omp critical
            {
                depth_vals.insert(depth_vals.end(), local_vals.begin(), local_vals.end());
                if (local_min < min_val) min_val = local_min;
                if (local_max > max_val) max_val = local_max;
            }
        }

        // 计算10%、50%、90%分位数（控制亮度分布）
        auto getQuantile = [&](double q) {
            if (depth_vals.empty()) return 0.0f;
            size_t idx = static_cast<size_t>(q * (depth_vals.size() - 1));
            std::nth_element(depth_vals.begin(), depth_vals.begin() + idx, depth_vals.end());
            return depth_vals[idx];
        };
        float q10 = getQuantile(0.10);
        float q50 = getQuantile(0.50);
        float q90 = getQuantile(0.90);

        // 分段缩放系数
        double scale10 = 0.25 * 255.0 / (q10 - min_val + 1e-6);
        double scale50 = 0.25 * 255.0 / (q50 - q10 + 1e-6);
        double scale90 = 0.25 * 255.0 / (q90 - q50 + 1e-6);
        double scaleMax = 0.25 * 255.0 / (max_val - q90 + 1e-6);

        // 生成基础灰度图（移除NEON，改用普通循环）
        visual_depth.create(height, width, CV_8UC3);
#pragma omp parallel for
        for (int r = 0; r < height; ++r) {
            const float *row_ptr = inv_depth.ptr<float>(r);
            cv::Vec3b *out_ptr = visual_depth.ptr<cv::Vec3b>(r);

            // 逐个处理像素（兼容所有架构）
            for (int c = 0; c < width; ++c) {
                float pixel = row_ptr[c];
                uint8_t val = 0;
                if (pixel > 0) {
                    if (pixel <= q10)
                        val = static_cast<uint8_t>((pixel - min_val) * scale10);
                    else if (pixel <= q50)
                        val = static_cast<uint8_t>(0.25 * 255 + (pixel - q10) * scale50);
                    else if (pixel <= q90)
                        val = static_cast<uint8_t>(0.5 * 255 + (pixel - q50) * scale90);
                    else
                        val = static_cast<uint8_t>(0.75 * 255 + (pixel - q90) * scaleMax);
                }
                out_ptr[c] = cv::Vec3b(val, val, val);
            }
        }

        // 应用JET伪彩色（outdoor模式默认配色）
        static cv::Mat lut;
        if (lut.empty()) {
            cv::Mat tmp(1, 256, CV_8UC1);
            for (int i = 0; i < 256; i++) tmp.at<uchar>(i) = i; // 正向映射
            cv::applyColorMap(tmp, lut, cv::COLORMAP_JET);
        }
        cv::LUT(visual_depth, lut, visual_depth);

        // 无效区域（深度=0）设为黑色
        cv::Mat mask = (depth_raw == 0);
        visual_depth.setTo(cv::Vec3b(0, 0, 0), mask);
    }

    // ====================== 3. 网格与深度标注 =======================
    const int set_num = 6; // 网格数量
    double font_scale = std::min(width, height) / 700.0; // 自适应字体大小
    int x_step = width / set_num;
    int y_step = height / set_num;

    // 绘制网格线
    for (int i = 1; i < set_num; ++i) {
        // 原始深度图网格
        cv::line(raw_depth_3ch, cv::Point(i * x_step, 0), cv::Point(i * x_step, height), 
                 cv::Scalar(255, 255, 255), 1);
        cv::line(raw_depth_3ch, cv::Point(0, i * y_step), cv::Point(width, i * y_step), 
                 cv::Scalar(255, 255, 255), 1);
        // 可视化深度图网格
        cv::line(visual_depth, cv::Point(i * x_step, 0), cv::Point(i * x_step, height), 
                 cv::Scalar(255, 255, 255), 1);
        cv::line(visual_depth, cv::Point(0, i * y_step), cv::Point(width, i * y_step), 
                 cv::Scalar(255, 255, 255), 1);
    }

    // 标注深度值（单位：米）
    for (int i = 1; i < set_num; ++i) {
        for (int j = 1; j < set_num; ++j) {
            int x = i * x_step;
            int y = j * y_step;
            float depth_val = depth_raw.at<uint16_t>(y, x) * 0.001f; // 毫米→米
            if (depth_val <= 0) continue;

            std::stringstream depth_text;
            depth_text << std::fixed << std::setprecision(2) << depth_val << "m";

            // 标注到原始深度图
            cv::putText(raw_depth_3ch, depth_text.str(), cv::Point(x + 5, y - 5), 
                        cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 255, 255), 2);
            // 标注到可视化深度图
            cv::putText(visual_depth, depth_text.str(), cv::Point(x + 5, y - 5), 
                        cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 255, 255), 2);
        }
    }

    // ====================== 4. 拼接并显示 ======================
    cv::Mat combined_img;
    cv::hconcat(raw_depth_3ch, visual_depth, combined_img); // 左：原始深度，右：可视化深度

    // 添加标题
    cv::putText(combined_img, "Raw Depth (Normalized)", cv::Point(20, 30), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 0), 2);
    cv::putText(combined_img, "Visual Depth (Outdoor)", cv::Point(width + 20, 30), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 0), 2);

    // 显示窗口
    cv::namedWindow("Depth Map (Raw + Visual)", cv::WINDOW_NORMAL);
    cv::resizeWindow("Depth Map (Raw + Visual)", width * 2, height); // 宽度翻倍，高度不变
    cv::imshow("Depth Map (Raw + Visual)", combined_img);
    cv::waitKey(1); // 非阻塞显示
}

#if 0

#if defined(__ARM_NEON__) || defined(__ARM_NEON)
#include <arm_neon.h>
#define USE_NEON 1
#else
#define USE_NEON 0
#endif

static bool compute_depth_percentiles(const cv::Mat &depth, double &p10, double &p50, double &p90)
{
    const int rows = depth.rows;
    const int cols = depth.cols;

    int num_threads = omp_get_max_threads();
    std::vector<std::vector<uint16_t>> locals(num_threads);

#pragma omp parallel
    {
        int tid = omp_get_thread_num();
        auto &buf = locals[tid];
        buf.reserve(depth.total() / num_threads);

#pragma omp for schedule(static)
        for (int y = 0; y < rows; ++y) {
            const uint16_t *ptr = depth.ptr<uint16_t>(y);
            for (int x = 0; x < cols; ++x) {
                if (ptr[x] > 0)
                    buf.push_back(ptr[x]);
            }
        }
    }

    size_t total = 0;
    for (auto &v : locals) total += v.size();
    if (total == 0) {
        p10 = p50 = p90 = -1.0;
        return false;
    }

    std::vector<uint16_t> valid;
    valid.reserve(total);
    for (auto &v : locals)
        valid.insert(valid.end(), v.begin(), v.end());

    size_t k10 = static_cast<size_t>(0.10 * valid.size());
    size_t k50 = static_cast<size_t>(0.50 * valid.size());
    size_t k90 = static_cast<size_t>(0.90 * valid.size());

    std::nth_element(valid.begin(), valid.begin() + k10, valid.end());
    p10 = valid[k10];

    std::nth_element(valid.begin(), valid.begin() + k50, valid.end());
    p50 = valid[k50];

    std::nth_element(valid.begin(), valid.begin() + k90, valid.end());
    p90 = valid[k90];

    return true;
}

void show_depth_map(uint16_t *depth_buffer,
                    size_t depth_size,
                    int width,
                    int height)
{
    if (!depth_buffer || depth_size == 0 || width <= 0 || height <= 0) {
        std::cerr << "Invalid depth buffer!" << std::endl;
        return;
    }


    size_t expected = static_cast<size_t>(width) * height * sizeof(uint16_t);
    if (depth_size != expected) {
        std::cerr << "Depth size mismatch! Expected: " << expected << ", Got: " << depth_size << std::endl;
        return;
    }

    cv::Mat depth_raw(height, width, CV_16UC1, depth_buffer);

    double p10, p50, p90;
    if (!compute_depth_percentiles(depth_raw, p10, p50, p90)) {
        std::cerr << "No valid depth data found!" << std::endl;
        return;
    }
    if (p10 <= 0 || p50 <= 0 || p90 <= 0) {
        std::cerr << "Invalid percentile values!" << std::endl;
        return;
    }


    double min_valid = 0.0;
    double max_valid = 0.0;

    std::vector<uint16_t> all_valid;
    for (int y = 0; y < height; ++y) {
        const uint16_t *dptr = depth_raw.ptr<uint16_t>(y);
        for (int x = 0; x < width; ++x) {
            if (dptr[x] > 0) {
                all_valid.push_back(dptr[x]);
            }
        }
    }
    if (!all_valid.empty()) {
        min_valid = *std::min_element(all_valid.begin(), all_valid.end());
        max_valid = *std::max_element(all_valid.begin(), all_valid.end());
    } else {
        std::cerr << "No valid depth data for min/max calculation!" << std::endl;
        return;
    }

    const double seg1_start = 0.0;
    const double seg1_end = 0.25;
    const double seg2_start = 0.25;
    const double seg2_end = 0.5;
    const double seg3_start = 0.5;
    const double seg3_end = 0.75;
    const double seg4_start = 0.75;
    const double seg4_end = 1.0;

    const float gray_scale = 255.0f;

    cv::Mat gray(height, width, CV_8UC1);


#pragma omp parallel for schedule(static)
    for (int y = 0; y < height; ++y) {
        const uint16_t *dptr = depth_raw.ptr<uint16_t>(y);
        uchar *optr = gray.ptr<uchar>(y);

#if USE_NEON

        int x = 0;
        float32x4_t v_min_valid = vdupq_n_f32((float)min_valid);
        float32x4_t v_p10 = vdupq_n_f32((float)p10);
        float32x4_t v_p50 = vdupq_n_f32((float)p50);
        float32x4_t v_p90 = vdupq_n_f32((float)p90);
        float32x4_t v_max_valid = vdupq_n_f32((float)max_valid);

        float32x4_t v_seg1_start = vdupq_n_f32((float)seg1_start);
        float32x4_t v_seg1_span = vdupq_n_f32((float)(seg1_end - seg1_start));
        float32x4_t v_seg2_start = vdupq_n_f32((float)seg2_start);
        float32x4_t v_seg2_span = vdupq_n_f32((float)(seg2_end - seg2_start));
        float32x4_t v_seg3_start = vdupq_n_f32((float)seg3_start);
        float32x4_t v_seg3_span = vdupq_n_f32((float)(seg3_end - seg3_start));
        float32x4_t v_seg4_start = vdupq_n_f32((float)seg4_start);
        float32x4_t v_seg4_span = vdupq_n_f32((float)(seg4_end - seg4_start));

        float32x4_t v_gray_scale = vdupq_n_f32(gray_scale);
        float32x4_t v_zero = vdupq_n_f32(0.0f);
        float32x4_t v_255 = vdupq_n_f32(255.0f);


        for (; x <= width - 4; x += 4) {

            uint16x4_t u16 = vld1_u16(dptr + x);
            uint32x4_t u32 = vmovl_u16(u16);
            float32x4_t v_z = vcvtq_f32_u32(u32);


            float32x4_t v_norm = vdupq_n_f32(0.0f);

            uint32x4_t mask1 = vcagtq_f32(v_p10, v_z);
            float32x4_t norm1 = vsubq_f32(v_z, v_min_valid);
            norm1 = vmulq_f32(norm1, vdivq_f32(v_seg1_span, vsubq_f32(v_p10, v_min_valid)));
            norm1 = vaddq_f32(norm1, v_seg1_start);
            v_norm = vbslq_f32(mask1, norm1, v_norm);

            uint32x4_t mask2_1 = vcagtq_f32(v_z, v_p10);
            uint32x4_t mask2_2 = vcagtq_f32(v_p50, v_z);
            uint32x4_t mask2 = vandq_u32(mask2_1, mask2_2);
            float32x4_t norm2 = vsubq_f32(v_z, v_p10);
            norm2 = vmulq_f32(norm2, vdivq_f32(v_seg2_span, vsubq_f32(v_p50, v_p10)));
            norm2 = vaddq_f32(norm2, v_seg2_start);
            v_norm = vbslq_f32(mask2, norm2, v_norm);

            uint32x4_t mask3_1 = vcagtq_f32(v_z, v_p50);
            uint32x4_t mask3_2 = vcagtq_f32(v_p90, v_z);
            uint32x4_t mask3 = vandq_u32(mask3_1, mask3_2);
            float32x4_t norm3 = vsubq_f32(v_z, v_p50);
            norm3 = vmulq_f32(norm3, vdivq_f32(v_seg3_span, vsubq_f32(v_p90, v_p50)));
            norm3 = vaddq_f32(norm3, v_seg3_start);
            v_norm = vbslq_f32(mask3, norm3, v_norm);

            uint32x4_t mask4 = vcagtq_f32(v_z, v_p90);
            float32x4_t norm4 = vsubq_f32(v_z, v_p90);
            norm4 = vmulq_f32(norm4, vdivq_f32(v_seg4_span, vsubq_f32(v_max_valid, v_p90)));
            norm4 = vaddq_f32(norm4, v_seg4_start);
            v_norm = vbslq_f32(mask4, norm4, v_norm);

            v_norm = vmulq_f32(v_norm, v_gray_scale);
            v_norm = vmaxq_f32(v_norm, v_zero);
            v_norm = vminq_f32(v_norm, v_255);

            uint32x4_t ui = vcvtq_u32_f32(v_norm);
            uint16x4_t u16n = vmovn_u32(ui);
            uint8x8_t u8 = vqmovn_u16(vcombine_u16(u16n, vdup_n_u16(0)));
            vst1_u8(optr + x, u8);
        }

        for (; x < width; ++x) {
            uint16_t z = dptr[x];
            float norm_val = 0.0f;

            if (z == 0) {
                optr[x] = 0;
                continue;
            }

            if (z <= p10) {
                norm_val = (z - min_valid) / (p10 - min_valid) * (seg1_end - seg1_start) + seg1_start;
            }
            else if (z > p10 && z <= p50) {
                norm_val = (z - p10) / (p50 - p10) * (seg2_end - seg2_start) + seg2_start;
            }
            else if (z > p50 && z <= p90) {
                norm_val = (z - p50) / (p90 - p50) * (seg3_end - seg3_start) + seg3_start;
            }
            else {
                norm_val = (z - p90) / (max_valid - p90) * (seg4_end - seg4_start) + seg4_start;
            }

            int gray_val = static_cast<int>(norm_val * gray_scale);
            gray_val = std::max(0, std::min(255, gray_val));
            optr[x] = static_cast<uchar>(gray_val);
        }
#else
        for (int x = 0; x < width; ++x) {
            uint16_t z = dptr[x];
            float norm_val = 0.0f;

            if (z == 0) {
                optr[x] = 0;
                continue;
            }
            if (z <= p10) {
                norm_val = (z - min_valid) / (p10 - min_valid) * (seg1_end - seg1_start) + seg1_start;
            }
            else if (z > p10 && z <= p50) {
                norm_val = (z - p10) / (p50 - p10) * (seg2_end - seg2_start) + seg2_start;
            }
            else if (z > p50 && z <= p90) {
                norm_val = (z - p50) / (p90 - p50) * (seg3_end - seg3_start) + seg3_start;
            }
            else {
                norm_val = (z - p90) / (max_valid - p90) * (seg4_end - seg4_start) + seg4_start;
            }
            int gray_val = static_cast<int>(norm_val * gray_scale);
            gray_val = std::max(0, std::min(255, gray_val));
            optr[x] = static_cast<uchar>(gray_val);
        }
#endif
    }

    cv::Mat color;
    cv::applyColorMap(gray, color, cv::COLORMAP_JET);

    cv::Mat mask = (depth_raw == 0);
    color.setTo(cv::Vec3b(0, 0, 0), mask);

    cv::imshow("Depth Color (Aligned Python)", color);
    cv::waitKey(1);
}

#endif