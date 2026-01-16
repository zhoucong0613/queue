#ifndef RENDER_DEPTH_H
#define RENDER_DEPTH_H

#include <opencv2/opencv.hpp>
#include <cstdint>
#include <cstddef>

/**
 * @brief 显示单个相机的NV12格式图像
 * @param cam_buffer 相机图像缓冲区（NV12格式）
 * @param cam_size 缓冲区字节大小
 * @param mode 相机模式（1: 1280x1088, 2: 1088x1280）
 * @param win_name 显示窗口名称
 */
void show_single_camera(uint8_t *cam_buffer, uint32_t cam_size, int mode, const char *win_name);

/**
 * @brief 显示深度图（原始归一化图 + Outdoor伪彩色可视化图）
 * @param depth_buffer 深度数据缓冲区（16位无符号整数，单位：毫米）
 * @param depth_size 缓冲区字节大小
 * @param width 深度图宽度
 * @param height 深度图高度
 */
void show_depth_map(uint16_t *depth_buffer, size_t depth_size, int width, int height);

#endif // RENDER_DEPTH_H