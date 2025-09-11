#include <stdio.h>
#include <getopt.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <sys/prctl.h>
#include <sched.h>
#include <pthread.h>
#include <signal.h>
#include <ctype.h>
#include <errno.h>
#include <stdint.h>
#include <net/if.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <png.h>
#include <opencv2/opencv.hpp>

#include "common.h"
#include "performance_test_util.h"
#include "synchronous_queue.h"
#include "cJSON.h"
#include "imu.h"

// 相机内参结构体
typedef struct {
    double fx, fy, cx, cy;
} CameraIntrinsics;

typedef struct {
    float x;
	float y;
	float z;
} PointCloud;

typedef struct {
	char *name;
	int available;
	uint32_t width;
	uint32_t height;
	uint32_t size;
	uint32_t offset;
	// uint32_t sub_node_num;
	// uint32_t sub_node_size;
}stream_node_info_t;

typedef struct {
	char model_version[32];
	uint32_t total_size;
	stream_node_info_t stream_nodes[STREAM_NODE_TYPE_NUM];
}server_stream_info_t;

static int client_fd;
static pthread_t client_data_thread;
static pthread_t consumer_thread;
static sync_queue_t recv_to_consumer;

static server_stream_info_t stream_info = {0};

static int32_t running = 0;

static CameraIntrinsics intr = {646.104, 418.067, 328.663, 174.894}; 

/* Debug Dump File */
static uint32_t dumpfile_mode = 0;
#define DUMP_LR_RAW_NV12			(1<<0)
#define DUMP_DEPTH_FILE				(1<<1)
#define DUMP_POINTCLOUD_FILE		(1<<2)

/* Debug Verbose */
static uint32_t verbose_mode = 0;
#define VERBOSE_STREAM				(1<<0)
#define VERBOSE_LR_RAW_NV12			(1<<1)
#define VERBOSE_DEPTH_POINTCLOUD	(1<<2)
#define VERBOSE_IMU					(1<<3)

static int preview_flag = 0;

void signal_handle(int signo) {
	running = 0;
}

#if 0
int read_png_depth(const char* filename, uint16_t **depth_data, int* width, int* height) {
    FILE *fp = fopen(filename, "rb");
    if (!fp) {
        fprintf(stderr, "无法打开文件: %s\n", filename);
        return -1;
    }

    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) {
        fclose(fp);
        return -1;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_read_struct(&png, NULL, NULL);
        fclose(fp);
        return -1;
    }

    if (setjmp(png_jmpbuf(png))) {
        png_destroy_read_struct(&png, &info, NULL);
        fclose(fp);
        return -1;
    }

    png_init_io(png, fp);
    png_read_info(png, info);

    *width = png_get_image_width(png, info);
    *height = png_get_image_height(png, info);
    png_byte color_type = png_get_color_type(png, info);
    png_byte bit_depth = png_get_bit_depth(png, info);

    if (color_type != PNG_COLOR_TYPE_GRAY || bit_depth != 16) {
        fprintf(stderr, "仅支持16位灰度PNG深度图\n");
        png_destroy_read_struct(&png, &info, NULL);
        fclose(fp);
        return -1;
    }

    *depth_data = (uint16_t*)malloc((*width) * (*height) * sizeof(uint16_t));
    png_bytep* row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * (*height));

    for (int y = 0; y < *height; y++) {
        row_pointers[y] = (png_byte*)png_malloc(png, png_get_rowbytes(png, info));
    }

    png_read_image(png, row_pointers);

    // 拷贝数据到连续数组（行优先存储）
    for (int y = 0; y < *height; y++) {
        uint16_t* row = (uint16_t*)row_pointers[y];
        for (int x = 0; x < *width; x++) {
            (*depth_data)[y * (*width) + x] = row[x];
        }
        png_free(png, row_pointers[y]);
    }

    free(row_pointers);
    png_destroy_read_struct(&png, &info, NULL);
    fclose(fp);
    return 0;
}
#endif

void depth_to_pointcloud(
    const uint16_t* depth_data,
    int rows,
    int cols,
    const CameraIntrinsics* intr,
    PointCloud* pc
)
{
	PointCloud *pointTmp = pc;

    for (int v = 0; v < rows; ++v) {
        for (int u = 0; u < cols; ++u) {
            uint16_t d = depth_data[v * cols + u];
            if (d > 0) {
				// 计算三维坐标（毫米转米）
				float x = (u - intr->cx) * d / intr->fx;
				float y = (v - intr->cy) * d / intr->fy;
				float z = d;
				// 存储坐标
				pointTmp->x = x;
				pointTmp->y = y;
				pointTmp->z = z;
			}
			pointTmp++;
        }
    }
}

void save_pointcloud_to_txt(const char* filename, PointCloud* pc, int rows, int cols)
{
    FILE* fp = fopen(filename, "w");
    if (!fp) {
        fprintf(stderr, "Open File Failed: %s\n", filename);
        return;
    }

	PointCloud *pointTmp = pc;

    for (int v = 0; v < rows; ++v) {
        for (int u = 0; u < cols; ++u) {
			if (fprintf(fp, "%f %f %f\n", pointTmp->x, pointTmp->y, pointTmp->z) < 0) {
				fprintf(stderr, "Write File Failed\n");
				break;
			}
			pointTmp++;
        }
    }
	fflush(fp);
    fclose(fp);
    printf("Save PointClode To %s.\n", filename);
}

#if 0
int main() {
    const char* depth_file = "depth000021.png";
    CameraIntrinsics intr = {208.503, 208.503, 316.668, 175.107};  // 内参

    // 读取深度图
    uint16_t* depth_data = NULL;
    int width, height;
    if (read_png_depth(depth_file, &depth_data, &width, &height) != 0) {
        return -1;
    }

    // 初始化点云容器
    PointCloud *pc = (PointCloud *)malloc(width * height * sizeof(PointCloud));

    // 转换深度图
    depth_to_pointcloud(depth_data, height, width, &intr, pc);

    // 保存结果
    save_pointcloud_to_txt("pointcloud222.txt", pc, height, width);

    // 释放资源
	if (pc)
		free(pc);
	if (depth_data)
    	free(depth_data);
    return 0;
}
#endif

static int dumpToFile(char *filename, char *srcBuf, unsigned int size)
{
	FILE *fd = NULL;
	fd = fopen(filename, "w+");
	if (fd == NULL) {
		printf("ERRopen(%s) fail", filename);
		return -1;
	}

	fwrite(srcBuf, 1, size, fd);
	fflush(fd);

	if (fd)
		fclose(fd);
	printf("filedump(%s, size(%d) is successed\n", filename, size);
	return 0;
}

/* 获取当前时间戳（微秒） */
static uint64_t get_current_timestamp_us() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}

static int client_check_stream_header(uint8_t *stream_buffer)
{
	stream_header_t *stream_header = NULL;
	stream_header = (stream_header_t *)stream_buffer;

	if (verbose_mode & VERBOSE_STREAM) {
		printf("[Client] Stream Magic = %x, Total_size = %d\n", stream_header->magic, stream_header->total_size);
	}

	if (stream_header->magic != STREAM_HEADER_MAGIC) {
		printf("Magic Error: %x\n", stream_header->magic);
		return -1;
	}

	// if (stream_header->total_size != size) {
	// 	printf("Size Error: %d != %d\n", stream_header->total_size, size);
	// 	return -1;
	// }

	return 0;
}

static void client_parse_imu_buffer(void *imu_buffer, uint32_t imu_size)
{
	uint32_t i;
	uint32_t imu_data_num = imu_size / sizeof(imu_subnode_data_t);
	imu_subnode_data_t *imu_subnode_data = static_cast<imu_subnode_data_t*>(imu_buffer);

	for (i = 0; i < imu_data_num; i++) {
		if (verbose_mode & VERBOSE_IMU) {
			printf("[IMU] Data received pts[%ld]: =======> \n", imu_subnode_data->timestamp);
			printf("  Accelerometer: [%f, %f, %f] m/s²\n", imu_subnode_data->ax, imu_subnode_data->ay, imu_subnode_data->az);
			printf("  Gyroscope:     [%f, %f, %f] rad/s\n", imu_subnode_data->gx, imu_subnode_data->gy, imu_subnode_data->gz);
		}
		imu_subnode_data++;
	}
}

static int client_get_buffer_size_by_type(uint8_t *stream_buffer, int type, void **data_buffer, uint32_t *size)
{
	if (type < 0 || type >= STREAM_NODE_TAIL) {
		printf("client_get_buffer_by_type: type error[%d]\n", type);
		return -1;
	}

	// No available
	if (stream_info.stream_nodes[type].available == 0) {
		return -1;
	}

	stream_header_t *stream_header = (stream_header_t *)stream_buffer;
	stream_node_header_t *node_header = (stream_node_header_t *)&stream_header->node_headers[type];
	uint8_t *data_ptr = stream_buffer + sizeof(stream_header_t);

	if (node_header->size <= 0) {
		return -1;
	}

	*data_buffer = data_ptr + node_header->offset;
	*size = node_header->size;
	return 0;
}

#if 0
static uint32_t client_get_frameid_by_type(uint8_t *stream_buffer, int type)
{
	if (type < 0 || type >= STREAM_NODE_TAIL) {
		printf("client_get_buffer_by_type: type error[%d]\n", type);
		return -1;
	}

	stream_header_t *stream_header = (stream_header_t *)stream_buffer;
	stream_node_header_t *node_header = (stream_node_header_t *)&stream_header->node_headers[type];

	return node_header->frame_id;
}
#endif

void *client_recv_data(void *context)
{
	int ret;
	prctl(PR_SET_NAME, "client_recv_data");
	printf("[Thread] client_recv_data ==> Start\n");

	sync_queue_t *sync_queue = &recv_to_consumer;
	data_item_t *data_item = NULL;
	sync_queue_info_t *sync_queue_info = &sync_queue->sync_queue_info;
	ssize_t len_recv;

	// printf("[Client] Recv Size = %d\n", sync_queue_info->data_item_size);

	while (running) {
		ret = sync_queue_get_unused_object(sync_queue, 2000, &data_item);
		if(ret != 0){
			printf("sync_queue_get_unused_object Failed\n");
			break;
		}

		len_recv = recv(client_fd, data_item->items, sync_queue_info->data_item_size, MSG_WAITALL);
        if (len_recv < 0) {
			printf("[Client] recv Err[%ld] %s", len_recv, strerror(errno));
			break;
		}
		else if (len_recv == 0) {
			printf("[Client] Closed.\n");
			break;
		}
		// else if (len_recv > 0) {
		// 	if (verbose_mode & VERBOSE_STREAM) {
		// 		uint8_t *ptr = (uint8_t *)data_item->items;
		// 		printf("[Client] Recv: len = %ld, %02x %02x %02x %02x\n", len_recv, ptr[0], ptr[1], ptr[2], ptr[3]);
		// 	}
		// }

        ret = sync_queue_save_inused_object(sync_queue, 2000, data_item);
		if(ret != 0){
			printf("sync_queue_save_inused_object Failed\n");
			break;
		}
	}

	printf("Thread: client_recv_data ==> Quit\n");
	running = 0;
	return NULL;
}

void *consumer_process(void *context)
{
	int ret;
	sync_queue_t *sync_queue = &recv_to_consumer;
	data_item_t *data_item = NULL;

	uint16_t *depth_buffer = NULL;
	uint32_t depth_size = 0;
	uint8_t *right_buffer = NULL;
	uint32_t right_size = 0;
	uint8_t *left_buffer = NULL;
	uint32_t left_size = 0;
	uint8_t *imu_buffer = NULL;
	uint32_t imu_size = 0;

	// debug dump file
	int dump_cnt = 0;
	int dump_file = 0;

	PointCloud *pc = (PointCloud *)malloc(sizeof(PointCloud) * STEREO_RES_WIDTH * STEREO_RES_HEIGHT);

    struct PerformanceTestParamSimple performace_total_consumer = {
    		.test_count = 0,
		.iteration_number = 30 * 60,
		.test_case = "consumer",
		.run_count = 0,
	};

	struct PerformanceTestParamSimple performace_total_depth2cloud = {
		.test_count = 0,
		.iteration_number = 30 * 60,
		.test_case = "depth2cloud",
		.run_count = 0,
	};

    while (running) {
		performance_test_start_simple(&performace_total_consumer);

		ret = sync_queue_obtain_inused_object(sync_queue, 5000, &data_item);
		if(ret != 0){
			printf("sync_queue_obtain_inused_object failed.\n");
			continue;
		}

		//
		/* Check Header */
		if (0 != client_check_stream_header((uint8_t *)data_item->items)){
			printf("[ClIent] Header Error\n");
			goto consumer_release;
		}
		
		
		static cv::Mat preview_buffer(STEREO_RES_HEIGHT * 2, STEREO_RES_WIDTH, CV_8UC3);
		static cv::Mat depth_gray_8u_prev;
		#if 1
		/* Depth To Preview */
		if (0 == client_get_buffer_size_by_type((uint8_t *)data_item->items, STREAM_NODE_DEPTH, (void **)&depth_buffer, &depth_size)) {
		
			if (preview_flag && depth_buffer != nullptr && depth_size > 0) {
	
				cv::Mat depth_gray(STEREO_RES_HEIGHT, STEREO_RES_WIDTH, CV_16UC1, depth_buffer);

			    	// 归一化 16bit -> 8bit
				cv::Mat depth_gray_8u;
			    	double min_val, max_val;
			    	cv::minMaxLoc(depth_gray, &min_val, &max_val);
			    	depth_gray.convertTo(depth_gray_8u, CV_8UC1, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));

				// EMA 平滑
				if (!depth_gray_8u_prev.empty()) {
					cv::addWeighted(depth_gray_8u, 0.5, depth_gray_8u_prev, 0.5, 0, depth_gray_8u);
				}
				depth_gray_8u.copyTo(depth_gray_8u_prev);

				// Jet伪彩色
				cv::Mat depth_color;
				cv::applyColorMap(depth_gray_8u, depth_color, cv::COLORMAP_JET);

				// 合成上下显示
				depth_color.copyTo(preview_buffer(cv::Rect(0, 0, STEREO_RES_WIDTH, STEREO_RES_HEIGHT)));
				cv::Mat depth_gray_bgr;
				cv::cvtColor(depth_gray_8u, depth_gray_bgr, cv::COLOR_GRAY2BGR);
				depth_gray_bgr.copyTo(preview_buffer(cv::Rect(0, STEREO_RES_HEIGHT, STEREO_RES_WIDTH, STEREO_RES_HEIGHT)));

				// 显示
				cv::imshow("Depth Preview (Top: Jet, Bottom: Gray)", preview_buffer);
				cv::waitKey(1);
			}
		}
		
		#endif


		#if 1
		/* Depth To Cloudpoint */
		if (0 == client_get_buffer_size_by_type((uint8_t *)data_item->items, STREAM_NODE_DEPTH, (void **)&depth_buffer, &depth_size)) {
			performance_test_start_simple(&performace_total_depth2cloud);

			depth_to_pointcloud(depth_buffer, STEREO_RES_HEIGHT, STEREO_RES_WIDTH, &intr, pc);

			performance_test_stop_simple(&performace_total_depth2cloud);

			// Verbose: Depth & PointCloud
			if (verbose_mode & VERBOSE_DEPTH_POINTCLOUD) {
				printf("[%ld] Client Get Depth: size[%d].\n", get_current_timestamp_us(), depth_size);
			}


			printf("dumpfile_mode == %d\n",dumpfile_mode);
			// Dump File
			if (dumpfile_mode != 0) {
				dump_cnt++;
				if (dump_cnt % 10 == 0) {
					dump_file = 1;
				}
			}

			printf("dump_file== %d\n",dump_file);

			if (dump_file && (dumpfile_mode & DUMP_DEPTH_FILE)) {
				char depth_pic_name[64] ={0};
				sprintf(depth_pic_name, "depth_cnt[%d].bin", dump_cnt);
				dumpToFile(depth_pic_name, (char *)depth_buffer, depth_size);
			}
			if (dump_file && (dumpfile_mode & DUMP_POINTCLOUD_FILE)) {
				char pointcloud_pic_name[64] ={0};
				sprintf(pointcloud_pic_name, "pointcloud_cnt[%d].txt", dump_cnt);
				save_pointcloud_to_txt(pointcloud_pic_name, pc, STEREO_RES_HEIGHT, STEREO_RES_WIDTH);
			}
		}
		
	
		
		/* Right Camera Data */
		if (0 == client_get_buffer_size_by_type((uint8_t *)data_item->items, STREAM_NODE_CAM_RIGHT, (void **)&right_buffer, &right_size)) {

		#if 0
			if (preview_flag && right_buffer != nullptr && right_size > 0) {
				// 初始化YUV图像（YUV420SP格式：分辨率 height x width，数据大小 = width*height*1.5）
				g_right_yuv = cv::Mat(height * 3 / 2, width, CV_8UC1, right_buffer).clone();
				// YUV420SP转BGR（嵌入式常用格式，对应cv::COLOR_YUV2BGR_NV12）
				cv::Mat right_bgr;
				cv::cvtColor(g_right_yuv, right_bgr, cv::COLOR_YUV2BGR_NV12);
				// 显示鼠标坐标（YUV图无需显示像素值，仅显示坐标）
				std::string right_txt = "Pos: (" + std::to_string(g_mousePos_right.x) + "," + 
									   std::to_string(g_mousePos_right.y) + ")";
				cv::putText(right_bgr, right_txt, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
				// 显示右图
				cv::imshow("Right Camera", right_bgr);
			}
		#endif
		
			// Verbose: Right
			if (verbose_mode & VERBOSE_LR_RAW_NV12) {
				printf("[%ld] Client Get Right Cam NV12: size[%d].\n", get_current_timestamp_us(), right_size);
			}
			// DumpFile: Right
			if (dump_file && (dumpfile_mode & DUMP_LR_RAW_NV12)) {
				char right_pic_name[64] ={0};
				// sprintf(right_pic_name, "right_%d.yuv", dump_cnt);
				snprintf(right_pic_name,  sizeof(right_pic_name), "Right_client_Raw_cnt[%d].yuv", dump_cnt);
				dumpToFile(right_pic_name, (char *)right_buffer, right_size);
			}
		}

		/* Left Camera Data */
		if (0 == client_get_buffer_size_by_type((uint8_t *)data_item->items, STREAM_NODE_CAM_LEFT, (void **)&left_buffer, &left_size)) {
		#if 0
			if (preview_flag && left_buffer != nullptr && left_size > 0) {
				g_left_yuv = cv::Mat(height * 3 / 2, width, CV_8UC1, left_buffer).clone();
				cv::Mat left_bgr;
				cv::cvtColor(g_left_yuv, left_bgr, cv::COLOR_YUV2BGR_NV12);
				std::string left_txt = "Pos: (" + std::to_string(g_mousePos_left.x) + "," + 
									  std::to_string(g_mousePos_left.y) + ")";
				cv::putText(left_bgr, left_txt, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
				cv::imshow("Left Camera", left_bgr);
			}
		#endif



			// Verbose: Left
			if (verbose_mode & VERBOSE_LR_RAW_NV12) {
				printf("[%ld] Client Get Left Cam NV12: size[%d].\n", get_current_timestamp_us(), left_size);
			}
			// DumpFile: Left
			if (dump_file && (dumpfile_mode & DUMP_LR_RAW_NV12)) {
				char left_pic_name[64] ={0};
				snprintf(left_pic_name,  sizeof(left_pic_name), "Left_client_Raw_cnt[%d].yuv", dump_cnt);
				dumpToFile(left_pic_name, (char *)left_buffer, left_size);
			}
		}

		/* IMU Data */
		if (0 == client_get_buffer_size_by_type((uint8_t *)data_item->items, STREAM_NODE_IMU, (void **)&imu_buffer, &imu_size)) {
			client_parse_imu_buffer(imu_buffer, imu_size);
		}
		
		#endif

		dump_file = 0;

consumer_release:
		ret = sync_queue_repay_unused_object(sync_queue, 5000, data_item);
		if(ret != 0){
			printf("sync_queue_obtain_inused_object failed.\n");
			continue;
		}

		performance_test_stop_simple(&performace_total_consumer);
    }

	printf("Thread: consumer_process ==> Quit\n");
	running = 0;
    return NULL;
}


static int client_get_parse_stream_info(server_stream_info_t *server_stream_info)
{
	char *info_json_str = static_cast<char*>(malloc(STREAM_INFO_JSON_STR_LEN));
	if (!info_json_str) {
		printf("[Client] malloc for JSON info Failed\n");
		return -1;
	}
	int len_recv = recv(client_fd, info_json_str, STREAM_INFO_JSON_STR_LEN, MSG_WAITALL);
	if (len_recv < 0) {
		printf("[Client] recv Err[%d] %s", len_recv, strerror(errno));
		free(info_json_str);
		return -1;
	}
	else if (len_recv == 0) {
		printf("[Client] Closed.\n");
		free(info_json_str);
		return -1;
	}

	int index = 0;
	stream_node_info_t *node_info = NULL;
	cJSON *node_json_info = NULL;

	cJSON *root = cJSON_Parse(info_json_str);
	if (root == NULL)
		return -1;

	cJSON *stream_info = cJSON_GetObjectItemCaseSensitive(root, "StreamInformation");
	if (stream_info) {
        cJSON *version = cJSON_GetObjectItemCaseSensitive(stream_info, "ModelVersion");
        if (cJSON_IsString(version))
            snprintf(server_stream_info->model_version, 32, "%s", version->valuestring);

        cJSON *total_size = cJSON_GetObjectItemCaseSensitive(stream_info, "TotalSize");
        if (cJSON_IsNumber(total_size))
            server_stream_info->total_size = total_size->valueint;

		cJSON *fx = cJSON_GetObjectItemCaseSensitive(stream_info, "fx");
        if (cJSON_IsNumber(fx))
            intr.fx = fx->valuedouble;
		cJSON *fy = cJSON_GetObjectItemCaseSensitive(stream_info, "fy");
        if (cJSON_IsNumber(fy))
            intr.fy = fy->valuedouble;
		cJSON *cx = cJSON_GetObjectItemCaseSensitive(stream_info, "cx");
        if (cJSON_IsNumber(cx))
            intr.cx = cx->valuedouble;
		cJSON *cy = cJSON_GetObjectItemCaseSensitive(stream_info, "cy");
        if (cJSON_IsNumber(cy))
            intr.cy = cy->valuedouble;
    }


	char right_name[]="Right-Cam",left_name[]="Left-Cam";
	char depth_name[]="Depth",imu_name[]="IMU";
	for (index = 0; index < STREAM_NODE_TYPE_NUM; index++) {
		node_info = (stream_node_info_t *)&server_stream_info->stream_nodes[index];
		switch (index) {
			case STREAM_NODE_CAM_RIGHT:
				node_info->name = right_name;
				break;
			case STREAM_NODE_CAM_LEFT:
				node_info->name = left_name;
				break;
			case STREAM_NODE_DEPTH:
				node_info->name = depth_name;
				break;
			case STREAM_NODE_IMU:
				node_info->name = imu_name;
				break;
		}
		node_json_info = cJSON_GetObjectItemCaseSensitive(root, node_info->name);
		if (node_json_info) {
			node_info->available = cJSON_GetObjectItem(node_json_info, "Available")->valueint;
			node_info->width = cJSON_GetObjectItem(node_json_info, "Width")->valueint;
			node_info->height = cJSON_GetObjectItem(node_json_info, "Height")->valueint;
			node_info->size = cJSON_GetObjectItem(node_json_info, "Size")->valueint;
			node_info->offset = cJSON_GetObjectItem(node_json_info, "Offset")->valueint;
		}
	}

	if (root)
		cJSON_free(root);

	/* Debug Print Stream Info */
	printf("================ Stream Information ================\n");
	printf("Model Verion: %s\n", server_stream_info->model_version);
	printf("Stream Total Size: %d\n", server_stream_info->total_size);
	printf("Node Num: %d\n", STREAM_NODE_TYPE_NUM);
	printf("fx: %f\n", intr.fx);
	printf("fy: %f\n", intr.fy);
	printf("cx: %f\n", intr.cx);
	printf("cy: %f\n", intr.cy);
	for (index = 0; index < STREAM_NODE_TYPE_NUM; index++) {
		node_info = (stream_node_info_t *)&server_stream_info->stream_nodes[index];
		printf("+++++++++++++++++++++++++++++++++\n");
		printf("+    [%d] %s\n", index, node_info->name);
		printf("+    Available: %d\n", node_info->available);
		printf("+    Resolution: %d x %d\n", node_info->width, node_info->height);
		printf("+    Size: %d\n", node_info->size);
		printf("+    Offset: %d\n", node_info->offset);
		printf("+++++++++++++++++++++++++++++++++\n");
	}
	printf("=========================================================\n");

	return 0;
}

static struct option const long_options[] = {
	{"interface", required_argument, NULL, 'i'},
	{"server", required_argument, NULL, 's'},
	{"verbose", required_argument, NULL, 'v'},
	{"preview", no_argument, NULL, 'p'},
	{"dump", required_argument, NULL, 'd'},
	{NULL, 0, NULL, 0}
};

static void print_help(void)
{
	printf("Usage: [Options]\n");
	printf("Options:\n");
	printf("-i, --interface=\"Network interface\"\n");
	printf("-s, --server=\"Server ip\"\n");
	printf("-v, --verbose\tEnable verbose mode\n");
	printf("-p, --preview\tPreview depth (JET) and RGB frames in real-time\n");
}


int main(int argc, char** argv)
{
	int ret;
	int c = 0;
	char ifname[32] = {0};
	char server_ip[64] = {0};

	if (argc <= 1) {
		print_help();
		return 0;
	}

	while ((c = getopt_long(argc, argv, "i:s:d:v:p", long_options, NULL)) != -1) {
		switch (c) {
		case 'i':
			strcpy(ifname, optarg);
			break;
		case 's':
			strcpy(server_ip, optarg);
			break;
		case 'v':
			verbose_mode = atoi(optarg);
			break;
		case 'd':
			dumpfile_mode = atoi(optarg);
			break;
		case 'p':
            		preview_flag = 1;
            		break;
		default:
			print_help();
			return 0;
		}

	}
	signal(SIGINT, signal_handle);

	if (0 != check_network(ifname)) {
		printf("Network [%s] is not Ready\n", ifname);
		return 0;
	}

	client_fd = client_init(server_ip);
	if (client_fd < 0) {
		printf("[Client] init error\n");
		return 0;
	}

	if (0 != client_get_parse_stream_info(&stream_info)) {
		printf("[Client] client_get_parse_stream_info error\n");
		close(client_fd);
		return 0;
	}

	char prod_name[]="recv_data",cons_name[]="consumer";
	sync_queue_info_t sync_queue_info_recv_consumer = {
		.productor_name = prod_name,
		.consumer_name = cons_name,

		.is_need_malloc_in_advance = 0,
		.is_external_buffer = 0,

		.queue_len = 3,
		.data_item_size = static_cast<int>(stream_info.total_size),
		.data_item_count = 1,

		.item_data_init_param = NULL,
		.item_data_init_func = NULL,
		.item_data_deinit_param = NULL,
		.item_data_deinit_func = NULL,
	};

	ret = sync_queue_create(&recv_to_consumer, &sync_queue_info_recv_consumer);
	if(ret != 0){
		printf("sync queue create failed for right cam.\n");
		goto main_err_out;
		return -1;
	}

	running = 1;
	pthread_create(&client_data_thread, NULL, client_recv_data, NULL);
	pthread_create(&consumer_thread, NULL, consumer_process, NULL);

	if (client_data_thread > 0) {
		pthread_join(client_data_thread, NULL);
		client_data_thread = 0;
	}
	if (consumer_thread > 0) {
		pthread_join(consumer_thread, NULL);
		consumer_thread = 0;
	}

main_err_out:
	close(client_fd);
	sync_queue_destory(&recv_to_consumer);

	return 0;
}

