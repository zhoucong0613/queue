#ifndef _STEREO_SIMPLE_H_
#define _STEREO_SIMPLE_H_

#include <stdint.h>
#include "common.h"
#include "eeprom_calib.h"

#define MODE_RAW_DEPTH	1
#define MODE_ISP_DEPTH	2
#define MODE_DEFAULT MODE_RAW_DEPTH
#define MODE_JSON_STR_LEN	128

typedef struct {
	char *name;
	int available;
	uint32_t width;
	uint32_t height;
	uint32_t size;
	uint32_t offset;
}stream_node_info_t;

typedef struct {
	char model_version[32];
	uint32_t total_size;
	stream_node_info_t stream_nodes[STREAM_NODE_TYPE_NUM];
}server_stream_info_t;

typedef struct {
    double fx, fy, cx, cy, baseline;
} CameraIntrinsics;

int stereo_client_create(const char* ifname, const char* server_ip);
int stereo_client_init(int client_fd, int selected_mode, server_stream_info_t* stream_info, CameraIntrinsics* intr, CompleteCalibData_t *calib_data);
void stereo_client_release(int client_fd);

#endif