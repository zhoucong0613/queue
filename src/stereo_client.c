#include <stdio.h>
#include <string.h>
#include <net/if.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdlib.h>

#include "stereo_client.h"
#include "eeprom_calib.h"
#include "cJSON.h"
#include "common.h"

int stereo_client_create(const char* ifname, const char* server_ip) {

    if (check_network(ifname) != 0) {
        printf("Network [%s] is not Ready\n", ifname);
        return -1; 
    }

    int client_fd = client_init(server_ip);
    if (client_fd < 0) {
        printf("[Client] init error\n");
        return -1;
    }

    return client_fd;
}

static int client_send_mode_to_server(int fd, int mode)
{
	if (fd < 0) {
		fprintf(stderr, "Invalid client fd for sending mode\n");
		return -1;
	}

	char mode_json[MODE_JSON_STR_LEN] = {0};
	snprintf(mode_json, MODE_JSON_STR_LEN, "{\"Mode\": %d}", mode);

	// size_t json_len = strlen(mode_json);
	// printf("[Client] Prepare to send mode msg (len=%zu): %s\n", json_len, mode_json);

	ssize_t sent_len = send(fd, mode_json, strlen(mode_json), 0);
	if (sent_len < 0) {
		fprintf(stderr, "Send mode to server failed: %s\n", strerror(errno));
		return -1;
	} else if (sent_len != strlen(mode_json)) {
		fprintf(stderr, "Send mode incomplete: sent %ld/%zu bytes\n", sent_len, strlen(mode_json));
		return -1;
	}

	printf("Successfully sent mode to server: %d (%s)\n", mode, mode == MODE_RAW_DEPTH ? "raw" : "isp");
	return 0;
}

static int client_recv_calib_sn_data(int fd, CompleteCalibData_t *calib_data)
{
    uint32_t recv_len = 0;
    ssize_t recv_ret = 0;

    recv_ret = recv(fd, &recv_len, sizeof(uint32_t), 0);
    if (recv_ret < 0) {
        fprintf(stderr, "[Client] Recv calib data len failed: %s\n", strerror(errno));
        return -1;
    } else if (recv_ret == 0) {
        fprintf(stderr, "[Client] Server disconnected when recv data len\n");
        return -1;
    } else if (recv_ret != sizeof(uint32_t)) {
        fprintf(stderr, "[Client] Recv calib len incomplete: got %ld/%zu bytes\n",
                recv_ret, sizeof(uint32_t));
        return -1;
    }

    if (recv_len != sizeof(CompleteCalibData_t)) {
        fprintf(stderr, "[Client] Calib data len mismatch! Expected: %zu, Got: %u\n",
                sizeof(CompleteCalibData_t), recv_len);
        return -1;
    }

    recv_ret = recv(fd, calib_data, recv_len, 0);
    if (recv_ret < 0) {
        fprintf(stderr, "[Client] Recv calib+SN data failed: %s\n", strerror(errno));
        return -1;
    } else if (recv_ret == 0) {
        fprintf(stderr, "[Client] Server disconnected when recv calib data\n");
        return -1;
    } else if (recv_ret != (ssize_t)recv_len) {
        fprintf(stderr, "[Client] Recv calib data incomplete: got %ld/%u bytes\n",
                recv_ret, recv_len);
        return -1;
    }

    if (eeprom_calib_verify(calib_data) != 0) {
        fprintf(stderr, "[Client] Verify calib data failed (invalid or corrupted)\n");
        return -1;
    }
    if (eeprom_calib_verify_sn(&calib_data->sn) != 0) {
        fprintf(stderr, "[Client] Verify SN data failed (invalid or corrupted)\n");
        return -1;
    }

    // eeprom_calib_print_sn(&calib_data.sn);

    if (eeprom_calib_save_to_yaml(calib_data) != 0) {
        fprintf(stderr, "[Client] Save calib data to YAML failed\n");
    }

    if (eeprom_calib_save_sn_to_file(&calib_data->sn) != 0) {
        fprintf(stderr, "[Client] Save SN data to file failed\n");
    }

    printf("[Client] Calib+SN data process completed (verify/save YAML/save SN file)\n");
    return 0;
}

static int client_get_parse_stream_info(int fd, server_stream_info_t *server_stream_info, CameraIntrinsics* intr)
{
	char *info_json_str = malloc(STREAM_INFO_JSON_STR_LEN);
	if (!info_json_str) {
		printf("[Client] malloc for JSON info Failed\n");
		return -1;
	}
	int len_recv = recv(fd, info_json_str, STREAM_INFO_JSON_STR_LEN, MSG_WAITALL);
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
            intr->fx = fx->valuedouble;
		cJSON *fy = cJSON_GetObjectItemCaseSensitive(stream_info, "fy");
        if (cJSON_IsNumber(fy))
            intr->fy = fy->valuedouble;
		cJSON *cx = cJSON_GetObjectItemCaseSensitive(stream_info, "cx");
        if (cJSON_IsNumber(cx))
            intr->cx = cx->valuedouble;
		cJSON *cy = cJSON_GetObjectItemCaseSensitive(stream_info, "cy");
        if (cJSON_IsNumber(cy))
            intr->cy = cy->valuedouble;
		cJSON *baseline = cJSON_GetObjectItemCaseSensitive(stream_info, "baseline");
        if (cJSON_IsNumber(baseline))
            intr->baseline = baseline->valuedouble;
    }

	for (index = 0; index < STREAM_NODE_TYPE_NUM; index++) {
		node_info = (stream_node_info_t *)&server_stream_info->stream_nodes[index];
		switch (index) {
			case STREAM_NODE_CAM_RIGHT:
				node_info->name = "Right-Cam";
				break;
			case STREAM_NODE_CAM_LEFT:
				node_info->name = "Left-Cam";
				break;
			case STREAM_NODE_DEPTH:
				node_info->name = "Depth";
				break;
			case STREAM_NODE_IMU:
				node_info->name = "IMU";
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
	printf("fx: %f\n", intr->fx);
	printf("fy: %f\n", intr->fy);
	printf("cx: %f\n", intr->cx);
	printf("cy: %f\n", intr->cy);
	printf("baseline: %f\n", intr->baseline);
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

int stereo_client_init(int client_fd, int selected_mode, server_stream_info_t* stream_info, CameraIntrinsics* intr, CompleteCalibData_t *calib_data)
{
    if (client_fd < 0) {
        fprintf(stderr, "Invalid client fd for stereo client init\n");
        return -1;
    }
    if (stream_info == NULL) {
        fprintf(stderr, "Null pointer for stream info\n");
        return -1;
    }
    if (intr == NULL) {
        fprintf(stderr, "Null pointer for CameraIntrinsics\n");
        return -1;
    }
    if (selected_mode != MODE_RAW_DEPTH && selected_mode != MODE_ISP_DEPTH) {
        fprintf(stderr, "Invalid working mode (only 1/2 supported)\n");
        return -1;
    }

    if (client_send_mode_to_server(client_fd, selected_mode) != 0) {
        fprintf(stderr, "Failed to send mode to server\n");
        return -1;
    }

    if (client_recv_calib_sn_data(client_fd, calib_data) != 0) {
        fprintf(stderr, "Failed to receive and verify calib+SN data\n");
        return -1;
    }

    if (client_get_parse_stream_info(client_fd, stream_info, intr) != 0) {
        printf("[Client] client_get_parse_stream_info error\n");
        return -1;
    }

    return 0;
}

void stereo_client_release(int client_fd)
{
    if (client_fd >= 0) {
        close(client_fd);
    }
}