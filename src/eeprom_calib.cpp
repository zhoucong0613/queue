#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cstdint>

#include "eeprom_calib.h"


static uint8_t calc_checksum(const void* data, uint32_t len) {
    if (!data || len == 0) return 0;
    uint32_t sum = 0;
    const uint8_t* ptr = (const uint8_t*)data;
    for (uint32_t i = 0; i < len; i++) {
        sum += ptr[i];
    }
    return (uint8_t)((sum % 0xFF) + 1);
}

static int sn_char_to_num(char c, int type) {

    c = toupper(c);
    
    if (c >= '1' && c <= '9') return c - '0';
    else if (type == 0 && c >= 'A' && c <= 'C') return 10 + (c - 'A');
    else if (type == 1 && c >= 'A' && c <= 'X') return 10 + (c - 'A');
    return -1;
}

int eeprom_calib_verify(CompleteCalibData_t* calib) {
    if (!calib) return -1;

    uint8_t calc_head_cksum = calc_checksum(&calib->head, 15);
    if (calc_head_cksum != calib->head.head_checksum) {
        printf("头校验和失败（存储:0x%02X, 计算:0x%02X）\n", calib->head.head_checksum, calc_head_cksum);
        return -1;
    }

    uint32_t lr_data_len = sizeof(MonoCamCalib_t)*2 + sizeof(StereoUnionParam_t) + sizeof(StereoExtrinsic_t);
    uint8_t calc_lr_cksum = calc_checksum(&calib->left_cam, lr_data_len);
    if (calc_lr_cksum != calib->lr_checksum) {
        printf("LR校验和失败（存储:0x%02X, 计算:0x%02X）\n", calib->lr_checksum, calc_lr_cksum);
        return -1;
    }

    printf("所有标定数据校验和验证通过！\n");
    return 0;
}

int eeprom_calib_verify_sn(const SnData_t* sn) {
    if (!sn) return -1;

    uint8_t calc_cksum = calc_checksum(sn, SN_DATA_LEN);
    uint8_t stored_cksum = (uint8_t)sn->checksum;
    
    if (calc_cksum != stored_cksum) {
        printf("SN校验和失败（存储:0x%02X, 计算:0x%02X）\n", stored_cksum, calc_cksum);
        return -1;
    }

    printf("SN校验和验证通过！\n");
    return 0;
}

void eeprom_calib_print_sn(const SnData_t* sn) {
    if (!sn) return;

    char vendor = toupper(sn->vendor);
    int year = 2000 + (sn->year_ten - '0') * 10 + (sn->year_unit - '0');
    int month_num = sn_char_to_num(sn->month, 0);
    int day_num = sn_char_to_num(sn->day, 1);

    printf("\n===================================== SN 信息 =====================================\n");
    printf("分辨率：%c（%s）\n", sn->resolution,
           sn->resolution == '0' ? "VGA" : (sn->resolution == '1' ? "HD" : "FHD"));
    printf("芯片厂商：%c（%s）\n", vendor,
           vendor == 'O' ? "OV" : (vendor == 'T' ? "思特威" : "未知"));
    printf("模组型号：%c%c%c\n", sn->model1, sn->model2, sn->model3);
    printf("生产日期：%d年%d月%d日（原始ASCII：%c月%c日）\n",
           year, month_num, day_num, sn->month, sn->day);
    printf("唯一标识：%.*s\n", 5, sn->unique);
    printf("校验和：0x%02X\n", (uint8_t)sn->checksum);
    printf("==================================================================================\n");
}

int eeprom_calib_save_to_yaml(CompleteCalibData_t* calib) {
    if (!calib) return -1;
    FILE* fp = fopen(YAML_FILENAME, "w");
    if (!fp) { perror("Err: Open YAML failed"); return -1; }

    #define FMT_FLOAT "%.16f"
    #define FMT_EPS   "%.16e"

    // YAML内容生成
    fprintf(fp, "%%YAML:1.0\nstereo0:\n");
    // 左相机
    fprintf(fp, "  cam0:\n    cam_overlaps: [1]\n    camera_model: pinhole\n");
    fprintf(fp, "    distortion_coeffs: [" FMT_FLOAT ", " FMT_FLOAT ", " FMT_EPS ", " FMT_EPS ", " FMT_FLOAT ", " FMT_FLOAT ", " FMT_FLOAT ", " FMT_FLOAT "]\n",
            calib->left_cam.k1, calib->left_cam.k2, calib->left_cam.p1, calib->left_cam.p2,
            calib->left_cam.k3, calib->left_cam.k4, calib->left_cam.k5, calib->left_cam.k6);
    fprintf(fp, "    distortion_model: rational_polynomial\n");
    fprintf(fp, "    intrinsics: [" FMT_FLOAT ", " FMT_FLOAT ", " FMT_FLOAT ", " FMT_FLOAT "]\n",
            calib->left_cam.fx, calib->left_cam.fy, calib->left_cam.cx, calib->left_cam.cy);
    fprintf(fp, "    resolution: [%d, %d]\n    rostopic: /cam0/image_raw\n",
            calib->left_cam.width, calib->left_cam.height);
    // 右相机
    fprintf(fp, "  cam1:\n    T_cn_cnm1:\n");
    fprintf(fp, "      - [" FMT_FLOAT ", " FMT_FLOAT ", " FMT_FLOAT ", " FMT_EPS "]\n",
            calib->extrinsic.R[0], calib->extrinsic.R[1], calib->extrinsic.R[2], calib->extrinsic.T[0]/1000.0);
    fprintf(fp, "      - [" FMT_FLOAT ", " FMT_FLOAT ", " FMT_FLOAT ", " FMT_EPS "]\n",
            calib->extrinsic.R[3], calib->extrinsic.R[4], calib->extrinsic.R[5], calib->extrinsic.T[1]/1000.0);
    fprintf(fp, "      - [" FMT_FLOAT ", " FMT_FLOAT ", " FMT_FLOAT ", " FMT_EPS "]\n",
            calib->extrinsic.R[6], calib->extrinsic.R[7], calib->extrinsic.R[8], calib->extrinsic.T[2]/1000.0);
    fprintf(fp, "      - [0.0, 0.0, 0.0, 1.0]\n");
    fprintf(fp, "    cam_overlaps: [0]\n    camera_model: pinhole\n");
    fprintf(fp, "    distortion_coeffs: [" FMT_FLOAT ", " FMT_FLOAT ", " FMT_EPS ", " FMT_EPS ", " FMT_FLOAT ", " FMT_FLOAT ", " FMT_FLOAT ", " FMT_FLOAT "]\n",
            calib->right_cam.k1, calib->right_cam.k2, calib->right_cam.p1, calib->right_cam.p2,
            calib->right_cam.k3, calib->right_cam.k4, calib->right_cam.k5, calib->right_cam.k6);
    fprintf(fp, "    distortion_model: rational_polynomial\n");
    fprintf(fp, "    intrinsics: [" FMT_FLOAT ", " FMT_FLOAT ", " FMT_FLOAT ", " FMT_FLOAT "]\n",
            calib->right_cam.fx, calib->right_cam.fy, calib->right_cam.cx, calib->right_cam.cy);
    fprintf(fp, "    resolution: [%d, %d]\n    rostopic: /cam1/image_raw\n",
            calib->right_cam.width, calib->right_cam.height);
    fprintf(fp, "    alpha: " FMT_FLOAT "\n", calib->stereo_param.alpha);

    #undef FMT_FLOAT
    #undef FMT_EPS
    fclose(fp);
    printf("YAML文件保存成功：%s\n", YAML_FILENAME);
    return 0;
}

int eeprom_calib_save_sn_to_file(const SnData_t* sn) {
    if (!sn) {
        fprintf(stderr, "Err: Invalid SN param\n");
        return -1;
    }

    FILE* fp = fopen(SN_FILENAME, "w");
    if (!fp) {
        perror("Err: Open SN file failed");
        return -1;
    }

    char vendor = toupper(sn->vendor);
    int year = 2000 + (sn->year_ten - '0') * 10 + (sn->year_unit - '0');
    int month_num = sn_char_to_num(sn->month, 0);
    int day_num = sn_char_to_num(sn->day, 1);

    fprintf(fp, "===================================== SN 信息 =====================================\n");
    fprintf(fp, "分辨率：%c（%s）\n", sn->resolution,
            sn->resolution == '0' ? "VGA" : (sn->resolution == '1' ? "HD" : "FHD"));
    fprintf(fp, "芯片厂商：%c（%s）\n", vendor,
            vendor == 'O' ? "OV" : (vendor == 'T' ? "思特威" : "未知"));
    fprintf(fp, "模组型号：%c%c%c\n", sn->model1, sn->model2, sn->model3);
    fprintf(fp, "生产日期：%d年%d月%d日（原始ASCII：%c月%c日）\n",
            year, month_num, day_num, sn->month, sn->day);
    fprintf(fp, "唯一标识：%.*s\n", 5, sn->unique);
    fprintf(fp, "校验和：0x%02X\n", (uint8_t)sn->checksum);
    fprintf(fp, "==================================================================================\n");

    fflush(fp);
    fclose(fp);
    printf("SN文件保存成功：%s\n", SN_FILENAME);
    return 0;
}