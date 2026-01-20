#ifndef __EEPROM_CALIB_H__
#define __EEPROM_CALIB_H__

#define YAML_FILENAME       "stereo_abh.yaml"
#define SN_FILENAME         "sn_info.txt"

/* ==================== 标定数据地址映射==================== */
#define ADDR_HEAD_INFO      0x0000  // 头信息（16字节）
#define ADDR_L_CALIB        0x0010  // 左相机参数（56字节）
#define ADDR_R_CALIB        0x0048  // 右相机参数（56字节）
#define ADDR_STEREO_PARAM   0x0080  // 双目联合参数（12字节）
#define ADDR_EXTRINSIC      0x008C  // 外参（48字节）
#define ADDR_HEAD_CHECKSUM  0x000F  // 头校验和
#define ADDR_LR_CHECKSUM    0x0124  // LR校验和

/* ==================== SN 地址映射 ==================== */
#define SN_START_ADDR       0x0125    // SN起始地址
#define SN_DATA_LEN         14        // 有效数据长度（0x0125-0x0132，14个字节，排除校验和）
#define SN_TOTAL_BYTES      15        // 总字节数（0x0125-0x0133，14个数据+1个校验和）

// 结构体定义（保持不变）
typedef struct __attribute__((packed)) {
    uint8_t  flag[8];
    uint8_t  module_type;
    uint8_t  calib_way;
    uint8_t  calib_version;
    uint8_t  reserved1;
    uint8_t  calib_rotate;
    uint8_t  calib_num;
    uint8_t  reserved2;
    uint8_t  head_checksum;
} EepromHeadInfo_t;

typedef struct __attribute__((packed)) {
    int32_t  width;
    int32_t  height;
    float    fx;
    float    fy;
    float    cx;
    float    cy;
    float    k1, k2, p1, p2;
    float    k3, k4, k5, k6;
} MonoCamCalib_t;

typedef struct __attribute__((packed)) {
    float    F_pixel;
    float    B_mm;
    float    alpha;
} StereoUnionParam_t;

typedef struct __attribute__((packed)) {
    float    R[9];
    float    T[3];
} StereoExtrinsic_t;

typedef struct __attribute__((packed)) {
    char resolution;
    char vendor;
    char model1;
    char model2;
    char model3;
    char year_ten;
    char year_unit;
    char month;
    char day;
    char unique[5];
    char checksum;
} SnData_t;

typedef struct {
    EepromHeadInfo_t     head;
    MonoCamCalib_t       left_cam;
    MonoCamCalib_t       right_cam;
    StereoUnionParam_t   stereo_param;
    StereoExtrinsic_t    extrinsic;
    uint8_t              lr_checksum;
    SnData_t             sn; 
} CompleteCalibData_t;


int eeprom_calib_verify(CompleteCalibData_t* calib);
int eeprom_calib_verify_sn(const SnData_t* sn);
void eeprom_calib_print_sn(const SnData_t* sn);
int eeprom_calib_save_to_yaml(CompleteCalibData_t* calib);
int eeprom_calib_save_sn_to_file(const SnData_t* sn);

#endif  // __EEPROM_CALIB_H__