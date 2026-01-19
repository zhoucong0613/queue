import ctypes
import os
import signal
import time
import socket
import json
from ctypes import (
    c_char, c_char_p, c_int, c_uint32, c_uint64, c_void_p, c_uint8, c_uint16,
    c_float, c_double, POINTER, cast, byref, string_at, create_string_buffer
)

# ************************ 1. 配置与加载.so库 ************************
# 配置.so库路径（替换为你的实际路径）
LIB_DIR = "./lib"
LIB_NAME = "libstereo.so"
lib_path = os.path.join(LIB_DIR, LIB_NAME)

# 设置环境变量，确保加载依赖
os.environ["LD_LIBRARY_PATH"] = f"{LIB_DIR}:{os.environ.get('LD_LIBRARY_PATH', '')}"

# 加载.so库
try:
    libstereo = ctypes.CDLL(lib_path)
except OSError as e:
    print(f"加载{LIB_NAME}失败：{e}")
    exit(1)

# ************************ 2. 复刻原C的所有宏定义（与原main.c完全一致） ************************
# 流节点类型（必须与原C/common.h一致）
STREAM_NODE_TYPE_NUM = 4
STREAM_NODE_CAM_RIGHT = 0
STREAM_NODE_CAM_LEFT = 1
STREAM_NODE_DEPTH = 2
STREAM_NODE_IMU = 3
STREAM_NODE_TAIL = 4

# Debug Dump File
DUMP_LR_RAW_NV12 = 1 << 0
DUMP_DEPTH_FILE = 1 << 1
DUMP_POINTCLOUD_FILE = 1 << 2

# Debug Verbose
VERBOSE_STREAM = 1 << 0
VERBOSE_LR_RAW_NV12 = 1 << 1
VERBOSE_DEPTH_POINTCLOUD = 1 << 2
VERBOSE_IMU = 1 << 3

# 工作模式
MODE_RAW_DEPTH = 1
MODE_ISP_DEPTH = 2
MODE_DEFAULT = MODE_RAW_DEPTH
MODE_JSON_STR_LEN = 128

# 分辨率（与原C/render_depth.h一致，替换为你的实际值）
STEREO_RES_WIDTH = 640
STEREO_RES_HEIGHT = 480

# 流头部魔术值（与原C/common.h一致）
STREAM_HEADER_MAGIC = 0x12345678

# ************************ 3. 复刻原C的所有结构体（字段、顺序与原C完全一致） ************************
# 3.1 原main.c：CameraIntrinsics
class CameraIntrinsics(ctypes.Structure):
    _fields_ = [
        ("fx", c_double),
        ("fy", c_double),
        ("cx", c_double),
        ("cy", c_double),
        ("baseline", c_double)
    ]

# 3.2 原main.c：PointCloud
class PointCloud(ctypes.Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float)
    ]

# 3.3 原main.c：stream_node_info_t
class stream_node_info_t(ctypes.Structure):
    _fields_ = [
        ("name", c_char_p),
        ("available", c_int),
        ("width", c_uint32),
        ("height", c_uint32),
        ("size", c_uint32),
        ("offset", c_uint32)
    ]

# 3.4 原main.c：server_stream_info_t
class server_stream_info_t(ctypes.Structure):
    _fields_ = [
        ("model_version", c_char * 32),
        ("total_size", c_uint32),
        ("stream_nodes", stream_node_info_t * STREAM_NODE_TYPE_NUM)
    ]

# 3.5 so库相关：stream_header_t（原C/common.h）
class stream_header_t(ctypes.Structure):
    _fields_ = [
        ("magic", c_uint32),
        ("total_size", c_uint32),
        ("node_headers", c_void_p)  # 简化，仅需magic和total_size
    ]

# 3.6 so库相关：imu_subnode_data_t（原C/imu.h）
class imu_subnode_data_t(ctypes.Structure):
    _fields_ = [
        ("timestamp", c_uint64),
        ("ax", c_float),
        ("ay", c_float),
        ("az", c_float),
        ("gx", c_float),
        ("gy", c_float),
        ("gz", c_float)
    ]

# 3.7 so库相关：CompleteCalibData_t（原C/eeprom_calib.h）
class CompleteCalibData_t(ctypes.Structure):
    _fields_ = [
        ("sn", c_void_p),  # 简化，so库内实现细节
        ("calib_data", c_void_p)
    ]

# 3.8 so库相关：sync_queue_t（原C/synchronous_queue.h）
class sync_queue_t(ctypes.Structure):
    _fields_ = [
        ("sync_queue_info", c_void_p),
        ("unused", c_void_p),
        ("inused", c_void_p)
    ]

# 3.9 so库相关：sync_queue_info_t（原C/synchronous_queue.h）
class sync_queue_info_t(ctypes.Structure):
    _fields_ = [
        ("productor_name", c_char_p),
        ("consumer_name", c_char_p),
        ("is_need_malloc_in_advance", c_int),
        ("is_external_buffer", c_int),
        ("queue_len", c_int),
        ("data_item_size", c_uint32),
        ("data_item_count", c_int),
        ("item_data_init_param", c_void_p),
        ("item_data_init_func", c_void_p),
        ("item_data_deinit_param", c_void_p),
        ("item_data_deinit_func", c_void_p)
    ]

# 3.10 so库相关：data_item_t（原C/synchronous_queue.h）
class data_item_t(ctypes.Structure):
    _fields_ = [
        ("items", POINTER(c_uint8)),
        ("size", c_uint32)
    ]

# 3.11 so库相关：PerformanceTestParamSimple（原C/performance_test_util.h）
class PerformanceTestParamSimple(ctypes.Structure):
    _fields_ = [
        ("test_count", c_int),
        ("iteration_number", c_int),
        ("test_case", c_char_p),
        ("run_count", c_int)
    ]

# ************************ 4. 复刻原C的全局变量（与原main.c完全一致） ************************
# 全局变量（对应原C的static全局变量）
client_fd = -1
recv_to_consumer = sync_queue_t()
stream_info = server_stream_info_t()
running = c_int(0)
intr = CameraIntrinsics()
dumpfile_mode = c_uint32(0)
verbose_mode = c_uint32(0)
preview_flag = c_int(0)
selected_mode = c_int(MODE_DEFAULT)

# ************************ 5. 映射so库中的底层函数（用户说明：未展示的函数都在so中） ************************
# 5.1 网络检查：int check_network(const char* ifname)
libstereo.check_network.argtypes = [c_char_p]
libstereo.check_network.restype = c_int

# 5.2 客户端初始化：int client_init(const char* server_ip)
libstereo.client_init.argtypes = [c_char_p]
libstereo.client_init.restype = c_int

# 5.3 深度图预览：void show_depth_map(uint16_t* depth_data, uint32_t size, uint32_t width, uint32_t height)
libstereo.show_depth_map.argtypes = [POINTER(c_uint16), c_uint32, c_uint32, c_uint32]
libstereo.show_depth_map.restype = None

# 5.4 EEPROM校准验证：int eeprom_calib_verify(CompleteCalibData_t* calib_data)
libstereo.eeprom_calib_verify.argtypes = [POINTER(CompleteCalibData_t)]
libstereo.eeprom_calib_verify.restype = c_int

# 5.5 SN验证：int eeprom_calib_verify_sn(void* sn)
libstereo.eeprom_calib_verify_sn.argtypes = [c_void_p]
libstereo.eeprom_calib_verify_sn.restype = c_int

# 5.6 保存校准数据到YAML：int eeprom_calib_save_to_yaml(CompleteCalibData_t* calib_data)
libstereo.eeprom_calib_save_to_yaml.argtypes = [POINTER(CompleteCalibData_t)]
libstereo.eeprom_calib_save_to_yaml.restype = c_int

# 5.7 保存SN到文件：int eeprom_calib_save_sn_to_file(void* sn)
libstereo.eeprom_calib_save_sn_to_file.argtypes = [c_void_p]
libstereo.eeprom_calib_save_sn_to_file.restype = c_int

# 5.8 同步队列创建：int sync_queue_create(sync_queue_t* queue, sync_queue_info_t* info)
libstereo.sync_queue_create.argtypes = [POINTER(sync_queue_t), POINTER(sync_queue_info_t)]
libstereo.sync_queue_create.restype = c_int

# 5.9 获取空闲队列项：int sync_queue_get_unused_object(sync_queue_t* queue, int timeout, data_item_t** item)
libstereo.sync_queue_get_unused_object.argtypes = [POINTER(sync_queue_t), c_int, POINTER(POINTER(data_item_t))]
libstereo.sync_queue_get_unused_object.restype = c_int

# 5.10 保存已用队列项：int sync_queue_save_inused_object(sync_queue_t* queue, int timeout, data_item_t* item)
libstereo.sync_queue_save_inused_object.argtypes = [POINTER(sync_queue_t), c_int, POINTER(data_item_t)]
libstereo.sync_queue_save_inused_object.restype = c_int

# 5.11 获取已用队列项：int sync_queue_obtain_inused_object(sync_queue_t* queue, int timeout, data_item_t** item)
libstereo.sync_queue_obtain_inused_object.argtypes = [POINTER(sync_queue_t), c_int, POINTER(POINTER(data_item_t))]
libstereo.sync_queue_obtain_inused_object.restype = c_int

# 5.12 归还空闲队列项：int sync_queue_repay_unused_object(sync_queue_t* queue, int timeout, data_item_t* item)
libstereo.sync_queue_repay_unused_object.argtypes = [POINTER(sync_queue_t), c_int, POINTER(data_item_t)]
libstereo.sync_queue_repay_unused_object.restype = c_int

# 5.13 销毁队列：void sync_queue_destory(sync_queue_t* queue)
libstereo.sync_queue_destory.argtypes = [POINTER(sync_queue_t)]
libstereo.sync_queue_destory.restype = None

# 5.14 性能测试开始：void performance_test_start_simple(PerformanceTestParamSimple* param)
libstereo.performance_test_start_simple.argtypes = [POINTER(PerformanceTestParamSimple)]
libstereo.performance_test_start_simple.restype = None

# 5.15 性能测试停止：void performance_test_stop_simple(PerformanceTestParamSimple* param)
libstereo.performance_test_stop_simple.argtypes = [POINTER(PerformanceTestParamSimple)]
libstereo.performance_test_stop_simple.restype = None

# ************************ 6. 复刻原main.c的所有辅助函数（逻辑、输出完全一致） ************************
def signal_handle(signum, frame):
    """复刻原C：void signal_handle(int signo)"""
    running.value = 0
    print("\n[Client] Received SIGINT, exiting...")

def depth_to_pointcloud(depth_data, rows, cols, intr_ptr, pc_ptr):
    """复刻原C：void depth_to_pointcloud(const uint16_t* depth_data, int rows, int cols, const CameraIntrinsics* intr, PointCloud* pc)"""
    # 转换指针类型
    depth_ptr = cast(depth_data, POINTER(c_uint16))
    intr_c = cast(intr_ptr, POINTER(CameraIntrinsics)).contents
    pointTmp = cast(pc_ptr, POINTER(PointCloud))

    # 复刻原C的双重循环与指针移动
    for v in range(rows):
        for u in range(cols):
            d = depth_ptr[v * cols + u]
            if d > 0:
                # 与原C完全一致的坐标计算（毫米转米逻辑）
                x = (u - intr_c.cx) * d / intr_c.fx
                y = (v - intr_c.cy) * d / intr_c.fy
                z = d

                # 赋值给点云
                pointTmp.contents.x = x
                pointTmp.contents.y = y
                pointTmp.contents.z = z

            # 指针后移（复刻原C：pointTmp++）
            pointTmp = cast(cast(pointTmp, c_void_p).value + ctypes.sizeof(PointCloud), POINTER(PointCloud))

def save_pointcloud_to_txt(filename, pc, rows, cols):
    """复刻原C：void save_pointcloud_to_txt(const char* filename, PointCloud* pc, int rows, int cols)"""
    try:
        # 复刻原C的fopen("w")
        with open(filename, "w") as fp:
            pointTmp = cast(pc, POINTER(PointCloud))

            # 复刻原C的双重循环
            for v in range(rows):
                for u in range(cols):
                    # 复刻原C的fprintf格式
                    line = f"{pointTmp.contents.x:.6f} {pointTmp.contents.y:.6f} {pointTmp.contents.z:.6f}\n"
                    fp.write(line)

                    # 指针后移
                    pointTmp = cast(cast(pointTmp, c_void_p).value + ctypes.sizeof(PointCloud), POINTER(PointCloud))

            # 复刻原C的fflush
            fp.flush()

        print(f"Save PointClode To {filename}.")
    except OSError as e:
        print(f"Open File Failed: {filename} ({e})")
    except Exception as e:
        print(f"Write File Failed ({e})")

def dumpToFile(filename, srcBuf, size):
    """复刻原C：static int dumpToFile(char *filename, char *srcBuf, unsigned int size)"""
    try:
        # 复刻原C的fopen("w+")
        with open(filename, "wb+") as fd:
            # 复刻原C的fwrite
            fd.write(srcBuf[:size])
            # 复刻原C的fflush
            fd.flush()

        print(f"filedump({filename}, size({size})) is successed")
        return 0
    except OSError as e:
        print(f"ERRopen({filename}) fail ({e})")
        return -1

def get_current_timestamp_us():
    """复刻原C：static uint64_t get_current_timestamp_us()"""
    # 复刻原C的clock_gettime(CLOCK_REALTIME)
    ts = time.time()
    return c_uint64(int(ts * 1000000))

def client_check_stream_header(stream_buffer):
    """复刻原C：static int client_check_stream_header(uint8_t *stream_buffer)"""
    # 转换指针类型
    stream_header = cast(stream_buffer, POINTER(stream_header_t))

    # 复刻原C的verbose打印
    if verbose_mode.value & VERBOSE_STREAM:
        print(f"[Client] Stream Magic = 0x{stream_header.contents.magic:08x}, Total_size = {stream_header.contents.total_size}")

    # 复刻原C的magic验证
    if stream_header.contents.magic != STREAM_HEADER_MAGIC:
        print(f"Magic Error: 0x{stream_header.contents.magic:08x} (expected 0x{STREAM_HEADER_MAGIC:08x})")
        return -1

    return 0

def client_parse_imu_buffer(imu_buffer, imu_size):
    """复刻原C：static void client_parse_imu_buffer(void *imu_buffer, uint32_t imu_size)"""
    # 计算IMU数据个数
    imu_data_size = ctypes.sizeof(imu_subnode_data_t)
    imu_data_num = imu_size // imu_data_size
    imu_subnode_data = cast(imu_buffer, POINTER(imu_subnode_data_t))

    # 复刻原C的循环解析
    for i in range(imu_data_num):
        if verbose_mode.value & VERBOSE_IMU:
            # 复刻原C的打印格式
            print(f"[IMU] Data received pts[{imu_subnode_data.contents.timestamp}]: =======> ")
            print(f"  Accelerometer: [{imu_subnode_data.contents.ax:.6f}, {imu_subnode_data.contents.ay:.6f}, {imu_subnode_data.contents.az:.6f}] m/s²")
            print(f"  Gyroscope:     [{imu_subnode_data.contents.gx:.6f}, {imu_subnode_data.contents.gy:.6f}, {imu_subnode_data.contents.gz:.6f}] rad/s")

        # 指针后移
        imu_subnode_data = cast(cast(imu_subnode_data, c_void_p).value + imu_data_size, POINTER(imu_subnode_data_t))

def client_get_buffer_size_by_type(stream_buffer, type_, data_buffer, size):
    """复刻原C：static int client_get_buffer_size_by_type(uint8_t *stream_buffer, int type, void **data_buffer, uint32_t *size)"""
    # 复刻原C的类型验证
    if type_ < 0 or type_ >= STREAM_NODE_TAIL:
        print(f"client_get_buffer_by_type: type error[{type_}]")
        return -1

    # 复刻原C的available验证
    node_info = stream_info.stream_nodes[type_]
    if node_info.available == 0:
        return -1

    # 转换指针类型
    stream_header = cast(stream_buffer, POINTER(stream_header_t))
    node_header = cast(stream_header.contents.node_headers, POINTER(c_void_p))  # 简化

    # 计算数据缓冲区地址（复刻原C逻辑）
    data_ptr = cast(stream_buffer, c_void_p).value + ctypes.sizeof(stream_header_t)
    node_offset = stream_info.stream_nodes[type_].offset
    node_size = stream_info.stream_nodes[type_].size

    # 复刻原C的大小验证
    if node_size <= 0:
        return -1

    # 赋值输出参数
    cast(data_buffer, POINTER(c_void_p)).contents.value = data_ptr + node_offset
    cast(size, POINTER(c_uint32)).contents.value = node_size

    return 0

# ************************ 7. 复刻原main.c的核心线程函数 ************************
def client_recv_data(context):
    """复刻原C：void *client_recv_data(void *context)"""
    print("[Thread] client_recv_data ==> Start")

    sync_queue = byref(recv_to_consumer)
    data_item = POINTER(data_item_t)()
    sync_queue_info = cast(recv_to_consumer.sync_queue_info, POINTER(sync_queue_info_t))

    while running.value:
        # 1. 复刻原C：sync_queue_get_unused_object
        ret = libstereo.sync_queue_get_unused_object(sync_queue, 2000, byref(data_item))
        if ret != 0:
            print("sync_queue_get_unused_object Failed")
            break

        # 2. 复刻原C：recv(client_fd, ..., MSG_WAITALL)
        if client_fd < 0:
            break
        try:
            # 转换为Python socket（复刻原C的client_fd）
            sock = socket.fromfd(client_fd, socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            data = sock.recv(sync_queue_info.contents.data_item_size, socket.MSG_WAITALL)

            len_recv = len(data)
            if len_recv < 0:
                print(f"[Client] recv Err[{len_recv}] {os.strerror(errno)}")
                break
            elif len_recv == 0:
                print("[Client] Closed.")
                break

            # 复制数据到队列项缓冲区
            ctypes.memmove(data_item.contents.items, data, len_recv)
        except Exception as e:
            print(f"[Client] recv Err: {e}")
            break

        # 3. 复刻原C：sync_queue_save_inused_object
        ret = libstereo.sync_queue_save_inused_object(sync_queue, 2000, data_item)
        if ret != 0:
            print("sync_queue_save_inused_object Failed")
            break

    # 4. 复刻原C：设置running=0
    running.value = 0
    print("Thread: client_recv_data ==> Quit")
    return None

def consumer_process(context):
    """复刻原C：void *consumer_process(void *context)"""
    # 复刻原C的局部变量
    ret = 0
    sync_queue = byref(recv_to_consumer)
    data_item = POINTER(data_item_t)()

    depth_buffer = c_void_p()
    depth_size = c_uint32(0)
    right_buffer = c_void_p()
    right_size = c_uint32(0)
    left_buffer = c_void_p()
    left_size = c_uint32(0)
    imu_buffer = c_void_p()
    imu_size = c_uint32(0)

    dump_cnt = 0
    dump_file = 0

    # 复刻原C：malloc点云缓冲区
    pc_size = STEREO_RES_WIDTH * STEREO_RES_HEIGHT
    pc = (PointCloud * pc_size)()
    pc_ptr = cast(pc, POINTER(PointCloud))

    # 复刻原C：性能测试结构体
    performace_total_consumer = PerformanceTestParamSimple(
        test_count=0,
        iteration_number=30 * 60,
        test_case=b"consumer",
        run_count=0
    )

    performace_total_depth2cloud = PerformanceTestParamSimple(
        test_count=0,
        iteration_number=30 * 60,
        test_case=b"depth2cloud",
        run_count=0
    )

    while running.value:
        # 1. 复刻原C：performance_test_start_simple
        libstereo.performance_test_start_simple(byref(performace_total_consumer))

        # 2. 复刻原C：sync_queue_obtain_inused_object
        ret = libstereo.sync_queue_obtain_inused_object(sync_queue, 5000, byref(data_item))
        if ret != 0:
            print("sync_queue_obtain_inused_object failed.")
            continue

        # 3. 复刻原C：检查流头部
        stream_buffer = data_item.contents.items
        if client_check_stream_header(stream_buffer) != 0:
            print("[Client] Header Error")
            goto_consumer_release = True
        else:
            goto_consumer_release = False

        if not goto_consumer_release:
            # 4. 复刻原C：深度图预览
            depth_buffer.value = 0
            depth_size.value = 0
            ret_depth = client_get_buffer_size_by_type(
                stream_buffer, STREAM_NODE_DEPTH,
                byref(depth_buffer), byref(depth_size)
            )

            if ret_depth == 0 and depth_buffer.value and depth_size.value > 0:
                if preview_flag.value and depth_buffer.value and depth_size.value > 0:
                    depth_ptr = cast(depth_buffer, POINTER(c_uint16))
                    libstereo.show_depth_map(
                        depth_ptr, depth_size.value,
                        STEREO_RES_WIDTH, STEREO_RES_HEIGHT
                    )

            # 5. 复刻原C：深度转点云
            depth_buffer.value = 0
            depth_size.value = 0
            ret_depth = client_get_buffer_size_by_type(
                stream_buffer, STREAM_NODE_DEPTH,
                byref(depth_buffer), byref(depth_size)
            )

            if ret_depth == 0 and depth_buffer.value and depth_size.value > 0:
                # 性能测试开始
                libstereo.performance_test_start_simple(byref(performace_total_depth2cloud))

                # 深度转点云
                depth_ptr = cast(depth_buffer, POINTER(c_uint16))
                depth_to_pointcloud(
                    depth_ptr, STEREO_RES_HEIGHT, STEREO_RES_WIDTH,
                    byref(intr), pc_ptr
                )

                # 性能测试停止
                libstereo.performance_test_stop_simple(byref(performace_total_depth2cloud))

                # 复刻原C：verbose打印
                if verbose_mode.value & VERBOSE_DEPTH_POINTCLOUD:
                    ts = get_current_timestamp_us()
                    print(f"[{ts.value}] Client Get Depth: size[{depth_size.value}].")

                # 复刻原C：转储判断
                if dumpfile_mode.value != 0:
                    dump_cnt += 1
                    dump_file = 1 if (dump_cnt % 10 == 0) else 0

                # 复刻原C：深度数据转储
                if dump_file and (dumpfile_mode.value & DUMP_DEPTH_FILE):
                    depth_pic_name = f"depth_cnt[{dump_cnt}].bin"
                    depth_buf = cast(depth_buffer, POINTER(c_uint8))
                    depth_data = string_at(depth_buf, depth_size.value)
                    dumpToFile(depth_pic_name, depth_data, depth_size.value)

                # 复刻原C：点云转储
                if dump_file and (dumpfile_mode.value & DUMP_POINTCLOUD_FILE):
                    pointcloud_pic_name = f"pointcloud_cnt[{dump_cnt}].txt"
                    save_pointcloud_to_txt(
                        pointcloud_pic_name, pc_ptr,
                        STEREO_RES_HEIGHT, STEREO_RES_WIDTH
                    )

            # 6. 复刻原C：右相机数据处理
            right_buffer.value = 0
            right_size.value = 0
            ret_right = client_get_buffer_size_by_type(
                stream_buffer, STREAM_NODE_CAM_RIGHT,
                byref(right_buffer), byref(right_size)
            )

            if ret_right == 0 and right_buffer.value and right_size.value > 0:
                # verbose打印
                if verbose_mode.value & VERBOSE_LR_RAW_NV12:
                    ts = get_current_timestamp_us()
                    print(f"[{ts.value}] Client Get Right Cam NV12: size[{right_size.value}].")

                # 转储
                if dump_file and (dumpfile_mode.value & DUMP_LR_RAW_NV12):
                    right_pic_name = f"Right_client_Raw_cnt[{dump_cnt}].yuv"
                    right_buf = cast(right_buffer, POINTER(c_uint8))
                    right_data = string_at(right_buf, right_size.value)
                    dumpToFile(right_pic_name, right_data, right_size.value)

            # 7. 复刻原C：左相机数据处理
            left_buffer.value = 0
            left_size.value = 0
            ret_left = client_get_buffer_size_by_type(
                stream_buffer, STREAM_NODE_CAM_LEFT,
                byref(left_buffer), byref(left_size)
            )

            if ret_left == 0 and left_buffer.value and left_size.value > 0:
                # verbose打印
                if verbose_mode.value & VERBOSE_LR_RAW_NV12:
                    ts = get_current_timestamp_us()
                    print(f"[{ts.value}] Client Get Left Cam NV12: size[{left_size.value}].")

                # 转储
                if dump_file and (dumpfile_mode.value & DUMP_LR_RAW_NV12):
                    left_pic_name = f"Left_client_Raw_cnt[{dump_cnt}].yuv"
                    left_buf = cast(left_buffer, POINTER(c_uint8))
                    left_data = string_at(left_buf, left_size.value)
                    dumpToFile(left_pic_name, left_data, left_size.value)

            # 8. 复刻原C：IMU数据处理
            imu_buffer.value = 0
            imu_size.value = 0
            ret_imu = client_get_buffer_size_by_type(
                stream_buffer, STREAM_NODE_IMU,
                byref(imu_buffer), byref(imu_size)
            )

            if ret_imu == 0 and imu_buffer.value and imu_size.value > 0:
                client_parse_imu_buffer(imu_buffer, imu_size.value)

            # 9. 复刻原C：重置dump_file
            dump_file = 0

        # 10. 复刻原C：goto consumer_release（归还数据项）
        ret = libstereo.sync_queue_repay_unused_object(sync_queue, 5000, data_item)
        if ret != 0:
            print("sync_queue_obtain_inused_object failed.")
            continue

        # 11. 复刻原C：performance_test_stop_simple
        libstereo.performance_test_stop_simple(byref(performace_total_consumer))

    # 12. 复刻原C：清理资源
    print("Thread: consumer_process ==> Quit")
    running.value = 0
    return None

# ************************ 8. 复刻原main.c的流程函数 ************************
def client_get_parse_stream_info(server_stream_info):
    """复刻原C：static int client_get_parse_stream_info(server_stream_info_t *server_stream_info)"""
    global intr

    # 1. 复刻原C：malloc JSON缓冲区
    info_json_str = create_string_buffer(STREAM_INFO_JSON_STR_LEN)

    # 2. 复刻原C：recv JSON数据
    if client_fd < 0:
        return -1

    try:
        sock = socket.fromfd(client_fd, socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        data = sock.recv(STREAM_INFO_JSON_STR_LEN, socket.MSG_WAITALL)
        len_recv = len(data)

        if len_recv < 0:
            print(f"[Client] recv Err[{len_recv}] {os.strerror(errno)}")
            return -1
        elif len_recv == 0:
            print("[Client] Closed.")
            return -1

        # 复制数据到缓冲区
        ctypes.memmove(info_json_str, data, len_recv)
    except Exception as e:
        print(f"[Client] recv Err: {e}")
        return -1

    # 3. 复刻原C：cJSON解析（用Python json模块替代，保持逻辑一致）
    try:
        json_data = json.loads(info_json_str.value.decode('utf-8', errors='ignore'))
    except json.JSONDecodeError as e:
        print(f"[Client] cJSON_Parse failed: {e}")
        return -1

    # 4. 复刻原C：提取StreamInformation
    if "StreamInformation" in json_data:
        stream_info_json = json_data["StreamInformation"]

        # 提取ModelVersion
        if "ModelVersion" in stream_info_json:
            model_version = stream_info_json["ModelVersion"].encode('utf-8')
            ctypes.memmove(server_stream_info.contents.model_version, model_version, min(31, len(model_version)))

        # 提取TotalSize
        if "TotalSize" in stream_info_json:
            server_stream_info.contents.total_size = c_uint32(stream_info_json["TotalSize"])

        # 提取内参intr
        if "fx" in stream_info_json:
            intr.fx = c_double(stream_info_json["fx"])
        if "fy" in stream_info_json:
            intr.fy = c_double(stream_info_json["fy"])
        if "cx" in stream_info_json:
            intr.cx = c_double(stream_info_json["cx"])
        if "cy" in stream_info_json:
            intr.cy = c_double(stream_info_json["cy"])
        if "baseline" in stream_info_json:
            intr.baseline = c_double(stream_info_json["baseline"])

    # 5. 复刻原C：循环处理每个stream node
    node_names = ["Right-Cam", "Left-Cam", "Depth", "IMU"]
    for index in range(STREAM_NODE_TYPE_NUM):
        node_info = server_stream_info.contents.stream_nodes[index]
        node_info.name = node_names[index].encode('utf-8')

        # 提取node json信息
        if node_names[index] in json_data:
            node_json = json_data[node_names[index]]
            node_info.available = c_int(node_json.get("Available", 0))
            node_info.width = c_uint32(node_json.get("Width", 0))
            node_info.height = c_uint32(node_json.get("Height", 0))
            node_info.size = c_uint32(node_json.get("Size", 0))
            node_info.offset = c_uint32(node_json.get("Offset", 0))

    # 6. 复刻原C：打印流信息（格式完全一致）
    print("================ Stream Information ================")
    print(f"Model Verion: {server_stream_info.contents.model_version.decode('utf-8', errors='ignore').strip()}")
    print(f"Stream Total Size: {server_stream_info.contents.total_size}")
    print(f"Node Num: {STREAM_NODE_TYPE_NUM}")
    print(f"fx: {intr.fx:.6f}")
    print(f"fy: {intr.fy:.6f}")
    print(f"cx: {intr.cx:.6f}")
    print(f"cy: {intr.cy:.6f}")
    print(f"baseline: {intr.baseline:.6f}")

    for index in range(STREAM_NODE_TYPE_NUM):
        node_info = server_stream_info.contents.stream_nodes[index]
        print("+++++++++++++++++++++++++++++++++")
        print(f"+    [{index}] {node_info.name.decode('utf-8')}")
        print(f"+    Available: {node_info.available}")
        print(f"+    Resolution: {node_info.width} x {node_info.height}")
        print(f"+    Size: {node_info.size}")
        print(f"+    Offset: {node_info.offset}")
        print("+++++++++++++++++++++++++++++++++")

    print("=========================================================")

    return 0

def client_send_mode_to_server(fd, mode):
    """复刻原C：static int client_send_mode_to_server(int fd, int mode)"""
    if fd < 0:
        print("Invalid client fd for sending mode")
        return -1

    # 1. 复刻原C：snprintf构造mode_json
    mode_json = create_string_buffer(MODE_JSON_STR_LEN)
    json_str = f'{{"Mode": {mode}}}'
    ctypes.memmove(mode_json, json_str.encode('utf-8'), min(MODE_JSON_STR_LEN-1, len(json_str)))

    # 2. 复刻原C：send数据
    try:
        sock = socket.fromfd(fd, socket.AF_INET, socket.SOCK_STREAM)
        sent_len = sock.send(mode_json.value)
        json_len = len(json_str)

        if sent_len < 0:
            print(f"Send mode to server failed: {os.strerror(errno)}")
            return -1
        elif sent_len != json_len:
            print(f"Send mode incomplete: sent {sent_len}/{json_len} bytes")
            return -1
    except Exception as e:
        print(f"Send mode to server failed: {e}")
        return -1

    # 3. 复刻原C：打印成功信息
    mode_str = "raw" if mode == MODE_RAW_DEPTH else "isp"
    print(f"Successfully sent mode to server: {mode} ({mode_str})")
    return 0

def client_recv_calib_sn_data(fd):
    """复刻原C：static int client_recv_calib_sn_data(int fd)"""
    if fd < 0:
        return -1

    calib_data = CompleteCalibData_t()
    recv_len = c_uint32(0)

    # 1. 复刻原C：recv calib数据长度
    try:
        sock = socket.fromfd(fd, socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)

        # 接收长度
        len_data = sock.recv(ctypes.sizeof(c_uint32))
        if len(len_data) != ctypes.sizeof(c_uint32):
            print(f"[Client] Recv calib len incomplete: got {len(len_data)}/{ctypes.sizeof(c_uint32)} bytes")
            return -1
        ctypes.memmove(byref(recv_len), len_data, ctypes.sizeof(c_uint32))

        # 验证长度
        calib_data_size = ctypes.sizeof(CompleteCalibData_t)
        if recv_len.value != calib_data_size:
            print(f"[Client] Calib data len mismatch! Expected: {calib_data_size}, Got: {recv_len.value}")
            return -1

        # 2. 复刻原C：recv calib数据
        calib_data_buf = sock.recv(recv_len.value)
        if len(calib_data_buf) != recv_len.value:
            print(f"[Client] Recv calib data incomplete: got {len(calib_data_buf)}/{recv_len.value} bytes")
            return -1
        ctypes.memmove(byref(calib_data), calib_data_buf, recv_len.value)

    except Exception as e:
        print(f"[Client] Recv calib+SN data failed: {e}")
        return -1

    # 3. 复刻原C：验证calib数据
    if libstereo.eeprom_calib_verify(byref(calib_data)) != 0:
        print("[Client] Verify calib data failed (invalid or corrupted)")
        return -1

    # 4. 复刻原C：验证SN数据
    if libstereo.eeprom_calib_verify_sn(calib_data.sn) != 0:
        print("[Client] Verify SN data failed (invalid or corrupted)")
        return -1

    # 5. 复刻原C：保存calib数据到YAML
    if libstereo.eeprom_calib_save_to_yaml(byref(calib_data)) != 0:
        print("[Client] Save calib data to YAML failed")

    # 6. 复刻原C：保存SN数据到文件
    if libstereo.eeprom_calib_save_sn_to_file(calib_data.sn) != 0:
        print("[Client] Save SN data to file failed")

    # 7. 复刻原C：打印完成信息
    print("[Client] Calib+SN data process completed (verify/save YAML/save SN file)")
    return 0

def print_help():
    """复刻原C：static void print_help(void)"""
    print("Usage: [Options]")
    print("Options:")
    print("-i, --interface=\"Network interface\"")
    print("-s, --server=\"Server ip\"")
    print("-v, --verbose\tEnable verbose mode")
    print("-m, --mode\tSet working mode (1: raw, 2: isp, default: 1)")
    print("-p, --preview\tPreview depth (JET) and RGB frames in real-time")
    print("-d, --dump\tEnable dump mode (bitmask)")

# ************************ 9. 复刻原main.c的主流程 ************************
def main():
    global client_fd, running, verbose_mode, dumpfile_mode, preview_flag, selected_mode

    # 1. 复刻原C：命令行参数解析（对应getopt_long）
    import argparse
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("-i", "--interface", required=False)
    parser.add_argument("-s", "--server", required=False)
    parser.add_argument("-v", "--verbose", type=int, default=0)
    parser.add_argument("-d", "--dump", type=int, default=0)
    parser.add_argument("-p", "--preview", action="store_true")
    parser.add_argument("-m", "--mode", type=int, default=MODE_DEFAULT)
    parser.add_argument("-h", "--help", action="store_true")

    args = parser.parse_args()

    # 复刻原C：无参数时打印帮助
    if args.help or (not args.interface and not args.server):
        print_help()
        return 0

    ifname = args.interface.encode('utf-8') if args.interface else b""
    server_ip = args.server.encode('utf-8') if args.server else b""
    verbose_mode.value = args.verbose
    dumpfile_mode.value = args.dump
    preview_flag.value = 1 if args.preview else 0
    selected_mode.value = args.mode

    # 2. 复刻原C：注册信号处理
    signal.signal(signal.SIGINT, signal_handle)

    # 3. 复刻原C：检查网络
    if libstereo.check_network(ifname) != 0:
        print(f"Network [{args.interface}] is not Ready")
        return 0

    # 4. 复刻原C：客户端初始化
    client_fd = libstereo.client_init(server_ip)
    if client_fd < 0:
        print("[Client] init error")
        return 0

    # 5. 复刻原C：发送工作模式
    if client_send_mode_to_server(client_fd, selected_mode.value) != 0:
        print("Failed to send mode to server")
        os.close(client_fd)
        return -1

    # 6. 复刻原C：接收calib+SN数据
    if client_recv_calib_sn_data(client_fd) != 0:
        print("Failed to receive and verify calib+SN data")
        os.close(client_fd)
        return -1

    # 7. 复刻原C：解析流信息
    if client_get_parse_stream_info(byref(stream_info)) != 0:
        print("[Client] client_get_parse_stream_info error")
        os.close(client_fd)
        return 0

    # 8. 复刻原C：构造队列信息
    sync_queue_info_recv_consumer = sync_queue_info_t(
        productor_name=b"recv_data",
        consumer_name=b"consumer",
        is_need_malloc_in_advance=0,
        is_external_buffer=0,
        queue_len=3,
        data_item_size=stream_info.total_size,
        data_item_count=1,
        item_data_init_param=None,
        item_data_init_func=None,
        item_data_deinit_param=None,
        item_data_deinit_func=None
    )

    # 9. 复刻原C：创建同步队列
    ret = libstereo.sync_queue_create(byref(recv_to_consumer), byref(sync_queue_info_recv_consumer))
    if ret != 0:
        print("sync queue create failed for right cam.")
        goto_main_err_out = True
    else:
        goto_main_err_out = False

    if goto_main_err_out:
        os.close(client_fd)
        libstereo.sync_queue_destory(byref(recv_to_consumer))
        return -1

    # 10. 复刻原C：创建线程（用Python threading替代pthread）
    import threading
    running.value = 1

    client_thread = threading.Thread(target=client_recv_data, args=(None,))
    consumer_thread = threading.Thread(target=consumer_process, args=(None,))

    client_thread.start()
    consumer_thread.start()

    # 11. 复刻原C：等待线程结束
    client_thread.join()
    consumer_thread.join()

    # 12. 复刻原C：资源释放（main_err_out）
    os.close(client_fd)
    libstereo.sync_queue_destory(byref(recv_to_consumer))

    return 0

if __name__ == "__main__":
    exit(main())