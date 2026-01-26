import argparse
import signal
import sys
import threading
import time 
import queue
import stereo_utils
import struct
import numpy as np
import stereo_client_py as stereo
from stereo_render import show_single_camera_py
from stereo_render import render_depth_py

VERBOSE_MODE_DEFAULT = 0
DUMPFILE_MODE_DEFAULT = 0
SELECTED_MODE_DEFAULT = stereo.MODE_DEFAULT
PREVIEW_FLAG_DEFAULT = False
IMU_STRUCT_SIZE = 32 

STEREO_RES_WIDTH = 640
STEREO_RES_HEIGHT = 352

client_fd = -1
stream_info = None
intr = None
calib_data = None
running = 0 
client_data_thread = None
consumer_thread = None
imu_data = None

latest_right = None
latest_left = None
latest_depth = None
latest_imu = None

# 锁
lock_right = threading.Lock()
lock_left = threading.Lock()
lock_depth = threading.Lock()
lock_imu = threading.Lock()

def signal_handle(signum, frame):
    print("\n=====================================")
    print("[Signal] Received Ctrl+C (SIGINT), preparing to exit gracefully...")
    
    global running
    running = 0

    print("[Signal] Set running=0, waiting threads to exit...")
    print("=====================================")

def print_help():
    help_info = """
Usage: stereo_client.py [Options]
Options:
  -i, --interface   Network interface (e.g. usb0)
  -s, --server      Server IP address (e.g. 192.168.5.5)
  -v, --verbose     Enable verbose mode (integer value, e.g. 1)
  -d, --dump        Enable dump file mode (integer value, e.g. 3)
  -p, --preview     Preview depth (JET) and RGB frames in real-time
  -m, --mode        Set working mode (1: raw, 2: isp, default: 1)
    """
    print(help_info)

def parse_args():

    parser = argparse.ArgumentParser(description="Stereo Client Python Wrapper")
    parser.add_argument('-i', '--interface', type=str, required=True, 
                        help="Network interface (e.g. usb0)")
    parser.add_argument('-s', '--server', type=str, required=True, 
                        help="Server IP address (e.g. 192.168.5.5)")
    parser.add_argument('-v', '--verbose', type=int, default=VERBOSE_MODE_DEFAULT, 
                        help="Enable verbose mode (integer value)")
    parser.add_argument('-d', '--dump', type=int, default=DUMPFILE_MODE_DEFAULT, 
                        help="Enable dump file mode (integer value)")
    parser.add_argument('-p', '--preview', action='store_true', default=PREVIEW_FLAG_DEFAULT, 
                        help="Preview depth (JET) and RGB frames in real-time")
    parser.add_argument('-m', '--mode', type=int, default=SELECTED_MODE_DEFAULT, 
                        help="Set working mode (1: raw, 2: isp, default: 1)")
    # 解析参数
    args = parser.parse_args()
    return args

def client_recv_data():
    global running, client_fd, stream_info, imu_data
    global latest_right, latest_left, latest_depth, latest_imu

    buffer_size = stream_info.total_size
    print(f"[Producer] Buffer size set to: {buffer_size} bytes")

    while running:
        recv_buffer = bytearray(buffer_size)
        mv = memoryview(recv_buffer)

        ret_bytes = stereo.stereo_client_recv_data(client_fd, mv)

        if stereo.client_check_stream_header(mv) != 0:
            print("[Client] Header Error")
            running = False
            break

        # Depth
        try:
            depth_mv, depth_size = stereo.client_get_buffer_size_by_type(mv, stereo.STREAM_NODE_DEPTH)
            depth_array = np.frombuffer(depth_mv, dtype=np.uint16).copy()
            with lock_depth:
                latest_depth = (depth_array, depth_size)
        except RuntimeError:
            pass

        # Right Camera
        try:
            right_mv, right_size = stereo.client_get_buffer_size_by_type(mv, stereo.STREAM_NODE_CAM_RIGHT)
            with lock_right:
                latest_right = (bytes(right_mv), right_size)
        except RuntimeError:
            pass

        # Left Camera
        try:
            left_mv, left_size = stereo.client_get_buffer_size_by_type(mv, stereo.STREAM_NODE_CAM_LEFT)
            with lock_left:
                latest_left = (bytes(left_mv), left_size)
        except RuntimeError:
            pass

        # IMU
        try:
            imu_mv, imu_size = stereo.client_get_buffer_size_by_type(mv, stereo.STREAM_NODE_IMU)
            stereo.stereo_parse_imu_buffer(imu_mv, imu_data)
            with lock_imu:
                latest_imu = imu_data
        except RuntimeError:
            pass

def consumer_process():
    global running, latest_right, latest_left, latest_depth, latest_imu

    while running:
        # Right
        with lock_right:
            if latest_right:
                mv, size = latest_right
                show_single_camera_py(mv, size, SELECTED_MODE_DEFAULT, "Right Camera")

        # Left
        with lock_left:
            if latest_left:
                mv, size = latest_left
                show_single_camera_py(mv, size, SELECTED_MODE_DEFAULT, "Left Camera")

        # Depth
        with lock_depth:
            if latest_depth:
                depth_buffer, depth_size = latest_depth
                # stereo.show_depth_map(depth_buffer, depth_size, STEREO_RES_WIDTH, STEREO_RES_HEIGHT)
                render_depth_py(depth_buffer, STEREO_RES_WIDTH, STEREO_RES_HEIGHT,
                                    need_speckle_filter=True, need_value_show=True,
                                    waitkey_time=1, win_name="Depth Map")

        # IMU
        with lock_imu:
            if latest_imu:
                imu_data_item = latest_imu
                print(f"时间戳 (timestamp): {imu_data_item.timestamp}")
                print(f"加速度计 - ax: {imu_data_item.ax:.6f}, ay: {imu_data_item.ay:.6f}, az: {imu_data_item.az:.6f}")
                print(f"陀螺仪 - gx: {imu_data_item.gx:.6f}, gy: {imu_data_item.gy:.6f}, gz: {imu_data_item.gz:.6f}")
                
        time.sleep(0.001)

def main():

    global client_fd, stream_info, intr, calib_data, running, imu_data
    global client_data_thread, consumer_thread

    signal.signal(signal.SIGINT, signal_handle)
    print("[Init] SIGINT signal (Ctrl+C) handler registered successfully.")

    args = parse_args()
    
    ifname = args.interface
    server_ip = args.server
    verbose_mode = args.verbose
    dumpfile_mode = args.dump
    preview_flag = args.preview
    selected_mode = args.mode
    
    stream_info = stereo.ServerStreamInfo() 
    intr = stereo.CameraIntrinsics()
    calib_data = stereo.CompleteCalibData()
    imu_data = stereo.ImuSubnodeData()
    
    print("=== Stereo Client Initializing ===")
    print(f"Interface: {ifname}")
    print(f"Server IP: {server_ip}")
    print(f"Working Mode: {selected_mode}")
    print("Press Ctrl+C to exit later...")
    
    client_fd = stereo.stereo_client_create(ifname, server_ip)
    if client_fd < 0:
        print("[Client] create error: failed to connect to server")
        return 1
    
    ret = stereo.stereo_client_init(
        client_fd,
        selected_mode,
        stream_info,
        intr,
        calib_data
    )
    if ret != 0:
        print("[Client] stereo client init failed")
        stereo.stereo_client_release(client_fd)
        return -1 

    print("[Client] stereo client create and init successfully!")
    print(f"Client FD: {client_fd}")

    # stereo_utils.print_all_init_values(args, client_fd, stream_info, intr, calib_data)

    running = 1

    client_data_thread = threading.Thread(target=client_recv_data, name="client_recv_data")
    consumer_thread = threading.Thread(target=consumer_process, name="consumer_process")

    client_data_thread.start()
    consumer_thread.start()
    print(f"Threads created: client_recv_data={client_data_thread.name}, consumer_process={consumer_thread.name}")

    if client_data_thread.is_alive():
        print("[Main] Waiting for client_recv_data thread to exit...")
        client_data_thread.join()
        client_data_thread = None
    if consumer_thread.is_alive():
        print("[Main] Waiting for consumer_process thread to exit...")
        consumer_thread.join()
        consumer_thread = None
    

    stereo.stereo_client_release(client_fd)
    print("[Client] stereo client resource released successfully!")
    return 0

if __name__ == "__main__":
    try:
        exit_code = main()
        exit(exit_code)
    except Exception as e:
        print(f"[Error] Unexpected error: {e}")
        print_help()
        if client_fd >= 0:
            stereo.stereo_client_release(client_fd)
        exit(1)