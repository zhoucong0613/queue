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
    print("[Thread] client_recv_data ==> Start")
    global running, client_fd, stream_info, imu_data

    # 设置缓冲区大小，增加兜底值防止total_size为0
    buffer_size = stream_info.total_size 
    print(f"[Producer] Buffer size set to: {buffer_size} bytes")

    while running:
        recv_buffer = bytearray(buffer_size)
        received_bytes = stereo.stereo_client_recv_data(client_fd, recv_buffer)
        time.sleep(0.001)

        header_check_ret = stereo.client_check_stream_header(recv_buffer)
        if header_check_ret != 0:
            print("[Client] Header Error")
            running = False
            break

        ret_code, depth_offset, depth_size = stereo.client_get_buffer_size_by_type(recv_buffer, stereo.STREAM_NODE_DEPTH)
        if ret_code == 0:
            depth_bytes = recv_buffer[depth_offset : depth_offset + depth_size]
            depth_buffer = np.frombuffer(depth_bytes, dtype=np.uint16)
            stereo.show_depth_map(depth_buffer, depth_size, STEREO_RES_WIDTH, STEREO_RES_HEIGHT)
                       
        ret_code, right_offset, right_size = stereo.client_get_buffer_size_by_type(recv_buffer, stereo.STREAM_NODE_CAM_RIGHT)
        if ret_code == 0:
            right_buffer = recv_buffer[right_offset : right_offset + right_size]
            stereo.show_single_camera(right_buffer, right_size, SELECTED_MODE_DEFAULT, "Right Camera")
        
        ret_code, left_offset, left_size = stereo.client_get_buffer_size_by_type(recv_buffer, stereo.STREAM_NODE_CAM_LEFT)
        if ret_code == 0:
            left_buffer = recv_buffer[left_offset : left_offset + left_size]
            stereo.show_single_camera(left_buffer, left_size, SELECTED_MODE_DEFAULT, "Left Camera")

        ret_code, imu_offset, imu_size = stereo.client_get_buffer_size_by_type(recv_buffer, stereo.STREAM_NODE_IMU)
        if ret_code == 0:
            imu_buffer = recv_buffer[imu_offset : imu_offset + imu_size]
            stereo.stereo_parse_imu_buffer(imu_buffer, imu_data)
            print(f"时间戳 (timestamp): {imu_data.timestamp}")
            print(f"加速度计 - ax: {imu_data.ax:.6f}, ay: {imu_data.ay:.6f}, az: {imu_data.az:.6f} (m/s²)")
            print(f"陀螺仪 - gx: {imu_data.gx:.6f}, gy: {imu_data.gy:.6f}, gz: {imu_data.gz:.6f} (rad/s)")
        else:
            print(f"获取IMU缓冲区失败，ret_code={ret_code}，imu_size={imu_size}")            
        time.sleep(0.0001)

    print("[Thread] client_recv_data ==> Quit")

def consumer_process():
    print("[Thread] consumer_process ==> Start")
    global running
    while running:
        time.sleep(0.001)
    print("[Thread] consumer_process ==> Quit")

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