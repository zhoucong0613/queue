import argparse
import signal
import sys
import threading
import time 
import queue
import stereo_utils
import struct
import numpy as np
from queue import Queue, Empty
import stereo_client_py as stereo
from stereo_render import show_single_camera_py
from stereo_render import render_depth_py

STEREO_RES_WIDTH = 640
STEREO_RES_HEIGHT = 352

DUMPFILE_MODE = 0
DUMP_LR_RAW_NV12 = 1 << 0
DUMP_DEPTH_FILE = 1 << 1

VERBOSE_MODE = 0
VERBOSE_LR_RAW_NV12 = 1 << 0
VERBOSE_DEPTH = 1 << 1
VERBOSE_IMU = 1 << 2

PREVIEW_MODE = 0
PREVIEW_DEPTH = 1 << 0 
PREVIEW_LEFT = 1 << 1
PREVIEW_RIGHT = 1 << 2

SELECTED_MODE_DEFAULT = stereo.MODE_DEFAULT

verbose_mode = VERBOSE_MODE
dumpfile_mode = DUMPFILE_MODE
preview_mode = PREVIEW_MODE
dump_depth_cnt = 0
dump_right_cnt = 0
dump_left_cnt = 0

client_fd = -1
running = 0 
stream_info = None
intr = None
calib_data = None
client_data_thread = None
consumer_thread = None

depth_q = Queue(maxsize=6)
left_q = Queue(maxsize=6)
right_q = Queue(maxsize=6)
imu_q = Queue(maxsize=50)

def dump_to_file(filename, src_buf, size):
    try:
        with open(filename, 'wb') as f:
            if isinstance(src_buf, np.ndarray):
                f.write(src_buf.tobytes())
            else:
                f.write(src_buf[:size])
        print(f"[Dump] Success: {filename}, size={size} bytes")
        return 0
    except Exception as e:
        print(f"[Dump] Error open/write {filename}: {e}", file=sys.stderr)
        return -1

def get_current_timestamp_us():
    return int(time.time() * 1000000)

def client_parse_imu_buffer(imu_data):
    global verbose_mode
    if not (verbose_mode & VERBOSE_IMU):
        return
    print(f"[IMU] Timestamp: {imu_data.timestamp} ===>")
    print(f"  Accelerometer: [{imu_data.ax:.6f}, {imu_data.ay:.6f}, {imu_data.az:.6f}] m/s²")
    print(f"  Gyroscope:     [{imu_data.gx:.6f}, {imu_data.gy:.6f}, {imu_data.gz:.6f}] rad/s")        

def client_recv_data():
    global running, client_fd, stream_info, imu_data

    buffer_size = stream_info.total_size
    print(f"[Producer] Buffer size set to: {buffer_size} bytes")

    imu_data = stereo.ImuSubnodeData()

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
            try:
                depth_q.put_nowait((depth_array, depth_size))
            except:
                pass
        except RuntimeError:
            pass

        # Right Camera
        try:
            right_mv, right_size = stereo.client_get_buffer_size_by_type(mv, stereo.STREAM_NODE_CAM_RIGHT)
            try:
                right_q.put_nowait((bytes(right_mv), right_size))
            except:
                pass
        except RuntimeError:
            pass

        # Left Camera
        try:
            left_mv, left_size = stereo.client_get_buffer_size_by_type(mv, stereo.STREAM_NODE_CAM_LEFT)
            try:
                left_q.put_nowait((bytes(left_mv), left_size))
            except:
                pass
        except RuntimeError:
            pass

        # IMU
        try:
            imu_mv, imu_size = stereo.client_get_buffer_size_by_type(mv, stereo.STREAM_NODE_IMU)
            stereo.stereo_parse_imu_buffer(imu_mv, imu_data)
            try:
                imu_q.put_nowait(imu_data)
            except:
                pass
        except RuntimeError:
            pass

def consumer_process():
    global running, preview_mode, dumpfile_mode, verbose_mode, dump_cnt, selected_mode
    global dump_depth_cnt, dump_right_cnt, dump_left_cnt

    while running:
        #depth
        try:
            depth_buffer, depth_size = depth_q.get_nowait()
            dump_depth_cnt += 1

            if preview_mode & PREVIEW_DEPTH:
                 # stereo.show_depth_map(depth_buffer, depth_size, STEREO_RES_WIDTH, STEREO_RES_HEIGHT)
                render_depth_py(depth_buffer, STEREO_RES_WIDTH, STEREO_RES_HEIGHT, need_speckle_filter=True, need_value_show=False, waitkey_time=1, win_name="Depth Map")

            if verbose_mode & VERBOSE_DEPTH:
                ts = get_current_timestamp_us()
                print(f"[{ts}] [Verbose] Depth data, size={depth_size}")

            if (dumpfile_mode & DUMP_DEPTH_FILE) and (dump_depth_cnt % 10 == 0):
                dump_to_file(f"depth_{dump_depth_cnt}.bin", depth_buffer, depth_size)
        except Empty:
            pass

        # Right
        try:
            right_buffer, right_size = right_q.get_nowait()
            dump_right_cnt += 1

            if preview_mode & PREVIEW_RIGHT:
                show_single_camera_py(right_buffer, right_size, selected_mode, "Right Camera")

            if verbose_mode & VERBOSE_LR_RAW_NV12:
                ts = get_current_timestamp_us()
                print(f"[{ts}] Right NV12, size={right_size}")

            if (dumpfile_mode & DUMP_LR_RAW_NV12) and (dump_right_cnt % 10 == 0):
                dump_to_file(f"Right_{dump_right_cnt}.yuv", right_buffer, right_size)

        except Empty:
            pass    

        try:
            left_buffer, left_size = left_q.get_nowait()
            dump_left_cnt += 1
            dump_left_file = (dumpfile_mode != 0 and dump_left_cnt % 10 == 0)

            if preview_mode & PREVIEW_LEFT:
                # stereo.show_single_camera(mv, size, selected_mode, "Left Camera"
                show_single_camera_py(left_buffer, left_size, selected_mode, "Left Camera")

            if verbose_mode & VERBOSE_LR_RAW_NV12:
                ts = get_current_timestamp_us()
                print(f"[{ts}] Left NV12, size={left_size}")

            if (dumpfile_mode & DUMP_LR_RAW_NV12) and (dump_left_cnt % 10 == 0):
                dump_to_file(f"Left_{dump_left_cnt}.yuv", left_buffer, left_size)
                
        except Empty:
            pass
                
        # IMU
        try:
            imu_item = imu_q.get_nowait()
            client_parse_imu_buffer(imu_item)
        except Empty:
            pass

        time.sleep(0.001)

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
  -i, --interface   Network interface (e.g. usb0) [required]
  -s, --server      Server IP address (e.g. 192.168.5.5) [required]
  -v, --verbose     Verbose mode (bit flag int, e.g. 8=VERBOSE_IMU, 3=STREAM+LR_RAW) [default:0]
  -d, --dump        Dump file mode (bit flag int, e.g. 3=LR_RAW+DEPTH,7=ALL) [default:0]
  -p, --preview     Preview mode (bit flag int, e.g. 1=DEPTH,3=DEPTH+LEFT) [default:0]
  -m, --mode        Working mode (1: raw, 2: isp) [default:1]
Bit Flag Reference:
  --verbose: 1=STREAM,2=LR_RAW,4=DEPTH,8=IMU
  --dump:    1=LR_RAW,2=DEPTH
  --preview: 1=DEPTH,2=LEFT,4=RIGHT
Example:
  python stereo_client.py -i usb0 -s 192.168.5.5 -p 7 -d 3 -v 8 -m 1
"""
    print(help_info)

def parse_args():
    parser = argparse.ArgumentParser(description="Stereo Client Python Wrapper", add_help=False)
    parser.add_argument('-i', '--interface', type=str, required=True,
                        help="Network interface (e.g. usb0)")
    parser.add_argument('-s', '--server', type=str, required=True,
                        help="Server IP address (e.g. 192.168.5.5)")
    parser.add_argument('-v', '--verbose', type=int, default=VERBOSE_MODE,
                        help="Verbose mode (bit flag integer, see --help for reference)")
    parser.add_argument('-d', '--dump', type=int, default=DUMPFILE_MODE,
                        help="Dump file mode (bit flag integer, see --help for reference)")
    parser.add_argument('-p', '--preview', type=int, default=PREVIEW_MODE,
                        help="Preview mode (bit flag integer, see --help for reference)")
    parser.add_argument('-m', '--mode', type=int, default=SELECTED_MODE_DEFAULT,
                        help="Set working mode (1: raw, 2: isp, default: 1)")
    parser.add_argument('-h', '--help', action='store_true', help="Show this help message and exit")
    
    args = parser.parse_args()
    if args.help:
        print_help()
        sys.exit(0)
    return args

def main():

    global client_fd, stream_info, intr, calib_data, running, imu_data
    global client_data_thread, consumer_thread, verbose_mode, dumpfile_mode, preview_mode, selected_mode

    signal.signal(signal.SIGINT, signal_handle)
    print("[Init] SIGINT signal (Ctrl+C) handler registered successfully.")

    args = parse_args()
    ifname = args.interface
    server_ip = args.server
    verbose_mode = args.verbose
    dumpfile_mode = args.dump
    preview_mode = args.preview
    selected_mode = args.mode
    
    stream_info = stereo.ServerStreamInfo() 
    intr = stereo.CameraIntrinsics()
    calib_data = stereo.CompleteCalibData()

    print("=== Stereo Client Initializing ===")
    print(f"Interface:    {ifname}")
    print(f"Server IP:    {server_ip}")
    print(f"Working Mode: {selected_mode}")
    print(f"Preview Mode: {preview_mode} (bit flag, 1=DEPTH,2=LEFT,4=RIGHT)")
    print(f"Dump Mode:    {dumpfile_mode} (bit flag,1=LR_RAW,2=DEPTH)")
    print(f"Verbose Mode: {verbose_mode} (bit flag,1=STREAM,2=LR_RAW,4=DEPTH,8=IMU)")
    print("Press Ctrl+C to exit later...")
    print("====================================================")
    
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