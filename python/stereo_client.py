import argparse
import signal
import sys
import threading
import time 
import stereo_utils
import stereo_client_py as stereo

VERBOSE_MODE_DEFAULT = 0
DUMPFILE_MODE_DEFAULT = 0
SELECTED_MODE_DEFAULT = stereo.MODE_DEFAULT
PREVIEW_FLAG_DEFAULT = False

client_fd = -1
stream_info = None
intr = None
calib_data = None
running = 0 
recv_to_consumer = None
client_data_thread = None
consumer_thread = None

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
    global running

    while running:
        time.sleep(0.01)
    print("[Thread] client_recv_data ==> Quit")

def consumer_process():
    print("[Thread] consumer_process ==> Start")
    global running
    while running:
        time.sleep(0.01)
    print("[Thread] consumer_process ==> Quit")

def main():

    global client_fd, stream_info, intr, calib_data, running, recv_to_consumer
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

    print("\n=== Sync Queue Initializing ===")
    sync_queue_info_recv_consumer = stereo.SyncQueueInfo()
    sync_queue_info_recv_consumer.productor_name = "recv_data"
    sync_queue_info_recv_consumer.consumer_name = "consumer"
    sync_queue_info_recv_consumer.is_need_malloc_in_advance = 0
    sync_queue_info_recv_consumer.is_external_buffer = 0
    sync_queue_info_recv_consumer.queue_len = 3
    sync_queue_info_recv_consumer.data_item_size = stream_info.total_size
    sync_queue_info_recv_consumer.data_item_count = 1
    sync_queue_info_recv_consumer.item_data_init_param = None
    sync_queue_info_recv_consumer.item_data_init_func = None
    sync_queue_info_recv_consumer.item_data_deinit_param = None
    sync_queue_info_recv_consumer.item_data_deinit_func = None
    
    recv_to_consumer = stereo.SyncQueue()

    ret = stereo.sync_queue_create(recv_to_consumer, sync_queue_info_recv_consumer)
    if ret != 0:
        print("sync queue create failed for right cam.")
        stereo.stereo_client_release(client_fd)
        return -1

    running = 1
    print(f"Sync queue create successfully! running={running}")

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

    stereo.sync_queue_destory(recv_to_consumer)
    print("Sync queue destroyed successfully!")
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
        if recv_to_consumer is not None:
            stereo.sync_queue_destory(recv_to_consumer)
        exit(1)