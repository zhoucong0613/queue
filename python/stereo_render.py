import cv2
import numpy as np

_windows_created = {}

def show_single_camera_py(cam_buffer, cam_size, mode, win_name):
    global _windows_created

    if cam_buffer is None or cam_size <= 0 or not win_name:
        print(f"[Camera] Invalid input for {win_name}!")
        return

    if mode not in (1, 2):
        print(f"[Camera] Unsupported mode {mode} for {win_name}")
        return

    if mode == 1:
        src_w, src_h = 1280, 1088
    else:
        src_w, src_h = 1088, 1280

    expected_size = src_w * src_h * 3 // 2
    if cam_size != expected_size:
        print(f"[Camera] {win_name} size mismatch! Expected {expected_size} bytes, got {cam_size}")
        return

    # 转 numpy
    nv12_array = np.frombuffer(cam_buffer, dtype=np.uint8).reshape((src_h * 3 // 2, src_w))
    bgr_mat = cv2.cvtColor(nv12_array, cv2.COLOR_YUV2BGR_NV12)

    if mode == 2:
        bgr_mat = cv2.rotate(bgr_mat, cv2.ROTATE_90_CLOCKWISE)

    # 只创建窗口一次
    if win_name not in _windows_created:
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        display_w = min(1280, bgr_mat.shape[1])
        display_h = int(bgr_mat.shape[0] * display_w / bgr_mat.shape[1])
        cv2.resizeWindow(win_name, display_w, display_h)
        _windows_created[win_name] = True

    # 更新图像
    cv2.imshow(win_name, bgr_mat)
    cv2.waitKey(1)
