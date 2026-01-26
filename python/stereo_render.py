import cv2
import numpy as np
import matplotlib.pyplot as plt

_windows_created = {}
_windows_created_depth = {}

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


def render_depth_py(depth_array: np.ndarray, width: int, height: int,
                    need_speckle_filter=True, need_value_show=True,
                    waitkey_time=1, win_name="Depth Map"):
    """
    Depth 渲染函数，保留 speckle filter、分位数归一化、Jet 伪彩色和网格+深度数值显示
    
    depth_array: np.ndarray, 1D 或 2D uint16 或 float32 深度图
    width, height: 深度图分辨率
    need_speckle_filter: 是否做 speckle filter
    need_value_show: 是否显示网格和深度数值
    waitkey_time: cv2.waitKey 时间
    win_name: 显示窗口名
    """

    if depth_array is None or depth_array.size == 0:
        print("[Depth] Invalid input")
        return

    # reshape
    if depth_array.ndim == 1:
        try:
            org_img = depth_array.reshape((height, width))
        except:
            print("[Depth] Reshape failed, check width/height")
            return
    else:
        org_img = depth_array.copy()

    # speckle filter
    if need_speckle_filter:
        norm_img = cv2.normalize(org_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
        cv2.filterSpeckles(norm_img, newVal=0, maxSpeckleSize=10, maxDiff=3)
        _, mask = cv2.threshold(norm_img, 0, 1, cv2.THRESH_BINARY)
        org_img = org_img * mask

    # flatten + percentile
    flattened_data = org_img.flatten()
    valid_data = flattened_data[flattened_data > 0]
    if valid_data.size == 0:
        print("[Depth] No valid depth values")
        return

    percentile1 = np.percentile(valid_data, 10)
    percentile2 = np.percentile(valid_data, 50)
    percentile3 = np.percentile(valid_data, 90)

    # split data into 4 ranges
    values1 = flattened_data[flattened_data <= percentile1]
    values2 = flattened_data[(flattened_data > percentile1) & (flattened_data <= percentile2)]
    values3 = flattened_data[(flattened_data > percentile2) & (flattened_data <= percentile3)]
    values4 = flattened_data[flattened_data > percentile3]

    # normalize each range to 0~1 in Jet colormap segments
    values1_norm = (values1 - np.min(values1)) / max(percentile1 - np.min(values1), 1e-6) * 0.25
    values2_norm = 0.25 + (values2 - percentile1) / max(percentile2 - percentile1, 1e-6) * 0.25
    values3_norm = 0.5 + (values3 - percentile2) / max(percentile3 - percentile2, 1e-6) * 0.25
    values4_norm = 0.75 + (values4 - percentile3) / max(np.max(values4) - percentile3, 1e-6) * 0.25

    # merge back to array
    norm_data = np.zeros_like(flattened_data, dtype=float)
    norm_data[flattened_data <= percentile1] = values1_norm
    norm_data[(flattened_data > percentile1) & (flattened_data <= percentile2)] = values2_norm
    norm_data[(flattened_data > percentile2) & (flattened_data <= percentile3)] = values3_norm
    norm_data[flattened_data > percentile3] = values4_norm
    norm_data = norm_data.reshape(org_img.shape)

    # colormap
    colormap = plt.cm.jet
    colored_img = colormap(norm_data)
    colored_img[org_img == 0] = (0, 0, 0, 1)  # 无效点置黑
    colored_img = (colored_img[:, :, :3] * 255).astype(np.uint8)
    colored_img = cv2.cvtColor(colored_img, cv2.COLOR_RGB2BGR)

    # draw grid and depth values
    if need_value_show:
        h, w, _ = colored_img.shape
        x_step_num = 6
        y_step_num = 12
        x_step = w // x_step_num
        y_step = h // y_step_num
        # draw grid lines
        for j in range(y_step_num):
            cv2.line(colored_img, (0, j * y_step), (w, j * y_step), (255, 255, 255), 1)
        for i in range(x_step_num):
            cv2.line(colored_img, (i * x_step, 0), (i * x_step, h), (255, 255, 255), 1)
        # draw depth values
        for i in range(1, x_step_num):
            for j in range(1, y_step_num):
                try:
                    depth_val = org_img[j * y_step, i * x_step]
                    font_size = 1.0 if w >= 1280 else 0.5
                    cv2.putText(colored_img, f'{depth_val/1000:.3f}m', (i*x_step+3, j*y_step-3),
                                cv2.FONT_HERSHEY_SIMPLEX, font_size, (255, 255, 255), 2)
                except:
                    continue

    # create window only once
    global _windows_created_depth
    if win_name not in _windows_created_depth:
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, 640, 400)
        _windows_created_depth[win_name] = True

    cv2.imshow(win_name, colored_img)
    cv2.waitKey(waitkey_time)
