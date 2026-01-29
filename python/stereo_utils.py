import stereo_client_py as stereo

VERBOSE_MODE_DEFAULT = 0
DUMPFILE_MODE_DEFAULT = 0
SELECTED_MODE_DEFAULT = stereo.MODE_DEFAULT
PREVIEW_FLAG_DEFAULT = False

# =====打印所有初始化值=====
def print_all_init_values(args, client_fd, stream_info, intr, calib_data):

    print("\n" + "="*80)
    print("============= 初始化完成 ============")
    print("="*80)
    
    # 1. 命令行参数解析结果
    print("\n【1. 命令行参数】")
    print(f"  网络接口 (interface): {args.interface}")
    print(f"  服务器IP (server): {args.server}")
    print(f"  详细模式 (verbose): {args.verbose} (默认: {VERBOSE_MODE_DEFAULT})")
    print(f"  转储模式 (dump): {args.dump} (默认: {DUMPFILE_MODE_DEFAULT})")
    print(f"  预览模式 (preview): {args.preview} (默认: {PREVIEW_FLAG_DEFAULT})")
    print(f"  工作模式 (mode): {args.mode} (默认: {SELECTED_MODE_DEFAULT})")
    
    # 2. 客户端创建结果
    print("\n【2. 客户端句柄信息】")
    print(f"  客户端文件描述符 (client_fd): {client_fd}")
    print(f"  客户端创建状态: {'成功' if client_fd >= 0 else '失败'}")
    
    # 3. CameraIntrinsics 结构体（相机内参）
    print("\n【3. CameraIntrinsics (相机内参)】")
    print(f"  结构体实例: {intr}")
    print(f"    fx (焦距x): {intr.fx}")
    print(f"    fy (焦距y): {intr.fy}")
    print(f"    cx (主点x): {intr.cx}")
    print(f"    cy (主点y): {intr.cy}")
    print(f"    baseline (基线距离): {intr.baseline}")
    
    # 4. ServerStreamInfo 结构体（流信息）
    print("\n【4. ServerStreamInfo (流信息)】")
    print(f"  结构体实例: {stream_info}")
    print(f"    total_size (流总大小): {stream_info.total_size}")
    
    # 4.1 model_version 数组
    print(f"    model_version (模型版本): ", end="")
    version_str = ""
    for i in range(stereo.MODEL_VERSION_LEN):
        try:
            char_val = stream_info.get_model_version(i)
            val = ord(char_val) if isinstance(char_val, str) else int(char_val)
            
            if 32 <= val <= 127:
                version_str += chr(val)
            elif val == 0:
                break
        except:
            pass
    print(version_str if version_str else "未知版本")
    
    # 4.2 stream_nodes 结构体数组（流节点信息）
    print(f"    stream_nodes (流节点数组, 共{stereo.STREAM_NODE_TYPE_NUM}个):")
    stream_node_names = ["右相机", "左相机", "深度图", "IMU"]
    for i in range(stereo.STREAM_NODE_TYPE_NUM):
        try:
            node = stream_info.get_stream_node(i)
            print(f"      节点[{i}] ({stream_node_names[i]}):")
            print(f"        name: {node.name if node.name else 'None'}")
            print(f"        available (是否可用): {node.available}")
            print(f"        width (宽度): {node.width}")
            print(f"        height (高度): {node.height}")
            print(f"        size (数据大小): {node.size}")
            print(f"        offset (数据偏移): {node.offset}")
        except Exception as e:
            print(f"      节点[{i}]: 访问失败 - {str(e)[:50]}")
    
    # 5. CompleteCalibData 结构体（完整标定数据，嵌套结构体）
    print("\n【5. CompleteCalibData (完整标定数据)】")
    print(f"  结构体实例: {calib_data}")
    
    # 5.1 嵌套：EepromHeadInfo（EEPROM头部信息，打印全部字段+flag转字符串）
    print(f"    5.1 EepromHeadInfo (EEPROM头部):")
    head = calib_data.head
    # 打印所有字段（包括reserved1/reserved2，无遗漏）
    print(f"      module_type (模块类型): {head.module_type}")
    print(f"      calib_way (标定方式): {head.calib_way}")
    print(f"      calib_version (标定版本): {head.calib_version}")
    print(f"      reserved1 (保留字段1): {head.reserved1}")
    print(f"      calib_rotate (标定旋转): {head.calib_rotate}")
    print(f"      calib_num (标定数量): {head.calib_num}")
    print(f"      reserved2 (保留字段2): {head.reserved2}")
    print(f"      head_checksum (头部校验和): {head.head_checksum}")

    # flag数组：同时打印16进制+可打印字符串（核心需求）
    print(f"      flag (标志位, 8字节): ")
    flag_hex = []   # 存储16进制值
    flag_str = []   # 存储可打印字符串
    for i in range(stereo.FLAG_LEN):
        try:
            val = int(head.get_flag(i))
            flag_hex.append(f"{hex(val)}")
            # 转换为可打印字符（ASCII 32-127），不可打印则用·代替
            flag_str.append(chr(val) if 32 <= val <= 127 else '·')
        except:
            flag_hex.append("0x??")
            flag_str.append('?')
    # 拼接输出：16进制列表 + 字符串形式
    print(f"        16进制: {' '.join(flag_hex)}")
    print(f"        字符串: {''.join(flag_str)}")
        
    # 5.2 嵌套：MonoCamCalib（单目相机标定，左/右相机）
    print(f"    5.2 MonoCamCalib (左相机标定):")
    left_cam = calib_data.left_cam
    print(f"      width: {left_cam.width}, height: {left_cam.height}")
    print(f"      fx: {left_cam.fx}, fy: {left_cam.fy}")
    print(f"      cx: {left_cam.cx}, cy: {left_cam.cy}")
    print(f"      畸变参数 (k1-k6): {left_cam.k1}, {left_cam.k2}, {left_cam.p1}, {left_cam.p2}, {left_cam.k3}, {left_cam.k4}, {left_cam.k5}, {left_cam.k6}")
    
    print(f"    5.3 MonoCamCalib (右相机标定):")
    right_cam = calib_data.right_cam
    print(f"      width: {right_cam.width}, height: {right_cam.height}")
    print(f"      fx: {right_cam.fx}, fy: {right_cam.fy}")
    print(f"      cx: {right_cam.cx}, cy: {right_cam.cy}")
    print(f"      畸变参数 (k1-k6): {right_cam.k1}, {right_cam.k2}, {right_cam.p1}, {right_cam.p2}, {right_cam.k3}, {right_cam.k4}, {right_cam.k5}, {right_cam.k6}")
    
    # 5.4 嵌套：StereoUnionParam（立体视觉公共参数）
    print(f"    5.4 StereoUnionParam (立体视觉参数):")
    stereo_param = calib_data.stereo_param
    print(f"      F_pixel (像素焦距): {stereo_param.F_pixel}")
    print(f"      B_mm (基线距离mm): {stereo_param.B_mm}")
    print(f"      alpha (校正系数): {stereo_param.alpha}")
    
    # 5.5 嵌套：StereoExtrinsic（外参矩阵 R/T）
    print(f"    5.5 StereoExtrinsic (外参矩阵):")
    extrinsic = calib_data.extrinsic
    print(f"      R 矩阵 (3x3, 9个元素): ", end="")
    for i in range(9):
        try:
            val = extrinsic.get_R(i)
            print(f"{val} ", end="")
            if (i+1) % 3 == 0: print(end="| ")
        except:
            print(f"未知值 ", end="")
            if (i+1) % 3 == 0: print(end="| ")
    print(f"\n      T 向量 (3个元素): ", end="")
    for i in range(3):
        try:
            val = extrinsic.get_T(i)
            print(f"{val} ", end="")
        except:
            print(f"未知值 ", end="")
    print()

    # 5.6 嵌套：SnData (序列号信息) - 复刻C语言 eeprom_calib_print_sn 逻辑
    print(f"    5.6 SnData (序列号信息):")
    sn = calib_data.sn

    def to_ascii(val):
        try:
            if isinstance(val, str) and len(val) == 1:
                return ord(val)
            return int(val)
        except (TypeError, ValueError):
            return 0

    def sn_char_to_num(c, type_num):
        try:
            c_upper = str(c).upper()[0] if isinstance(c, (str, int)) else ' '
            if c_upper >= '1' and c_upper <= '9':
                return int(c_upper) - int('0')
            # 月份(type=0)：A-C → 10/11/12
            elif type_num == 0 and c_upper >= 'A' and c_upper <= 'C':
                return 10 + (ord(c_upper) - ord('A'))
            # 日期(type=1)：A-X → 10-43
            elif type_num == 1 and c_upper >= 'A' and c_upper <= 'X':
                return 10 + (ord(c_upper) - ord('A'))
            return -1
        except:
            return -1

    res_char = str(sn.resolution)
    res_name = "VGA" if res_char == '0' else ("HD" if res_char == '1' else "FHD")
    res_ascii = to_ascii(sn.resolution)
    print(f"      分辨率：{res_char}（{res_name}） (0x{res_ascii:02X})")

    vendor_char = str(sn.vendor).upper()
    vendor_name = "OV" if vendor_char == 'O' else ("思特威" if vendor_char == 'T' else "未知")
    vendor_ascii = to_ascii(sn.vendor)
    print(f"      芯片厂商：{vendor_char}（{vendor_name}） (0x{vendor_ascii:02X})")

    model1_char = chr(to_ascii(sn.model1)) if 32 <= to_ascii(sn.model1) <= 127 else '?'
    model2_char = chr(to_ascii(sn.model2)) if 32 <= to_ascii(sn.model2) <= 127 else '?'
    model3_char = chr(to_ascii(sn.model3)) if 32 <= to_ascii(sn.model3) <= 127 else '?'
    model_full = f"{model1_char}{model2_char}{model3_char}"
    model1_ascii = to_ascii(sn.model1)
    model2_ascii = to_ascii(sn.model2)
    model3_ascii = to_ascii(sn.model3)
    print(f"      模组型号：{model_full} (0x{model1_ascii:02X}, 0x{model2_ascii:02X}, 0x{model3_ascii:02X})")

    # 年份计算：2000 + (year_ten-'0')*10 + (year_unit-'0')
    year_ten_num = sn_char_to_num(sn.year_ten, -1)  # type=-1仅取数字
    year_unit_num = sn_char_to_num(sn.year_unit, -1)
    full_year = 2000 + year_ten_num * 10 + year_unit_num if (year_ten_num != -1 and year_unit_num != -1) else "未知"

    month_num = sn_char_to_num(sn.month, 0)
    day_num = sn_char_to_num(sn.day, 1)
    month_raw = str(sn.month)
    day_raw = str(sn.day)
    print(f"      生产日期：{full_year}年{month_num if month_num != -1 else '未知'}月{day_num if day_num != -1 else '未知'}日（原始ASCII：{month_raw}月{day_raw}日）")

    print(f"      唯一标识：", end="")
    unique_chars = []
    unique_hex = []
    for i in range(5):
        try:
            val = sn.get_unique(i)
            val_char = chr(to_ascii(val)) if 32 <= to_ascii(val) <= 127 else '?'
            val_hex = f"0x{to_ascii(val):02X}"
            unique_chars.append(val_char)
            unique_hex.append(val_hex)
        except:
            unique_chars.append('?')
            unique_hex.append("0x??")

    print(f"{''.join(unique_chars)} ({' '.join(unique_hex)})")

    checksum_ascii = to_ascii(sn.checksum)
    print(f"      校验和：0x{checksum_ascii:02X}（{checksum_ascii}）")
        
    # 5.7 其他字段
    print(f"    5.7 其他: lr_checksum = {calib_data.lr_checksum}")
    
    # 6. 整体状态汇总
    print("\n【6. 初始化整体状态汇总】")
    status = "全部成功" if client_fd >= 0 else "客户端创建失败"
    print(f"  最终状态: {status}")
    print("\n" + "="*80)
    print("============= 全量数值打印结束 ============")
    print("="*80 + "\n")