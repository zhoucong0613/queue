#include <pybind11/pybind11.h>
#include <pybind11/buffer_info.h>
#include <string>

extern "C" {
    #include "performance_test_util.h"
    #include "synchronous_queue.h"
    #include "cJSON.h"
    #include "imu.h"
    #include "eeprom_calib.h"
    #include "render_depth.h"
    #include "common.h"
    #include "stereo_client.h"
}

namespace py = pybind11;

PYBIND11_MODULE(stereo_client_py, m) {
      m.doc() = "Python binding for stereo client";

      py::class_<EepromHeadInfo_t>(m, "EepromHeadInfo")
            .def(py::init<>())
            .def_readwrite("module_type", &EepromHeadInfo_t::module_type)
            .def_readwrite("calib_way", &EepromHeadInfo_t::calib_way)
            .def_readwrite("calib_version", &EepromHeadInfo_t::calib_version)
            .def_readwrite("reserved1", &EepromHeadInfo_t::reserved1)
            .def_readwrite("calib_rotate", &EepromHeadInfo_t::calib_rotate)
            .def_readwrite("calib_num", &EepromHeadInfo_t::calib_num)
            .def_readwrite("reserved2", &EepromHeadInfo_t::reserved2)
            .def_readwrite("head_checksum", &EepromHeadInfo_t::head_checksum)
            .def("get_flag", [](const EepromHeadInfo_t &self, size_t idx) {
                  if (idx >= 8) throw py::index_error("flag array index out of range (0-7)");
                  return self.flag[idx];
            }, "Get flag element by index (0-7)", py::arg("idx"))
            .def("set_flag", [](EepromHeadInfo_t &self, size_t idx, uint8_t value) {
                  if (idx >= 8) throw py::index_error("flag array index out of range (0-7)");
                  self.flag[idx] = value;
            }, "Set flag element by index (0-7)", py::arg("idx"), py::arg("value"));

      py::class_<MonoCamCalib_t>(m, "MonoCamCalib")
            .def(py::init<>())
            .def_readwrite("width", &MonoCamCalib_t::width)
            .def_readwrite("height", &MonoCamCalib_t::height)
            .def_readwrite("fx", &MonoCamCalib_t::fx)
            .def_readwrite("fy", &MonoCamCalib_t::fy)
            .def_readwrite("cx", &MonoCamCalib_t::cx)
            .def_readwrite("cy", &MonoCamCalib_t::cy)
            .def_readwrite("k1", &MonoCamCalib_t::k1)
            .def_readwrite("k2", &MonoCamCalib_t::k2)
            .def_readwrite("p1", &MonoCamCalib_t::p1)
            .def_readwrite("p2", &MonoCamCalib_t::p2)
            .def_readwrite("k3", &MonoCamCalib_t::k3)
            .def_readwrite("k4", &MonoCamCalib_t::k4)
            .def_readwrite("k5", &MonoCamCalib_t::k5)
            .def_readwrite("k6", &MonoCamCalib_t::k6);

      py::class_<StereoUnionParam_t>(m, "StereoUnionParam")
            .def(py::init<>())
            .def_readwrite("F_pixel", &StereoUnionParam_t::F_pixel)
            .def_readwrite("B_mm", &StereoUnionParam_t::B_mm)
            .def_readwrite("alpha", &StereoUnionParam_t::alpha);


      py::class_<StereoExtrinsic_t>(m, "StereoExtrinsic")
            .def(py::init<>())
            .def("get_R", [](const StereoExtrinsic_t &self, size_t idx) {
                  if (idx >= 9) throw py::index_error("R array index out of range (0-8)");
                  return self.R[idx];
            }, "Get R matrix element by index (0-8)", py::arg("idx"))
            .def("set_R", [](StereoExtrinsic_t &self, size_t idx, float value) {
                  if (idx >= 9) throw py::index_error("R array index out of range (0-8)");
                  self.R[idx] = value;
            }, "Set R matrix element by index (0-8)", py::arg("idx"), py::arg("value"))
            .def("get_T", [](const StereoExtrinsic_t &self, size_t idx) {
                  if (idx >= 3) throw py::index_error("T array index out of range (0-2)");
                  return self.T[idx];
            }, "Get T vector element by index (0-2)", py::arg("idx"))
            .def("set_T", [](StereoExtrinsic_t &self, size_t idx, float value) {
                  if (idx >= 3) throw py::index_error("T array index out of range (0-2)");
                  self.T[idx] = value;
            }, "Set T vector element by index (0-2)", py::arg("idx"), py::arg("value"));

      py::class_<SnData_t>(m, "SnData")
            .def(py::init<>())
            .def_readwrite("resolution", &SnData_t::resolution)
            .def_readwrite("vendor", &SnData_t::vendor)
            .def_readwrite("model1", &SnData_t::model1)
            .def_readwrite("model2", &SnData_t::model2)
            .def_readwrite("model3", &SnData_t::model3)
            .def_readwrite("year_ten", &SnData_t::year_ten)
            .def_readwrite("year_unit", &SnData_t::year_unit)
            .def_readwrite("month", &SnData_t::month)
            .def_readwrite("day", &SnData_t::day)
            .def_readwrite("checksum", &SnData_t::checksum)
            .def("get_unique", [](const SnData_t &self, size_t idx) {
                  if (idx >= 5) throw py::index_error("unique array index out of range (0-4)");
                  return self.unique[idx];
            }, "Get unique array element by index (0-4)", py::arg("idx"))
            .def("set_unique", [](SnData_t &self, size_t idx, char value) {
                  if (idx >= 5) throw py::index_error("unique array index out of range (0-4)");
                  self.unique[idx] = value;
            }, "Set unique array element by index (0-4)", py::arg("idx"), py::arg("value"));
            
      py::class_<CameraIntrinsics>(m, "CameraIntrinsics")
            .def(py::init<>())
            .def_readwrite("fx", &CameraIntrinsics::fx)
            .def_readwrite("fy", &CameraIntrinsics::fy)
            .def_readwrite("cx", &CameraIntrinsics::cx)
            .def_readwrite("cy", &CameraIntrinsics::cy)
            .def_readwrite("baseline", &CameraIntrinsics::baseline);
      
      py::class_<stream_node_info_t>(m, "StreamNodeInfo")
            .def(py::init<>())
            .def_readwrite("name", &stream_node_info_t::name)
            .def_readwrite("available", &stream_node_info_t::available)
            .def_readwrite("width", &stream_node_info_t::width)
            .def_readwrite("height", &stream_node_info_t::height)
            .def_readwrite("size", &stream_node_info_t::size)
            .def_readwrite("offset", &stream_node_info_t::offset);      

      py::class_<server_stream_info_t>(m, "ServerStreamInfo")
            .def(py::init<>())
            .def_readwrite("total_size", &server_stream_info_t::total_size)
            .def("get_model_version", [](const server_stream_info_t &self, size_t idx) {
                  if (idx >= 32) throw py::index_error("model_version array index out of range (0-31)");
                  return static_cast<uint8_t>(self.model_version[idx]);
            }, "Get model_version element by index (0-31) (return ASCII value)", py::arg("idx"))
            .def("set_model_version", [](server_stream_info_t &self, size_t idx, char value) {
                  if (idx >= 32) throw py::index_error("model_version array index out of range (0-31)");
                  self.model_version[idx] = value;
            }, "Set model_version element by index (0-31)", py::arg("idx"), py::arg("value"))
            .def("get_stream_node", [](server_stream_info_t &self, size_t idx) {
                  if (idx >= STREAM_NODE_TYPE_NUM) throw py::index_error("stream_nodes array index out of range");
                  return &self.stream_nodes[idx];
            }, "Get StreamNodeInfo by index", py::arg("idx"), py::return_value_policy::reference_internal);

      py::class_<CompleteCalibData_t>(m, "CompleteCalibData")
            .def(py::init<>())
            .def_readwrite("head", &CompleteCalibData_t::head)
            .def_readwrite("left_cam", &CompleteCalibData_t::left_cam)
            .def_readwrite("right_cam", &CompleteCalibData_t::right_cam)
            .def_readwrite("stereo_param", &CompleteCalibData_t::stereo_param)
            .def_readwrite("extrinsic", &CompleteCalibData_t::extrinsic)
            .def_readwrite("lr_checksum", &CompleteCalibData_t::lr_checksum)
            .def_readwrite("sn", &CompleteCalibData_t::sn);

      py::enum_<stream_node_type_t>(m, "StreamNodeType")
            .value("STREAM_NODE_CAM_RIGHT", STREAM_NODE_CAM_RIGHT)
            .value("STREAM_NODE_CAM_LEFT", STREAM_NODE_CAM_LEFT)
            .value("STREAM_NODE_DEPTH", STREAM_NODE_DEPTH)
            .value("STREAM_NODE_IMU", STREAM_NODE_IMU)
            .value("STREAM_NODE_TAIL", STREAM_NODE_TAIL)
            .export_values();    
            
      m.attr("STREAM_NODE_TYPE_NUM") = STREAM_NODE_TYPE_NUM;

      m.attr("MODE_DEFAULT") = MODE_DEFAULT;
      m.attr("MODEL_VERSION_LEN") = 32;
      m.attr("FLAG_LEN") = 8;

      m.def("stereo_client_create", 
            &stereo_client_create, 
            py::arg("ifname"), py::arg("server_ip"),
            "Create stereo client, return client_fd (negative if failed)");

      m.def("stereo_client_init",
            &stereo_client_init,
            py::arg("client_fd"), py::arg("mode"), py::arg("stream_info"), py::arg("intr"), py::arg("calib_data"),
            "Init stereo client, return 0 on success");

      m.def("stereo_client_release",
            &stereo_client_release,
            py::arg("client_fd"),
            "Release stereo client resource");

      m.def("stereo_client_recv_data",
      [](int client_fd, py::buffer buffer) {
            py::buffer_info info = buffer.request();

            if (info.format != py::format_descriptor<uint8_t>::format()) {
                  throw py::type_error("stereo_client_recv_data requires a bytearray buffer (uint8_t type)");
            }
            if (info.ndim != 1) {
                  throw py::value_error("stereo_client_recv_data requires a 1-dimensional buffer (only bytearray supported)");
            }

            ssize_t ret = stereo_client_recv_data(client_fd, info.ptr, info.size);

            if (ret < 0) {
                  throw std::runtime_error("stereo_client_recv_data failed with error code: " + std::to_string(ret));
            }

            return ret;
      },
      py::arg("client_fd"), py::arg("buffer"),
      "Receive data from stereo client to Python bytearray buffer\n"
      "Args:\n"
      "  client_fd: Valid client file descriptor from stereo_client_create\n"
      "  buffer: Writable bytearray to store received data (pre-allocated)\n"
      "Returns:\n"
      "  Number of bytes successfully received (non-negative integer)\n"
      "Raises:\n"
      "  TypeError: If buffer is not a bytearray\n"
      "  ValueError: If buffer is not 1-dimensional\n"
      "  RuntimeError: If C function returns negative error code");
      
      
            // 封装 client_check_stream_header 函数
      m.def("client_check_stream_header",
            [](py::buffer buffer) {
                  py::buffer_info info = buffer.request();
                  
                  // 校验缓冲区类型和维度
                  if (info.format != py::format_descriptor<uint8_t>::format()) {
                        throw py::type_error("client_check_stream_header requires a uint8_t (byte) buffer");
                  }
                  if (info.ndim != 1) {
                        throw py::value_error("client_check_stream_header requires a 1-dimensional buffer");
                  }
                  if (info.ptr == nullptr) {
                        throw std::runtime_error("client_check_stream_header buffer is null");
                  }

                  // 调用C函数
                  int ret = client_check_stream_header(static_cast<uint8_t*>(info.ptr));
                  return ret;
            },
            py::arg("stream_buffer"),
            "Check the validity of stream buffer header\n"
            "Args:\n"
            "  stream_buffer: 1-dimensional bytearray buffer containing stream data\n"
            "Returns:\n"
            "  int: 0 for valid header, negative for error, positive for other status (depends on C implementation)\n"
            "Raises:\n"
            "  TypeError: If buffer is not byte type\n"
            "  ValueError: If buffer is not 1-dimensional\n"
            "  RuntimeError: If buffer is null");

      m.def("client_get_buffer_size_by_type",
            [](py::buffer stream_buffer, int type) {
                  // 1. 获取缓冲区信息并严格校验
                  py::buffer_info info = stream_buffer.request();
                  
                  // 校验缓冲区类型：必须是uint8_t（byte）
                  if (info.format != py::format_descriptor<uint8_t>::format()) {
                        throw py::type_error("stream_buffer must be a uint8_t (byte) buffer");
                  }
                  // 校验缓冲区维度：必须是一维
                  if (info.ndim != 1) {
                        throw py::value_error("stream_buffer must be 1-dimensional (bytearray)");
                  }
                  // 校验缓冲区指针非空
                  if (info.ptr == nullptr) {
                        throw std::runtime_error("stream_buffer pointer is null");
                  }

                  // 2. 初始化输出参数（对应C代码中传入空指针，由函数赋值）
                  void* data_buffer = nullptr;  // 输出：指向stream_buffer内数据的指针
                  uint32_t size = 0;            // 输出：数据长度

                  // 3. 调用C函数（严格传递输入输出参数）
                  int ret_code = client_get_buffer_size_by_type(
                  static_cast<uint8_t*>(info.ptr),  // 输入：stream_buffer起始地址
                  type,                             // 输入：流类型（如STREAM_NODE_DEPTH）
                  &data_buffer,                     // 输出：data_buffer指针的地址
                  &size                             // 输出：size变量的地址
                  );

                  // 4. 转换输出参数为Python易用格式（指针→偏移量，避免裸指针风险）
                  // data_buffer是相对于stream_buffer的内部指针，转换为字节偏移量
                  uint64_t data_offset = 0;
                  if (data_buffer != nullptr) {
                        // 计算偏移量：目标指针 - 缓冲区起始指针
                        data_offset = reinterpret_cast<uint8_t*>(data_buffer) - static_cast<uint8_t*>(info.ptr);
                  }

                  // 5. 返回结果：(函数返回码, 数据偏移量, 数据大小)
                  return py::make_tuple(ret_code, data_offset, size);
            },
            py::arg("stream_buffer"), py::arg("type"),
            R"(Get buffer pointer and size by stream type (strictly match C function semantics)
            
            Args:
            stream_buffer (bytearray): 1-dimensional uint8_t buffer (input)
            type (int): Stream type (use StreamNodeType enum, e.g. STREAM_NODE_DEPTH) (input)
            
            Returns:
            tuple: (ret_code: int, data_offset: int, size: uint32_t)
                  - ret_code: 0 = success, negative = error (C function return value)
                  - data_offset: Offset of target data in stream_buffer (bytes, output from data_buffer)
                  - size: Size of target data (bytes, output from size)
            
            Note:
            Equivalent to C code: 
                  int client_get_buffer_size_by_type(uint8_t *stream_buffer, int type, void **data_buffer, uint32_t *size);
            - data_buffer (C output) → data_offset (Python, offset in stream_buffer)
            - size (C output) → size (Python)
            )");

      m.def("client_get_frameid_by_type",
            [](py::buffer stream_buffer, int type) {
                  py::buffer_info info = stream_buffer.request();
                  
                  if (info.format != py::format_descriptor<uint8_t>::format()) {
                        throw py::type_error("client_get_frameid_by_type requires a uint8_t (byte) buffer");
                  }
                  if (info.ndim != 1) {
                        throw py::value_error("client_get_frameid_by_type requires a 1-dimensional buffer");
                  }
                  if (info.ptr == nullptr) {
                        throw std::runtime_error("client_get_frameid_by_type buffer is null");
                  }

                  uint32_t frame_id = client_get_frameid_by_type(static_cast<uint8_t*>(info.ptr), type);
                  return frame_id;
            },
            py::arg("stream_buffer"), py::arg("type"),
            "Get frame ID by stream type\n"
            "Args:\n"
            "  stream_buffer: 1-dimensional bytearray buffer containing stream data\n"
            "  type: Stream type (use StreamNodeType enum values)\n"
            "Returns:\n"
            "  uint32_t: Frame ID of the specified stream type\n"
            "Raises:\n"
            "  TypeError: If buffer is not byte type\n"
            "  ValueError: If buffer is not 1-dimensional\n"
            "  RuntimeError: If buffer is null");

      // ========== 新增：封装 show_single_camera 函数 ==========
      m.def("show_single_camera",
            [](py::buffer cam_buffer, uint32_t cam_size, int mode, const std::string& win_name) {
                  // 校验缓冲区类型（uint8_t/byte）
                  py::buffer_info info = cam_buffer.request();
                  if (info.format != py::format_descriptor<uint8_t>::format()) {
                        throw py::type_error("show_single_camera requires uint8_t (byte) buffer for cam_buffer");
                  }
                  if (info.ndim != 1) {
                        throw py::value_error("show_single_camera requires 1-dimensional byte buffer");
                  }
                  if (info.ptr == nullptr) {
                        throw std::runtime_error("cam_buffer pointer is null");
                  }

                  // 调用C函数
                  show_single_camera(
                        static_cast<uint8_t*>(info.ptr),
                        cam_size,
                        mode,
                        win_name.c_str()
                  );
            },
            py::arg("cam_buffer"), py::arg("cam_size"), py::arg("mode"), py::arg("win_name"),
            R"(显示单个相机的NV12格式图像
            Args:
                cam_buffer: 1维bytearray（uint8_t），相机图像缓冲区（NV12格式）
                cam_size: 缓冲区字节大小（uint32_t）
                mode: 相机模式（1: 1280x1088, 2: 1088x1280）
                win_name: 显示窗口名称（字符串）
            )");

      // ========== 新增：封装 show_depth_map 函数 ==========
      m.def("show_depth_map",
            [](py::buffer depth_buffer, size_t depth_size, int width, int height) {
                  // 校验缓冲区类型（uint16_t）
                  py::buffer_info info = depth_buffer.request();
                  if (info.format != py::format_descriptor<uint16_t>::format()) {
                        throw py::type_error("show_depth_map requires uint16_t buffer for depth_buffer");
                  }
                  if (info.ndim != 1) {
                        throw py::value_error("show_depth_map requires 1-dimensional buffer");
                  }
                  if (info.ptr == nullptr) {
                        throw std::runtime_error("depth_buffer pointer is null");
                  }

                  // 调用C函数
                  show_depth_map(
                        static_cast<uint16_t*>(info.ptr),
                        depth_size,
                        width,
                        height
                  );
            },
            py::arg("depth_buffer"), py::arg("depth_size"), py::arg("width"), py::arg("height"),
            R"(显示深度图（原始归一化图 + Outdoor伪彩色可视化图）
            Args:
                depth_buffer: 1维缓冲区（uint16_t），深度数据（单位：毫米）
                depth_size: 缓冲区字节大小（size_t）
                width: 深度图宽度（int）
                height: 深度图高度（int）
            )");

      py::class_<imu_subnode_data_t>(m, "ImuSubnodeData")
            .def(py::init<>())  // 默认构造函数
            .def_readwrite("timestamp", &imu_subnode_data_t::timestamp)
            .def_readwrite("ax", &imu_subnode_data_t::ax)
            .def_readwrite("ay", &imu_subnode_data_t::ay)
            .def_readwrite("az", &imu_subnode_data_t::az)
            .def_readwrite("gx", &imu_subnode_data_t::gx)
            .def_readwrite("gy", &imu_subnode_data_t::gy)
            .def_readwrite("gz", &imu_subnode_data_t::gz);

      m.def("stereo_parse_imu_buffer",
            [](py::buffer imu_buffer, py::object imu_subnode_data_obj) {
                  // 1. 校验 IMU 缓冲区类型和维度（必须是 1 维 bytearray）
                  py::buffer_info imu_buf_info = imu_buffer.request();
                  if (imu_buf_info.format != py::format_descriptor<uint8_t>::format()) {
                        throw py::type_error("stereo_parse_imu_buffer: imu_buffer 必须是 bytearray (uint8_t 类型)");
                  }
                  if (imu_buf_info.ndim != 1) {
                        throw py::value_error("stereo_parse_imu_buffer: imu_buffer 必须是 1 维缓冲区");
                  }
                  if (imu_buf_info.ptr == nullptr) {
                        throw std::runtime_error("stereo_parse_imu_buffer: imu_buffer 指针为空");
                  }

                  // 2. 校验输出参数是 ImuSubnodeData 实例，并获取底层 C 指针
                  imu_subnode_data_t* imu_data_ptr = imu_subnode_data_obj.cast<imu_subnode_data_t*>();
                  if (imu_data_ptr == nullptr) {
                        throw py::type_error("stereo_parse_imu_buffer: imu_subnode_data 必须是 ImuSubnodeData 实例");
                  }

                  // 3. 调用 C 函数（类型匹配，无编译/运行错误）
                  stereo_parse_imu_buffer(imu_buf_info.ptr, imu_data_ptr);
            },
            py::arg("imu_buffer"), py::arg("imu_subnode_data"),
            R"(解析 IMU 缓冲区数据到 ImuSubnodeData 对象
            功能：将 IMU 原始缓冲区的第一个数据拷贝到 ImuSubnodeData 实例中
            参数：
            imu_buffer: bytearray - 1 维 uint8_t 缓冲区，存放 IMU 原始数据
            imu_subnode_data: ImuSubnodeData - 用于接收解析后数据的实例
            异常：
            TypeError: 缓冲区类型错误 / 输出参数不是 ImuSubnodeData 实例
            ValueError: 缓冲区维度不是 1 维
            RuntimeError: 缓冲区指针为空
            )"); 
}