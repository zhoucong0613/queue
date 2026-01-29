#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

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

static inline uint8_t* buf_ptr(py::buffer b, ssize_t &size) {
    py::buffer_info info = b.request();
    size = info.size;
    return static_cast<uint8_t*>(info.ptr);
}

PYBIND11_MODULE(stereo_client_py, m) {

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
        .def("get_flag", [](EepromHeadInfo_t &s, size_t i){ return s.flag[i]; })
        .def("set_flag", [](EepromHeadInfo_t &s, size_t i, uint8_t v){ s.flag[i] = v; });

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
        .def("get_R", [](StereoExtrinsic_t &s, size_t i){ return s.R[i]; })
        .def("set_R", [](StereoExtrinsic_t &s, size_t i, float v){ s.R[i] = v; })
        .def("get_T", [](StereoExtrinsic_t &s, size_t i){ return s.T[i]; })
        .def("set_T", [](StereoExtrinsic_t &s, size_t i, float v){ s.T[i] = v; });

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
        .def("get_unique", [](SnData_t &s, size_t i){ return s.unique[i]; })
        .def("set_unique", [](SnData_t &s, size_t i, char v){ s.unique[i] = v; });

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
        .def("get_model_version", [](server_stream_info_t &s, size_t i){ return s.model_version[i]; })
        .def("get_stream_node", [](server_stream_info_t &s, size_t i){
            return &s.stream_nodes[i];
        }, py::return_value_policy::reference_internal);

    py::class_<CompleteCalibData_t>(m, "CompleteCalibData")
        .def(py::init<>())
        .def_readwrite("head", &CompleteCalibData_t::head)
        .def_readwrite("left_cam", &CompleteCalibData_t::left_cam)
        .def_readwrite("right_cam", &CompleteCalibData_t::right_cam)
        .def_readwrite("stereo_param", &CompleteCalibData_t::stereo_param)
        .def_readwrite("extrinsic", &CompleteCalibData_t::extrinsic)
        .def_readwrite("lr_checksum", &CompleteCalibData_t::lr_checksum)
        .def_readwrite("sn", &CompleteCalibData_t::sn);

    py::class_<imu_subnode_data_t>(m, "ImuSubnodeData")
        .def(py::init<>())
        .def_readwrite("timestamp", &imu_subnode_data_t::timestamp)
        .def_readwrite("ax", &imu_subnode_data_t::ax)
        .def_readwrite("ay", &imu_subnode_data_t::ay)
        .def_readwrite("az", &imu_subnode_data_t::az)
        .def_readwrite("gx", &imu_subnode_data_t::gx)
        .def_readwrite("gy", &imu_subnode_data_t::gy)
        .def_readwrite("gz", &imu_subnode_data_t::gz);

    py::enum_<stream_node_type_t>(m, "StreamNodeType")
        .value("STREAM_NODE_CAM_RIGHT", STREAM_NODE_CAM_RIGHT)
        .value("STREAM_NODE_CAM_LEFT", STREAM_NODE_CAM_LEFT)
        .value("STREAM_NODE_DEPTH", STREAM_NODE_DEPTH)
        .value("STREAM_NODE_IMU", STREAM_NODE_IMU)
        .value("STREAM_NODE_TAIL", STREAM_NODE_TAIL)
        .export_values();

    m.attr("STREAM_NODE_TYPE_NUM") = STREAM_NODE_TYPE_NUM;
    m.attr("MODE_RAW_DEPTH") = MODE_RAW_DEPTH;
    m.attr("MODE_ISP_DEPTH") = MODE_ISP_DEPTH;
    m.attr("MODE_DEFAULT") = MODE_DEFAULT;
    m.attr("MODEL_VERSION_LEN") = 32;
    m.attr("FLAG_LEN") = 8;

    m.def("stereo_client_create", &stereo_client_create);
    m.def("stereo_client_init", &stereo_client_init);
    m.def("stereo_client_release", &stereo_client_release);

    m.def("stereo_client_recv_data",
        [](int fd, py::memoryview mv){
            ssize_t sz;
            uint8_t* ptr = buf_ptr(mv, sz);
            return stereo_client_recv_data(fd, ptr, sz);
        });

    m.def("client_check_stream_header",
        [](py::memoryview mv){
            ssize_t sz;
            uint8_t* ptr = buf_ptr(mv, sz);
            return client_check_stream_header(ptr);
        });

    m.def("client_get_buffer_size_by_type",
        [](py::memoryview mv, int type) -> std::tuple<py::memoryview, uint32_t> {
            ssize_t sz;
            uint8_t* ptr = buf_ptr(mv, sz);

            void* data_ptr = nullptr;
            uint32_t data_size = 0;

            int ret = client_get_buffer_size_by_type(ptr, type, &data_ptr, &data_size);

            if (ret != 0 || data_ptr == nullptr) {
                  throw std::runtime_error("Failed to get buffer by type");
            }

            // Return a memoryview pointing to the sub-buffer
            return std::make_tuple(
                  py::memoryview::from_memory(data_ptr, data_size),
                  data_size
            );
        });

    m.def("client_get_frameid_by_type",
        [](py::memoryview mv, int type){
            ssize_t sz;
            uint8_t* ptr = buf_ptr(mv, sz);
            return client_get_frameid_by_type(ptr, type);
        });

    m.def("show_single_camera",
        [](py::memoryview mv, uint32_t sz, int mode, const std::string& w){
            ssize_t _;
            uint8_t* ptr = buf_ptr(mv, _);
            show_single_camera(ptr, sz, mode, w.c_str());
        });

    m.def("show_depth_map",
        [](py::array_t<uint16_t> &arr, size_t sz, int w, int h){
            show_depth_map(arr.mutable_data(), sz, w, h);
        });

    m.def("stereo_parse_imu_buffer",
        [](py::memoryview mv, imu_subnode_data_t &imu){
            ssize_t _;
            uint8_t* ptr = buf_ptr(mv, _);
            stereo_parse_imu_buffer(ptr, &imu);
        });
}
