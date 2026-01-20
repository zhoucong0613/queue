#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <unordered_map>
#include <mutex>
#include <tuple>

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

static char* py_str_to_c_str(const std::string& s) {
    if (s.empty()) return nullptr;
    char* buf = new char[s.size() + 1];
    std::strcpy(buf, s.c_str());
    return buf;
}

static void free_c_str(char* ptr) {
    if (ptr) delete[] ptr;
}

static std::unordered_map<uint64_t, py::function> g_sync_queue_cb_map;
static std::mutex g_sync_queue_cb_mutex;
static uint64_t g_cb_id = 0;

static int sync_queue_cb_wrapper(void* param, void* data) {
    std::lock_guard<std::mutex> lock(g_sync_queue_cb_mutex);
    uint64_t cb_id = reinterpret_cast<uint64_t>(param);
    auto it = g_sync_queue_cb_map.find(cb_id);
    if (it == g_sync_queue_cb_map.end()) return -1;
    
    try {
        return it->second(py::capsule(param), py::capsule(data)).cast<int>();
    } catch (const py::error_already_set& e) {
        py::print("Sync queue callback error:", e.what());
        return -1;
    }
}

static uint64_t register_sync_queue_cb(py::function cb) {
    std::lock_guard<std::mutex> lock(g_sync_queue_cb_mutex);
    uint64_t id = ++g_cb_id;
    g_sync_queue_cb_map[id] = cb;
    return id;
}

static void unregister_sync_queue_cb(uint64_t cb_id) {
    std::lock_guard<std::mutex> lock(g_sync_queue_cb_mutex);
    g_sync_queue_cb_map.erase(cb_id);
}

std::tuple<int, py::capsule> sync_queue_get_unused_object_wrapper(sync_queue_t* sync_queue, uint32_t timeout_ms) {
    data_item_t *data_item = nullptr;
    int ret = sync_queue_get_unused_object(sync_queue, timeout_ms, &data_item);
    py::capsule cap(data_item, [](void *p) {});
    return std::make_tuple(ret, cap);
}

std::tuple<int, py::capsule> sync_queue_obtain_inused_object_wrapper(sync_queue_t* sync_queue, uint32_t timeout_ms) {
    data_item_t *data_item = nullptr;
    int ret = sync_queue_obtain_inused_object(sync_queue, timeout_ms, &data_item);
    py::capsule cap(data_item, [](void *p) {});
    return std::make_tuple(ret, cap);
}

std::tuple<int, py::capsule> sync_queue_obtain_inused_object_width_user_wrapper(sync_queue_t* sync_queue, uint32_t timeout_ms, int user_flag) {
    data_item_t *data_item = nullptr;
    int ret = sync_queue_obtain_inused_object_width_user(sync_queue, timeout_ms, &data_item, user_flag);
    py::capsule cap(data_item, [](void *p) {});
    return std::make_tuple(ret, cap);
}

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


      m.attr("SYNC_QUEUE_MAX_USER") = SYNC_QUEUE_MAX_USER;

      py::class_<data_item_t>(m, "DataItem")
            .def(py::init<>())
            .def_readwrite("is_init_added", &data_item_t::is_init_added)
            .def_readwrite("item_count", &data_item_t::item_count)
            .def_readwrite("ref_repay", &data_item_t::ref_repay)
            .def_readwrite("ref_obtain", &data_item_t::ref_obtain)
            .def_readwrite("index", &data_item_t::index)
            .def_readwrite("inused_update_time_ms", &data_item_t::inused_update_time_ms)
            .def_readwrite("unused_update_time_ms", &data_item_t::unused_update_time_ms)
            .def_readwrite("inused_frame_index", &data_item_t::inused_frame_index)
            .def_readwrite("unused_frame_index", &data_item_t::unused_frame_index)
            .def("get_items", [](const data_item_t &self) {
                  return reinterpret_cast<uint64_t>(self.items);
            }, "Get items pointer (as uint64_t address)")
            .def("set_items", [](data_item_t &self, uint64_t addr) {
                  self.items = reinterpret_cast<void*>(addr);
            }, "Set items pointer (from uint64_t address)", py::arg("addr"))
            .def("get_user_recoder", [](const data_item_t &self, size_t idx) {
                  if (idx >= SYNC_QUEUE_MAX_USER) throw py::index_error("user_recoder index out of range");
                  return self.user_recoder[idx];
            }, "Get user_recoder element by index", py::arg("idx"))
            .def("set_user_recoder", [](data_item_t &self, size_t idx, uint8_t value) {
                  if (idx >= SYNC_QUEUE_MAX_USER) throw py::index_error("user_recoder index out of range");
                  self.user_recoder[idx] = value;
            }, "Set user_recoder element by index", py::arg("idx"), py::arg("value"));


      py::class_<sync_queue_info_t>(m, "SyncQueueInfo")
            .def(py::init<>())
            .def_property("productor_name",
                [](const sync_queue_info_t& self) {
                    return self.productor_name ? std::string(self.productor_name) : "";
                },
                [](sync_queue_info_t& self, const std::string& s) {
                    if (self.productor_name) free_c_str(self.productor_name);
                    self.productor_name = py_str_to_c_str(s);
                })
            .def_property("consumer_name",
                [](const sync_queue_info_t& self) {
                    return self.consumer_name ? std::string(self.consumer_name) : "";
                },
                [](sync_queue_info_t& self, const std::string& s) {
                    if (self.consumer_name) free_c_str(self.consumer_name);
                    self.consumer_name = py_str_to_c_str(s);
                })
            .def_readwrite("is_need_malloc_in_advance", &sync_queue_info_t::is_need_malloc_in_advance)
            .def_readwrite("is_external_buffer", &sync_queue_info_t::is_external_buffer)
            .def_readwrite("queue_len", &sync_queue_info_t::queue_len)
            .def_readwrite("data_item_size", &sync_queue_info_t::data_item_size)
            .def_readwrite("data_item_count", &sync_queue_info_t::data_item_count)
            .def_readwrite("item_data_init_param", &sync_queue_info_t::item_data_init_param)
            .def_readwrite("item_data_deinit_param", &sync_queue_info_t::item_data_deinit_param)
            .def_property("item_data_init_func",
                [](const sync_queue_info_t& self) {
                    return self.item_data_init_func ? reinterpret_cast<uint64_t>(self.item_data_init_param) : 0ULL;
                },
                [](sync_queue_info_t& self, py::object cb_obj) {
                    if (cb_obj.is_none() || !py::isinstance<py::function>(cb_obj)) {
                        self.item_data_init_func = nullptr;
                        self.item_data_init_param = nullptr;
                        return;
                    }
                    py::function cb = cb_obj.cast<py::function>();
                    uint64_t cb_id = register_sync_queue_cb(cb);
                    self.item_data_init_func = &sync_queue_cb_wrapper;
                    self.item_data_init_param = reinterpret_cast<void*>(cb_id);
                })
            .def_property("item_data_deinit_func",
                [](const sync_queue_info_t& self) {
                    return self.item_data_deinit_func ? reinterpret_cast<uint64_t>(self.item_data_deinit_param) : 0ULL;
                },
                [](sync_queue_info_t& self, py::object cb_obj) {
                    if (cb_obj.is_none() || !py::isinstance<py::function>(cb_obj)) {
                        self.item_data_deinit_func = nullptr;
                        self.item_data_deinit_param = nullptr;
                        return;
                    }
                    py::function cb = cb_obj.cast<py::function>();
                    uint64_t cb_id = register_sync_queue_cb(cb);
                    self.item_data_deinit_func = &sync_queue_cb_wrapper;
                    self.item_data_deinit_param = reinterpret_cast<void*>(cb_id);
                });

      py::class_<sync_queue_t>(m, "SyncQueue")
            .def(py::init<>())
            .def_readwrite("user_count", &sync_queue_t::user_count)
            .def_readwrite("inused_queue_count", &sync_queue_t::inused_queue_count)
            .def_readwrite("sync_queue_info", &sync_queue_t::sync_queue_info)
            .def_property("ununsed_queue", nullptr, nullptr)
            .def_property("inused_queue", nullptr, nullptr);

      m.def("sync_queue_unregister_cb", &unregister_sync_queue_cb, py::arg("cb_id"), "Unregister sync queue callback");
      m.def("sync_queue_add_user", &sync_queue_add_user, py::arg("sync_queue"), "Add user to sync queue");
      m.def("sync_queue_create", &sync_queue_create, py::arg("sync_queue"), py::arg("sync_queue_info"), "Create sync queue (compatible with C logic)");
      m.def("sync_queue_create_multi_user", &sync_queue_create_multi_user, py::arg("sync_queue"), py::arg("sync_queue_info"), "Create multi-user sync queue");
      m.def("sync_queue_destory", &sync_queue_destory, py::arg("sync_queue"), "Destroy sync queue");
      // 生产者函数
      m.def("sync_queue_get_unused_object", &sync_queue_get_unused_object_wrapper, 
            py::arg("sync_queue"), py::arg("timeout_ms"), 
            "Get unused object (producer), return (ret, data_item_capsule)");
      m.def("sync_queue_save_inused_object", &sync_queue_save_inused_object, 
            py::arg("sync_queue"), py::arg("timeout_ms"), py::arg("data_item"), 
            "Save inused object (producer)");
      
      // 消费者函数
      m.def("sync_queue_obtain_inused_object", &sync_queue_obtain_inused_object_wrapper, 
            py::arg("sync_queue"), py::arg("timeout_ms"), 
            "Obtain inused object (consumer), return (ret, data_item_capsule)");
      m.def("sync_queue_obtain_inused_object_width_user", &sync_queue_obtain_inused_object_width_user_wrapper, 
            py::arg("sync_queue"), py::arg("timeout_ms"), py::arg("user_flag"), 
            "Obtain inused object with user flag, return (ret, data_item_capsule)");
      m.def("sync_queue_repay_unused_object", &sync_queue_repay_unused_object, 
            py::arg("sync_queue"), py::arg("timeout_ms"), py::arg("data_item"), 
            "Repay unused object (consumer)");

      py::class_<PerformanceTestParamSimple>(m, "PerformanceTestParamSimple")
      .def(py::init<>())
      .def_readwrite("test_count", &PerformanceTestParamSimple::test_count)
      .def_readwrite("iteration_number", &PerformanceTestParamSimple::iteration_number)
      .def_readwrite("run_count", &PerformanceTestParamSimple::run_count)
      .def_readwrite("test_start_time_us", &PerformanceTestParamSimple::test_start_time_us)
      .def_readwrite("test_end_time_us", &PerformanceTestParamSimple::test_end_time_us)
      .def_readwrite("start_time_us", &PerformanceTestParamSimple::start_time_us)
      .def_readwrite("min_diff", &PerformanceTestParamSimple::min_diff)
      .def_readwrite("max_diff", &PerformanceTestParamSimple::max_diff)
      .def_property("test_case",
            [](const PerformanceTestParamSimple& self) {
                  return self.test_case ? std::string(self.test_case) : "";
            },
            [](PerformanceTestParamSimple& self, const std::string& s) {
                  static std::unordered_map<const char*, std::string> test_case_cache;
                  if (self.test_case && test_case_cache.count(self.test_case)) {
                  delete[] const_cast<char*>(self.test_case);
                  test_case_cache.erase(self.test_case);
                  }
                  char* new_str = new char[s.size() + 1];
                  std::strcpy(new_str, s.c_str());
                  self.test_case = const_cast<const char*>(new_str);
                  test_case_cache[self.test_case] = s;
            });

      m.def("performance_test_start_simple",
            &performance_test_start_simple,
            py::arg("param"),
            "Start performance test (simple version), param: PerformanceTestParamSimple instance");

      m.def("performance_test_stop_simple",
            &performance_test_stop_simple,
            py::arg("param"),
            "Stop performance test (simple version), param: PerformanceTestParamSimple instance");     

}