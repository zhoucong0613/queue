cmake_minimum_required(VERSION 3.10)

# 工程名
project(TestProject LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 打开警告和优化
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

# 头文件目录
include_directories(${PROJECT_SOURCE_DIR}/include)

# 查找 OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 源文件
set(SRC_FILES
    ${PROJECT_SOURCE_DIR}/main.cpp
    ${PROJECT_SOURCE_DIR}/src/cJSON.cpp
    ${PROJECT_SOURCE_DIR}/src/common.cpp
    ${PROJECT_SOURCE_DIR}/src/mqueue.cpp
    ${PROJECT_SOURCE_DIR}/src/performance_test_util.cpp
    ${PROJECT_SOURCE_DIR}/src/synchronous_queue.cpp
)

# 生成可执行文件
add_executable(${PROJECT_NAME} ${SRC_FILES})

# 链接 OpenCV 库
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# 输出路径设置为 cmake 执行时的构建目录
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

set(SCRIPT_FILES
    ${PROJECT_SOURCE_DIR}/test.sh    # 项目根目录的 test.sh
    ${PROJECT_SOURCE_DIR}/delete.sh  # 项目根目录的 delete.sh
)

# 遍历脚本文件，逐个复制到编译目录
foreach(SCRIPT ${SCRIPT_FILES})
    # 检查脚本文件是否存在（避免编译错误）
    if(EXISTS ${SCRIPT})
        # 获取脚本文件名（不含路径）
        get_filename_component(SCRIPT_NAME ${SCRIPT} NAME)
        # 复制脚本到编译目录，并设置可执行权限
        add_custom_command(
            TARGET ${PROJECT_NAME}  # 绑定到目标：生成可执行文件后自动复制
            POST_BUILD             # 编译完成后执行
            COMMAND ${CMAKE_COMMAND} -E copy ${SCRIPT} ${CMAKE_BINARY_DIR}/${SCRIPT_NAME}
            COMMAND chmod +x ${CMAKE_BINARY_DIR}/${SCRIPT_NAME}  # 赋予可执行权限
            COMMENT "Copying ${SCRIPT_NAME} to build directory and setting execute permission"
        )
    else()
        message(WARNING "${SCRIPT} not found! Skipping copy.")
    endif()
endforeach()

