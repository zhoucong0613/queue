cmake_minimum_required(VERSION 3.10)

project(stereo_client 
    LANGUAGES CXX
    VERSION 1.0.0
    DESCRIPTION "Stereo Client with arch-specific optimizations"
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


set(COMMON_COMPILE_FLAGS
    -Wall
    -O2        
    -fPIC
)

set(ARCH "${CMAKE_HOST_SYSTEM_PROCESSOR}")
message(STATUS "Detected host architecture: ${ARCH}")

if(ARCH MATCHES "arm|aarch64|ARM|AARCH64")
    set(IS_ARM 1)
    set(IS_X86 0)
    message(STATUS "Architecture: ARM (${ARCH})")

    set(ARM_COMPILE_FLAGS "")
    
    if(ARCH MATCHES "aarch64|AARCH64")
        set(ENABLE_NEON 1)
        list(APPEND ARM_COMPILE_FLAGS -march=armv8-a)
        message(STATUS "ARM64 detected: NEON enabled by default")
    else()
        set(ENABLE_NEON 1)
        list(APPEND ARM_COMPILE_FLAGS
            -march=armv7-a
            -mfpu=neon
            -mfloat-abi=hard
        )
        message(STATUS "ARM32 detected: NEON enabled with -mfpu=neon")
    endif()

    add_compile_definitions(USE_NEON=${ENABLE_NEON})
    add_compile_definitions(__ARM_NEON__)

elseif(ARCH MATCHES "x86_64|amd64|i386|i686|X86")
    set(IS_ARM 0)
    set(IS_X86 1)
    message(STATUS "Architecture: x86/x86_64 (${ARCH})")

    set(X86_COMPILE_FLAGS
        -march=native
        -mtune=native
    )

    add_compile_definitions(USE_NEON=0)

else()
    set(IS_ARM 0)
    set(IS_X86 0)
    message(WARNING "Unknown architecture: ${ARCH}, using generic compile flags")
    add_compile_definitions(USE_NEON=0)
endif()

find_package(OpenCV REQUIRED QUIET)
if(OpenCV_FOUND)
    message(STATUS "Found OpenCV: ${OpenCV_VERSION} (${OpenCV_INCLUDE_DIRS})")
endif()

find_package(OpenMP REQUIRED QUIET)
if(OpenMP_FOUND)
    message(STATUS "Found OpenMP: ${OpenMP_VERSION}")
endif()


set(SRC_FILES
    ${PROJECT_SOURCE_DIR}/main.cpp
    ${PROJECT_SOURCE_DIR}/src/cJSON.cpp
    ${PROJECT_SOURCE_DIR}/src/common.cpp
    ${PROJECT_SOURCE_DIR}/src/mqueue.cpp
    ${PROJECT_SOURCE_DIR}/src/performance_test_util.cpp
    ${PROJECT_SOURCE_DIR}/src/synchronous_queue.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_include_directories(${PROJECT_NAME}
    PRIVATE ${PROJECT_SOURCE_DIR}/include
    PRIVATE ${OpenCV_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME}
    PRIVATE ${COMMON_COMPILE_FLAGS}
    PRIVATE $<$<BOOL:${IS_ARM}>:${ARM_COMPILE_FLAGS}>
    PRIVATE $<$<BOOL:${IS_X86}>:${X86_COMPILE_FLAGS}>
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE ${OpenCV_LIBS}
    PRIVATE OpenMP::OpenMP_CXX
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    OUTPUT_NAME "stereo_client"
    DEBUG_POSTFIX "_debug"
)

set(SCRIPT_FILES
    ${PROJECT_SOURCE_DIR}/test.sh
    ${PROJECT_SOURCE_DIR}/delete.sh
)

foreach(SCRIPT ${SCRIPT_FILES})
    if(EXISTS ${SCRIPT})
        get_filename_component(SCRIPT_NAME ${SCRIPT} NAME)
        set(DEST_SCRIPT "${CMAKE_BINARY_DIR}/${SCRIPT_NAME}")
        
        add_custom_command(
            TARGET ${PROJECT_NAME}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SCRIPT} ${DEST_SCRIPT}
            COMMAND chmod +x ${DEST_SCRIPT}
            COMMENT "Copying and chmod ${SCRIPT_NAME} to build dir"
            VERBATIM 
        )
    else()
        message(WARNING "Script file not found: ${SCRIPT} (skipped)")
    endif()
endforeach()

message(STATUS "=====================================")
message(STATUS "Final compile flags for ${PROJECT_NAME}:")
get_target_property(COMPILE_FLAGS ${PROJECT_NAME} COMPILE_OPTIONS)
message(STATUS "  ${COMPILE_FLAGS}")
message(STATUS "=====================================")