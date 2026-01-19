# ==============================================================================
# C/C++混合编译Makefile（支持纯C调用C++ OpenCV）
# 项目：stereo_client
# 功能：纯C（main.c）调用C++ OpenCV接口，兼容混合编译
# 修正点：1. 修复scripts目标shell循环语法 2. 规范变量引用 3. 补充续行符避免分号错误
# ==============================================================================

# -------------------------- 基础配置（区分C/C++文件） --------------------------
PROJECT_NAME   := stereo_client
SRC_DIR        := ./src
INCLUDE_DIR    := ./include
# 1. 纯C文件（.c）：main.c + src下所有.c文件（自动排除.cpp文件，无需额外处理）
C_SRCS         := main.c $(wildcard $(SRC_DIR)/*.c)
# 2. C++文件（.cpp，依赖OpenCV）：src下所有.cpp文件（如render_depth.cpp）
CPP_SRCS       := $(wildcard $(SRC_DIR)/*.cpp)
SCRIPT_FILES   := test.sh delete.sh

# 目标文件配置（核心：避免重复收集，直接转换，无重叠）
# C文件对应.o文件（仅转换一次，无重复）
C_OBJS         := $(patsubst %.c, %.o, $(C_SRCS))
# C++文件对应.o文件（仅转换一次，无重复）
CPP_OBJS       := $(patsubst %.cpp, %.o, $(CPP_SRCS))
# 所有目标文件（C+CPP，唯一无重复）
ALL_OBJS       := $(C_OBJS) $(CPP_OBJS)

# 编译工具配置（区分C/C++编译器，链接用g++）
CC             := gcc            # 纯C编译器
CXX            := g++            # C++编译器（编译OpenCV相关代码）
RM             := rm -f          # 删除命令
CHMOD          := chmod +x       # 脚本授权命令

# -------------------------- 架构检测 & 编译选项配置 --------------------------
ARCH           := $(shell uname -m)
COMMON_CFLAGS  := -Wall -O2 -fPIC -std=c99  # C编译标准（C99）
COMMON_CXXFLAGS:= -Wall -O2 -fPIC -std=c++11 # C++编译标准（支持OpenCV）
DEFINES        :=
ARCH_CFLAGS    :=
ARCH_CXXFLAGS  :=

# 架构分支判断（C/C++分别配置架构选项，补充-GNU_SOURCE宏）
ifneq (, $(findstring arm, $(ARCH))$(findstring aarch64, $(ARCH)))
    DEFINES      += -DUSE_NEON=1 -D__ARM_NEON__ -D_GNU_SOURCE
    ifeq (, $(findstring aarch64, $(ARCH)))
        ARCH_CFLAGS += -march=armv7-a -mfpu=neon -mfloat-abi=hard
        ARCH_CXXFLAGS += -march=armv7-a -mfpu=neon -mfloat-abi=hard
        $(info STATUS: Architecture: ARM32 ($(ARCH)) - NEON enabled with -mfpu=neon)
    else
        ARCH_CFLAGS += -march=armv8-a
        ARCH_CXXFLAGS += -march=armv8-a
        $(info STATUS: Architecture: ARM64 ($(ARCH)) - NEON enabled by default)
    endif
else ifneq (, $(findstring x86_64, $(ARCH))$(findstring i386, $(ARCH))$(findstring i686, $(ARCH)))
    DEFINES      += -DUSE_NEON=0 -D_GNU_SOURCE  # 核心补充：-D_GNU_SOURCE，解决网络编程定义问题
    ARCH_CFLAGS  += -march=native -mtune=native
    ARCH_CXXFLAGS += -march=native -mtune=native
    $(info STATUS: Architecture: x86/x86_64 ($(ARCH)))
else
    DEFINES      += -DUSE_NEON=0 -D_GNU_SOURCE
    $(warning WARNING: Unknown architecture: $(ARCH), using generic compile flags)
endif

# -------------------------- 依赖配置（OpenCV + OpenMP，支持C++） --------------------------
ifneq ($(shell pkg-config --exists opencv4; echo $$?), 0)
    ifneq ($(shell pkg-config --exists opencv; echo $$?), 0)
        $(error ERROR: OpenCV not found! Please install OpenCV and configure pkg-config path.)
    else
        OPENCV_CFLAGS := $(shell pkg-config --cflags opencv)
        OPENCV_LIBS   := $(shell pkg-config --libs opencv)
    endif
else
    OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4)
    OPENCV_LIBS   := $(shell pkg-config --libs opencv4)
endif
OPENMP_FLAGS   := -fopenmp

# -------------------------- 最终编译/链接选项汇总 --------------------------
# C编译选项（纯C，无OpenCV头文件，补充-D_GNU_SOURCE，避免C编译器报错）
CFLAGS         := $(COMMON_CFLAGS) $(ARCH_CFLAGS) $(DEFINES) $(OPENMP_FLAGS)
# C++编译选项（包含OpenCV头文件，支持C++特性）
CXXFLAGS       := $(COMMON_CXXFLAGS) $(ARCH_CXXFLAGS) $(DEFINES) $(OPENCV_CFLAGS) $(OPENMP_FLAGS)
# 头文件包含路径
INCLUDES       := -I$(INCLUDE_DIR)
# 链接选项（用g++链接，包含OpenCV和OpenMP库，确保C++标准库依赖）
LDFLAGS        := $(OPENCV_LIBS) $(OPENMP_FLAGS)
# 可执行文件（根目录）
TARGET         := $(PROJECT_NAME)

# -------------------------- 构建规则（混合编译，链接用g++） --------------------------
# 默认目标：构建所有产物
all: $(TARGET) scripts
	$(info STATUS: Build completed successfully! Target: ./$(TARGET))
	$(info STATUS: Final C compile flags: $(CFLAGS))
	$(info STATUS: Final C++ compile flags: $(CXXFLAGS))

# 构建可执行文件（关键：用g++链接，仅引用一次ALL_OBJS，无重复目标文件）
$(TARGET): $(ALL_OBJS)
	$(CXX) $(ALL_OBJS) -o $(TARGET) $(LDFLAGS)
	$(info STATUS: Linked target file with g++ (C++/OpenCV support): ./$(TARGET))

# 编译纯C文件：匹配所有.c文件（main.c + src/*.c），生成对应.o文件
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	$(info STATUS: Compiled C file: $< → $@)

# 编译C++文件：匹配所有.cpp文件（src/*.cpp），生成对应.o文件
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@
	$(info STATUS: Compiled C++ file: $< → $@)

# 清理目标（删除所有.o文件和可执行文件，无残留）
clean:
	$(RM) $(C_OBJS)
	$(RM) $(CPP_OBJS)
	$(RM) $(TARGET)
	$(info STATUS: Cleaned all build artifacts successfully.)

# -------------------------- 伪目标声明（避免与同名文件冲突） --------------------------
.PHONY: all clean scripts