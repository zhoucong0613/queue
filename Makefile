PROJECT_NAME   := stereo_client
SRC_DIR        := ./src
INCLUDE_DIR    := ./include

LIB_NAME       := libstereo.so
LIB_DIR        := ./lib
LIB_FILE       := $(LIB_DIR)/$(LIB_NAME)

SRC_C_FILES    := $(wildcard $(SRC_DIR)/*.c)
SRC_CPP_FILES  := $(wildcard $(SRC_DIR)/*.cpp)
MAIN_SRC       := main.c
MAIN_OBJ       := main.o

SRC_C_OBJS     := $(patsubst %.c, %.o, $(SRC_C_FILES))
SRC_CPP_OBJS   := $(patsubst %.cpp, %.o, $(SRC_CPP_FILES))
SRC_ALL_OBJS   := $(SRC_C_OBJS) $(SRC_CPP_OBJS)

CC             := gcc
CXX            := g++
RM             := rm -f
MKDIR          := mkdir -p

ARCH           := $(shell uname -m)
COMMON_CFLAGS  := -Wall -O2 -fPIC -std=c99
COMMON_CXXFLAGS:= -Wall -O2 -fPIC -std=c++11
DEFINES        :=
ARCH_CFLAGS    :=
ARCH_CXXFLAGS  :=

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
    DEFINES      += -DUSE_NEON=0 -D_GNU_SOURCE
    ARCH_CFLAGS  += -march=native -mtune=native
    ARCH_CXXFLAGS += -march=native -mtune=native
    $(info STATUS: Architecture: x86/x86_64 ($(ARCH)))
else
    DEFINES      += -DUSE_NEON=0 -D_GNU_SOURCE
    $(warning WARNING: Unknown architecture: $(ARCH), using generic compile flags)
endif

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

CFLAGS         := $(COMMON_CFLAGS) $(ARCH_CFLAGS) $(DEFINES) $(OPENMP_FLAGS)
CXXFLAGS       := $(COMMON_CXXFLAGS) $(ARCH_CXXFLAGS) $(DEFINES) $(OPENCV_CFLAGS) $(OPENMP_FLAGS)
INCLUDES       := -I$(INCLUDE_DIR)
LIB_LDFLAGS    := -shared
MAIN_LDFLAGS   := -L$(LIB_DIR) -lstereo
LDFLAGS        := $(OPENCV_LIBS) $(OPENMP_FLAGS)
TARGET         := $(PROJECT_NAME)

all: create_lib_dir $(LIB_FILE) $(TARGET)
	$(info STATUS: Build completed successfully! Target: ./$(TARGET))
	$(info STATUS: Shared library: $(LIB_FILE))

create_lib_dir:
	$(MKDIR) $(LIB_DIR)
	$(info STATUS: Created library directory: $(LIB_DIR))

$(LIB_FILE): $(SRC_ALL_OBJS)
	$(CXX) $(LIB_LDFLAGS) $(SRC_ALL_OBJS) -o $(LIB_FILE) $(LDFLAGS)
	$(info STATUS: Generated shared library: $(LIB_FILE))

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	$(info STATUS: Compiled C file (shared lib): $< → $@)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@
	$(info STATUS: Compiled C++ file (shared lib): $< → $@)

$(MAIN_OBJ): $(MAIN_SRC)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	$(info STATUS: Compiled main.c (caller): $< → $@)

$(TARGET): $(MAIN_OBJ) $(LIB_FILE)
	$(CC) $(MAIN_OBJ) -o $(TARGET) $(MAIN_LDFLAGS) $(LDFLAGS)
	$(info STATUS: Linked executable: ./$(TARGET))

clean:
	$(RM) $(SRC_ALL_OBJS) $(MAIN_OBJ)
	$(RM) $(LIB_FILE) $(TARGET)
	$(RM) -r $(LIB_DIR)
	$(info STATUS: Cleaned all build artifacts successfully.)

.PHONY: all clean create_lib_dir