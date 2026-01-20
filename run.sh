#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
LIB_DIR="${SCRIPT_DIR}/lib"
PY_SCRIPT="${SCRIPT_DIR}/python/stereo_client.py"

RUN_ARGS="-i usb0 -s 192.168.5.5 -m 1"

export PYTHONPATH="${LIB_DIR}:${PYTHONPATH}"
export LD_LIBRARY_PATH="${LIB_DIR}:${LD_LIBRARY_PATH}"

if [ ! -f "${PY_SCRIPT}" ]; then
    echo -e "\033[31mERROR: Python脚本不存在 → ${PY_SCRIPT}\033[0m"
    echo "请确认 python 目录下有 stereo_client.py 文件！"
    exit 1
fi

if [ ! -f "${LIB_DIR}/stereo_client_py.so" ]; then
    echo -e "\033[31mERROR: Python绑定模块不存在 → ${LIB_DIR}/stereo_client_py.so\033[0m"
    echo "请先在根目录执行 make 编译生成模块！"
    exit 1
fi

if [ ! -f "${LIB_DIR}/libstereo.so" ]; then
    echo -e "\033[31mERROR: 核心库不存在 → ${LIB_DIR}/libstereo.so\033[0m"
    echo "请先在根目录执行 make 编译生成库文件！"
    exit 1
fi


python3 "${PY_SCRIPT}" ${RUN_ARGS}

EXIT_CODE=$?
if [ ${EXIT_CODE} -eq 0 ]; then
    echo -e "\n\033[32mINFO: 脚本执行成功！\033[0m"
else
    echo -e "\n\033[31mERROR: 脚本执行失败，退出码 → ${EXIT_CODE}\033[0m"
    exit ${EXIT_CODE}
fi