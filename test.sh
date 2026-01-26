#!/bin/bash

INTERFACE=usb0
CLIENT_IP="192.168.5.10"
SERVER_IP="192.168.5.5"

SCRIPT_DIR=$(cd $(dirname $0); pwd)
LIB_DIR="${SCRIPT_DIR}/lib"

export LD_LIBRARY_PATH="${LIB_DIR}:${LD_LIBRARY_PATH}"

sudo ifconfig ${INTERFACE} mtu 9000

./stereo_client -i ${INTERFACE} -s ${SERVER_IP} -m 1 -p
