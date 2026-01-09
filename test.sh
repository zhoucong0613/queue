INTERFACE=usb0
CLIENT_IP="192.168.5.10"
SERVER_IP="192.168.5.5"

sudo ifconfig ${INTERFACE} mtu 9000

./stereo_client -i ${INTERFACE} -s ${SERVER_IP} -p -m 1
