INTERFACE=usb0
CLIENT_IP="192.168.5.10"
SERVER_IP="192.168.5.5"

sudo ifconfig ${INTERFACE} mtu 9000

./TestProject -i ${INTERFACE} -s ${SERVER_IP} -p
