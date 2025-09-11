#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <net/if.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <sys/socket.h>
#include <errno.h>

#include "common.h"

// 检查网卡是否存在
static int is_interface_exist(const char *ifname)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ);

    // 通过获取MAC地址判断接口是否存在
    int ret = ioctl(sock, SIOCGIFHWADDR, &ifr);
    close(sock);
    return (ret == 0) ? 1 : 0;
}

// 检查是否配置IP地址
static int is_interface_has_ip(const char *ifname)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ);

    // 获取IPv4地址
    if (ioctl(sock, SIOCGIFADDR, &ifr) == 0) {
        struct sockaddr_in *sin = (struct sockaddr_in *)&ifr.ifr_addr;
        close(sock);
        return (sin->sin_addr.s_addr != INADDR_ANY) ? 1 : 0;
    }
    close(sock);
    return 0;
}

int check_network(const char *ifname)
{
    // 第一步：检测usb0是否存在
    int exist = is_interface_exist(ifname);
    if (exist == 1) {
        printf("[+] %s Network exist\n", ifname);

        // 第二步：检测IP配置状态
        int has_ip = is_interface_has_ip(ifname);
        if (has_ip == 1) {
            struct sockaddr_in sin;
            char ip_str[INET_ADDRSTRLEN];

            // 获取并打印具体IP地址
            int sock = socket(AF_INET, SOCK_DGRAM, 0);
            struct ifreq ifr;
            strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
            ioctl(sock, SIOCGIFADDR, &ifr);
            memcpy(&sin, &ifr.ifr_addr, sizeof(sin));
            inet_ntop(AF_INET, &sin.sin_addr, ip_str, INET_ADDRSTRLEN);
            close(sock);

            printf("[+] IP Configed: %s\n", ip_str);
        } else {
            printf("[-] IP Not Configed\n");
			return -1;
        }
    } else if (exist == 0) {
        printf("[-] %s Network Not exist\n", ifname);
		return -1;
    } else {
        perror("Network Detect Error\n");
		return -1;
    }
    return 0;
}

int client_init(const char *server_ip)
{
	int client_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (client_fd < 0) {
		printf("[Client] Socket Error: %s\n", strerror(errno));
		return -1;
	}

	// int flag = fcntl(client_fd, F_GETFL);
	// flag |= O_NONBLOCK;
	// if (fcntl(client_fd, F_SETFL, flag) == -1) {
	// 	printf("[Client] fcntl Error: %s\n", strerror(errno));
	// 	close(client_fd);
	// 	return -1;
	// }

	int ret;
    struct sockaddr_in addr;
    
    addr.sin_family = AF_INET;
    addr.sin_port = htons(SERVER_PORT);
    addr.sin_addr.s_addr = inet_addr(server_ip);
 
	ret = connect(client_fd, (struct sockaddr*)&addr, sizeof(addr));
	if (ret != 0) {
		printf("[Client] Connect server[%s] err(%d) %s", server_ip, ret, strerror(errno));
		close(client_fd);
		return -1;
	}

	return client_fd;
}