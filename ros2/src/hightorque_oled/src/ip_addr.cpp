#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include "hightorque_oled/ip_addr.hpp"

uint32_t ip_array[5];

void update_ip_addr(void)
{
    int fd;
    struct ifreq ifr;
    char* net_adapter[3] = {LO_NET, ETHERNET, WLAN};

    fd = socket(AF_INET, SOCK_DGRAM, 0);

    /*get an IPv4 IP address */
    ifr.ifr_addr.sa_family = AF_INET;

    for(int i = 0; i < 3; i ++)
    {
        strncpy(ifr.ifr_name, net_adapter[i], IFNAMSIZ-1);
        if(ioctl(fd, SIOCGIFADDR, &ifr) != -1)
        {
            ip_array[i] = ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;
        }
        else
        {
            ip_array[i] = 0;
        }
    }

    close(fd);
}

uint32_t get_ip_data_u32(uint8_t i)
{
    return ip_array[i];
}

uint32_t* get_ip_data_u32_all(void)
{
    return ip_array;
}
