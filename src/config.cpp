#include "config.h"

wiz_NetInfo own_network_info = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56},  // MAC address
    .ip = {192, 168, 100, 3},                     // IP address
    .sn = {255, 255, 255, 0},                     // Subnet Mask
    .gw = {192, 168, 100, 1},                     // Gateway
    .dns = {8, 8, 8, 8},                          // DNS server
    .dhcp = NETINFO_STATIC                        // DHCP enable/disable
};

const uint8_t dest_ip[4] = {192, 168, 100, 2};  // Destination IP address
const uint16_t dest_port = 5000;                // Destination port