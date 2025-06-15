#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "wizchip_conf.h"

// 自分のネットワーク情報
wiz_NetInfo own_network_info = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56},  // MAC address
    .ip = {192, 168, 100, 3},                     // IP address
    .sn = {255, 255, 255, 0},                     // Subnet Mask
    .gw = {192, 168, 100, 1},                     // Gateway
    .dns = {8, 8, 8, 8},                          // DNS server
    .dhcp = NETINFO_STATIC                        // DHCP enable/disable
};

#ifdef __cplusplus
}
#endif