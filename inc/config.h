#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "wizchip_conf.h"

// 自分のネットワーク情報
extern wiz_NetInfo own_network_info;
extern const uint8_t dest_ip[4];
extern const uint16_t dest_port;

#ifdef __cplusplus
}
#endif