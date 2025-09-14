#pragma once

#include "socket.h"
#include "timer.h"
#include "wizchip_spi.h"
// #include "w5x00_gpio_irq.h"
// #include "w5x00_spi.h"
#include "wizchip_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

void eth_setup(void);
void eth_send(char *data, size_t len, uint8_t *ip, uint16_t port);
int32_t eth_recv(char *buf, size_t len);

#ifdef __cplusplus
}
#endif