#pragma once

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "socket.h"
#include "timer.h"
#include "w5x00_gpio_irq.h"
#include "w5x00_spi.h"
#include "wizchip_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

void eth_setup(void);
void eth_send(char *data, size_t len, const uint8_t *ip, uint16_t port);
void eth_recv_task(void *pvParameters);

extern TaskHandle_t eth_send_task_handle;
extern TaskHandle_t eth_recv_task_handle;

extern QueueHandle_t recv_queue;

#ifdef __cplusplus
}
#endif