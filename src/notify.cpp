#include "notify.h"

#include <FreeRTOS.h>
#include <stdio.h>
#include <task.h>
#include <time.h>

#include "eth.h"
#include "hardware/spi.h"
#include "packet.h"
#include "pico/stdlib.h"

char limit_sw_pin[] = {6, 7, 8, 9, 10, 11, 12, 13};

void notify_task(void *pvParameters) {
  for (int i = 0; i < sizeof(limit_sw_pin); i++) {
    gpio_init(limit_sw_pin[i]);
    gpio_set_dir(limit_sw_pin[i], GPIO_IN);
  }

  spi_init(spi0, 100 * 1000);
  gpio_set_function(2, GPIO_FUNC_SPI);  // SCK
  gpio_set_function(3, GPIO_FUNC_SPI);  // MOSI
  gpio_set_function(4, GPIO_FUNC_SPI);  // MISO
  gpio_set_function(5, GPIO_FUNC_SPI);  // CS
  spi_set_format(spi0, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

  while (1) {
    int16_t absenc_val = 0;
    spi_read16_blocking(spi0, 0x0000, (uint16_t *)&absenc_val, 1);

    uint8_t limit_sw_val = 0;
    for (int i = 0; i < sizeof(limit_sw_pin); i++) {
      if (gpio_get(limit_sw_pin[i])) {
        limit_sw_val |= (1 << i);
      }
    }

    UplinkPacket uplink_packet = {
        .abs_enc = absenc_val,
        .limit_sw = limit_sw_val,
    };
    if (xQueueSend(recv_queue, &uplink_packet, 0) != pdTRUE) {
      printf("Failed to send packet to queue\n");
    } else {
      // printf("Sent packet: abs_enc=%d, limit_sw=0x%02X\n", absenc_val,
      // limit_sw_val);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}