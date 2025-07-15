#include "output.h"

#include <FreeRTOS.h>
#include <queue.h>
#include <stdio.h>
#include <task.h>
#include <time.h>

#include "config.h"
#include "eth.h"
#include "hardware/spi.h"
#include "packet.h"
#include "pico/stdlib.h"

#define NEOPIXEL_PIN 15
#define EMERGENCY_STOP_PIN 14

void output_task(void *pvParameters) {
  gpio_init(EMERGENCY_STOP_PIN);
  gpio_init(NEOPIXEL_PIN);
  gpio_set_dir(EMERGENCY_STOP_PIN, GPIO_OUT);
  gpio_set_dir(NEOPIXEL_PIN, GPIO_OUT);

  gpio_put(EMERGENCY_STOP_PIN, 1);

  while (1) {
    DownlinkPacket packet;
    if (xQueuePeek(recv_queue, &packet, portMAX_DELAY) != pdTRUE) {
      printf("Failed to receive packet from queue\n");
      continue;
    }

    if (packet.emergency_stop) {
      gpio_put(EMERGENCY_STOP_PIN, 0);
      printf("Emergency stop activated\n");
    } else {
      gpio_put(EMERGENCY_STOP_PIN, 1);
    }

    // printf("Received packet: led_pattern=%d, emergency_stop=%d\n",
    //        packet.led_pattern, packet.emergency_stop);

    // Output processing logic here
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
