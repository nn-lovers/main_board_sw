#include "eth.h"

#include <FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <time.h>

#include "config.h"
#include "packet.h"
#include "timer.h"

#ifdef __cplusplus
extern "C" {
#endif

static uint8_t recvbuf[2048];

static void recv_intr_callback();

void eth_setup(void) {
  wizchip_spi_initialize();
  wizchip_cris_initialize();

  wizchip_reset();
  wizchip_initialize();
  wizchip_check();
  // wizchip_gpio_interrupt_initialize(0, gpio_callback);

  network_initialize(own_network_info);
  print_network_information(own_network_info);
  int retval = socket(0, Sn_MR_UDP, 5000, 0);
  if (retval != 0) {
    printf("socket() failed: %d\n", retval);
    return;
  }
}

void eth_send(char *data, size_t len, uint8_t *ip, uint16_t port) {
  if (len > 2048) {
    printf("Data length exceeds buffer size\n");
    return;
  }

  int32_t sent_len = sendto(0, (uint8_t *)data, len, ip, port);
  if (sent_len < 0) {
    printf("sendto() failed: %d\n", sent_len);
  } else {
    // printf("Sent %d bytes to %d.%d.%d.%d:%d\n", sent_len, ip[0], ip[1],
    //        ip[2], ip[3], port);
  }
}

void eth_recv_task(void *pvParameters) {
  uint16_t reg_val;
  int32_t recv_len = 0;
  uint8_t queuebuf = 0;

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ctlwizchip(CW_GET_INTERRUPT, &reg_val);
    if (!(reg_val & 0b1)) continue;
    reg_val = (SIK_CONNECTED | SIK_DISCONNECTED | SIK_TIMEOUT | SIK_RECEIVED) &
              0x00ff;
    ctlsocket(0, CS_CLR_INTERRUPT, &reg_val);
    if (!(reg_val & SIK_RECEIVED)) continue;

    uint8_t addr[4] = {0};
    uint16_t port = 0;

    if (xSemaphoreTake(eth_mutex, portMAX_DELAY) != pdTRUE) {
      printf("Failed to take eth_mutex\n");
      continue;
    }
    recv_len = recvfrom(0, recvbuf, sizeof(recvbuf), addr, &port);
    xSemaphoreGive(eth_mutex);

    if (recv_len > 0) {
      // printf("recv: %d\n", recv_len);
      // recvbuf[recv_len] = '\0';
      // printf("recv from %d.%d.%d.%d:%d -> %s\n", addr[0], addr[1], addr[2],
      //        addr[3], port, recvbuf);
      // for (int i = 0; i < recv_len; i++) {
      //   printf("%02x ", recvbuf[i]);
      // }
      // gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
      // printf("\n");

      DownlinkPacket packet;
      packet.led_pattern = recvbuf[0];
      packet.emergency_stop = recvbuf[1] != 0;
      if (xQueueOverwrite(recv_queue, &packet) != pdTRUE) {
        printf("Failed to send packet to queue\n");
      } else {
        // printf("Packet sent to queue: led_pattern=%d, emergency_stop=%d\n",
        //  packet.led_pattern, packet.emergency_stop);
      }
    } else {
      printf("recv() failed: %d\n", recv_len);
    }
    vTaskDelay(100);
  }
}

#ifdef __cplusplus
}
#endif