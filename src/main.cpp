#include <FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <task.h>
#include <time.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"
#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "socket.h"
#include "timer.h"
#include "w5x00_gpio_irq.h"
#include "w5x00_spi.h"
#include "wizchip_conf.h"

#define PLL_SYS_KHZ (133 * 1000)

// Task handle
TaskHandle_t task0_handle = NULL;
TaskHandle_t recv_task_handle = NULL;

QueueHandle_t recv_queue = NULL;
uint8_t recvbuf[2048];

static wiz_NetInfo g_net_info = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56},  // MAC address
    .ip = {192, 168, 100, 3},                     // IP address
    .sn = {255, 255, 255, 0},                     // Subnet Mask
    .gw = {192, 168, 100, 1},                     // Gateway
    .dns = {8, 8, 8, 8},                          // DNS server
    .dhcp = NETINFO_STATIC                        // DHCP enable/disable
};

static void set_clock_khz(void);
static void recv_intr_callback(void);

int64_t alarm_callback(alarm_id_t id, void *user_data) {
  // Put your timeout handler code in here
  return 0;
}

void task0(void *pvParameters) {
  wizchip_spi_initialize();
  wizchip_cris_initialize();

  printf("hoge\n");
  wizchip_reset();
  wizchip_initialize();
  wizchip_check();

  printf("hoge\n");
  // wizchip_gpio_interrupt_initialize(0, gpio_callback);

  network_initialize(g_net_info);
  print_network_information(g_net_info);
  int retval = socket(0, Sn_MR_UDP, 5000, 0);
  if (retval != 0) {
    printf("socket() failed: %d\n", retval);
    return;
  }

  char msg[] = "Hello, world!";
  uint8_t ip[4] = {192, 168, 100, 2};  // Destination IP address

  while (1) {
    printf("Task 0\n");
    sendto(0, (uint8_t *)msg, strlen(msg), ip, 50001);
    // sleep_ms(1000);
    vTaskDelay(1000);
  }
}

void recv_task(void *pvParameters) {
  uint16_t reg_val;
  int32_t recv_len = 0;
  uint8_t queuebuf = 0;

  sleep_ms(5000);

  wizchip_gpio_interrupt_initialize(0, recv_intr_callback);

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
    recv_len = recvfrom(0, recvbuf, sizeof(recvbuf), addr, &port);
    if (recv_len > 0) {
      printf("recv: %d\n", recv_len);
      recvbuf[recv_len] = '\0';
      printf("recv from %d.%d.%d.%d:%d -> %s\n", addr[0], addr[1], addr[2],
             addr[3], port, recvbuf);
      // for (int i = 0; i < recv_len; i++) {
      //   printf("%02x ", recvbuf[i]);
      // }
      gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
      printf("\n");
    } else {
      printf("recv() failed: %d\n", recv_len);
    }
    vTaskDelay(100);
  }
}

void start_core1() { vTaskStartScheduler(); }

int counter = 0;
int main() {
  set_clock_khz();
  stdio_init_all();
  while (!stdio_usb_connected()) {
    sleep_ms(1);
  }

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);

  // recv_queue = xQueueCreate(1, sizeof(uint8_t));
  // if (recv_queue == NULL) {
  //   printf("Failed to create queue\n");
  //   return -1;
  // }
  // prvRuntimeInitializer();
  xTaskCreate(task0, "Task_0", 256, NULL, 1, &task0_handle);
  xTaskCreate(recv_task, "RecvTask", 256, NULL, 1, &recv_task_handle);
  // multicore_launch_core1(start_core1);
  vTaskStartScheduler();

  // Timer example code - This example fires off the callback after 2000ms
  // add_alarm_in_ms(2000, alarm_callback, NULL, false);
  // For more examples of timer use see
  // https://github.com/raspberrypi/pico-examples/tree/master/timer
  while (true) {
    // printf("Hello, world! %d\n", counter++);
    // sleep_ms(1000);
  }
}

static void set_clock_khz(void) {
  // set a system clock frequency in khz
  set_sys_clock_khz(PLL_SYS_KHZ, true);

  // configure the specified clock
  clock_configure(
      clk_peri,
      0,                                                 // No glitchless mux
      CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  // System PLL on AUX
                                                         // mux
      PLL_SYS_KHZ * 1000,                                // Input frequency
      PLL_SYS_KHZ * 1000  // Output (must be same as no divider)
  );
}

/* GPIO */
static void recv_intr_callback(void) {
  signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  // gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
  vTaskNotifyGiveFromISR(recv_task_handle, &xHigherPriorityTaskWoken);
  uint8_t buf = 0;
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
