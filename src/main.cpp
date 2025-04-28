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
#include "pico/stdlib.h"
#include "socket.h"
#include "timer.h"
#include "w5x00_gpio_irq.h"
#include "w5x00_spi.h"
#include "wizchip_conf.h"

#define PLL_SYS_KHZ (133 * 1000)

/* Semaphore */
static xSemaphoreHandle recv_sem = NULL;

/* Timer  */
static volatile uint32_t g_msec_cnt = 0;

static void set_clock_khz(void);
static void gpio_callback(void);
static void repeating_timer_callback(void);
static time_t millis(void);

int64_t alarm_callback(alarm_id_t id, void *user_data) {
  // Put your timeout handler code in here
  return 0;
}

void task0(void *pvParameters) {
  while (1) {
    printf("Task 0\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void task1(void *pvParameters) {
  while (1) {
    printf("Task 1\n");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

int counter = 0;
int main() {
  set_clock_khz();
  stdio_init_all();
  while (!stdio_usb_connected()) {
    sleep_ms(1);
  }

  wizchip_spi_initialize();
  wizchip_cris_initialize();

  wizchip_reset();
  wizchip_initialize();
  wizchip_check();

  wizchip_1ms_timer_initialize(repeating_timer_callback);
  wizchip_gpio_interrupt_initialize(Sn_MR_UDP, gpio_callback);

  xTaskCreate(task0, "Task_0", 256, NULL, 1, NULL);
  xTaskCreate(task1, "Task_1", 256, NULL, 1, NULL);
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
static void gpio_callback(void) {
  signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR(recv_sem, &xHigherPriorityTaskWoken);
  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/* Timer */
static void repeating_timer_callback(void) { g_msec_cnt++; }

static time_t millis(void) { return g_msec_cnt; }