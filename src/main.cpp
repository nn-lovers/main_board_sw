#include <FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <task.h>
#include <time.h>

#include "eth.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"
#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#define PLL_SYS_KHZ (133 * 1000)

// Task handle
TaskHandle_t task0_handle = NULL;
TaskHandle_t recv_task_handle = NULL;

static void set_clock_khz(void);

int64_t alarm_callback(alarm_id_t id, void *user_data) {
  // Put your timeout handler code in here
  return 0;
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

  eth_setup();

  // prvRuntimeInitializer();
  // xTaskCreate(task0, "Task_0", 256, NULL, 1, &task0_handle);
  xTaskCreate(eth_recv_task, "RecvTask", 256, NULL, 1, &eth_recv_task_handle);
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
