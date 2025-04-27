#include <stdio.h>

#include "hardware/timer.h"
#include "pico/stdlib.h"

int64_t alarm_callback(alarm_id_t id, void *user_data) {
  // Put your timeout handler code in here
  return 0;
}

int counter = 0;
int main() {
  stdio_init_all();
  while (!stdio_usb_connected()) {
    sleep_ms(1000);
  }

  // Timer example code - This example fires off the callback after 2000ms
  add_alarm_in_ms(2000, alarm_callback, NULL, false);
  // For more examples of timer use see
  // https://github.com/raspberrypi/pico-examples/tree/master/timer
  while (true) {
    printf("Hello, world! %d\n", counter++);
    sleep_ms(1000);
  }
}
