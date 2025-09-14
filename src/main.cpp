#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// #include "eth.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
// #include "notify.h"
// #include "output.h"
#include "config.h"
#include "packet.h"
#include "pico/binary_info.h"
#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "wizchip_spi.h"
#define PLL_SYS_KHZ (133 * 1000)

static wiz_NetInfo g_net_info = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56},  // MAC address
    .ip = {192, 168, 100, 3},                     // IP address
    .sn = {255, 255, 255, 0},                     // Subnet Mask
    .gw = {192, 168, 100, 1},                     // Gateway
    .dns = {8, 8, 8, 8},                          // DNS server
    .dhcp = NETINFO_STATIC};

char limit_sw_pin[] = {6, 7, 8, 9, 10, 11, 12, 13};

static void set_clock_khz(void);

int64_t alarm_callback(alarm_id_t id, void *user_data) {
  // Put your timeout handler code in here
  return 0;
}

int counter = 0;
int main() {
  set_clock_khz();
  stdio_init_all();
  // while (!stdio_usb_connected()) {
  //   sleep_ms(1);
  // }
  //
  printf("Hello, world!\n");

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);

  for (int i = 0; i < sizeof(limit_sw_pin); i++) {
    gpio_init(limit_sw_pin[i]);
    gpio_set_dir(limit_sw_pin[i], GPIO_IN);
  }

  sleep_ms(3000);
  wizchip_spi_initialize();
  wizchip_cris_initialize();

  wizchip_reset();
  wizchip_initialize();
  wizchip_check();

  network_initialize(g_net_info);

  /* Get network information */
  print_network_information(g_net_info);
  int retval = socket(0, Sn_MR_UDP, 5000, 0);
  if (retval != 0) {
    printf("socket() failed: %d\n", retval);
    return 0;
  }
  uint8_t mode = SOCK_IO_NONBLOCK;
  ctlsocket(0, CS_SET_IOMODE, (void *)&mode);

  uint8_t ip[4] = {192, 168, 100, 1};
  uint16_t port = 5001;

  uint8_t cnt = 0;
  while (true) {
    // printf("Hello, world! %d\n", counter++);
    sleep_ms(20);

    // uint8_t buf[2048];
    // uint8_t dest_ip[4];
    // uint16_t dest_port;
    // retval = recvfrom(0, buf, sizeof(buf), dest_ip, &dest_port);
    // if (retval > 0) {
    //   buf[retval] = '\0';
    //   printf("Received %d bytes from %d.%d.%d.%d:%d: %s\n", retval,
    //   dest_ip[0],
    //          dest_ip[1], dest_ip[2], dest_ip[3], dest_port, buf);
    // }
    uint8_t limit_sw_val = 0;
    for (int i = 0; i < sizeof(limit_sw_pin); i++) {
      if (gpio_get(limit_sw_pin[i])) {
        limit_sw_val |= (1 << i);
      }
    }

    UplinkPacket uplink_packet = {
        .abs_enc = counter++,
        .limit_sw = limit_sw_val,
    };

    retval =
        sendto(0, (uint8_t *)&uplink_packet, sizeof(uplink_packet), ip, port);
    if (retval < 0) {
      printf(" Loopback error : %d\n", retval);

      while (1);
    }
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
