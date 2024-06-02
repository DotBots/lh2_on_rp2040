/**
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the lighthouse v2 chip (TS4231) using the RP2040 microcontroller.
 *
 * Load this program on your board. with a TS4231 connected to pins 15 (Data) and 16 (Envelope).
 *
 * @date 2024
 *
 * @copyright Inria, 2024
 *
 */
#include "hardware/pio.h"
#include "lh2/lh2.h"
#include "pico/stdlib.h"
#include "ts4231_capture.pio.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>

//=========================== defines ==========================================

#define LH2_0_DATA_PIN 3 // The Envelope pin will be (Data pin + 1)
#define LH2_0_ENV_PIN (LH2_0_DATA_PIN+1) // The Envelope pin will be (Data pin + 1)

//=========================== variables ========================================

static db_lh2_t _lh2;

//=========================== prototypes ========================================

//=========================== main =============================================

int main() {

  // LH2 config
  db_lh2_init(&_lh2, LH2_0_DATA_PIN, LH2_0_ENV_PIN);
  // Configure the PIO
  PIO pio = pio0;
  uint offset = pio_add_program(pio, &ts4231_capture_program);
  uint sm = pio_claim_unused_sm(pio, true);
  ts4231_capture_program_init(pio, sm, offset, LH2_0_DATA_PIN);
  // sleep_ms(250);
  uint32_t test = pio->rxf[sm];
  uint32_t test2 = pio->rxf[sm];

  stdio_init_all();
  if (cyw43_arch_init()) {
    printf("Wi-Fi init failed");
    return -1;
  }
  while (true) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(250);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(750);
  }
}

//=========================== functions =============================================

