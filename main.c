/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/pio.h"
#include "lh2/lh2.h"
#include "pico/stdlib.h"
#include "ts4231_capture.pio.h"
#include <stdio.h>

#define LH2_0_DATA_PIN 15 // The Envelope pin will be (Data pin + 1)

int main() {

  // LH2 config
  db_lh2_init();

  // Configure the PIO
  PIO pio = pio0;
  uint offset = pio_add_program(pio, &ts4231_capture_program);
  uint sm = pio_claim_unused_sm(pio, true);
  ts4231_capture_program_init(pio, sm, offset, LH2_0_DATA_PIN);

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
