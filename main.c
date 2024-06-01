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
// #include "lh2/lh2.h"
#include "pico/stdlib.h"
#include "ts4231_capture.pio.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>

//=========================== defines ==========================================

#define LH2_0_DATA_PIN 3 // The Envelope pin will be (Data pin + 1)
#define LH2_0_ENV_PIN (LH2_0_DATA_PIN+1) // The Envelope pin will be (Data pin + 1)

//=========================== variables ========================================

//=========================== prototypes ========================================

void _initialize_ts4231(const uint8_t gpio_d, const uint8_t gpio_e);

//=========================== main =============================================

int main() {

  // LH2 config
  // db_lh2_init();
  _initialize_ts4231(LH2_0_DATA_PIN, LH2_0_ENV_PIN);

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

//=========================== functions! =============================================

void _initialize_ts4231(const uint8_t gpio_d, const uint8_t gpio_e) {

    // Filip's code define these pins as inputs, and then changes them quickly to outputs. Not sure why, but it works.
    gpio_init(gpio_d);
    gpio_init(gpio_e);
    gpio_set_dir(gpio_d, GPIO_IN);
    gpio_set_dir(gpio_e, GPIO_IN);

    // start the TS4231 initialization
    // Wiggle the Envelope and Data pins
    gpio_set_dir(gpio_e, GPIO_OUT);
    sleep_us(10);
    gpio_put(gpio_e, 1);
    sleep_us(10);
    gpio_put(gpio_e, 0);
    sleep_us(10);
    gpio_put(gpio_e, 1);
    sleep_us(10);
    gpio_set_dir(gpio_d, GPIO_OUT);
    sleep_us(10);
    gpio_put(gpio_d, 1);
    sleep_us(10);
    // Turn the pins back to inputs
    gpio_set_dir(gpio_d, GPIO_IN);
    gpio_set_dir(gpio_e, GPIO_IN);
    // finally, wait 1 milisecond
    sleep_us(1000);

    // Send the configuration magic number/sequence
    uint16_t config_val = 0x392B;
    // Turn the Data and Envelope lines back to outputs and clear them.
    gpio_set_dir(gpio_d, GPIO_OUT);
    gpio_set_dir(gpio_e, GPIO_OUT);
    sleep_us(10);
    gpio_put(gpio_d, 0);
    sleep_us(10);
    gpio_put(gpio_e, 0);
    sleep_us(10);
    // Send the magic configuration value, MSB first.
    for (uint8_t i = 0; i < 15; i++) {

        config_val = config_val << 1;
        if ((config_val & 0x8000) > 0) {
            gpio_put(gpio_d, 1);
        } else {
            gpio_put(gpio_d, 0);
        }

        // Toggle the Envelope line as a clock.
        sleep_us(10);
        gpio_put(gpio_e, 1);
        sleep_us(10);
        gpio_put(gpio_e, 0);
        sleep_us(10);
    }
    // Finish send sequence and turn pins into inputs again.
    gpio_put(gpio_d, 0);
    sleep_us(10);
    gpio_put(gpio_e, 1);
    sleep_us(10);
    gpio_put(gpio_d, 1);
    sleep_us(10);
    gpio_set_dir(gpio_d, GPIO_IN);
    gpio_set_dir(gpio_e, GPIO_IN);
    // Finish by waiting 10usec
    sleep_us(10);

    // Now read back the sequence that the TS4231 answers.
    gpio_set_dir(gpio_d, GPIO_OUT);
    gpio_set_dir(gpio_e, GPIO_OUT);
    sleep_us(10);
    gpio_put(gpio_d, 0);
    sleep_us(10);
    gpio_put(gpio_e, 0);
    sleep_us(10);
    gpio_put(gpio_d, 1);
    sleep_us(10);
    gpio_put(gpio_e, 1);
    sleep_us(10);
    // Set Data pin as an input, to receive the data
    gpio_set_dir(gpio_d, GPIO_IN);
    sleep_us(10);
    gpio_put(gpio_e, 0);
    sleep_us(10);
    // Use the Envelope pin to output a clock while the data arrives.
    for (uint8_t i = 0; i < 14; i++) {
        gpio_put(gpio_e, 1);
        sleep_us(10);
        gpio_put(gpio_e, 0);
        sleep_us(10);
    }

    // Finish the configuration procedure
    gpio_set_dir(gpio_d, GPIO_OUT);
    sleep_us(10);
    gpio_put(gpio_e, 1);
    sleep_us(10);
    gpio_put(gpio_d, 1);
    sleep_us(10);

    gpio_put(gpio_e, 0);
    sleep_us(10);
    gpio_put(gpio_d, 0);
    sleep_us(10);
    gpio_put(gpio_e, 1);
    sleep_us(10);

    gpio_set_dir(gpio_d, GPIO_IN);
    gpio_set_dir(gpio_e, GPIO_IN);

    sleep_us(50000);
}