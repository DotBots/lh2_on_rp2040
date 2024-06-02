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
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <stdlib.h>

//=========================== defines ==========================================

#define LH2_0_DATA_PIN 3                     // The Envelope pin will be (Data pin + 1)
#define LH2_0_ENV_PIN  (LH2_0_DATA_PIN + 1)  // The Envelope pin will be (Data pin + 1)

//=========================== variables ========================================

static db_lh2_t _lh2;

//=========================== prototypes ========================================

//=========================== main =============================================

int main() {

    // TODO - configure the clock for 128MHz

    // LH2 config
    db_lh2_init(&_lh2, LH2_0_DATA_PIN, LH2_0_ENV_PIN);

    // test gpio
    gpio_init(10);
    gpio_set_dir(10, GPIO_OUT);
    gpio_put(10, 1);

    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    while (1) {
        // wait until something happens e.g. an SPI interrupt
        // __WFE();

        // the location function has to be running all the time
        db_lh2_process_location(&_lh2);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    // __NOP();
}

//=========================== functions =============================================
