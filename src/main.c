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
#include "pico/time.h"
#include "lh2/lh2.h"
#include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"
#include <stdio.h>
#include <stdlib.h>

#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

//=========================== defines ==========================================

#define LH2_0_DATA_PIN 10                     // 
#define LH2_0_ENV_PIN  (LH2_0_DATA_PIN + 1)  // The Envelope pin will be (Data pin + 1)
#define LH2_1_DATA_PIN 12                     // 
#define LH2_1_ENV_PIN  (LH2_1_DATA_PIN + 1)  // The Envelope pin will be (Data pin + 1)
#define LH2_2_DATA_PIN 18                    // 
#define LH2_2_ENV_PIN  (LH2_2_DATA_PIN + 1)  // The Envelope pin will be (Data pin + 1)
#define LH2_3_DATA_PIN 20                   // 
#define LH2_3_ENV_PIN  (LH2_3_DATA_PIN + 1)  // The Envelope pin will be (Data pin + 1)
#define TIMER_DELAY_US 100000



//=========================== variables ========================================

// is nedeed so the variable is accesible to both cores [1]
db_lh2_t        _lh2_0;
db_lh2_t        _lh2_1;
db_lh2_t        _lh2_2;
db_lh2_t        _lh2_3;
absolute_time_t timer_0;
bool            clk_conf_OK;

uint8_t sensor_0 = 0;
uint8_t sensor_1 = 1;
uint8_t sensor_2 = 2;
uint8_t sensor_3 = 3;

//=========================== prototypes ========================================

void core1_entry();

//=========================== main core #0 =============================================

int main() {
    // configure the clock for 128MHz
    clk_conf_OK = set_sys_clock_khz(128000, true);

    // // power up sensor 1
    // gpio_init(4);
    // gpio_set_dir(4, GPIO_OUT);
    // gpio_put(4, 1);

    // // power up sensor 2
    // gpio_init(14);
    // gpio_set_dir(14, GPIO_OUT);
    // gpio_put(14, 1);

    // // power up sensor 3
    // gpio_init(20);
    // gpio_set_dir(20, GPIO_OUT);
    // gpio_put(20, 1);

    // init the USB UART
    stdio_init_all();
    sleep_ms(3000);
    printf("Start code\n");

    // set-up the on-board LED
    // cyw43_arch_init();
    // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // LH2 config, before starting the second core
    db_lh2_init(&_lh2_0, sensor_0, LH2_0_DATA_PIN, LH2_0_ENV_PIN);
    db_lh2_init(&_lh2_1, sensor_1, LH2_1_DATA_PIN, LH2_1_ENV_PIN);

    // Launch the second core
    multicore_launch_core1(core1_entry);

    // Debug Gpio
    gpio_init(0);
    gpio_set_dir(0, GPIO_OUT);
    gpio_init(1);
    gpio_set_dir(1, GPIO_OUT);
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    gpio_init(3);
    gpio_set_dir(3, GPIO_OUT);
    gpio_put(0, 1);
    gpio_put(1, 1);
    gpio_put(2, 1);
    gpio_put(3, 1);

    timer_0 = get_absolute_time();

    while (true) {

        // the location function has to be running all the time
        db_lh2_process_location(&_lh2_0);
        db_lh2_process_location(&_lh2_1);

        if (absolute_time_diff_us(timer_0, get_absolute_time()) > TIMER_DELAY_US) {

            // // Print the first two base stations of all 4 sensors
            // printf("sen_0 (%d-%d %d-%d %d-%d %d-%d)   \tsen_1 (%d-%d %d-%d %d-%d %d-%d)   \tsen_2 (%d-%d %d-%d %d-%d %d-%d)   \tsen_3 (%d-%d %d-%d %d-%d %d-%d)\n",
            //        _lh2_0.locations[0][0].selected_polynomial, _lh2_0.locations[0][0].lfsr_location, _lh2_0.locations[1][0].selected_polynomial, _lh2_0.locations[1][0].lfsr_location,
            //        _lh2_0.locations[0][1].selected_polynomial, _lh2_0.locations[0][1].lfsr_location, _lh2_0.locations[1][1].selected_polynomial, _lh2_0.locations[1][1].lfsr_location,
            //        _lh2_1.locations[0][0].selected_polynomial, _lh2_1.locations[0][0].lfsr_location, _lh2_1.locations[1][0].selected_polynomial, _lh2_1.locations[1][0].lfsr_location,
            //        _lh2_1.locations[0][1].selected_polynomial, _lh2_1.locations[0][1].lfsr_location, _lh2_1.locations[1][1].selected_polynomial, _lh2_1.locations[1][1].lfsr_location,
            //        _lh2_2.locations[0][0].selected_polynomial, _lh2_2.locations[0][0].lfsr_location, _lh2_2.locations[1][0].selected_polynomial, _lh2_2.locations[1][0].lfsr_location,
            //        _lh2_2.locations[0][1].selected_polynomial, _lh2_2.locations[0][1].lfsr_location, _lh2_2.locations[1][1].selected_polynomial, _lh2_2.locations[1][1].lfsr_location,
            //        _lh2_3.locations[0][0].selected_polynomial, _lh2_3.locations[0][0].lfsr_location, _lh2_3.locations[1][0].selected_polynomial, _lh2_3.locations[1][0].lfsr_location,
            //        _lh2_3.locations[0][1].selected_polynomial, _lh2_3.locations[0][1].lfsr_location, _lh2_3.locations[1][1].selected_polynomial, _lh2_3.locations[1][1].lfsr_location);
            
            // Print the first sweep of all basestations of Sensor 0
            printf("%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d\t%d-%d)\n",
                _lh2_0.locations[0][0].selected_polynomial, _lh2_0.locations[0][0].lfsr_location,
                _lh2_0.locations[0][1].selected_polynomial, _lh2_0.locations[0][1].lfsr_location,
                _lh2_0.locations[0][2].selected_polynomial, _lh2_0.locations[0][2].lfsr_location,
                _lh2_0.locations[0][3].selected_polynomial, _lh2_0.locations[0][3].lfsr_location,
                _lh2_0.locations[0][4].selected_polynomial, _lh2_0.locations[0][4].lfsr_location,
                _lh2_0.locations[0][5].selected_polynomial, _lh2_0.locations[0][5].lfsr_location,
                _lh2_0.locations[0][6].selected_polynomial, _lh2_0.locations[0][6].lfsr_location,
                _lh2_0.locations[0][7].selected_polynomial, _lh2_0.locations[0][7].lfsr_location,
                _lh2_0.locations[0][8].selected_polynomial, _lh2_0.locations[0][8].lfsr_location,
                _lh2_0.locations[0][9].selected_polynomial, _lh2_0.locations[0][9].lfsr_location,
                _lh2_0.locations[0][10].selected_polynomial, _lh2_0.locations[0][10].lfsr_location,
                _lh2_0.locations[0][11].selected_polynomial, _lh2_0.locations[0][11].lfsr_location,
                _lh2_0.locations[0][12].selected_polynomial, _lh2_0.locations[0][12].lfsr_location,
                _lh2_0.locations[0][13].selected_polynomial, _lh2_0.locations[0][13].lfsr_location,
                _lh2_0.locations[0][14].selected_polynomial, _lh2_0.locations[0][14].lfsr_location,
                _lh2_0.locations[0][15].selected_polynomial, _lh2_0.locations[0][15].lfsr_location);

            timer_0 = get_absolute_time();
        }
    }
}

//=========================== main core #0 =============================================

void core1_entry() {

    db_lh2_init(&_lh2_2, sensor_2, LH2_2_DATA_PIN, LH2_2_ENV_PIN);
    db_lh2_init(&_lh2_3, sensor_3, LH2_3_DATA_PIN, LH2_3_ENV_PIN);

    while (true) {
        db_lh2_process_location(&_lh2_2);
        db_lh2_process_location(&_lh2_3);
    }
}

//=========================== references ========================================
