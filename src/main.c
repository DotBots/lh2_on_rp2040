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
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <stdlib.h>
#include "hardware/dma.h"

//=========================== defines ==========================================

#define LH2_0_DATA_PIN 3                     // The Envelope pin will be (Data pin + 1)
#define LH2_0_ENV_PIN  (LH2_0_DATA_PIN + 1)  // The Envelope pin will be (Data pin + 1)
#define TIMER_DELAY_US 500000

//=========================== variables ========================================

static db_lh2_t _lh2;
absolute_time_t timer;
uint8_t         ring_buffer_count;

//=========================== prototypes ========================================

//=========================== main =============================================

int main() {
    stdio_init_all();
    sleep_ms(3000);
    printf("Start code\n");
    // TODO - configure the clock for 128MHz

    // LH2 config
    db_lh2_init(&_lh2, LH2_0_DATA_PIN, LH2_0_ENV_PIN);

    // test gpio
    gpio_init(10);
    gpio_set_dir(10, GPIO_OUT);
    gpio_put(10, 1);

    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    timer = get_absolute_time();

    while (1) {
        // wait until something happens e.g. an SPI interrupt
        // __WFE();

        // the location function has to be running all the time
        db_lh2_process_location(&_lh2);
        // printf("PIO0->IRQ = %08b,   PIO0->INTR = %012b,   PIO0->IRQ0_INTE = %012b,   SM_PC = %04x   FIFO RX = %d\n", pio0_hw->irq, pio0_hw->intr, pio0_hw->inte0, pio_sm_get_pc(pio0,0) ,pio_sm_get_rx_fifo_level(pio0,0));
        if (absolute_time_diff_us(timer, get_absolute_time()) > TIMER_DELAY_US) {

            printf("Out: PIO0->IRQ = %08b, FIFO RX = %d, RingBuff = %d, DMA_tx_count = %d\n", pio0_hw->irq, pio_sm_get_rx_fifo_level(pio0, 0), *_lh2.spi_ring_buffer_count_ptr, dma_hw->ch[0].transfer_count);
            printf("poly00: %d, index00: %d, poly10: %d, index10: %d, data_ready=%d%d%d%d%d%d%d%d\n", _lh2.locations[0][0].selected_polynomial, _lh2.locations[0][0].lfsr_location, _lh2.locations[1][0].selected_polynomial, _lh2.locations[1][0].lfsr_location,
            _lh2.data_ready[0][0],
            _lh2.data_ready[1][0],
            _lh2.data_ready[0][1],
            _lh2.data_ready[1][1],
            _lh2.data_ready[0][2],
            _lh2.data_ready[1][2],
            _lh2.data_ready[0][3],
            _lh2.data_ready[1][3]);
            timer = get_absolute_time();
        }
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    // __NOP();
}
