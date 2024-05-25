/**
 * @file
 * @ingroup bsp_lh2
 *
 * @brief  nRF52833-specific definition of the "lh2" bsp module.
 *
 * @author Filip Maksimovic <filip.maksimovic@inria.fr>
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <nrf.h>

#include "gpio.h"
#include "lh2.h"
#include "timer_hf.h"

//=========================== defines =========================================

#define SPIM_INTERRUPT_PRIORITY                2                                                              ///< Interrupt priority, as high as it will go
#define SPI_BUFFER_SIZE                        64                                                             ///< Size of buffers used for SPI communications
#define SPI_FAKE_SCK_PIN                       6                                                              ///< NOTE: SPIM needs an SCK pin to be defined, P1.6 is used because it's not an available pin in the BCM module.
#define SPI_FAKE_SCK_PORT                      1                                                              ///< NOTE: SPIM needs an SCK pin to be defined, P1.6 is used because it's not an available pin in the BCM module.
#define FUZZY_CHIP                             0xFF                                                           ///< not sure what this is about
#define LH2_LOCATION_ERROR_INDICATOR           0xFFFFFFFF                                                     ///< indicate the location value is false
#define LH2_POLYNOMIAL_ERROR_INDICATOR         0xFF                                                           ///< indicate the polynomial index is invalid
#define POLYNOMIAL_BIT_ERROR_INITIAL_THRESHOLD 4                                                              ///< initial threshold of polynomial error
#define LH2_BUFFER_SIZE                        10                                                             ///< Amount of lh2 frames the buffer can contain
#define GPIOTE_CH_IN_ENV_HiToLo                1                                                              ///< falling edge gpio channel
#define GPIOTE_CH_IN_ENV_LoToHi                2                                                              ///< rising edge gpio channel
#define PPI_SPI_START_CHAN                     2                                                              ///< PPI channel for starting the GPIOTE to SPI capture ppi
#define PPI_SPI_GROUP                          0                                                              ///< PPI group for automatically dissabling the ppi after a successful spi capture
#define HASH_TABLE_BITS                        11                                                             ///< How many bits will be used for the hashtable for the _end_buffers
#define HASH_TABLE_SIZE                        (1 << HASH_TABLE_BITS)                                         ///< How big will the hashtable for the _end_buffers
#define HASH_TABLE_MASK                        ((1 << HASH_TABLE_BITS) - 1)                                   ///< Mask selecting the HAS_TABLE_BITS least significant bits
#define NUM_LSFR_COUNT_CHECKPOINTS             64                                                             ///< How many lsfr checkpoints are per polynomial
#define DISTANCE_BETWEEN_LSFR_CHECKPOINTS      2048                                                           ///< How many lsfr checkpoints are per polynomial
#define CHECKPOINT_TABLE_BITS                  6                                                              ///< How many bits will be used for the checkpoint table for the lfsr search
#define CHECKPOINT_TABLE_MASK_LOW              ((1 << CHECKPOINT_TABLE_BITS) - 1)                             ///< How big will the checkpoint table for the lfsr search
#define CHECKPOINT_TABLE_MASK_HIGH             (((1 << CHECKPOINT_TABLE_BITS) - 1) << CHECKPOINT_TABLE_BITS)  ///< Mask selecting the CHECKPOINT_TABLE_BITS least significant bits
#define LH2_MAX_DATA_VALID_TIME_US             2000000                                                        //< Data older than this is considered outdate and should be erased (in microseconds)
#define LH2_SWEEP_PERIOD_US                    20000                                                          ///< time, in microseconds, between two full rotations of the LH2 motor
#define LH2_SWEEP_PERIOD_THRESHOLD_US          1000                                                           ///< How close a LH2 pulse must arrive relative to LH2_SWEEP_PERIOD_US, to be considered the same type of sweep (first sweep or second second). (in microseconds)
#define LH2_TIMER_DEV                          2                                                              ///< Timer device used for LH2

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define NRF_SPIM         NRF_SPIM4_S
#define SPIM_IRQ         SPIM4_IRQn
#define SPIM_IRQ_HANDLER SPIM4_IRQHandler
#define NRF_GPIOTE       NRF_GPIOTE0_S
#define NRF_PPI          NRF_DPPIC_S
#else
#define NRF_SPIM         NRF_SPIM3
#define SPIM_IRQ         SPIM3_IRQn
#define SPIM_IRQ_HANDLER SPIM3_IRQHandler
#endif

typedef struct {
    uint8_t  buffer[LH2_BUFFER_SIZE][SPI_BUFFER_SIZE];  ///< arrays of bits for local storage, contents of SPI transfer are copied into this
    uint32_t timestamps[LH2_BUFFER_SIZE];               ///< arrays of timestamps of when different SPI transfers happened
    uint8_t  writeIndex;                                // Index for next write
    uint8_t  readIndex;                                 // Index for next read
    uint8_t  count;                                     // Number of arrays in buffer
} lh2_ring_buffer_t;

typedef struct {
    uint8_t           spi_rx_buffer[SPI_BUFFER_SIZE];  ///< buffer where data coming from SPI are stored
    lh2_ring_buffer_t data;                            ///< array containing demodulation data of each locations
} lh2_vars_t;

//=========================== variables ========================================

static const uint32_t _polynomials[LH2_POLYNOMIAL_COUNT] = {
    0x0001D258,
    0x00017E04,
    0x0001FF6B,
    0x00013F67,
    0x0001B9EE,
    0x000198D1,
    0x000178C7,
    0x00018A55,
};

static const uint32_t _end_buffers[LH2_POLYNOMIAL_COUNT][NUM_LSFR_COUNT_CHECKPOINTS] = {
    {
        // p0
        0x00000000000000001,  // [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1] starting seed, little endian
        0b10000000000011010,
        0b10000001010110001,
        0b10111000000110000,
        0b10101010110011101,
        0b10100001010111011,
        0b01011000111100110,
        0b00011011100000011,
        0b10001010101011010,
        0b01001010001111101,
        0b00010010100011000,
        0b00000100111000011,
        0b11001100100000010,
        0b00111111101111011,
        0b10110010010110001,
        0b01110001000110111,
        0b01100101100011111,
        0b11010001101010100,
        0b00011011101110011,
        0b10001110100000011,
        0b10010001101011110,
        0b11011011010100011,
        0b11011111100110001,
        0b00010100010110011,
        0b10100011001011111,
        0b00111110110000100,
        0b10001100001010011,
        0b11100011011100000,
        0b11110001010110001,
        0b11101000000111110,
        0b00011010010010011,
        0b10101010010100101,
        0b10111000110011011,
        0b10111111000001001,
        0b00101101110010101,
        0b10100100100011100,
        0b10100110100011110,
        0b11001000101110010,
        0b11100111100101101,
        0b00111101010100010,
        0b11001101100010000,
        0b00001001001100101,
        0b10001010111010001,
        0b01000111000001001,
        0b01000101110011111,
        0b00111100011000011,
        0b11111111000001001,
        0b11101101110000010,
        0b11100101011110101,
        0b01000010100110110,
        0b10110111000100001,
        0b10010110011001011,
        0b01001001110110111,
        0b01011011110010010,
        0b01000110110010010,
        0b01100101110010010,
        0b11011100110011101,
        0b01000011010111101,
        0b10010110101000000,
        0b01011111011001111,
        0b10000110101101011,
        0b00101100111011001,
        0b10010101110100110,
        0b00001110011011111,
    },
    {
        // p1
        0x00000000000000001,  // [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1] starting seed, little endian
        0b00000001111100000,
        0b11111111000111010,
        0b10111000111011000,
        0b11010000110111110,
        0b10001100001111010,
        0b10010111100011101,
        0b10011100011010010,
        0b10110111100111100,
        0b11101101100100000,
        0b11110110110010101,
        0b01110110111101110,
        0b11000010101101111,
        0b01101111000011101,
        0b01000000000111001,
        0b01010101101000101,
        0b00101110001101110,
        0b11010000111010000,
        0b00001100001001010,
        0b11011100001000011,
        0b01000011000110100,
        0b00001101001011100,
        0b11010100111100011,
        0b10100011000111100,
        0b00010001010011110,
        0b01011011010101010,
        0b10101110010010010,
        0b10100001010111011,
        0b10100101111010001,
        0b01010111100110110,
        0b11110110001100000,
        0b10000101000110111,
        0b10011000000100001,
        0b00110001110110000,
        0b10100001010001101,
        0b11100111111101000,
        0b01110011011010110,
        0b11010000101111011,
        0b10101001001000100,
        0b00010000111001111,
        0b00100011101000011,
        0b00110110001000011,
        0b11110011101001111,
        0b10100101011110000,
        0b10111011010000101,
        0b11001010000111101,
        0b11101011111010011,
        0b00000001110111100,
        0b00110010100110110,
        0b00101111010110001,
        0b11101110010000001,
        0b10011100010011101,
        0b01000111111100110,
        0b01100111101100011,
        0b10100100000010011,
        0b11011100110100101,
        0b10001101000111011,
        0b01001010000111000,
        0b11100110011110111,
        0b11111000000100100,
        0b00111100110011100,
        0b11011000110000110,
        0b00011011111011000,
        0b10011000101010001,
    },
    {
        // p2
        0x00000000000000001,  // [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1] starting seed, little endian
        0b11001110010011010,
        0b00111011100000111,
        0b10011010101111011,
        0b00011011011000100,
        0b01100001111001001,
        0b00100011000101100,
        0b01110100001110110,
        0b01011101010010110,
        0b00011111001011100,
        0b11111101101001011,
        0b00110100010100010,
        0b11001011001101010,
        0b01100110111001010,
        0b01000010000110001,
        0b00011011111101001,
        0b01110001111011010,
        0b01011010100110111,
        0b10010110101101111,
        0b01001001010111011,
        0b10110110011111010,
        0b10010001010110000,
        0b11001011000000110,
        0b00001011000001000,
        0b10110001110000001,
        0b11111010001101100,
        0b01110110010001010,
        0b00100000010110111,
        0b10001001011101001,
        0b00100101110000111,
        0b10001100100001100,
        0b10000011101111100,
        0b00000010011101011,
        0b01010000011010001,
        0b01011110000100010,
        0b00100100000010110,
        0b01100010101111011,
        0b01101010010110001,
        0b00110111001111111,
        0b11010110101011110,
        0b00111000001101111,
        0b01000010110010001,
        0b01010110110011001,
        0b00110011110100111,
        0b10101011100111000,
        0b11010010001000100,
        0b11001011111100111,
        0b01001000000110001,
        0b01111110101111111,
        0b01111000110111110,
        0b11101100001001001,
        0b00011010000000010,
        0b01000011110101010,
        0b11111111111010011,
        0b10001011001010010,
        0b00100100100111000,
        0b01001011100000011,
        0b11111101110110110,
        0b10111111010001011,
        0b10101111001100000,
        0b00010110111101110,
        0b10100010011011000,
        0b01111001011100111,
        0b10000110110011101,
    },
    {
        // p3
        0x00000000000000001,  // [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1] starting seed, little endian
        0b00101100000100010,
        0b10001100001111000,
        0b10001110100110011,
        0b11011011110010110,
        0b00010010100101011,
        0b01001100111011111,
        0b00110011010010101,
        0b11000100000001101,
        0b01010011010101100,
        0b01111100110100001,
        0b10011011011110001,
        0b11100011000010110,
        0b00010000010000010,
        0b10110110011111000,
        0b00110111011100001,
        0b00011111010001100,
        0b00000101100001100,
        0b01011000011110010,
        0b10111011011111111,
        0b11000001011110011,
        0b11011000000000011,
        0b00011011000101100,
        0b10100101100010011,
        0b10011101110001010,
        0b11110010001000101,
        0b10001011101110001,
        0b01010010000000110,
        0b00001011001111000,
        0b10000000110111001,
        0b00010001010100001,
        0b10100111110001010,
        0b00111100010000101,
        0b11001101011101111,
        0b11000010010010010,
        0b01100010001001001,
        0b01001111001010100,
        0b10000000010000100,
        0b11010011011101100,
        0b01000111000101000,
        0b01011010010110011,
        0b10100000101001111,
        0b10101110100001100,
        0b00010101100111011,
        0b11111101010001100,
        0b01100000101101000,
        0b10001011110011100,
        0b10001100100101101,
        0b00110101011011111,
        0b11010110000110010,
        0b11101011101110000,
        0b00010111001011011,
        0b01110110010101011,
        0b00111001000001010,
        0b10111101110100011,
        0b11001111000000011,
        0b00010000110100010,
        0b01111110100101011,
        0b00011001010001101,
        0b11111001111101101,
        0b00010111110101110,
        0b10011101110100010,
        0b10011100001001111,
        0b00101011101001101,
    },
    {
        // p4
        0x00000000000000001,  // [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1] starting seed, little endian
        0b00111101010111100,
        0b01010011100100100,
        0b01100001100011000,
        0b11000011111110101,
        0b00100100011111011,
        0b00010000110000011,
        0b10001000111100101,
        0b01111110000100101,
        0b11010110111101011,
        0b01110001000011100,
        0b10101100100110100,
        0b01011100011111100,
        0b11001111011011111,
        0b00100101010010110,
        0b00010011000100100,
        0b11001000111100000,
        0b10110111000100001,
        0b00110000001111000,
        0b10101100001001111,
        0b00010001010010110,
        0b01001001110100010,
        0b01010001110000110,
        0b00101110010101101,
        0b00100010100111011,
        0b01000010110100110,
        0b01110110111010010,
        0b10101110001110110,
        0b10111101100101000,
        0b01000010111011011,
        0b11010100011001110,
        0b01010011110010111,
        0b01000101010111010,
        0b11110110100000001,
        0b01101011000100010,
        0b01101101111000101,
        0b10111101100011011,
        0b10011110001001000,
        0b00110101110101111,
        0b10011111101011101,
        0b11110100111101000,
        0b10110000011111110,
        0b00001100010101110,
        0b11100011111011001,
        0b11100010000111100,
        0b11001001011110111,
        0b10000011010010101,
        0b01010001101001001,
        0b11010010101101010,
        0b01001101001100111,
        0b00110110011011100,
        0b00111000010100101,
        0b01101100110010010,
        0b01000010000010001,
        0b01001000111110101,
        0b10101110010001100,
        0b11111000100010000,
        0b10111001000011000,
        0b00100101111101011,
        0b00111011000100010,
        0b00111011101100011,
        0b10001010110100100,
        0b10011010101100001,
        0b10111100101010110,
    },
    {
        // p5
        0x00000000000000001,  // [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1] starting seed, little endian
        0b10000110010011101,
        0b10010100110011101,
        0b11101111001011001,
        0b11001000010011110,
        0b01111111110111110,
        0b00100011010001111,
        0b01011101110101000,
        0b10011010101100010,
        0b00111101111110011,
        0b00110010000110101,
        0b10100111011001111,
        0b01100111010001011,
        0b00100010110011110,
        0b00011010000101110,
        0b11001001101001100,
        0b11011011100110100,
        0b00000010111000100,
        0b11111011010010001,
        0b00100100000011000,
        0b01001000001001100,
        0b10111011101111001,
        0b00111001100101000,
        0b00111011000011100,
        0b11000001001111010,
        0b10010010011100111,
        0b11000001101100001,
        0b11010010110000101,
        0b11001000010010000,
        0b00010111101011110,
        0b01000110001101000,
        0b01110001010000100,
        0b10001110110110000,
        0b00111111101100011,
        0b01000000111011101,
        0b11000001100011101,
        0b11111001001100101,
        0b10000110101001011,
        0b01011111100010011,
        0b11110001000101010,
        0b01100001010010111,
        0b01101011100011111,
        0b01101110000100100,
        0b00101101001000111,
        0b01000110101100010,
        0b10010000010110000,
        0b01111011100011110,
        0b00110011110101011,
        0b11010011000101000,
        0b01011110110001100,
        0b00101011110111001,
        0b00010100001111011,
        0b01011001111000111,
        0b00010100011100111,
        0b11011110010111101,
        0b11111000111011101,
        0b10011010110011010,
        0b01101010000110010,
        0b10101111110000010,
        0b00110111001010101,
        0b00001001000001110,
        0b00000010110111000,
        0b11010000101110001,
        0b01101010111000011,
    },
    {
        // p6
        0x00000000000000001,  // [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1] starting seed, little endian
        0b00010010000111001,
        0b01001101111111100,
        0b00010100111011111,
        0b01110110101110110,
        0b01010101000001110,
        0b00111001110010110,
        0b00011000111001010,
        0b00111111000100011,
        0b00111010001001001,
        0b10010001101110110,
        0b10011110001110100,
        0b10011010110110011,
        0b11110010100110101,
        0b11100001000101111,
        0b11010000111111101,
        0b11110001001110000,
        0b10010001111000110,
        0b10100011001000001,
        0b10001000011101000,
        0b10001101001000111,
        0b11100111010110100,
        0b00001010100010001,
        0b01110010010010001,
        0b01001000110110000,
        0b01011010000010000,
        0b01100110100001100,
        0b10111001000100001,
        0b01000011100101101,
        0b10111010001110010,
        0b11011001111000001,
        0b10110000010001111,
        0b00100110001001001,
        0b11000011000110001,
        0b11110011100010110,
        0b01011101101010000,
        0b11001011101000100,
        0b11100001011010110,
        0b11111111100000101,
        0b10000100111001101,
        0b11001011011100111,
        0b11101110011001010,
        0b11001100100001001,
        0b10001001000101101,
        0b11001101100101000,
        0b01011001111000100,
        0b01111010110011010,
        0b00110011110100011,
        0b00100011001101100,
        0b11101010000001011,
        0b11110100100010110,
        0b11011011000111000,
        0b01011101001100000,
        0b11000100010110001,
        0b10000111011101111,
        0b10010001111111101,
        0b11111001011001111,
        0b11101011111101111,
        0b00100101001100010,
        0b10100000011000110,
        0b01001101000100110,
        0b11000110110101010,
        0b01101010101001110,
        0b00010101100100011,
    },
    {
        // p7
        0x00000000000000001,  // [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1] starting seed, little endian
        0b11111111111001011,
        0b10101101101100110,
        0b00100101101010000,
        0b01000010111001111,
        0b10010001000110001,
        0b10010100011110001,
        0b10010110111000011,
        0b00011111000011111,
        0b00011000111000111,
        0b01101000010001001,
        0b00001010011011000,
        0b00100101010100111,
        0b01110010111010110,
        0b00101101000111111,
        0b11111010001011001,
        0b10001110011000110,
        0b10010110011101110,
        0b11001101111111010,
        0b01001101110010110,
        0b01000011011111110,
        0b11101100100011100,
        0b10000101011010011,
        0b11111111100101101,
        0b01001001000001101,
        0b11001101110001101,
        0b01000000110011101,
        0b11000001101011011,
        0b01110011110100100,
        0b01110011011101110,
        0b01010001111111000,
        0b01011101100100000,
        0b10010000110101010,
        0b11111001001110010,
        0b10000001100101101,
        0b10000001000010111,
        0b01010001101010011,
        0b10110010100011000,
        0b11001101110101011,
        0b01000101000110100,
        0b00011011000110101,
        0b10101100110101110,
        0b10010110011010111,
        0b00110100110001110,
        0b11001100101110100,
        0b11110100100010010,
        0b11011011001101110,
        0b10100101000101111,
        0b10011000101101111,
        0b10000111001010110,
        0b01001100000100100,
        0b00101010011110001,
        0b10100001000111100,
        0b11010101101110010,
        0b11111000011101010,
        0b11000101010011110,
        0b11110100001010000,
        0b00000111000000100,
        0b01000100101100001,
        0b01101001011000111,
        0b01010010010011110,
        0b10001111110000100,
        0b11100110010111101,
        0b10000101011010000,
    },
};

static uint16_t _end_buffers_hashtable[HASH_TABLE_SIZE] = { 0 };

// Dynamic checkpoint
static uint32_t _lfsr_checkpoint_bits[LH2_POLYNOMIAL_COUNT][LH2_SWEEP_COUNT]  = { 0 };  ///<
static uint32_t _lfsr_checkpoint_count[LH2_POLYNOMIAL_COUNT][LH2_SWEEP_COUNT] = { 0 };
static uint32_t _lsfr_checkpoint_average                                      = 0;

///! NOTE: SPIM needs an SCK pin to be defined, P1.6 is used because it's not an available pin in the BCM module
static const gpio_t _lh2_spi_fake_sck_gpio = {
    .port = 1,
    .pin  = 3,
};

///< Encodes in two bits which sweep slot of a particular basestation is empty, (1 means data, 0 means empty)
typedef enum {
    LH2_SWEEP_BOTH_SLOTS_EMPTY,   ///< Both sweep slots are empty
    LH2_SWEEP_SECOND_SLOT_EMPTY,  ///< Only the second sweep slot is empty
    LH2_SWEEP_FIRST_SLOT_EMPTY,   ///< Only the first sweep slot is empty
    LH2_SWEEP_BOTH_SLOTS_FULL,    ///< Both sweep slots are filled with raw data
} db_lh2_sweep_slot_state_t;

static lh2_vars_t _lh2_vars;  ///< local data of the LH2 driver

//=========================== prototypes =======================================

// these functions are called in the order written to perform the LH2 localization
/**
 * @brief wiggle the data and envelope lines in a magical way to configure the TS4231 to continuously read for LH2 sweep signals.
 *
 * @param[in]   gpio_d  pointer to gpio data
 * @param[in]   gpio_e  pointer to gpio event
 */
void _initialize_ts4231(const gpio_t *gpio_d, const gpio_t *gpio_e);

/**
 * @brief
 * @param[in] sample_buffer: SPI samples loaded into a local buffer
 * @return chipsH: 64-bits of demodulated data
 */
uint64_t _demodulate_light(uint8_t *sample_buffer);

/**
 * @brief from a 17-bit sequence and a polynomial, generate up to 64-17=47 bits as if the LFSR specified by poly were run forwards for numbits cycles, all little endian
 *
 * @param[in] poly:     17-bit polynomial
 * @param[in] bits:     starting seed
 * @param[in] numbits:  number of bits
 *
 * @return sequence of bits resulting from running the LFSR forward
 */
uint64_t _poly_check(uint32_t poly, uint32_t bits, uint8_t numbits);

/**
 * @brief find out which LFSR polynomial the bit sequence is a member of
 *
 * @param[in] chipsH1: input sequences of bits from demodulation
 * @param[in] start_val: number of bits between the envelope falling edge and the beginning of the sequence where valid data has been found
 *
 * @return polynomial, indicating which polynomial was found, or FF for error (polynomial not found).
 */
uint8_t _determine_polynomial(uint64_t chipsH1, int8_t *start_val);

/**
 * @brief counts the number of 1s in a 64-bit
 *
 * @param[in] bits_in: arbitrary bits
 *
 * @return cumulative number of 1s inside of bits_in
 */
uint64_t _hamming_weight(uint64_t bits_in);

/**
 * @brief finds the position of a 17-bit sequence (bits) in the sequence generated by polynomial3 with initial seed 1
 *
 * @param[in] index: index of polynomial
 * @param[in] bits: 17-bit sequence
 *
 * @return count: location of the sequence
 */
uint32_t _reverse_count_p(uint8_t index, uint32_t bits);

/**
 * @brief Set a gpio as an INPUT with no pull-up or pull-down
 * @param[in] gpio: pin to configure as input [0-31]
 */
void _lh2_pin_set_input(const gpio_t *gpio);

/**
 * @brief Set a gpio as an OUTPUT with standard drive
 * @param[in] gpio: gpio to configure as input [0-31]
 */
void _lh2_pin_set_output(const gpio_t *gpio);

/**
 * @brief set-up GPIOTE so that events are configured for falling (GPIOTE_CH_IN) and rising edges (GPIOTE_CH_IN_ENV_HiToLo) of the envelope signal
 *
 * @param[in]   gpio_e  pointer to gpio event
 */
void _gpiote_setup(const gpio_t *gpio_e);

/**
 * @brief start SPI3 at falling edge of envelope, start timer2 at falling edge of envelope, stop/capture timer2 at rising edge of envelope
 */
void _ppi_setup(void);

/**
 * @brief spi3 setup
 *
 * @param[in]   gpio_d  pointer to gpio data
 */
void _spi_setup(const gpio_t *gpio_d);

/**
 * @brief add one element to the ring buffer for spi captures
 *
 * @param[in]   cb          pointer to ring buffer structure
 * @param[in]   data        pointer to the data array to save in the ring buffer
 * @param[in]   timestamp   timestamp of when the LH2 measurement was taken. (taken with timer_hf_now())
 */
void _add_to_spi_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, uint32_t timestamp);

/**
 * @brief retreive the oldest element from the ring buffer for spi captures
 *
 * @param[in]    cb          pointer to ring buffer structure
 * @param[out]   data        pointer to the array where the ring buffer data will be saved
 * @param[out]   timestamp   timestamp of when the LH2 measurement was taken. (taken with timer_hf_now())
 */
bool _get_from_spi_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, uint32_t *timestamp);

/**
 * @brief generates a hashtable from the LSFR checpoints and stores it in an array.
 *
 * @param[in]   hash_table  pointer to the array where the hashtable will be stored
 */
void _fill_hash_table(uint16_t *hash_table);

/**
 * @brief Accesses the global tables _lfsr_checkpoint_hashtable & _lfsr_checkpoint_count
 *        and updates them with the last found polynomial count
 *
 * @param[in] polynomial: index of polynomial
 * @param[in] bits: 17-bit sequence
 * @param[in] count: position of the received laser sweep in the LSFR sequence
 */
void _update_lfsr_checkpoints(uint8_t polynomial, uint32_t bits, uint32_t count);

/**
 * @brief LH2 sweeps come with an almost perfect 20ms difference.
 *        this function uses the timestamps to figure to which sweep-slot the new LH2 data belongs to.
 *
 * @param[in] lh2 pointer to the lh2 instance
 * @param[in] polynomial: index of found polynomia
 * @param[in] timestamp: timestamp of the SPI capture
 */
uint8_t _select_sweep(db_lh2_t *lh2, uint8_t polynomial, uint32_t timestamp);

//=========================== public ===========================================

void db_lh2_init(db_lh2_t *lh2, const gpio_t *gpio_d, const gpio_t *gpio_e) {
    // Initialize the TS4231 on power-up - this is only necessary when power-cycling
    _initialize_ts4231(gpio_d, gpio_e);

    // Configure the necessary Pins in the GPIO peripheral  (MOSI and CS not needed)
    _lh2_pin_set_input(gpio_d);                    // Data_pin will become the MISO pin
    _lh2_pin_set_output(&_lh2_spi_fake_sck_gpio);  // set SCK as Output.

    _spi_setup(gpio_d);

    // Setup the LH2 local variables
    memset(_lh2_vars.spi_rx_buffer, 0, SPI_BUFFER_SIZE);
    // initialize the spi ring buffer
    memset(&_lh2_vars.data, 0, sizeof(lh2_ring_buffer_t));

    // Setup LH2 data
    lh2->spi_ring_buffer_count_ptr = &_lh2_vars.data.count;  // pointer to the size of the spi ring buffer,

    for (uint8_t sweep = 0; sweep < LH2_SWEEP_COUNT; sweep++) {
        for (uint8_t basestation = 0; basestation < LH2_SWEEP_COUNT; basestation++) {
            lh2->raw_data[sweep][basestation].bits_sweep           = 0;
            lh2->raw_data[sweep][basestation].selected_polynomial  = LH2_POLYNOMIAL_ERROR_INDICATOR;
            lh2->raw_data[sweep][basestation].bit_offset           = 0;
            lh2->locations[sweep][basestation].selected_polynomial = LH2_POLYNOMIAL_ERROR_INDICATOR;
            lh2->locations[sweep][basestation].lfsr_location       = LH2_LOCATION_ERROR_INDICATOR;
            lh2->timestamps[sweep][basestation]                    = 0;
            lh2->data_ready[sweep][basestation]                    = DB_LH2_NO_NEW_DATA;
        }
    }
    memset(_lh2_vars.data.buffer[0], 0, LH2_BUFFER_SIZE);

    // Initialize the hash table for the lsfr checkpoints
    _fill_hash_table(_end_buffers_hashtable);

    // initialize GPIOTEs
    _gpiote_setup(gpio_e);

    // initialize PPI
    _ppi_setup();
}

void db_lh2_start(void) {

    NRF_PPI->TASKS_CHG[PPI_SPI_GROUP].EN = 1;
}

void db_lh2_stop(void) {

    NRF_PPI->TASKS_CHG[PPI_SPI_GROUP].DIS = 1;
}

void db_lh2_reset(db_lh2_t *lh2) {
    // compute LFSR locations and detect invalid packets
    for (uint8_t basestation = 0; basestation < LH2_BASESTATION_COUNT; basestation++) {
        for (uint8_t sweep = 0; sweep < 2; sweep++) {

            // Remove the flags indicating available data
            lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
            lh2->timestamps[sweep][basestation] = 0;
            // We won't actually clear the data, it's not worth the computational effort.
        }
    }
}

void db_lh2_process_raw_data(db_lh2_t *lh2) {
    if (_lh2_vars.data.count == 0) {
        return;
    }

    // Get value before it's overwritten by the ringbuffer.
    uint8_t temp_spi_bits[SPI_BUFFER_SIZE * 2] = { 0 };  // The temp buffer has to be 128 long because _demodulate_light() expects it to be so
                                                         // Making it smaller causes a hardfault
                                                         // I don't know why, the SPI buffer is clearly 64bytes long.
                                                         // should ask fil about this

    // stop the interruptions while you're reading the data.
    uint32_t temp_timestamp = 0;  // default timestamp
    if (!_get_from_spi_ring_buffer(&_lh2_vars.data, temp_spi_bits, &temp_timestamp)) {
        return;
    }
    // perform the demodulation + poly search on the received packets
    // convert the SPI reading to bits via zero-crossing counter demodulation and differential/biphasic manchester decoding
    uint64_t temp_bits_sweep = _demodulate_light(temp_spi_bits);

    // figure out which polynomial each one of the two samples come from.
    int8_t  temp_bit_offset          = 0;  // default offset
    uint8_t temp_selected_polynomial = _determine_polynomial(temp_bits_sweep, &temp_bit_offset);

    // If there was an error with the polynomial, leave without updating anything
    if (temp_selected_polynomial == LH2_POLYNOMIAL_ERROR_INDICATOR) {
        return;
    }

    // Figure in which of the two sweep slots we should save the new data.
    uint8_t sweep = _select_sweep(lh2, temp_selected_polynomial, temp_timestamp);

    // Put the newly read polynomials in the data structure (polynomial 0,1 must map to LH0, 2,3 to LH1. This can be accomplish by  integer-dividing the selected poly in 2, a shift >> accomplishes this.)
    // This structur always holds the two most recent sweeps from any lighthouse
    lh2->raw_data[sweep][temp_selected_polynomial >> 1].bit_offset          = temp_bit_offset;
    lh2->raw_data[sweep][temp_selected_polynomial >> 1].selected_polynomial = temp_selected_polynomial;
    lh2->raw_data[sweep][temp_selected_polynomial >> 1].bits_sweep          = temp_bits_sweep;
    lh2->timestamps[sweep][temp_selected_polynomial >> 1]                   = temp_timestamp;
    lh2->data_ready[sweep][temp_selected_polynomial >> 1]                   = DB_LH2_RAW_DATA_AVAILABLE;
}

void db_lh2_process_location(db_lh2_t *lh2) {
    if (_lh2_vars.data.count == 0) {
        return;
    }

    //*********************************************************************************//
    //                              Prepare Raw Data                                   //
    //*********************************************************************************//

    // Get value before it's overwritten by the ringbuffer.
    uint8_t temp_spi_bits[SPI_BUFFER_SIZE * 2] = { 0 };  // The temp buffer has to be 128 long because _demodulate_light() expects it to be so
                                                         // Making it smaller causes a hardfault
                                                         // I don't know why, the SPI buffer is clearly 64bytes long.
                                                         // should ask fil about this

    // stop the interruptions while you're reading the data.
    uint32_t temp_timestamp = 0;  // default timestamp
    if (!_get_from_spi_ring_buffer(&_lh2_vars.data, temp_spi_bits, &temp_timestamp)) {
        return;
    }
    // perform the demodulation + poly search on the received packets
    // convert the SPI reading to bits via zero-crossing counter demodulation and differential/biphasic manchester decoding
    uint64_t temp_bits_sweep = _demodulate_light(temp_spi_bits);

    // figure out which polynomial each one of the two samples come from.
    int8_t  temp_bit_offset          = 0;  // default offset
    uint8_t temp_selected_polynomial = _determine_polynomial(temp_bits_sweep, &temp_bit_offset);

    // If there was an error with the polynomial, leave without updating anything
    if (temp_selected_polynomial == LH2_POLYNOMIAL_ERROR_INDICATOR) {
        return;
    }

    // Figure in which of the two sweep slots we should save the new data.
    uint8_t sweep = _select_sweep(lh2, temp_selected_polynomial, temp_timestamp);

    // Put the newly read polynomials in the data structure (polynomial 0,1 must map to LH0, 2,3 to LH1. This can be accomplish by  integer-dividing the selected poly in 2, a shift >> accomplishes this.)
    // This structur always holds the two most recent sweeps from any lighthouse
    uint8_t basestation = temp_selected_polynomial >> 1;

    //*********************************************************************************//
    //                           Compute Polynomial Count                              //
    //*********************************************************************************//

    // Sanity check, make sure you don't start the LFSR search with a bit-sequence full of zeros.
    if ((temp_bits_sweep >> (47 - temp_bit_offset)) == 0x000000) {
        // Mark the data as wrong and keep going
        lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
        return;
    }

    // Compute and save the lsfr location.
    uint32_t lfsr_loc_temp = _reverse_count_p(
                                 temp_selected_polynomial,
                                 temp_bits_sweep >> (47 - temp_bit_offset)) -
                             temp_bit_offset;

    //*********************************************************************************//
    //                                 Store results                                   //
    //*********************************************************************************//

    // Save raw data information
    lh2->raw_data[sweep][basestation].bit_offset          = temp_bit_offset;
    lh2->raw_data[sweep][basestation].selected_polynomial = temp_selected_polynomial;
    lh2->raw_data[sweep][basestation].bits_sweep          = temp_bits_sweep;
    lh2->timestamps[sweep][basestation]                   = temp_timestamp;
    // Save processed location information
    lh2->locations[sweep][basestation].lfsr_location       = lfsr_loc_temp;
    lh2->locations[sweep][basestation].selected_polynomial = temp_selected_polynomial;
    // Mark the data point as processed
    lh2->data_ready[sweep][basestation] = DB_LH2_PROCESSED_DATA_AVAILABLE;
}

//=========================== private ==========================================

void _initialize_ts4231(const gpio_t *gpio_d, const gpio_t *gpio_e) {

    // Configure the wait timer
    db_timer_hf_init(LH2_TIMER_DEV);

    // Filip's code define these pins as inputs, and then changes them quickly to outputs. Not sure why, but it works.
    _lh2_pin_set_input(gpio_d);
    _lh2_pin_set_input(gpio_e);

    // start the TS4231 initialization
    // Wiggle the Envelope and Data pins
    _lh2_pin_set_output(gpio_e);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;  // set pin HIGH
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;  // set pin LOW
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    _lh2_pin_set_output(gpio_d);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTSET = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    // Turn the pins back to inputs
    _lh2_pin_set_input(gpio_d);
    _lh2_pin_set_input(gpio_e);
    // finally, wait 1 milisecond
    db_timer_hf_delay_us(LH2_TIMER_DEV, 1000);

    // Send the configuration magic number/sequence
    uint16_t config_val = 0x392B;
    // Turn the Data and Envelope lines back to outputs and clear them.
    _lh2_pin_set_output(gpio_e);
    _lh2_pin_set_output(gpio_d);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTCLR = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    // Send the magic configuration value, MSB first.
    for (uint8_t i = 0; i < 15; i++) {

        config_val = config_val << 1;
        if ((config_val & 0x8000) > 0) {
            nrf_port[gpio_d->port]->OUTSET = 1 << gpio_d->pin;
        } else {
            nrf_port[gpio_d->port]->OUTCLR = 1 << gpio_d->pin;
        }

        // Toggle the Envelope line as a clock.
        db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
        nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
        db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
        nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
        db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    }
    // Finish send sequence and turn pins into inputs again.
    nrf_port[gpio_d->port]->OUTCLR = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTSET = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    _lh2_pin_set_input(gpio_d);
    _lh2_pin_set_input(gpio_e);
    // Finish by waiting 10usec
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);

    // Now read back the sequence that the TS4231 answers.
    _lh2_pin_set_output(gpio_e);
    _lh2_pin_set_output(gpio_d);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTCLR = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTSET = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    // Set Data pin as an input, to receive the data
    _lh2_pin_set_input(gpio_d);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    // Use the Envelope pin to output a clock while the data arrives.
    for (uint8_t i = 0; i < 14; i++) {
        nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
        db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
        nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
        db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    }

    // Finish the configuration procedure
    _lh2_pin_set_output(gpio_d);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTSET = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);

    nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTCLR = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);

    _lh2_pin_set_input(gpio_d);
    _lh2_pin_set_input(gpio_e);

    db_timer_hf_delay_us(LH2_TIMER_DEV, 50000);
}

uint64_t _demodulate_light(uint8_t *sample_buffer) {  // bad input variable name!!
    // TODO: rename sample_buffer
    // TODO: make it a void and have chips be a modified pointer thingie
    // FIXME: there is an edge case where I throw away an initial "1" and do not count it in the bit-shift offset, resulting in an incorrect error of 1 in the LFSR location
    uint8_t chip_index;
    uint8_t local_buffer[128];
    uint8_t zccs_1[128];
    uint8_t chips1[128];  // TODO: give this a better name.
    uint8_t temp_byte_N;  // TODO: bad variable name "temp byte"
    uint8_t temp_byte_M;  // TODO: bad variable name "temp byte"

    // initialize loop variables
    uint8_t  ii = 0x00;
    int      jj = 0;
    int      kk = 0;
    uint64_t gg = 0;

    // initialize temporary "ones counter" variable that counts consecutive ones
    int ones_counter = 0;

    // initialize result:
    uint64_t chipsH1 = 0;

    // FIND ZERO CROSSINGS
    chip_index         = 0;
    zccs_1[chip_index] = 0x01;

    memcpy(local_buffer, sample_buffer, 128);

    // for loop over bytes of the SPI buffer (jj), nested with a for loop over bits in each byte (ii)
    for (jj = 0; jj < 128; jj++) {
        // edge case - check if last bit (LSB) of previous byte is the same as first bit (MSB) of current byte
        // if it is not, increment chip_index and reset count
        if (jj != 0) {
            temp_byte_M = (local_buffer[jj - 1]) & (0x01);   // previous byte's LSB
            temp_byte_N = (local_buffer[jj] >> 7) & (0x01);  // current byte's MSB
            if (temp_byte_M != temp_byte_N) {
                chip_index++;
                zccs_1[chip_index] = 1;
            } else {
                zccs_1[chip_index] += 1;
            }
        }
        // look at one byte at a time
        for (ii = 7; ii > 0; ii--) {
            temp_byte_M = ((local_buffer[jj]) >> (ii)) & (0x01);      // bit shift by ii and mask
            temp_byte_N = ((local_buffer[jj]) >> (ii - 1)) & (0x01);  // bit shift by ii-1 and mask
            if (temp_byte_M == temp_byte_N) {
                zccs_1[chip_index] += 1;
            } else {
                chip_index++;
                zccs_1[chip_index] = 1;
            }
        }
    }

    // threshold the zero crossings into: likely one chip, likely two zero chips, or fuzzy
    for (jj = 0; jj < 128; jj++) {
        // not memory efficient, but ok for readability, turn ZCCS into chips by thresholding
        if (zccs_1[jj] >= 5) {
            chips1[jj] = 0;  // it's a very likely zero
        } else if (zccs_1[jj] <= 3) {
            chips1[jj] = 1;  // it's a very likely one
        } else {
            chips1[jj] = FUZZY_CHIP;  // fuzzy
        }
    }
    // final bit is bugged, make it fuzzy:
    // chips1[127] = 0xFF;

    // DEMODULATION:
    // basic principles, in descending order of importance:
    //  1) an odd number of ones in a row is not allowed - this must be avoided at all costs
    //  2) finding a solution to #1 given a set of data is quite cumbersome without certain assumptions
    //    a) a fuzzy before an odd run of 1s is almost always a 1
    //    b) a fuzzy between two even runs of 1s is almost always a 0
    //    c) a fuzzy after an even run of 1s is usually a a 0
    //  3) a detected 1 is rarely wrong, but detected 0s can be, this is especially common in low-SNR readings
    //    exception: if the first bit is a 1 it is NOT reliable because the capture is asynchronous
    //  4) this is not perfect, but the earlier the chip, the more likely that it is correct. Polynomials can be used to fix bit errors later in the reading
    // known bugs/issues:
    //  1) if there are many ones at the very beginning of the reading, the algorithm will mess it up
    //  2) in some instances, the count value will be off by approximately 5, the origin of this bug is unknown at the moment
    // DEMODULATE PACKET:

    // reset variables:
    kk           = 0;
    ones_counter = 0;
    jj           = 0;
    for (jj = 0; jj < 128;) {      // TODO: 128 is such an easy magic number to get rid of...
        gg = 0;                    // TODO: this is not used here?
        if (chips1[jj] == 0x00) {  // zero, keep going, reset state
            jj++;
            ones_counter = 0;
        }
        if (chips1[jj] == 0x01) {  // one, keep going, keep track of the # of ones
                                   // k_msleep(10);
            if (jj == 0) {         // edge case - first chip = 1 is unreliable, do not increment 1s counter
                jj++;
            } else {
                jj           = jj + 1;
                ones_counter = ones_counter + 1;
            }
        }

        if ((jj == 127) & (chips1[jj] == FUZZY_CHIP)) {
            chips1[jj] = 0x00;
        } else if ((chips1[jj] == FUZZY_CHIP) & (ones_counter == 0)) {  // fuzz after a zero
                                                                        // k_msleep(10);
            if (chips1[jj + 1] == 0) {                                  // zero then fuzz then zero -> fuzz is a zero
                jj++;
                chips1[jj - 1] = 0;
            } else if (chips1[jj + 1] == FUZZY_CHIP) {  // zero then fuzz then fuzz -> just move on, you're probably screwed
                // k_msleep(10);
                jj += 2;
            } else if (chips1[jj + 1] == 1) {  // zero then fuzz then one -> investigate
                kk           = 1;
                ones_counter = 0;
                while (chips1[jj + kk] == 1) {
                    ones_counter++;
                    kk++;
                }
                if (ones_counter % 2 == 1) {  // fuzz -> odd ones, the fuzz is a 1
                    jj++;
                    chips1[jj - 1] = 1;
                    ones_counter   = 1;
                } else if (ones_counter % 2 == 0) {  // fuzz -> even ones, move on for now, it's indeterminate
                    jj++;
                    ones_counter = 0;  // temporarily treat as a 0 for counting purposes
                } else {               // catch statement
                    jj++;
                }
            }
        } else if ((chips1[jj] == FUZZY_CHIP) & (ones_counter != 0)) {  // ones then fuzz
                                                                        // k_msleep(10);
            if ((ones_counter % 2 == 0) & (chips1[jj + 1] == 0)) {      // even ones then fuzz then zero, fuzz is a zero
                jj++;
                chips1[jj - 1] = 0;
                ones_counter   = 0;
            }
            if ((ones_counter % 2 == 0) & (chips1[jj + 1] != 0)) {  // even ones then fuzz then not zero - investigate
                if (chips1[jj + 1] == 1) {                          // subsequent bit is a 1
                    kk = 1;
                    while (chips1[jj + kk] == 1) {
                        ones_counter++;
                        kk++;
                    }
                    if (ones_counter % 2 == 1) {  // indicates an odd # of 1s, so the fuzzy has to be a 1
                        jj++;
                        chips1[jj - 1] = 1;
                        ones_counter   = 1;              // not actually 1, but it's ok for modulo purposes
                    } else if (ones_counter % 2 == 0) {  // even ones -> fuzz -> even ones, indeterminate
                        jj++;
                        ones_counter = 0;
                    }
                } else if (chips1[jj + 1] == FUZZY_CHIP) {  // subsequent bit is a fuzzy - skip for now...
                    jj++;
                }
            } else if ((ones_counter % 2 == 1) & (chips1[jj + 1] == FUZZY_CHIP)) {  // odd ones then fuzz then fuzz, fuzz is 1 then 0
                jj += 2;
                chips1[jj - 1] = 0;
                chips1[jj - 2] = 1;
                ones_counter   = 0;
            } else if ((ones_counter % 2 == 1) & (chips1[jj + 1] != 0)) {  // odd ones then fuzz then not zero - the fuzzy has to be a 1
                jj++;
                ones_counter++;
                chips1[jj - 1] = 1;
            } else {  // catch statement
                jj++;
            }
        }
    }
    // finish up demodulation, pick off straggling fuzzies and odd runs of 1s
    for (jj = 0; jj < 128;) {
        if (chips1[jj] == 0x00) {                   // zero, keep going, reset state
            if (ones_counter % 2 == 1) {            // implies an odd # of 1s
                chips1[jj - ones_counter - 1] = 1;  // change the bit before the run of 1s to a 1 to make it even
            }
            jj++;
            ones_counter = 0;
        } else if (chips1[jj] == 0x01) {  // one, keep going, keep track of the # of ones
            if (jj == 0) {                // edge case - first chip = 1 is unreliable, do not increment 1s counter
                jj++;
            } else {
                jj           = jj + 1;
                ones_counter = ones_counter + 1;
            }
        } else if (chips1[jj] == FUZZY_CHIP) {
            // if (ones_counter==0) { // fuzz after zeros, if the next chip is a 1, make it a 1, else make it a zero
            //     if (chips1[jj+1]==1) {
            //         jj+1;
            //         chips1[jj-1] = 1;
            //         ones_counter++;
            //     }
            //     else {
            //         jj++;
            //     }
            // }  <---- this is commented out because this is a VERY rare edge case and seems to be causing occasional problems w/ otherwise clean packets
            if ((ones_counter != 0) & (ones_counter % 2 == 0)) {  // fuzz after even ones - at this point this is almost always a 0
                jj++;
                chips1[jj - 1] = 0;
                ones_counter   = 0;
            } else if (ones_counter % 2 == 1) {  // fuzz after odd ones - exceedingly uncommon at this point, make it a 1
                jj++;
                chips1[jj - 1] = 1;
                ones_counter++;
            } else {  // catch statement
                jj++;
            }
        } else {  // catch statement
            jj++;
        }
    }

    // next step in demodulation: take the resulting array of 1 and 0 chips and put them into a single 64-bit unsigned int
    // this is primarily for easy manipulation for polynomial searching
    chip_index = 0;  // TODO: rename "chip index" it's not descriptive
    chipsH1    = 0;
    gg         = 0;    // looping/while break indicating variable, reset to 0
    while (gg < 64) {  // very last one - make all remaining fuzzies 0 and load it into two 64-bit longs
        if (chip_index > 127) {
            gg = 65;  // break
        }
        if ((chip_index == 0) & (chips1[chip_index] == 0x01)) {  // first bit is a 1 - ignore it
            chip_index = chip_index + 1;
        } else if ((chip_index == 0) & (chips1[chip_index] == FUZZY_CHIP)) {  // first bit is fuzzy - ignore it
            chip_index = chip_index + 1;
        } else if (gg == 63) {  // load the final bit
            if (chips1[chip_index] == 0) {
                chipsH1 &= 0xFFFFFFFFFFFFFFFE;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == FUZZY_CHIP) {
                chipsH1 &= 0xFFFFFFFFFFFFFFFE;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == 0x01) {
                chipsH1 |= 0x0000000000000001;
                gg         = gg + 1;
                chip_index = chip_index + 2;
            }
        } else {  // load the bit in!!
            if (chips1[chip_index] == 0) {
                chipsH1 &= 0xFFFFFFFFFFFFFFFE;
                chipsH1    = chipsH1 << 1;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == FUZZY_CHIP) {
                chipsH1 &= 0xFFFFFFFFFFFFFFFE;
                chipsH1    = chipsH1 << 1;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == 0x01) {
                chipsH1 |= 0x0000000000000001;
                chipsH1    = chipsH1 << 1;
                gg         = gg + 1;
                chip_index = chip_index + 2;
            }
        }
    }
    return chipsH1;
}

uint64_t _poly_check(uint32_t poly, uint32_t bits, uint8_t numbits) {
    uint64_t bits_out      = 0;
    uint8_t  shift_counter = 1;
    uint8_t  b1            = 0;
    uint32_t buffer        = bits;   // mask to prevent bit overflow
    poly &= 0x00001FFFF;             // mask to prevent silliness
    bits_out |= buffer;              // initialize 17 LSBs of result
    bits_out &= 0x00000000FFFFFFFF;  // mask because I didn't want to re-cast the buffer

    while (shift_counter <= numbits) {
        bits_out = bits_out << 1;  // shift left (forward in time) by 1

        b1     = __builtin_popcount(buffer & poly) & 0x01;  // mask the buffer w/ the selected polynomial
        buffer = ((buffer << 1) | b1) & (0x0001FFFF);

        bits_out |= ((b1) & (0x01));  // put result of the XOR operation into the new bit
        shift_counter++;
    }
    return bits_out;
}

uint8_t _determine_polynomial(uint64_t chipsH1, int8_t *start_val) {
    // check which polynomial the bit sequence is part of
    // TODO: make function a void and modify memory directly
    // TODO: rename chipsH1 to something relevant... like bits?

    *start_val = 8;  // TODO: remove this? possible that I modify start value during the demodulation process

    int32_t  bits_N_for_comp                      = 47 - *start_val;
    uint32_t bit_buffer1                          = (uint32_t)(((0xFFFF800000000000) & chipsH1) >> 47);
    uint64_t bits_from_poly[LH2_POLYNOMIAL_COUNT] = { 0 };
    uint64_t weights[LH2_POLYNOMIAL_COUNT]        = { 0xFFFFFFFFFFFFFFFF };
    uint8_t  selected_poly                        = LH2_POLYNOMIAL_ERROR_INDICATOR;  // initialize to error condition
    uint8_t  min_weight_idx                       = LH2_POLYNOMIAL_ERROR_INDICATOR;
    uint64_t min_weight                           = LH2_POLYNOMIAL_ERROR_INDICATOR;
    uint64_t bits_to_compare                      = 0;
    int32_t  threshold                            = POLYNOMIAL_BIT_ERROR_INITIAL_THRESHOLD;

    // try polynomial vs. first buffer bits
    // this search takes 17-bit sequences and runs them forwards through the polynomial LFSRs.
    // if the remaining detected bits fit well with the chosen 17-bit sequence and a given polynomial, it is treated as "correct"
    // in case of bit errors at the beginning of the capture, the 17-bit sequence is shifted (to a max of 8 bits)
    // in case of bit errors at the end of the capture, the ending bits are removed (to a max of
    // removing bits reduces the threshold correspondingly, as incorrect packet detection will cause a significant delay in location estimate

    // run polynomial search on the first capture
    while (1) {

        // TODO: do this math stuff in multiple operations to: (a) make it readable (b) ensure order-of-execution
        bit_buffer1     = (uint32_t)(((0xFFFF800000000000 >> (*start_val)) & chipsH1) >> (64 - 17 - (*start_val)));
        bits_to_compare = (chipsH1 & (0xFFFFFFFFFFFFFFFF << (64 - 17 - (*start_val) - bits_N_for_comp)));
        // reset the minimum polynomial match found
        min_weight_idx = LH2_POLYNOMIAL_ERROR_INDICATOR;
        min_weight     = LH2_POLYNOMIAL_ERROR_INDICATOR;
        // Check against all the known polynomials
        for (uint8_t i = 0; i < LH2_POLYNOMIAL_COUNT; i++) {
            bits_from_poly[i] = (((_poly_check(_polynomials[i], bit_buffer1, bits_N_for_comp)) << (64 - 17 - (*start_val) - bits_N_for_comp)) | (chipsH1 & (0xFFFFFFFFFFFFFFFF << (64 - (*start_val)))));
            // weights[i]        = _hamming_weight(bits_from_poly[i] ^ bits_to_compare);
            weights[i] = __builtin_popcount(bits_from_poly[i] ^ bits_to_compare);
            // Keep track of the minimum weight value and which polinimial generated it.
            if (weights[i] < min_weight) {
                min_weight_idx = i;
                min_weight     = weights[i];
            }
        }

        // If you found a sufficiently good value, then return which polinomial generated it
        if (min_weight <= (uint64_t)threshold) {
            selected_poly = min_weight_idx;
            break;
            // match failed, try again removing bits from the end
        } else if (*start_val > 8) {
            *start_val      = 8;
            bits_N_for_comp = bits_N_for_comp - 9;
            if (threshold > 2) {
                threshold = threshold - 1;
            } else if (threshold == 2) {  // keep threshold at ones, but you're probably screwed with an unlucky bit error
                threshold = 2;
            }
        } else {
            *start_val = *start_val + 1;
            bits_N_for_comp -= 1;
        }

        // too few bits to reliably compare, give up
        if (bits_N_for_comp < 19) {
            selected_poly = LH2_POLYNOMIAL_ERROR_INDICATOR;  // mark the poly as "wrong"
            break;
        }
    }
    return selected_poly;
}

uint64_t _hamming_weight(uint64_t bits_in) {  // TODO: bad name for function? or is it, it might be a good name for a function, because it describes exactly what it does
    uint64_t weight = bits_in;
    weight          = weight - ((weight >> 1) & 0x5555555555555555);                         // find # of 1s in every 2-bit block
    weight          = (weight & 0x3333333333333333) + ((weight >> 2) & 0x3333333333333333);  // find # of 1s in every 4-bit block
    weight          = (weight + (weight >> 4)) & 0x0F0F0F0F0F0F0F0F;                         // find # of 1s in every 8-bit block
    weight          = (weight + (weight >> 8)) & 0x00FF00FF00FF00FF;                         // find # of 1s in every 16-bit block
    weight          = (weight + (weight >> 16)) & 0x0000FFFF0000FFFF;                        // find # of 1s in every 32-bit block
    weight          = (weight + (weight >> 32));                                             // add the two 32-bit block results together
    weight          = weight & 0x000000000000007F;                                           // mask final result, max value of 64, 0'b01000000
    return weight;
}

uint32_t _reverse_count_p(uint8_t index, uint32_t bits) {

    bits                 = bits & 0x0001FFFF;  // initialize buffer to initial bits, masked
    uint32_t buffer_down = bits;
    uint32_t buffer_up   = bits;

    uint32_t count_down      = 0;
    uint32_t count_up        = 0;
    uint32_t b17             = 0;
    uint32_t b1              = 0;
    uint32_t masked_buff     = 0;
    uint8_t  hash_index_down = 0;
    uint8_t  hash_index_up   = 0;

    // Copy const variables (Flash) into local variables (RAM) to speed up execution.
    uint32_t _end_buffers_local[NUM_LSFR_COUNT_CHECKPOINTS] = { 0 };
    uint32_t polynomials_local                              = _polynomials[index];
    for (size_t i = 0; i < NUM_LSFR_COUNT_CHECKPOINTS; i++) {
        _end_buffers_local[i] = _end_buffers[index][i];
    }

    while (buffer_up != _end_buffers_local[0])  // do until buffer reaches one of the saved states
    {

        //
        // CHECKPOINT CHECKING
        //

        // Check end_buffer backward count
        // Lower hash option in the hash table
        hash_index_down = _end_buffers_hashtable[(buffer_down >> 2) & HASH_TABLE_MASK] & CHECKPOINT_TABLE_MASK_LOW;
        if (buffer_down == _end_buffers_local[hash_index_down]) {
            count_down = count_down + 2048 * hash_index_down - 1;
            _update_lfsr_checkpoints(index, bits, count_down);
            return count_down;
        }
        // Upper hash option in the hash table
        hash_index_down = (_end_buffers_hashtable[(buffer_down >> 2) & HASH_TABLE_MASK] & CHECKPOINT_TABLE_MASK_HIGH) >> CHECKPOINT_TABLE_BITS;
        if (buffer_down == _end_buffers_local[hash_index_down]) {
            count_down = count_down + 2048 * hash_index_down - 1;
            _update_lfsr_checkpoints(index, bits, count_down);
            return count_down;
        }

        // Check end_buffer forward count
        // Lower hash option in the hash table
        hash_index_up = _end_buffers_hashtable[(buffer_up >> 2) & HASH_TABLE_MASK] & CHECKPOINT_TABLE_MASK_LOW;
        if (buffer_up == _end_buffers_local[hash_index_up]) {
            count_up = 2048 * hash_index_up - count_up - 1;
            _update_lfsr_checkpoints(index, bits, count_up);
            return count_up;
        }
        // Upper hash option in the hash table
        hash_index_up = (_end_buffers_hashtable[(buffer_up >> 2) & HASH_TABLE_MASK] & CHECKPOINT_TABLE_MASK_HIGH) >> CHECKPOINT_TABLE_BITS;
        if (buffer_up == _end_buffers_local[hash_index_up]) {
            count_up = 2048 * hash_index_up - count_up - 1;
            _update_lfsr_checkpoints(index, bits, count_up);
            return count_up;
        }

        // Check the dynamical checkpoints, backward
        if (buffer_down == _lfsr_checkpoint_bits[index][0]) {
            count_down = count_down + _lfsr_checkpoint_count[index][0];
            _update_lfsr_checkpoints(index, bits, count_down);
            return count_down;
        }
        if (buffer_down == _lfsr_checkpoint_bits[index][1]) {
            count_down = count_down + _lfsr_checkpoint_count[index][1];
            _update_lfsr_checkpoints(index, bits, count_down);
            return count_down;
        }

        // Check the dynamical checkpoints, forward
        if (buffer_up == _lfsr_checkpoint_bits[index][0]) {
            count_up = _lfsr_checkpoint_count[index][0] - count_up;
            _update_lfsr_checkpoints(index, bits, count_up);
            return count_up;
        }
        if (buffer_up == _lfsr_checkpoint_bits[index][1]) {
            count_up = _lfsr_checkpoint_count[index][1] - count_up;
            _update_lfsr_checkpoints(index, bits, count_up);
            return count_up;
        }

        //
        // LSFR UPDATE
        //
        // LSFR backward update
        b17         = buffer_down & 0x00000001;                                                      // save the "newest" bit of the buffer
        buffer_down = (buffer_down & (0x0001FFFE)) >> 1;                                             // shift the buffer right, backwards in time
        masked_buff = (buffer_down) & (polynomials_local);                                           // mask the buffer w/ the selected polynomial
        buffer_down = buffer_down | (((__builtin_popcount(masked_buff) ^ b17) & 0x00000001) << 16);  // This weird line propagates the LSFR one bit into the past
        count_down++;

        // LSFR forward update
        b1        = __builtin_popcount(buffer_up & polynomials_local) & 0x01;  // mask the buffer w/ the selected polynomial
        buffer_up = ((buffer_up << 1) | b1) & (0x0001FFFF);
        count_up++;
    }
    return count_up;
}

void _lh2_pin_set_input(const gpio_t *gpio) {
    // Configure Data pin as INPUT, with no pullup or pull down.
    nrf_port[gpio->port]->PIN_CNF[gpio->pin] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                               (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                               (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos);
}

void _lh2_pin_set_output(const gpio_t *gpio) {
    // Configure Data pin as OUTPUT, with standar power drive current.
    nrf_port[gpio->port]->PIN_CNF[gpio->pin] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |   // Set Pin as output
                                               (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos);  // Activate high current gpio mode.
}

void _gpiote_setup(const gpio_t *gpio_e) {
    NRF_GPIOTE->CONFIG[GPIOTE_CH_IN_ENV_HiToLo] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                                  (gpio_e->pin << GPIOTE_CONFIG_PSEL_Pos) |
                                                  (gpio_e->port << GPIOTE_CONFIG_PORT_Pos) |
                                                  (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->CONFIG[GPIOTE_CH_IN_ENV_LoToHi] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                                  (gpio_e->pin << GPIOTE_CONFIG_PSEL_Pos) |
                                                  (gpio_e->port << GPIOTE_CONFIG_PORT_Pos) |
                                                  (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);
}

void _ppi_setup(void) {
#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)

    // Add the selected DPPI Channel to a Group to be able to disable it with a Task.
    NRF_PPI->CHG[PPI_SPI_GROUP] = (1 << PPI_SPI_START_CHAN);

    // Publish an Event when the Envelope line goes from HIGH to LOW.
    NRF_GPIOTE->PUBLISH_IN[GPIOTE_CH_IN_ENV_HiToLo] = (GPIOTE_PUBLISH_IN_EN_Enabled << GPIOTE_PUBLISH_IN_EN_Pos) |
                                                      (PPI_SPI_START_CHAN << GPIOTE_PUBLISH_IN_CHIDX_Pos);

    // Subscription to trigger the SPI transfer and disable the PPI system
    NRF_SPIM->SUBSCRIBE_START = (SPIM_SUBSCRIBE_START_EN_Enabled << SPIM_SUBSCRIBE_START_EN_Pos) |
                                (PPI_SPI_START_CHAN << SPIM_SUBSCRIBE_START_CHIDX_Pos);

    NRF_PPI->SUBSCRIBE_CHG[PPI_SPI_GROUP].DIS = (DPPIC_SUBSCRIBE_CHG_DIS_EN_Enabled << DPPIC_SUBSCRIBE_CHG_DIS_EN_Pos) |
                                                (PPI_SPI_START_CHAN << DPPIC_SUBSCRIBE_CHG_DIS_CHIDX_Pos);

#else

    // Add all the ppi setup to group 0 to be able to enable and disable it automatically.
    NRF_PPI->CHG[PPI_SPI_GROUP] = (1 << PPI_SPI_START_CHAN);

    uint32_t envelope_input_HiToLo        = (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_IN_ENV_HiToLo];
    uint32_t spi_start_task_addr          = (uint32_t)&NRF_SPIM->TASKS_START;
    uint32_t ppi_group0_disable_task_addr = (uint32_t)&NRF_PPI->TASKS_CHG[0].DIS;

    NRF_PPI->CH[PPI_SPI_START_CHAN].EEP   = envelope_input_HiToLo;         // envelope down
    NRF_PPI->CH[PPI_SPI_START_CHAN].TEP   = spi_start_task_addr;           // start spi3 transfer
    NRF_PPI->FORK[PPI_SPI_START_CHAN].TEP = ppi_group0_disable_task_addr;  // Disable the PPI group

#endif
}

void _spi_setup(const gpio_t *gpio_d) {
    // Define the necessary Pins in the SPIM peripheral
    NRF_SPIM->PSEL.MISO = gpio_d->pin << SPIM_PSEL_MISO_PIN_Pos |                          // Define pin number for MISO pin
                          gpio_d->port << SPIM_PSEL_MISO_PORT_Pos |                        // Define pin port for MISO pin
                          SPIM_PSEL_MISO_CONNECT_Connected << SPIM_PSEL_MISO_CONNECT_Pos;  // Enable the MISO pin

    NRF_SPIM->PSEL.SCK = _lh2_spi_fake_sck_gpio.pin << SPIM_PSEL_SCK_PIN_Pos |          // Define pin number for SCK pin
                         _lh2_spi_fake_sck_gpio.port << SPIM_PSEL_SCK_PORT_Pos |        // Define pin port for SCK pin
                         SPIM_PSEL_SCK_CONNECT_Connected << SPIM_PSEL_SCK_CONNECT_Pos;  // Enable the SCK pin

    NRF_SPIM->PSEL.MOSI = (4UL) << SPIM_PSEL_MOSI_PIN_Pos |
                          1 << SPIM_PSEL_MOSI_PORT_Pos |
                          SPIM_PSEL_MOSI_CONNECT_Connected << SPIM_PSEL_MOSI_CONNECT_Pos;

    // Configure the Interruptions
    NVIC_ClearPendingIRQ(SPIM_IRQ);
    NVIC_DisableIRQ(SPIM_IRQ);  // Disable interruptions while configuring

    // Configure the SPIM peripheral
    NRF_SPIM->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M32;                         // Set SPI frequency to 32MHz
    NRF_SPIM->CONFIG    = SPIM_CONFIG_ORDER_MsbFirst << SPIM_CONFIG_ORDER_Pos;  // Set MsB out first

    // Configure the EasyDMA channel, only using RX
    NRF_SPIM->RXD.MAXCNT = SPI_BUFFER_SIZE;                    // Set the size of the input buffer.
    NRF_SPIM->RXD.PTR    = (uint32_t)_lh2_vars.spi_rx_buffer;  // Set the input buffer pointer.

    NRF_SPIM->INTENSET = SPIM_INTENSET_END_Enabled << SPIM_INTENSET_END_Pos;  // Enable interruption for when a packet arrives
    NVIC_SetPriority(SPIM_IRQ, SPIM_INTERRUPT_PRIORITY);                      // Set priority for Radio interrupts to 1
    // Enable SPIM interruptions
    NVIC_EnableIRQ(SPIM_IRQ);

    // Enable the SPIM peripheral
    NRF_SPIM->ENABLE = SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos;
}

void _add_to_spi_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, uint32_t timestamp) {

    memcpy(cb->buffer[cb->writeIndex], data, SPI_BUFFER_SIZE);
    cb->timestamps[cb->writeIndex] = timestamp;
    cb->writeIndex                 = (cb->writeIndex + 1) % LH2_BUFFER_SIZE;

    if (cb->count < LH2_BUFFER_SIZE) {
        cb->count++;
    } else {
        // Overwrite oldest data, adjust readIndex
        cb->readIndex = (cb->readIndex + 1) % LH2_BUFFER_SIZE;
    }
}

bool _get_from_spi_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, uint32_t *timestamp) {
    if (cb->count == 0) {
        // Buffer is empty
        return false;
    }

    memcpy(data, cb->buffer[cb->readIndex], SPI_BUFFER_SIZE);
    *timestamp    = cb->timestamps[cb->readIndex];
    cb->readIndex = (cb->readIndex + 1) % LH2_BUFFER_SIZE;
    cb->count--;

    return true;
}

void _fill_hash_table(uint16_t *hash_table) {

    // Iterate over all the checkpoints and save the HASH_TABLE_BITS 11 bits as a a index for the hashtable
    for (size_t poly = 0; poly < LH2_POLYNOMIAL_COUNT; poly++) {
        for (size_t checkpoint = 1; checkpoint < NUM_LSFR_COUNT_CHECKPOINTS; checkpoint++) {
            if (hash_table[(_end_buffers[poly][checkpoint] >> 2) & HASH_TABLE_MASK] == 0) {  // We shift by 2 to the right because we precomputed that that hash has the least amount of collisions in the hash table

                hash_table[(_end_buffers[poly][checkpoint] >> 2) & HASH_TABLE_MASK] = checkpoint & CHECKPOINT_TABLE_MASK_LOW;  // that element of the hash table is empty, copy the checkpoint into the lower 6 bits
            } else {
                hash_table[(_end_buffers[poly][checkpoint] >> 2) & HASH_TABLE_MASK] |= (checkpoint << CHECKPOINT_TABLE_BITS) & CHECKPOINT_TABLE_MASK_HIGH;  // If the element is already occupied, use the upper 6 bits
            }
        }
    }
}

void _update_lfsr_checkpoints(uint8_t polynomial, uint32_t bits, uint32_t count) {

    // Update the current running weighted sum. 75% of old value +25% of new value
    _lsfr_checkpoint_average = (((_lsfr_checkpoint_average * 3) >> 2) + (count >> 2));

    // Is the new count higher or lower than the current running average.
    uint8_t index = count <= _lsfr_checkpoint_average ? 0 : 1;

    // Save the new count in the correct place in the checkpoint array
    _lfsr_checkpoint_bits[polynomial][index]  = bits;
    _lfsr_checkpoint_count[polynomial][index] = count;
}

uint8_t _select_sweep(db_lh2_t *lh2, uint8_t polynomial, uint32_t timestamp) {
    // TODO: check the exact, per-mode period of each polynomial instead of using a blanket 20ms

    uint8_t  basestation = polynomial >> 1;  ///< each base station uses 2 polynomials. integer dividing by 2 maps the polynomial number to the basestation number.
    uint32_t now         = db_timer_hf_now(LH2_TIMER_DEV);
    // check that current data stored is not too old.
    for (size_t sweep = 0; sweep < 2; sweep++) {
        if (now - lh2->timestamps[0][basestation] > LH2_MAX_DATA_VALID_TIME_US) {
            // Remove data that is too old.
            lh2->raw_data[sweep][basestation].bits_sweep          = 0;
            lh2->raw_data[sweep][basestation].selected_polynomial = LH2_POLYNOMIAL_ERROR_INDICATOR;
            lh2->raw_data[sweep][basestation].bit_offset          = 0;
            lh2->timestamps[sweep][basestation]                   = 0;
            lh2->data_ready[sweep][basestation]                   = DB_LH2_NO_NEW_DATA;
            // I don't think it's worth it to remove the location data. It is already marked as "No new data"
        }
    }

    ///< Encode in two bits which sweep slot of this basestation is empty, (1 means data, 0 means empty)
    uint8_t sweep_slot_state = ((lh2->timestamps[1][basestation] != 0) << 1) | (lh2->timestamps[0][basestation] != 0);
    // by default, select the first slot
    uint8_t selected_sweep = 0;

    switch (sweep_slot_state) {

        case LH2_SWEEP_BOTH_SLOTS_EMPTY:
        {
            // use the first slot
            selected_sweep = 0;
            break;
        }

        case LH2_SWEEP_FIRST_SLOT_EMPTY:
        {
            // check that the filled slot is not a perfect 20ms match to the new data.
            uint32_t diff = (timestamp - lh2->timestamps[1][basestation]) % LH2_SWEEP_PERIOD_US;
            diff          = diff < LH2_SWEEP_PERIOD_US - diff ? diff : LH2_SWEEP_PERIOD_US - diff;

            if (diff < LH2_SWEEP_PERIOD_THRESHOLD_US) {
                // match: use filled slot
                selected_sweep = 1;
            } else {
                // no match: use empty slot
                selected_sweep = 0;
            }
            break;
        }

        case LH2_SWEEP_SECOND_SLOT_EMPTY:
        {
            // check that the filled slot is not a perfect 20ms match to the new data.
            uint32_t diff = (timestamp - lh2->timestamps[0][basestation]) % LH2_SWEEP_PERIOD_US;
            diff          = diff < LH2_SWEEP_PERIOD_US - diff ? diff : LH2_SWEEP_PERIOD_US - diff;

            if (diff < LH2_SWEEP_PERIOD_THRESHOLD_US) {
                // match: use filled slot
                selected_sweep = 0;
            } else {
                // no match: use empty slot
                selected_sweep = 1;
            }
            break;
        }

        case LH2_SWEEP_BOTH_SLOTS_FULL:
        {
            // How far away is this new pulse from the already stored data
            uint32_t diff_0 = (timestamp - lh2->timestamps[0][basestation]) % LH2_SWEEP_PERIOD_US;
            diff_0          = diff_0 < LH2_SWEEP_PERIOD_US - diff_0 ? diff_0 : LH2_SWEEP_PERIOD_US - diff_0;
            uint32_t diff_1 = (timestamp - lh2->timestamps[1][basestation]) % LH2_SWEEP_PERIOD_US;
            diff_1          = diff_1 < LH2_SWEEP_PERIOD_US - diff_1 ? diff_1 : LH2_SWEEP_PERIOD_US - diff_1;

            // Use the one that is closest to 20ms
            if (diff_0 <= diff_1) {
                selected_sweep = 0;
            } else {
                selected_sweep = 1;
            }
            break;
        }

        default:
        {
            // By default, use he first slot
            selected_sweep = 0;
            break;
        }
    }

    return selected_sweep;
}

//=========================== interrupts =======================================

void SPIM_IRQ_HANDLER(void) {
    // Check if the interrupt was caused by a fully send package
    if (NRF_SPIM->EVENTS_END) {
        // Clear the Interrupt flag
        NRF_SPIM->EVENTS_END = 0;
        // Reenable the PPI channel
        db_lh2_start();
        // Read the current time.
        uint32_t timestamp = db_timer_hf_now(LH2_TIMER_DEV);
        // Add new reading to the ring buffer
        _add_to_spi_ring_buffer(&_lh2_vars.data, _lh2_vars.spi_rx_buffer, timestamp);
    }
}
