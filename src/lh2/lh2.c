/**
 * @file
 * @ingroup bsp_lh2
 *
 * @brief  RP2040-specific definition of the "lh2" bsp module.
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

#include "ts4231_capture.pio.h"
#include "hardware/gpio.h"
#include "lh2.h"
#include "lh2_decoder.h"
#include "lh2_checkpoints.h"
#include "pico/time.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

//=========================== defines =========================================

#define TS4231_SENSOR_QTY                      4                                                              ///< Max amount of TS4231 sensors the library supports
#define TS4231_CAPTURE_BUFFER_SIZE             64                                                             ///< Size of buffers used for SPI communications
#define LH2_LOCATION_ERROR_INDICATOR           0xFFFFFFFF                                                     ///< indicate the location value is false
#define LH2_POLYNOMIAL_ERROR_INDICATOR         0xFF                                                           ///< indicate the polynomial index is invalid
#define LH2_BUFFER_SIZE                        10                                                             ///< Amount of lh2 frames the buffer can contain
#define LH2_MAX_DATA_VALID_TIME_US             2000000                                                        //< Data older than this is considered outdate and should be erased (in microseconds)
#define LH2_SWEEP_PERIOD_US                    20000                                                          ///< time, in microseconds, between two full rotations of the LH2 motor
#define LH2_SWEEP_PERIOD_THRESHOLD_US          1000                                                           ///< How close a LH2 pulse must arrive relative to LH2_SWEEP_PERIOD_US, to be considered the same type of sweep (first sweep or second second). (in microseconds)

// Ring buffer for the ts4231 raw data capture
typedef struct {
    uint8_t         buffer[LH2_BUFFER_SIZE][TS4231_CAPTURE_BUFFER_SIZE];  ///< arrays of bits for local storage, contents of SPI transfer are copied into this
    absolute_time_t timestamps[LH2_BUFFER_SIZE];                          ///< arrays of timestamps of when different SPI transfers happened
    uint8_t         writeIndex;                                           ///< Index for next write
    uint8_t         readIndex;                                            ///< Index for next read
    uint8_t         count;                                                ///< Number of arrays in buffer
} lh2_ring_buffer_t;

typedef struct {
    uint8_t            spi_rx_buffer[TS4231_CAPTURE_BUFFER_SIZE];  ///< buffer where data coming from SPI are stored
    lh2_ring_buffer_t  data;                                       ///< array containing demodulation data of each locations
    _lfsr_checkpoint_t checkpoint;                                 ///< Dynamic checkpoints for the lsfr index search
    int                dma_channel;                                ///< dma channel that sends the data from the PIO capture to the ring_buffer.
    PIO                pio;                                        ///< PIO device used for this sensor
    uint8_t            sm;                                         ///< State machine used for this sensor
} lh2_vars_t;

typedef struct {
    bool init_flag;  ///< is true if the program has already been stored in the pio memory
    uint offset[2];  ///< offsets for the pio programs in pio0 and pio1
} pio_vars_t;

//=========================== variables ========================================

///< Encodes in two bits which sweep slot of a particular basestation is empty, (1 means data, 0 means empty)
typedef enum {
    LH2_SWEEP_BOTH_SLOTS_EMPTY,   ///< Both sweep slots are empty
    LH2_SWEEP_SECOND_SLOT_EMPTY,  ///< Only the second sweep slot is empty
    LH2_SWEEP_FIRST_SLOT_EMPTY,   ///< Only the first sweep slot is empty
    LH2_SWEEP_BOTH_SLOTS_FULL,    ///< Both sweep slots are filled with raw data
} db_lh2_sweep_slot_state_t;

static lh2_vars_t _lh2_vars[TS4231_SENSOR_QTY];  ///< local data of the LH2 driver, one copy per sensor

static pio_vars_t _pio_vars = { 0 };  ///< stores the status of the one-off configurations of the pio programs

//=========================== prototypes =======================================

// these functions are called in the order written to perform the LH2 localization
/**
 * @brief wiggle the data and envelope lines in a magical way to configure the TS4231 to continuously read for LH2 sweep signals.
 *
 * @param[in]   gpio_d  pointer to gpio data
 * @param[in]   gpio_e  pointer to gpio event
 */
void _initialize_ts4231(const uint8_t gpio_d, const uint8_t gpio_e);

/**
 * @brief Configure the DMA to automatically retrieve data from the PIO TS4231 capture. And send it to the ring buffer
 *
 * @param[in] sensor:   which TS4231 sensor is associated with this data structure (valid values [0-3])
 */
void _init_dma_pio_capture(uint8_t sensor);

/**
 * @brief add one element to the ring buffer for spi captures
 *
 * @param[in]   cb          pointer to ring buffer structure
 * @param[in]   data        pointer to the data array to save in the ring buffer
 * @param[in]   timestamp   timestamp of when the LH2 measurement was taken. (taken with timer_hf_now())
 */
void _add_to_ts4231_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, absolute_time_t timestamp);

/**
 * @brief retreive the oldest element from the ring buffer for spi captures
 *
 * @param[in]    cb          pointer to ring buffer structure
 * @param[out]   data        pointer to the array where the ring buffer data will be saved
 * @param[out]   timestamp   timestamp of when the LH2 measurement was taken. (taken with timer_hf_now())
 */
bool _get_from_ts4231_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, absolute_time_t *timestamp);

/**
 * @brief Accesses the global tables _lfsr_checkpoint_hashtable & _lfsr_checkpoint_count
 *        and updates them with the last found polynomial count
 *
 * @param[in] sensor:   which TS4231 sensor is associated with this data structure (valid values [0-3])
 * @param[in] polynomial: index of polynomial
 * @param[in] bits: 17-bit sequence
 * @param[in] count: position of the received laser sweep in the LSFR sequence
 * @param[in] sweep: 0 for the first sweep, and 1 for the second sweep
 */
void _update_lfsr_checkpoints(uint8_t sensor, uint8_t polynomial, uint32_t bits, uint32_t count, uint8_t sweep);

/**
 * @brief LH2 sweeps come with an almost perfect 20ms difference.
 *        this function uses the timestamps to figure to which sweep-slot the new LH2 data belongs to.
 *
 * @param[in] lh2 pointer to the lh2 instance
 * @param[in] polynomial: index of found polynomia
 * @param[in] timestamp: timestamp of the SPI capture
 */
uint8_t _select_sweep(db_lh2_t *lh2, uint8_t polynomial, absolute_time_t timestamp);

/**
 * @brief ISR that copies the data generated by the PIO capture of the TS4231 into a ring buffer
 */
void _pio_irq_handler_generic(uint8_t sensor);
void pio_irq_handler_0(void);
void pio_irq_handler_1(void);
void pio_irq_handler_2(void);
void pio_irq_handler_3(void);

//=========================== public ===========================================

void db_lh2_init(db_lh2_t *lh2, uint8_t sensor, const uint8_t gpio_d, const uint8_t gpio_e) {
    // Initialize the TS4231 on power-up - this is only necessary when power-cycling
    _initialize_ts4231(gpio_d, gpio_e);

    // Setup the LH2 local variables
    memset(_lh2_vars[sensor].spi_rx_buffer, 0, TS4231_CAPTURE_BUFFER_SIZE);
    // initialize the spi ring buffer
    memset(&_lh2_vars[sensor].data, 0, sizeof(lh2_ring_buffer_t));

    // Setup LH2 data
    lh2->spi_ring_buffer_count_ptr = &_lh2_vars[sensor].data.count;  // pointer to the size of the spi ring buffer,
    lh2->sensor                    = sensor;                         // store the sensor number inside the public lh2 structure.

    for (uint8_t sweep = 0; sweep < LH2_SWEEP_COUNT; sweep++) {
        for (uint8_t basestation = 0; basestation < LH2_SWEEP_COUNT; basestation++) {
            lh2->raw_data[sweep][basestation].bits_sweep           = 0;
            lh2->raw_data[sweep][basestation].selected_polynomial  = LH2_POLYNOMIAL_ERROR_INDICATOR;
            lh2->raw_data[sweep][basestation].bit_offset           = 0;
            lh2->locations[sweep][basestation].selected_polynomial = LH2_POLYNOMIAL_ERROR_INDICATOR;
            lh2->locations[sweep][basestation].lfsr_location       = LH2_LOCATION_ERROR_INDICATOR;
            lh2->timestamps[sweep][basestation]                    = nil_time;
            lh2->data_ready[sweep][basestation]                    = DB_LH2_NO_NEW_DATA;
        }
    }
    memset(_lh2_vars[sensor].data.buffer[0], 0, LH2_BUFFER_SIZE);

    // Configure the PIO and the DMA for the TS4231 capture
    // Retrieve pio and sm dinamically, and store them in the global variable.

    // do the per-sensor configuration
    PIO    pio;
    uint   sm;
    int8_t pio_irq;

    switch (sensor) {
        case 0:
            pio     = pio0;
            sm      = 0;
            pio_irq = PIO0_IRQ_0;
            // Enable interrupt
            pio_set_irq0_source_enabled(pio, pis_interrupt0, true);  // Connect the SM interrupt to system IRQ
            irq_set_exclusive_handler(pio_irq, pio_irq_handler_0);
            break;

        case 1:
            pio     = pio0;
            sm      = 1;
            pio_irq = PIO0_IRQ_1;
            // Enable interrupt
            pio_set_irq1_source_enabled(pio, pis_interrupt1, true);  // Connect the SM interrupt to system IRQ
            irq_set_exclusive_handler(pio_irq, pio_irq_handler_1);
            break;

        case 2:
            pio     = pio1;
            sm      = 0;
            pio_irq = PIO1_IRQ_0;
            // Enable interrupt
            pio_set_irq0_source_enabled(pio, pis_interrupt0, true);  // Connect the SM interrupt to system IRQ
            irq_set_exclusive_handler(pio_irq, pio_irq_handler_2);
            break;

        case 3:
            pio     = pio1;
            sm      = 1;
            pio_irq = PIO1_IRQ_1;
            // Enable interrupt
            pio_set_irq1_source_enabled(pio, pis_interrupt1, true);  // Connect the SM interrupt to system IRQ
            irq_set_exclusive_handler(pio_irq, pio_irq_handler_3);
            break;

        default:
            break;
    }

    // only once per reboot, save the pio program to pio memory
    if (!_pio_vars.init_flag) {
        _pio_vars.offset[0] = pio_add_program(pio0, &ts4231_capture_program);
        _pio_vars.offset[1] = pio_add_program(pio1, &ts4231_capture_program);
        _pio_vars.init_flag = true;
    }

    // Save the correct pio and sm values
    _lh2_vars[sensor].pio = pio;
    _lh2_vars[sensor].sm  = sm;

    // retrieve the correct offset
    uint offset = (pio == pio0) ? _pio_vars.offset[0] : _pio_vars.offset[1];
    irq_set_enabled(pio_irq, true);  // Enable the IRQ
    // Enable PIO and DMA
    _init_dma_pio_capture(sensor);
    ts4231_capture_program_init(pio, sm, offset, gpio_d);
}

void db_lh2_start(void) {

    // NRF_PPI->TASKS_CHG[PPI_SPI_GROUP].EN = 1;
}

void db_lh2_stop(void) {

    // NRF_PPI->TASKS_CHG[PPI_SPI_GROUP].DIS = 1;
}

void db_lh2_process_location(db_lh2_t *lh2) {
    uint8_t sensor = lh2->sensor;  // Make a local copy of the sensor number, for readability's sake

    if (_lh2_vars[sensor].data.count == 0) {
        return;
    }

    //*********************************************************************************//
    //                              Prepare Raw Data                                   //
    //*********************************************************************************//

    // Get value before it's overwritten by the ringbuffer.
    uint8_t temp_spi_bits[TS4231_CAPTURE_BUFFER_SIZE * 2] = { 0 };  // The temp buffer has to be 128 long because _demodulate_light() expects it to be so
                                                                    // Making it smaller causes a hardfault
                                                                    // I don't know why, the SPI buffer is clearly 64bytes long.
                                                                    // should ask fil about this

    // stop the interruptions while you're reading the data.
    absolute_time_t temp_timestamp = nil_time;  // default timestamp
    if (!_get_from_ts4231_ring_buffer(&_lh2_vars[sensor].data, temp_spi_bits, &temp_timestamp)) {
        return;
    }
    // perform the demodulation + poly search on the received packets
    // convert the SPI reading to bits via zero-crossing counter demodulation and differential/biphasic manchester decoding.
    gpio_put(1, 1);
    uint64_t temp_bits_sweep = _demodulate_light(temp_spi_bits);
    gpio_put(1, 0);

    // figure out which polynomial each one of the two samples come from.
    int8_t temp_bit_offset = 0;  // default offset
    gpio_put(2, 1);
    uint8_t temp_selected_polynomial = _determine_polynomial(temp_bits_sweep, &temp_bit_offset);
    gpio_put(2, 0);

    // If there was an error with the polynomial, leave without updating anything
    if (temp_selected_polynomial == LH2_POLYNOMIAL_ERROR_INDICATOR) {
        return;
    }

    // Figure in which of the two sweep slots we should save the new data.
    uint8_t sweep = _select_sweep(lh2, temp_selected_polynomial, temp_timestamp);

    // Put the newly read polynomials in the data structure (polynomial 0,1 must map to LH0, 2,3 to LH1. This can be accomplish by  integer-dividing the selected poly in 2, a shift >> accomplishes this.)
    // This structure always holds the two most recent sweeps from any lighthouse
    uint8_t basestation = temp_selected_polynomial >> 1;

    //*********************************************************************************//
    //                             Compute LFSR Position                               //
    //*********************************************************************************//

    // Select the valid bits of the lfsr by applying the offset (he first few bits might be invalid, as detected by _determine_polynomial())
    uint32_t temp_lfsr_bits = temp_bits_sweep >> (47 - temp_bit_offset);

    // Sanity check, make sure you don't start the LFSR search with a bit-sequence full of zeros.
    if (temp_lfsr_bits == 0x000000) {
        // Mark the data as wrong and keep going
        lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
        return;
    }

    // Compute the lfsr location.
    gpio_put(3, 1);
    uint32_t temp_lfsr_loc = _lfsr_index_search(&_lh2_vars[sensor].checkpoint,
                                                temp_selected_polynomial,
                                                temp_lfsr_bits);
    gpio_put(3, 0);

    // Check that the count didn't fall on an illegal value
    if (temp_lfsr_loc != 0) {
        // Save a new dynamic checkpoint
        _update_lfsr_checkpoints(sensor, temp_selected_polynomial, temp_lfsr_bits, temp_lfsr_loc, sweep);
    } else {
        // Mark the data as wrong and keep going
        lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
        return;
    }

    // Undo the bit offset introduced above, to get the LFSR position of the first bit that hit the sensor.
    temp_lfsr_loc -= temp_bit_offset;

    //*********************************************************************************//
    //                                 Store results                                   //
    //*********************************************************************************//

    // Save raw data information
    lh2->raw_data[sweep][basestation].bit_offset          = temp_bit_offset;
    lh2->raw_data[sweep][basestation].selected_polynomial = temp_selected_polynomial;
    lh2->raw_data[sweep][basestation].bits_sweep          = temp_bits_sweep;
    lh2->timestamps[sweep][basestation]                   = temp_timestamp;
    // Save processed location information
    lh2->locations[sweep][basestation].lfsr_location       = temp_lfsr_loc;
    lh2->locations[sweep][basestation].selected_polynomial = temp_selected_polynomial;
    // Mark the data point as processed
    lh2->data_ready[sweep][basestation] = DB_LH2_PROCESSED_DATA_AVAILABLE;
}

//=========================== private ==========================================

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

void _init_dma_pio_capture(uint8_t sensor) {
    // TODO: The problem is here
    // Make local copies of important variables, for readability.
    PIO     pio = _lh2_vars[sensor].pio;
    uint8_t sm  = _lh2_vars[sensor].sm;

    // Configure PIO->Temp Buffer DMA
    int chan                      = dma_claim_unused_channel(true);
    _lh2_vars[sensor].dma_channel = chan;  // save which channel belongs to which sensor

    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);      // Transfer 32 bits at a time (max possible)
    channel_config_set_read_increment(&c, false);               // reading from PIO FIFO, no need to increment
    channel_config_set_write_increment(&c, true);               // writing to temp buffer, increment
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));  // tie the DMA channel to the PIO capture
    // channel_config_set_ring(&c, true, 4);                       // 4 means reset address after (1 << 4) = 16 words transfers, or 64 bytes (TS4231_CAPTURE_BUFFER_SIZE)

    dma_channel_configure(
        chan,                             // Channel to be configured
        &c,                               // The configuration we just created
        _lh2_vars[sensor].spi_rx_buffer,  // The initial write address (temp buffer for the PIO capture)
        &pio->rxf[sm],                    // The initial read address (PIO RX FIFO)
        64,                               // Transfer 1 word per data request.
        true                              // Start immediately.
    );
}

void _add_to_ts4231_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, absolute_time_t timestamp) {

    memcpy(cb->buffer[cb->writeIndex], data, TS4231_CAPTURE_BUFFER_SIZE);
    cb->timestamps[cb->writeIndex] = timestamp;
    cb->writeIndex                 = (cb->writeIndex + 1) % LH2_BUFFER_SIZE;

    if (cb->count < LH2_BUFFER_SIZE) {
        cb->count++;
    } else {
        // Overwrite oldest data, adjust readIndex
        cb->readIndex = (cb->readIndex + 1) % LH2_BUFFER_SIZE;
    }
}

bool _get_from_ts4231_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, absolute_time_t *timestamp) {
    if (cb->count == 0) {
        // Buffer is empty
        return false;
    }

    memcpy(data, cb->buffer[cb->readIndex], TS4231_CAPTURE_BUFFER_SIZE);
    *timestamp    = cb->timestamps[cb->readIndex];
    cb->readIndex = (cb->readIndex + 1) % LH2_BUFFER_SIZE;
    cb->count--;

    return true;
}

void _update_lfsr_checkpoints(uint8_t sensor, uint8_t polynomial, uint32_t bits, uint32_t count, uint8_t sweep) {

    // Save the new count in the correct place in the checkpoint array
    _lh2_vars[sensor].checkpoint.bits[polynomial][sweep]  = bits;
    _lh2_vars[sensor].checkpoint.count[polynomial][sweep] = count;
}

uint8_t _select_sweep(db_lh2_t *lh2, uint8_t polynomial, absolute_time_t timestamp) {
    // TODO: check the exact, per-mode period of each polynomial instead of using a blanket 20ms

    uint8_t         basestation = polynomial >> 1;  ///< each base station uses 2 polynomials. integer dividing by 2 maps the polynomial number to the basestation number.
    absolute_time_t now         = get_absolute_time();
    // check that current data stored is not too old.
    for (size_t sweep = 0; sweep < 2; sweep++) {
        if (absolute_time_diff_us(lh2->timestamps[0][basestation], now) > LH2_MAX_DATA_VALID_TIME_US) {
            // Remove data that is too old.
            lh2->raw_data[sweep][basestation].bits_sweep          = 0;
            lh2->raw_data[sweep][basestation].selected_polynomial = LH2_POLYNOMIAL_ERROR_INDICATOR;
            lh2->raw_data[sweep][basestation].bit_offset          = 0;
            lh2->timestamps[sweep][basestation]                   = nil_time;
            lh2->data_ready[sweep][basestation]                   = DB_LH2_NO_NEW_DATA;
            // I don't think it's worth it to remove the location data. It is already marked as "No new data"
        }
    }

    ///< Encode in two bits which sweep slot of this basestation is empty, (1 means data, 0 means empty)
    uint8_t sweep_slot_state = (!is_nil_time(lh2->timestamps[1][basestation]) << 1) | (!is_nil_time(lh2->timestamps[0][basestation]));
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
            int64_t diff = (absolute_time_diff_us(lh2->timestamps[1][basestation], timestamp) % LH2_SWEEP_PERIOD_US);
            diff         = diff < LH2_SWEEP_PERIOD_US - diff ? diff : LH2_SWEEP_PERIOD_US - diff;

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
            int64_t diff = (absolute_time_diff_us(lh2->timestamps[0][basestation], timestamp) % LH2_SWEEP_PERIOD_US);
            diff         = diff < LH2_SWEEP_PERIOD_US - diff ? diff : LH2_SWEEP_PERIOD_US - diff;

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
            int64_t diff_0 = (absolute_time_diff_us(lh2->timestamps[0][basestation], timestamp) % LH2_SWEEP_PERIOD_US);
            diff_0         = diff_0 < LH2_SWEEP_PERIOD_US - diff_0 ? diff_0 : LH2_SWEEP_PERIOD_US - diff_0;
            int64_t diff_1 = (absolute_time_diff_us(lh2->timestamps[1][basestation], timestamp) % LH2_SWEEP_PERIOD_US);
            diff_1         = diff_1 < LH2_SWEEP_PERIOD_US - diff_1 ? diff_1 : LH2_SWEEP_PERIOD_US - diff_1;

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

void _pio_irq_handler_generic(uint8_t sensor) {
    // Make local copies of important variables, for readability.
    PIO     pio = _lh2_vars[sensor].pio;
    uint8_t sm  = _lh2_vars[sensor].sm;

    gpio_put(0, 1);
    // Read the current time.
    absolute_time_t timestamp = get_absolute_time();
    // Add new reading to the ring buffer
    _add_to_ts4231_ring_buffer(&_lh2_vars[sensor].data, _lh2_vars[sensor].spi_rx_buffer, timestamp);
    pio_sm_clear_fifos(pio, sm);  // Purge the PIO FIFO from any straggling bits
    // reset the DMA channel
    dma_channel_set_trans_count(_lh2_vars[sensor].dma_channel, 64, false);
    dma_channel_set_write_addr(_lh2_vars[sensor].dma_channel, _lh2_vars[sensor].spi_rx_buffer, true);
    // Clear the PIO interrupt
    pio_interrupt_clear(pio, sm);

    gpio_put(0, 0);
}

// Each ISR calls the same handler, for the appropriate sensor index.
void pio_irq_handler_0(void) {
    _pio_irq_handler_generic(0);
}

void pio_irq_handler_1(void) {
    _pio_irq_handler_generic(1);
}

void pio_irq_handler_2(void) {
    _pio_irq_handler_generic(2);
}

void pio_irq_handler_3(void) {
    _pio_irq_handler_generic(3);
}
