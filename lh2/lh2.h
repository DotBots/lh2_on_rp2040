#ifndef __LH2_H_
#define __LH2_H_

/**
 * @defgroup    bsp_lh2 LightHouse 2 support
 * @ingroup     bsp
 * @brief       Control the LH2 sensor
 *
 * @{
 * @file
 * @author Filip Maksimovic <filip.maksimovic@inria.fr>
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022-present
 * @}
 */

#include <nrf.h>
#include <stdint.h>
#include <stdbool.h>

#include "gpio.h"

//=========================== defines ==========================================

#define LH2_BASESTATION_COUNT 4                          ///< Number of supported concurrent basestations
#define LH2_POLYNOMIAL_COUNT  LH2_BASESTATION_COUNT * 2  ///< Number of supported LFSR polynomials, two per basestation
#define LH2_SWEEP_COUNT       2                          ///< Number of laser sweeps per basestations rotation

/// LH2 data ready buffer state
typedef enum {
    DB_LH2_NO_NEW_DATA,               ///< The data occupying this spot of the buffer has already been sent.
    DB_LH2_RAW_DATA_AVAILABLE,        ///< The data occupying this spot of the buffer is new and ready to send.
    DB_LH2_PROCESSED_DATA_AVAILABLE,  ///< The data occupying this spot of the buffer is new and ready to send.
} db_lh2_data_ready_state_t;

/// LH2 raw data
typedef struct __attribute__((packed)) {
    uint64_t bits_sweep;           ///< bits sweep is the result of the demodulation, sweep_N indicates which SPI transfer those bits are associated with
    uint8_t  selected_polynomial;  ///< selected poly is the polyomial # (between 0 and 31) that the demodulation code thinks the demodulated bits are a part of, initialize to error state
    int8_t   bit_offset;           ///< bit_offset indicates an offset between the start of the packet, as indicated by envelope dropping, and the 17-bit sequence that is verified to be in a known LFSR sequence
} db_lh2_raw_data_t;

/// LH2 raw data location
typedef struct __attribute__((packed)) {
    uint8_t  selected_polynomial;  ///< selected poly is the polyomial # (between 0 and 31) that the demodulation code thinks the demodulated bits are a part of, initialize to error state
    uint32_t lfsr_location;        ///< LFSR location is the position in a given polynomial's LFSR that the decoded data is, initialize to error state
} db_lh2_location_t;

/// LH2 instance (one row per laser sweep, and one column per basestation)
typedef struct {
    db_lh2_raw_data_t         raw_data[LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];    ///< raw data decoded from the lighthouse
    db_lh2_location_t         locations[LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];   ///< buffer holding the computed locations
    uint32_t                  timestamps[LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];  ///< timestamp of when the raw data was received
    db_lh2_data_ready_state_t data_ready[LH2_SWEEP_COUNT][LH2_BASESTATION_COUNT];  ///< Is the data in the buffer ready to send over radio, or has it already been sent ?
    uint8_t                  *spi_ring_buffer_count_ptr;                           ///< pointer to the SPI rung buffer packet count, so the user application can read how many spi captures are waiting to be processed.
} db_lh2_t;

//=========================== public ===========================================

/**
 * @brief Initialize LH2
 *
 * @param[in]   lh2 pointer to the lh2 instance
 * @param[in]   gpio_d  pointer to gpio data
 * @param[in]   gpio_e  pointer to gpio event
 */
void db_lh2_init(db_lh2_t *lh2, const gpio_t *gpio_d, const gpio_t *gpio_e);

/**
 * @brief Process raw data coming from the lighthouse, but skip the polynomial count calculation.
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_process_raw_data(db_lh2_t *lh2);

/**
 * @brief Compute the location based on raw data coming from the lighthouse
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_process_location(db_lh2_t *lh2);

/**
 * @brief Start the LH2 frame acquisition
 *
 */
void db_lh2_start(void);

/**
 * @brief Stop the LH2 frame acquisition
 *
 */
void db_lh2_stop(void);

/**
 * @brief Reset the lh2 internal state so new location computation can be made
 *
 * @param[in]   lh2 pointer to the lh2 instance
 */
void db_lh2_reset(db_lh2_t *lh2);

#endif /* __LH2_H_ */
