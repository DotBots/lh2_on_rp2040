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

#include "lh2.h"
#include "lh2_decoder.h"
#include "lh2_checkpoints.h"

//=========================== defines =========================================

#define FUZZY_CHIP                             0xFF  ///< not sure what this is about
#define POLYNOMIAL_BIT_ERROR_INITIAL_THRESHOLD 0     ///< tolerate no errors in received data
// #define POLYNOMIAL_BIT_ERROR_INITIAL_THRESHOLD 4                             ///< initial threshold of polynomial error
#define HASH_TABLE_BITS 6                             ///< How many bits will be used for the hashtable for the _end_buffers
#define HASH_TABLE_MASK ((1 << HASH_TABLE_BITS) - 1)  ///< Mask selecting the HAS_TABLE_BITS least significant bits

// Define variables useful for debuging
typedef uint32_t lfsr_17bits_t;

//=========================== variables ========================================

//=========================== prototypes =======================================

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
 * @param[in] checkpoints: structure with dynamic checkpoints
 * @param[in] index: index of polynomial
 * @param[in] bits: 17-bit sequence
 *
 * @return count: location of the sequence
 */
uint32_t _lfsr_index_search(_lfsr_checkpoint_t *checkpoint, uint8_t index, uint32_t bits);

//=========================== public ===========================================

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

    // Handle the edgecase of a full zero input
    if (chipsH1 == 0x00) {
        return LH2_POLYNOMIAL_ERROR_INDICATOR;
    }

    *start_val = 8;  // TODO: remove this? possible that I modify start value during the demodulation process

    int32_t  bits_N_for_comp                      = 47 - *start_val;
    uint32_t bit_buffer1                          = (uint32_t)(((0xFFFF800000000000) & chipsH1) >> 47);
    uint64_t bits_from_poly[LH2_POLYNOMIAL_COUNT] = { 0 };
    uint8_t  weights[LH2_POLYNOMIAL_COUNT]        = { 0xFF };
    uint8_t  selected_poly                        = LH2_POLYNOMIAL_ERROR_INDICATOR;  // initialize to error condition
    uint8_t  min_weight_idx                       = LH2_POLYNOMIAL_ERROR_INDICATOR;
    uint64_t min_weight                           = LH2_POLYNOMIAL_ERROR_INDICATOR;
    uint64_t bits_to_compare                      = 0;
    int32_t  threshold                            = POLYNOMIAL_BIT_ERROR_INITIAL_THRESHOLD;

#if defined(LH2_MOCAP_FILTER)
    // tighten threshold if we expect MoCap interference
    threshold = 0;
#endif

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
            weights[i]        = __builtin_popcount(bits_from_poly[i] ^ bits_to_compare);
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

// uint8_t _determine_polynomial_2(uint64_t input_bits, int8_t *start_val) {
//     // check which polynomial the bit sequence is part of
//     // TODO: make function a void and modify memory directly

//     // Handle the edgecase of a full zero input
//     if (input_bits == 0x00) {
//         return LH2_POLYNOMIAL_ERROR_INDICATOR;
//     }

//     *start_val                                    = 8;                               // how many bits to ignore from the START of the sequence (MSB)
//     uint8_t  end_val                              = 0;                               // how many bits to ignore from the END of the sequence (LSB)
//     int32_t  bits_N_for_comp                      = 64 - 17 - *start_val - end_val;  // How many bits will be used to comparison. 64bit - start and end offset - 17bit from the sequence extended.
//     uint64_t bits_to_compare                      = 0;                               // input bit sub-sequence used for the comparison.
//     uint32_t lfsr_buffer                          = 0;                               // 17 bits sequence that will be extended using the known polynomials
//     uint64_t predicted_bits[LH2_POLYNOMIAL_COUNT] = { 0 };                           // bits predicted using the known polynomials, based on the input
//     uint8_t  weights[LH2_POLYNOMIAL_COUNT]        = { 0xFF };                        // score of how well each polynomial matches the input bits (lower the better, 0 is a perfect match)
//     uint8_t  selected_poly                        = LH2_POLYNOMIAL_ERROR_INDICATOR;  // initialize to error condition
//     uint8_t  min_weight_idx                       = LH2_POLYNOMIAL_ERROR_INDICATOR;
//     uint64_t min_weight                           = LH2_POLYNOMIAL_ERROR_INDICATOR;
//     int32_t  threshold                            = 0;  // we don't tolerate a single bit error between the input bits and the predicted data.

//     // try polynomial vs. first buffer bits
//     // this search takes 17-bit sequences and runs them forwards through the polynomial LFSRs.
//     // if the remaining detected bits fit well with the chosen 17-bit sequence and a given polynomial, it is treated as "correct"
//     // in case of bit errors at the beginning of the capture, the 17-bit sequence is shifted (to a max of 8 bits)
//     // in case of bit errors at the end of the capture, the ending bits are removed (to a max of
//     // removing bits reduces the threshold correspondingly, as incorrect packet detection will cause a significant delay in location estimate

//     // run polynomial search on the first capture
//     while (1) {

//         // Select which 17bit sequence will be used to predict the rest of the sequence
//         uint64_t mask_lfsr_buffer = 0xFFFF800000000000 >> (*start_val);                        // create a mask in the proper place
//         mask_lfsr_buffer          = mask_lfsr_buffer & input_bits;                             // apply the mask
//         lfsr_buffer               = (uint32_t)(mask_lfsr_buffer >> (64 - 17 - (*start_val)));  // Move the masked bit to the right (right justify-them)

//         // TODO: do this math stuff in multiple operations to: (a) make it readable (b) ensure order-of-execution
//         bits_to_compare = (input_bits & (0xFFFFFFFFFFFFFFFF << (64 - 17 - (*start_val) - bits_N_for_comp)));
//         // reset the minimum polynomial match found
//         min_weight_idx = LH2_POLYNOMIAL_ERROR_INDICATOR;
//         min_weight     = LH2_POLYNOMIAL_ERROR_INDICATOR;
//         // Check against all the known polynomials
//         for (uint8_t i = 0; i < LH2_POLYNOMIAL_COUNT; i++) {
//             predicted_bits[i] = (((_poly_check(_polynomials[i], bit_buffer1, bits_N_for_comp)) << (64 - 17 - (*start_val) - bits_N_for_comp)) | (chipsH1 & (0xFFFFFFFFFFFFFFFF << (64 - (*start_val)))));
//             weights[i]        = __builtin_popcount(predicted_bits[i] ^ bits_to_compare);
//             // Keep track of the minimum weight value and which polinimial generated it.
//             if (weights[i] < min_weight) {
//                 min_weight_idx = i;
//                 min_weight     = weights[i];
//             }
//         }

//         // If you found a sufficiently good value, then return which polinomial generated it
//         if (min_weight <= (uint64_t)threshold) {
//             selected_poly = min_weight_idx;
//             break;
//             // match failed, try again removing bits from the end
//         } else if (*start_val > 8) {
//             *start_val      = 8;
//             bits_N_for_comp = bits_N_for_comp - 9;
//             if (threshold > 2) {
//                 threshold = threshold - 1;
//             } else if (threshold == 2) {  // keep threshold at ones, but you're probably screwed with an unlucky bit error
//                 threshold = 2;
//             }
//         } else {
//             *start_val = *start_val + 1;
//             bits_N_for_comp -= 1;
//         }

//         // too few bits to reliably compare, give up
//         if (bits_N_for_comp < 19) {
//             selected_poly = LH2_POLYNOMIAL_ERROR_INDICATOR;  // mark the poly as "wrong"
//             break;
//         }
//     }
//     return selected_poly;
// }

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

uint32_t _lfsr_index_search(_lfsr_checkpoint_t *checkpoint, uint8_t index, uint32_t bits) {

    lfsr_17bits_t bits_local  = bits & 0x0001FFFF;  // initialize buffer to initial bits, masked
    lfsr_17bits_t buffer_down = bits_local;
    lfsr_17bits_t buffer_up   = bits_local;

    uint32_t count_down  = 0;
    uint32_t count_up    = 0;
    int32_t  count_final = 0;  // There is a chance for overflow near the roll-over point of the lfsr sequence (sequence very high (>120k), hitting a very low check point (~20))
    uint32_t b17         = 0;
    uint32_t b1          = 0;
    uint32_t masked_buff = 0;
    uint8_t  hash_down   = 0;
    uint8_t  hash_up     = 0;
    bool     success     = false;  // True if the LFSR search produced a valid result

    // Copy const variables (Flash) into local variables (RAM) to speed up execution.
    uint32_t _lfsr_hash_table_local[NUM_LSFR_COUNT_CHECKPOINTS]  = { 0 };
    uint32_t _lfsr_index_table_local[NUM_LSFR_COUNT_CHECKPOINTS] = { 0 };
    uint32_t polynomials_local                                   = _polynomials[index];
    for (size_t i = 0; i < NUM_LSFR_COUNT_CHECKPOINTS; i++) {
        _lfsr_hash_table_local[i]  = _lfsr_hash_table[index][i];
        _lfsr_index_table_local[i] = _lfsr_index_table[index][i];
    }

    // Start the iterative search
    while (buffer_up != 0x00 && buffer_down != 0x00) {  // Check that the count has not fallen into an invalid state    {

        //
        // CHECKPOINT CHECKING
        //

        // Check lfsr backward count against precomputed checkpoints
        hash_down = (buffer_down >> 11) & HASH_TABLE_MASK;
        if (buffer_down == _lfsr_hash_table_local[hash_down]) {
            count_final = count_down + _lfsr_index_table_local[hash_down];
            success     = true;  // mark the success of the search
            break;
        }

        // Check lfsr forward count against precomputed checkpoints
        hash_up = (buffer_up >> 11) & HASH_TABLE_MASK;
        if (buffer_up == _lfsr_hash_table_local[hash_up]) {
            count_final = _lfsr_index_table_local[hash_up] - count_up;
            success     = true;  // mark the success of the search
            break;
        }

        // Check the dynamical checkpoints, backward
        // Sweep 0
        if (buffer_down == checkpoint->bits[index][0]) {
            count_final = count_down + checkpoint->count[index][0];
            success     = true;  // mark the success of the search
            break;
        }
        // Sweep 1
        if (buffer_down == checkpoint->bits[index][1]) {
            count_final = count_down + checkpoint->count[index][1];
            success     = true;  // mark the success of the search
            break;
        }

        // Check the dynamical checkpoints, forward
        // Sweep 0
        if (buffer_up == checkpoint->bits[index][0]) {
            count_final = checkpoint->count[index][0] - count_up;
            success     = true;  // mark the success of the search
            break;
        }
        // Sweep 1
        if (buffer_up == checkpoint->bits[index][1]) {
            count_final = checkpoint->count[index][1] - count_up;
            success     = true;  // mark the success of the search
            break;
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

    // Return the found lfsr value
    if (success) {
        // Handle overflows
        if (count_final < 0) {
            // wrap around the number
            count_final += 131071;  // 2^17 -1 (the full lenght of the sequence)
        }
        // Turn result back to unsigned before returning
        return (uint32_t)count_final;
    } else {
        return LH2_LFSR_SEARCH_ERROR_INDICATOR;
    }
}

bool _check_mocap_interference(uint8_t *arr, uint8_t size) {

    // Qualysis Mocap cameras pulse IR light modulated with a regular square wave at 1Mhz.
    // At the 32Mhz speed we sample the SPI, that corresponds to an alternating 0xFF,0xFF,0xFF,0x00,0x00,0x00 pattern

    // Check only the bottom half of the array, that should be enough to catch an error.
    for (int i = 0; i < size / 2; i++) {
        // Check for 3 consecutive 0xFF
        if (arr[i] == 0xFF && arr[i + 1] == 0xFF && arr[i + 2] == 0xFF) {
            return true;  // Error for 3 consecutive 0xFF
        }

        // Check for 3 consecutive 0x00
        if (arr[i] == 0x00 && arr[i + 1] == 0x00 && arr[i + 2] == 0x00) {
            return true;  // Error for 3 consecutive 0x00
        }
    }
    return false;  // No error found
}
//=========================== private ==========================================

//=========================== interrupts =======================================
