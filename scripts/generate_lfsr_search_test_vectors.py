import numpy as np
import itertools

# Define polynomials (same as MATLAB input)
polynomials = [
    '0001D258','00017E04','0001FF6B','00013F67','0001B9EE','000198D1',
    '000178C7','00018A55','00015777','0001D911','00015769','0001991F',
    '00012BD0','0001CF73','0001365D','000197F5','000194A0','0001B279',
    '00013A34','0001AE41','000180D4','00017891','00012E64','00017C72',
    '00019C6D','00013F32','0001AE14','00014E76','00013C97','000130CB',
    '00013750','0001CB8D'
]

def compute_full_lfsr_sequence(num_poly=0):
    # Convert hex character to 4-bit binary string
    hex_to_bits = {
        '0': [0,0,0,0], '1': [0,0,0,1], '2': [0,0,1,0], '3': [0,0,1,1],
        '4': [0,1,0,0], '5': [0,1,0,1], '6': [0,1,1,0], '7': [0,1,1,1],
        '8': [1,0,0,0], '9': [1,0,0,1], 'A': [1,0,1,0], 'B': [1,0,1,1],
        'C': [1,1,0,0], 'D': [1,1,0,1], 'E': [1,1,1,0], 'F': [1,1,1,1]
    }

    # Convert all polynomials to binary bits
    polybits = np.zeros((32, 32), dtype=int)
    for jj, hex_str in enumerate(polynomials):
        for kk, char in enumerate(hex_str):
            polybits[jj, (kk * 4):(kk + 1) * 4] = hex_to_bits[char]

    # Select polynomial and truncate to 17 bits (remove first 15 bits)
    selected_poly = polybits[num_poly, 15:32]  # Keep last 17 bits

    # Generate LFSR sequence
    sequence_length = 131071  # 2^17 - 1
    poly_brute = np.zeros(sequence_length, dtype=int)
    state = np.array([0] * 16 + [1], dtype=int)  # Initial state: 16 zeros + 1
    poly_brute[:17] = state[::-1]  # Store initial state reversed

    for i in range(17, sequence_length):
        feedback = np.sum(state * selected_poly) % 2
        state = np.roll(state, -1)
        state[-1] = feedback
        poly_brute[i] = feedback

    return poly_brute

def naive_checkpoints_indices(num_of_checkpoints):
    # Return a list of indices for regularly spaced checkpoints
    # indices = np.round(np.linspace(0, 2**17, num_of_checkpoints + 1)).astype(int)[0:-1]
    indices = np.round(np.linspace(0, 110e3, num_of_checkpoints + 1)).astype(int)[0:-1]
    indices[0] += 1  # Make sure the first element is 1 and not 0
    return indices

def indices_2_lfsr(full_lfsr, indices):

    start = indices.reshape((-1,)) -1
    stop = indices.reshape((-1,)) + 16

    sequences = np.zeros((indices.shape[0],17)).astype(int)

    for i in range(indices.shape[0]):
        sequences[i] = full_lfsr[start[i]:stop[i]]

    return sequences

def print_c_file_header(file, num_vectors):
    
    # Print headers of the file
    file_header = f"""\
#ifndef __LH2_TEST_VECTORS_H_
#define __LH2_TEST_VECTORS_H_

/**
* @brief  Hand-picked python-generated checkpoints to test the LFSR index search
*         Generated with: scripts/generate_lfsr_search_test_vectors.py
*
* @{{
* @file
* @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
* @copyright Inria, 2025-present
* @}}
*/

#include "lh2.h"

//=========================== defines =========================================

#define NUM_LSFR_TEST_VECTORS {num_vectors}                            ///< How many lsfr checkpoints are per polynomial

//=========================== variables ========================================

static const uint32_t test_polynomials[LH2_POLYNOMIAL_COUNT] = {{
"""
    file.write(file_header)

    # Add polynomials
    for poly in polynomials:
        file.write(f"    0x{poly},\n")
    file.write("};\n\n")

    # Add the header of the Hash table
    file.write("static const uint32_t test_lfsr_vector_table[LH2_POLYNOMIAL_COUNT][NUM_LSFR_TEST_VECTORS] = {\n")

def print_c_vector_table(file, indices, lfsr, poly_num):
    # Add the hash table
    # Go polynomial by polynomial

    # Print which polynomial
    file.write("    {\n")
    file.write(f"        // Polynomial: {poly_num}\n")

    # Print each checkpoint
    for check in range(lfsr.shape[0]):
        file.write("        0b")
        file.write("".join(map(str, lfsr[check])))
        file.write(f",    // lfsr position: {indices[check]}\n")
        
    # Add closing braket
    file.write("    },\n")

def print_c_close_vector_table():
    file.write("};\n\n")

def print_c_index_table_header(file):
    # Add the header of the Hash table
    file.write("static const uint32_t test_lfsr_index_table[NUM_LSFR_TEST_VECTORS] = {\n")

def print_c_index_table(file, indices, lfsr, poly_num):
    # Add the hash table
    # Go polynomial by polynomial

    # Print which polynomial
    # file.write("    {\n")
    # file.write(f"        // Polynomial: {poly_num}\n")

    # Print each checkpoint
    for check in range(lfsr.shape[0]):
        file.write(f"    {indices[check]}")
        # bits = "".join(map(str, lfsr[check]))
        # file.write(f",    // lfsr bits: 0b{bits}\n")
        file.write(f",\n")
        
    # Add closing braket
    # file.write("    },\n")

def print_c_close_index_table():
    file.write("};\n\n#endif /* __LH2_TEST_VECTORS_H_ */")

def generate_test_vectors(num_poly=0, num_vectors=100):

    num_checkpoints = num_vectors
    full_lfsr = compute_full_lfsr_sequence(num_poly)
    naive_indices = naive_checkpoints_indices(num_checkpoints)

    naive_lfsr = indices_2_lfsr(full_lfsr, naive_indices) 

    print(f"checkpoint indices = {naive_indices}")

    return naive_indices, naive_lfsr



if __name__ == "__main__":

    # Options
    num_vectors = 100
    poly_list = range(len(polynomials))

    # Store the computed values
    indices_list = []
    lfsr_list = []

    with open("lh2_lfsr_search_test_vectors.h", "w") as file:

        # Print the start of the file
        print_c_file_header(file, num_vectors)

        # Generate the checkpoint table, one polynomial at a time.
        for poly in poly_list:
            print(f"\nPolynomial: {poly}")
            # Generate checkpoints
            indices, lfsr = generate_test_vectors(num_poly=poly, num_vectors=num_vectors)
            # Print checkpoints to a file
            print_c_vector_table(file, indices, lfsr, poly)

            # Store values
            indices_list.append(indices)
            lfsr_list.append(lfsr)
        print_c_close_vector_table()

        
        # Print the matching index table
        print_c_index_table_header(file)
        print_c_index_table(file, indices_list[0], lfsr_list[0], 0)
        print_c_close_index_table()

            