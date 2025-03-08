import numpy as np
from common import polynomials, compute_full_lfsr_sequence, indices_2_lfsr, naive_checkpoints_indices


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

def print_c_close_vector_table(file):
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

def print_c_close_index_table(file):
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
        print_c_close_vector_table(file)

        
        # Print the matching index table
        print_c_index_table_header(file)
        print_c_index_table(file, indices_list[0], lfsr_list[0], 0)
        print_c_close_index_table(file)

            