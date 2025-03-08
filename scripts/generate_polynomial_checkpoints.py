import numpy as np
from common import polynomials, compute_full_lfsr_sequence, indices_2_lfsr, naive_checkpoints_indices


def create_offset_matrix(lfsr_sequence, indices, hash_bits):
    "Returns a matrix. each line is an checkpoint index, and each column is the offest needed to arrive at a particular hash"

    offset_matrix = np.ones((indices.shape[0], 2**hash_bits)).astype(int)
    paint_value = 2**(hash_bits+1) * 10
    offset_matrix *= paint_value # paint the matrix with an arbitrarily large number

    max_offset = 2**(hash_bits+1) # From where to where to check the lfsr sequence

    # Go checkpoint by checkpoint
    for idx,checkpoint in enumerate(indices):

        # If this is the first checkpoint, enforce only positive offsets
        if checkpoint == 1:
            offset_bot = 0
            offset_top = 2*max_offset
        else:
            offset_bot = -max_offset
            offset_top = max_offset

        # check many lfsr positions before and after the checkpoint
        for offset in range(offset_bot, offset_top):

            # extract current lfsr section
            start = checkpoint -1  + offset
            stop = checkpoint + 16 + offset
            lfsr_chunk = lfsr_sequence[start:stop]

            # compute hash
            hash = lfsr_chunk[:hash_bits]
            # convert the hash array into an actual unsigned integer
            hash_int = np.dot(hash,1 << np.arange(len(hash) - 1, -1, -1)) 

            # save the value to the offset matrix if there is not an smaller value already present
            if abs(offset) < abs(offset_matrix[idx][hash_int]):
                offset_matrix[idx][hash_int] = offset

    return offset_matrix

def find_min_offsets(offset_matrix, indices, hash_bits):

    # start the variables for the random optimizator
    print_n = 0
    print_done = 0
    best_candidate = 0
    best_kpi = 2**hash_bits *10

    # Set the seed for the simulation
    # np.random.seed(42)
    # np.random.seed(40)
    np.random.seed(133)

    # Precreate the row index
    row_idx = np.arange(offset_matrix.shape[0])

    print(f"kpi = ", end="")
    while(True):
        # Sample a new random index for the offset matrix
        sample = np.random.permutation(offset_matrix.shape[1])[:offset_matrix.shape[0]]  # sample, and remove one if tha matrix is not square, because the offset matrix is (63,64)
        # Get the vector of candidate offsets
        candidate = offset_matrix[row_idx,sample]

        # if the proposed offset for the first checkpoint (0x001) is negative, discard the solution
        # We are only interested in positive offsets for the first checkpoint.
        if candidate[0] < 0: continue

        # Evaluate the solution
        if np.abs(candidate).max() < best_kpi:
            best_kpi = np.abs(candidate).max()
            best_candidate = candidate

        # Print progress every once in a while
        print_n += 1
        if print_n % 100000 == 0:
            print(f"{best_kpi}, ", end="", flush=True)

        # Stop if you hit a miracle
        if best_kpi <= 40: break

        # Stop after too many iterations
        if print_n > 100000 * 200: break # This should give 200 prints, or 20M iterations

    print()
    return best_candidate

def lfsr_2_hashes(sequences, hash_bits):

    # Get the Hash
    hashes = sequences[:,:hash_bits]
    hashes_int = np.dot(hashes,1 << np.arange(hashes.shape[1] - 1, -1, -1))

    return hashes_int

def print_c_file_header(file, hash_bits):
    
    # Print headers of the file
    file_header = f"""\
#ifndef __LH2_CHECKPOINTS_H_
#define __LH2_CHECKPOINTS_H_

/**
* @brief  Precomputed checkpoints for the LFSR index search
*
* @{{
* @file
* @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
* @copyright Inria, 2025-present
* @}}
*/

#include "lh2.h"

//=========================== defines =========================================

#define NUM_LSFR_COUNT_CHECKPOINTS {2**hash_bits}                            ///< How many lsfr checkpoints are per polynomial

//=========================== variables ========================================

static const uint32_t _polynomials[LH2_POLYNOMIAL_COUNT] = {{
"""
    file.write(file_header)

    # Add polynomials
    for poly in polynomials:
        file.write(f"    0x{poly},\n")
    file.write("};\n\n")

    # Add the header of the Hash table
    file.write("static const uint32_t _lfsr_hash_table[LH2_POLYNOMIAL_COUNT][NUM_LSFR_COUNT_CHECKPOINTS] = {\n")

def print_c_hash_table(file, indices, lfsr, hashes, poly_num):
    # Add the hash table
    # Go polynomial by polynomial

    # Print which polynomial
    file.write("    {\n")
    file.write(f"        // Polynomial: {poly_num}\n")

    h_map = hashes.argsort()
    # Print each checkpoint
    for check in range(lfsr.shape[0]):
        file.write("        0b")
        file.write("".join(map(str, lfsr[h_map[check]])))
        file.write(f",    // lfsr position: {indices[h_map[check]]}\n")
        
    # Add closing braket
    file.write("    },\n")

def print_c_close_hash_table():
    file.write("};\n\n")

def print_c_index_table_header(file):
    # Add the header of the Hash table
    file.write("static const uint32_t _lfsr_index_table[LH2_POLYNOMIAL_COUNT][NUM_LSFR_COUNT_CHECKPOINTS] = {\n")

def print_c_index_table(file, indices, lfsr, hashes, poly_num):
    # Add the hash table
    # Go polynomial by polynomial

    # Print which polynomial
    file.write("    {\n")
    file.write(f"        // Polynomial: {poly_num}\n")

    h_map = hashes.argsort()
    # Print each checkpoint
    for check in range(lfsr.shape[0]):
        file.write(f"        {indices[h_map[check]]}")
        bits = "".join(map(str, lfsr[h_map[check]]))
        file.write(f",    // lfsr bits: 0b{bits}\n")
        
    # Add closing braket
    file.write("    },\n")

def print_c_close_index_table():
    file.write("};\n\n#endif /* __LH2_CHECKPOINTS_H_ */")

def generate_checkpoints(num_poly=0, hash_bits=6):

    num_checkpoints = 2**hash_bits
    full_lfsr = compute_full_lfsr_sequence(num_poly)
    naive_indices = naive_checkpoints_indices(num_checkpoints)

    # Compute the matrix of lfsr hash offset
    offset_matrix = create_offset_matrix(full_lfsr, naive_indices, hash_bits)
    # Find the best combination of offsets
    offset_vector = find_min_offsets(offset_matrix, naive_indices, hash_bits)

    # Apply the offsets:
    optimal_indices = naive_indices + offset_vector

    # Get the lfsr section for the optimal checkpoints
    optimal_lfsr = indices_2_lfsr(full_lfsr, optimal_indices) 
    optimal_hash = lfsr_2_hashes(optimal_lfsr, hash_bits)

    naive_lfsr = indices_2_lfsr(full_lfsr, naive_indices) 
    naive_hash  = lfsr_2_hashes(naive_lfsr, hash_bits) 


    print(f"checkpoint indices = {optimal_indices}")
    # print(optimal_hash)
    print(f"offset vector = {offset_vector}")
    # print(naive_indices)

    return optimal_indices, optimal_lfsr, optimal_hash



if __name__ == "__main__":

    # Options
    hash_bits = 6
    poly_list = range(32)

    # Store the computed values
    indices_list = []
    lfsr_list = []
    hashes_list = []

    with open("lh2_checkpoints.h", "w") as file:

        # Print the start of the file
        print_c_file_header(file, hash_bits)

        # Generate the checkpoint table, one polynomial at a time.
        for poly in poly_list:
            print(f"\nPolynomial: {poly}")
            # Generate checkpoints
            indices, lfsr, hashes = generate_checkpoints(num_poly=poly, hash_bits=hash_bits)
            # Print checkpoints to a file
            print_c_hash_table(file, indices, lfsr, hashes, poly)

            # Store values
            indices_list.append(indices)
            lfsr_list.append(lfsr)
            hashes_list.append(hashes)
        print_c_close_hash_table()

        
        # Print the matching index table
        print_c_index_table_header(file)
        for poly in poly_list:
            # Print the index table to a file
            print_c_index_table(file, indices_list[poly], lfsr_list[poly], hashes_list[poly], poly)
        print_c_close_index_table()

            