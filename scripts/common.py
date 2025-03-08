import numpy as np

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
    poly_brute[:17] = state[:]  # Store initial state

    for i in range(17, sequence_length):
        feedback = np.sum(state * selected_poly) % 2
        state = np.roll(state, -1)
        state[-1] = feedback
        poly_brute[i] = feedback

    return poly_brute

def indices_2_lfsr(full_lfsr, indices):

    start = indices.reshape((-1,))
    stop = indices.reshape((-1,)) + 17

    sequences = np.zeros((indices.shape[0],17)).astype(int)

    for i in range(indices.shape[0]):
        sequences[i] = full_lfsr[start[i]:stop[i]]

    return sequences

def naive_checkpoints_indices(num_of_checkpoints):
    # Return a list of indices for regularly spaced checkpoints
    # indices = np.round(np.linspace(0, 2**17, num_of_checkpoints + 1)).astype(int)[0:-1]
    indices = np.round(np.linspace(0, 110e3, num_of_checkpoints + 1)).astype(int)[0:-1]
    return indices