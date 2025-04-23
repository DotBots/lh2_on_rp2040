# GDB Pretty printer for the _lfsr_index_search() function.
# It prints the lfsr sequence in nice 17bits format, and also uses python to search the position of the current sequence and prints it as well.
#
#

import numpy as np
import gdb.printing


class Bits17Print:
    """Pretty-printer for printing uint32_t variables as 17bit binary numbers"""
    def __init__(self, val):
        self.val = val
    def to_string(self):
        index = int(gdb.parse_and_eval("index"))
        # Find the current position in the LFSR sequence
        lfsr = int(self.val)
        lfsr_int = find_lfsr_sequence(lfsr, index)
        # Print it nicely
        return f"0b{format(lfsr,'017b')} <> poly: {index} <> seq: {lfsr_int}"
    

# This matches the pretty printer to the name of some variables
def lookup_function(val):
    # This function is called for every variable in the context
    if str(val.type) == 'lfsr_17bits_t': return Bits17Print(val)

    return None

# Define polynomials (same as MATLAB input)
polynomials = [
    '0001D258','00017E04','0001FF6B','00013F67','0001B9EE','000198D1',
    '000178C7','00018A55','00015777','0001D911','00015769','0001991F',
    '00012BD0','0001CF73','0001365D','000197F5','000194A0','0001B279',
    '00013A34','0001AE41','000180D4','00017891','00012E64','00017C72',
    '00019C6D','00013F32','0001AE14','00014E76','00013C97','000130CB',
    '00013750','0001CB8D'
]

def find_lfsr_sequence(lfsr, num_poly):
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
    sequence_length = 131071 * 2 # 2^17 - 1
    state = np.array([0] * 16 + [1], dtype=int)  # Initial state: 16 zeros + 1

    # Check the first lfsr value
    if lfsr == 1:
        return 0


    result = -1
    for i in range(17, sequence_length):
        feedback = np.sum(state * selected_poly) % 2
        state = np.roll(state, -1)
        state[-1] = feedback

        # Get the unsigned version of the lfsr sequence.
        lfsr_int = np.dot(state,1 << np.arange(len(state) - 1, -1, -1)) 

        if lfsr == lfsr_int:
            result = i - 16
            break

    return result


gdb.printing.register_pretty_printer(gdb.current_objfile(), lookup_function, replace=True)
