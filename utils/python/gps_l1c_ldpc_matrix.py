"""
gps_l1c_ldpc_matrix.py

Generates C++ source code for the LDPC sparse matrices for GPS L1C.

José Antonio Mayo, 2026. contact@tatjam.eu

-----------------------------------------------------------------------------

GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
This file is part of GNSS-SDR.

Copyright (C) 2022  (see AUTHORS file for a list of contributors)
SPDX-License-Identifier: GPL-3.0-or-later

----------------------------------------------------------------------------
"""

import sys
import os

import numpy as np

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
import gps_l1c_ldpc_matrix_raw_data as d


def build_matrix(a, b, c, d, e, t, rows, cols):
    out = np.zeros((rows, cols), dtype=int)

    a_cols = cols// 2
    b_cols = 1
    a_rows = rows - 1

    # Note indices in the datasheet start at 1
    for row, col in a:
        out[row - 1, col - 1] = 1
    for row, col in b:
        out[row - 1, col - 1 + a_cols] = 1
    for row, col in c:
        out[row - 1 + a_rows, col - 1] = 1
    for row, col in d:
        out[row - 1 + a_rows, col - 1 + a_cols] = 1
    for row, col in e:
        out[row - 1 + a_rows, col - 1 + a_cols + b_cols] = 1
    for row, col in t:
        out[row - 1, col - 1 + a_cols + b_cols] = 1

    return out

# In order of ascending row, the non-zero column indices.
# Intuitively, this is the concatenation of the list of VN neighbors
# to each CN, in order of increasing CN index
def build_luts(mat):
    # cn_neighbors maps each edge to a VN (col)
    cn_neighbors = []

    # cn_row_ptr[i, i+1] maps the CN (row) i, to the range
    # of edges it's connected to 
    cn_row_ptr = [0]

    for row in range(mat.shape[0]):
        neighbors = []
        for col in range(mat.shape[1]):
            if mat[row, col] != 0:
                neighbors.append(col)

        cn_neighbors += neighbors
        cn_row_ptr.append(len(cn_neighbors))

    permutation = []
    vn_col_ptr = [0]

    for col in range(mat.shape[1]):
        permutations = []
        for edge, vn in enumerate(cn_neighbors):
            if vn == col:
                permutations.append(edge)

        permutation += permutations
        vn_col_ptr.append(len(permutation))
        
    return cn_neighbors, cn_row_ptr, permutation, vn_col_ptr

# Converts an array of numbers to a hex string of densely packed, big-endian
# 12 / 16 bit hex string values
def array_to_hex_str(arr, size):
    out = ""
    for entry in arr:
        if size == 16:
            assert entry >= 0 and entry < 65536
            out += f"{entry:04X}"
        else:
            assert entry >= 0 and entry < 4096
            out += f"{entry:03X}"
    
    return out
    
def dump_wrapped_cpp_hexstr(full, name):
    chunks = [full[i : i + 73] for i in range(0, len(full), 73)]

    print(f"constexpr int {name}_LENGTH = {len(full)}; //!< Not including zero terminator")
    print(f"constexpr char {name}_STR[{name}_LENGTH + 1] =")
    for chunk in chunks[:-1]:
        print(f'    "{chunk}"')
    print(f'    "{chunks[-1]}";')

def dump_matrix(mat, c_name):
    cn_neighbors, cn_row_ptr, permutation, vn_col_ptr = build_luts(mat)

    needs_16bit = np.count_nonzero(mat) > 4096
    bit_size = 16 if np.count_nonzero(mat) > 4096 else 12
    cn_neighbors_hex = array_to_hex_str(cn_neighbors, bit_size)
    cn_row_ptr_hex = array_to_hex_str(cn_row_ptr, bit_size)
    permutation_hex = array_to_hex_str(permutation, bit_size)
    vn_col_ptr_hex = array_to_hex_str(vn_col_ptr, bit_size)

    print(f"constexpr bool {c_name}_IS_16_BIT_HEXSTR = {str(needs_16bit).lower()};")
    print("")
    dump_wrapped_cpp_hexstr(cn_neighbors_hex, f"{c_name}_CN_NEIGHBORS")
    print("")
    dump_wrapped_cpp_hexstr(cn_row_ptr_hex, f"{c_name}_CN_ROW_PTR")
    print("")
    dump_wrapped_cpp_hexstr(permutation_hex, f"{c_name}_PERMUTATION")
    print("")
    dump_wrapped_cpp_hexstr(vn_col_ptr_hex, f"{c_name}_VN_COL_PTR")


def main():
    sf2 = build_matrix(
        d.SUBMATRIX_A_SF_2_NON_ZERO_ENTRIES,
        d.SUBMATRIX_B_SF_2_NON_ZERO_ENTRIES,
        d.SUBMATRIX_C_SF_2_NON_ZERO_ENTRIES,
        d.SUBMATRIX_D_SF_2_NON_ZERO_ENTRIES,
        d.SUBMATRIX_E_SF_2_NON_ZERO_ENTRIES,
        d.SUBMATRIX_T_SF_2_NON_ZERO_ENTRIES,
        600,
        1200,
    )

    sf3 = build_matrix(
        d.SUBMATRIX_A_SF_3_NON_ZERO_ENTRIES,
        d.SUBMATRIX_B_SF_3_NON_ZERO_ENTRIES,
        d.SUBMATRIX_C_SF_3_NON_ZERO_ENTRIES,
        d.SUBMATRIX_D_SF_3_NON_ZERO_ENTRIES,
        d.SUBMATRIX_E_SF_3_NON_ZERO_ENTRIES,
        d.SUBMATRIX_T_SF_3_NON_ZERO_ENTRIES,
        274,
        548,
    )

    dump_matrix(sf2, "GPS_L1C_LDPC_SF2")
    print("")
    print("")
    print("")
    dump_matrix(sf3, "GPS_L1C_LDPC_SF3")


if __name__ == "__main__":
    main()
