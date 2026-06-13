"""
gps_l1c_ranging_codes.py

Generates GPS L1C ranging codes for PRNs 1 through 64 given the configuration tables,
in a format appropriate for direct insertion into C++ source code.

José Antonio Mayo, 2026. contact@tatjam.eu

-----------------------------------------------------------------------------

GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
This file is part of GNSS-SDR.

Copyright (C) 2022  (see AUTHORS file for a list of contributors)
SPDX-License-Identifier: GPL-3.0-or-later

-----------------------------------------------------------------------------
"""

# Contains x**2 mod 10223 for all needed numbers
legendre_residues = {x * x % 10223 for x in range(10223)}

# L(0) = 0
# L(t) = 1 if x exists s.t. t mod 10223 = x**2 mod 10223
# L(t) = 0 otherwise
legendre_lut = [1 if t in legendre_residues else 0 for t in range(10223)]
legendre_lut[0] = 0


def weil_code(w) -> list[int]:
    # Note that a XOR b for booleans is just "not equals"
    return [
        1 if legendre_lut[t] != legendre_lut[(t + w) % 10223] else 0
        for t in range(10223)
    ]


def main():
    l1c_p_str_chunks = {}
    l1c_d_str_chunks = {}

    for idx in range(0, 63):
        prn = idx + 1
        # Bare 10223 bit weil codes
        l1c_p_seq = weil_code(GPS_L1C_P_WEIL_INDICES[idx])
        l1c_d_seq = weil_code(GPS_L1C_D_WEIL_INDICES[idx])

        # Full ranging codes with inserted sequence, note that the insertion index is 0-indexed
        inserted_sequence = [0, 1, 1, 0, 1, 0, 0]
        p_idx = GPS_L1C_P_INSERTION_INDICES[idx]
        d_idx = GPS_L1C_D_INSERTION_INDICES[idx]

        l1c_p_seq[p_idx:p_idx] = inserted_sequence
        l1c_d_seq[d_idx:d_idx] = inserted_sequence

        # As integers
        l1c_p_seq_int = int("".join(map(str, l1c_p_seq)), 2)
        l1c_d_seq_int = int("".join(map(str, l1c_d_seq)), 2)

        # NOTE: We have 10230 bits, which fit in 2258 hex chars, but one of the chars is "half-full".
        #       To simplify the C++ code, we shift left 2 bits, so that the last hex char contains the
        #       zero padding instead of the first one
        raw_p_str = f"{l1c_p_seq_int<<2:02558X}"
        raw_d_str = f"{l1c_d_seq_int<<2:02558X}"
        p_str_chunks = [raw_p_str[i : i + 73] for i in range(0, len(raw_p_str), 73)]
        d_str_chunks = [raw_d_str[i : i + 73] for i in range(0, len(raw_d_str), 73)]

        l1c_p_str_chunks[prn] = p_str_chunks
        l1c_d_str_chunks[prn] = d_str_chunks

    p = [
        (l1c_p_str_chunks, "GPS_L1C_P_RANGE_CODE"),
        (l1c_d_str_chunks, "GPS_L1C_D_RANGE_CODE"),
    ]

    print(
        "constexpr size_t GPS_L1C_RANGE_CODE_STR_LENGTH = 2558; //!< Does not include the null terminator"
    )
    for chunk_array, name in p:
        # We dump the raw chip sequences written in hexadecimal string, but because 10230 is not
        # divisible by 4, the last hexadecimal symbol contains 0 padding at its LSB!
        print(
            f"constexpr char {name}[GPS_L1C_NUMBER_OF_PRN][GPS_L1C_RANGE_CODE_STR_LENGTH + 1] = {{"
        )
        for idx in range(0, 63):
            prn = idx + 1
            chunks = chunk_array[prn]
            for chunk in chunks[:-1]:
                print(f'    "{chunk}"')

            print(f'    "{chunks[-1]}", // PRN {prn}')

        print("};")


###############################################################
# Raw data tables taken from IS-GPS-800J 01-AUG-2022

# Index into the array is [PRN - 1]
GPS_L1C_P_WEIL_INDICES = [
    5111,
    5109,
    5108,
    5106,
    5103,
    5101,
    5100,
    5098,
    5095,
    5094,
    5093,
    5091,
    5090,
    5081,
    5080,
    5069,
    5068,
    5054,
    5044,
    5027,
    5026,
    5014,
    5004,
    4980,
    4915,
    4909,
    4893,
    4885,
    4832,
    4824,
    4591,
    3706,
    5092,
    4986,
    4965,
    4920,
    4917,
    4858,
    4847,
    4790,
    4770,
    4318,
    4126,
    3961,
    3790,
    4911,
    4881,
    4827,
    4795,
    4789,
    4725,
    4675,
    4539,
    4535,
    4458,
    4197,
    4096,
    3484,
    3481,
    3393,
    3175,
    2360,
    1852,
]

# Index into the array is [PRN - 1]
GPS_L1C_P_INSERTION_INDICES = [
    412,
    161,
    1,
    303,
    207,
    4971,
    4496,
    5,
    4557,
    485,
    253,
    4676,
    1,
    66,
    4485,
    282,
    193,
    5211,
    729,
    4848,
    982,
    5955,
    9805,
    670,
    464,
    29,
    429,
    394,
    616,
    9457,
    4429,
    4771,
    365,
    9705,
    9489,
    4193,
    9947,
    824,
    864,
    347,
    677,
    6544,
    6312,
    9804,
    278,
    9461,
    444,
    4839,
    4144,
    9875,
    197,
    1156,
    4674,
    10035,
    4504,
    5,
    9937,
    430,
    5,
    355,
    909,
    1622,
    6284,
]

# Index into the array is [PRN - 1]
GPS_L1C_D_WEIL_INDICES = [
    5097,
    5110,
    5079,
    4403,
    4121,
    5043,
    5042,
    5104,
    4940,
    5035,
    4372,
    5064,
    5084,
    5048,
    4950,
    5019,
    5076,
    3736,
    4993,
    5060,
    5061,
    5096,
    4983,
    4783,
    4991,
    4815,
    4443,
    4769,
    4879,
    4894,
    4985,
    5056,
    4921,
    5036,
    4812,
    4838,
    4855,
    4904,
    4753,
    4483,
    4942,
    4813,
    4957,
    4618,
    4669,
    4969,
    5031,
    5038,
    4740,
    4073,
    4843,
    4979,
    4867,
    4964,
    5025,
    4579,
    4390,
    4763,
    4612,
    4784,
    3716,
    4703,
    4851,
]

# Index into the array is [PRN - 1]
GPS_L1C_D_INSERTION_INDICES = [
    181,
    359,
    72,
    1110,
    1480,
    5034,
    4622,
    1,
    4547,
    826,
    6284,
    4195,
    368,
    1,
    4796,
    523,
    151,
    713,
    9850,
    5734,
    34,
    6142,
    190,
    644,
    467,
    5384,
    801,
    594,
    4450,
    9437,
    4307,
    5906,
    378,
    9448,
    9432,
    5849,
    5547,
    9546,
    9132,
    403,
    3766,
    3,
    684,
    9711,
    333,
    6124,
    10216,
    4251,
    9893,
    9884,
    4627,
    4449,
    9798,
    985,
    4272,
    126,
    10024,
    434,
    1029,
    561,
    289,
    638,
    4353,
]

if __name__ == "__main__":
    main()
