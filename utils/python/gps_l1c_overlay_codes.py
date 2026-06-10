"""
gps_l1c_overlay_codes.py

Generates GPS L1C overlay codes for PRNs 1 through 64 given the configuration tables,
in a format appropriate for direct insertion into C++ source code.

José Antonio Mayo, 2026. contact@tatjam.eu

-----------------------------------------------------------------------------

GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
This file is part of GNSS-SDR.

Copyright (C) 2022  (see AUTHORS file for a list of contributors)
SPDX-License-Identifier: GPL-3.0-or-later

----------------------------------------------------------------------------
"""


def lfsr(registers0i, mi):
    registers = [int(x, 2) == 1 for x in f"{registers0i:011b}"]
    registers.reverse()

    # Note taps contains 12 bits, not 11
    taps = [int(x, 2) == 1 for x in f"{mi:012b}"]
    taps.reverse()

    out = []
    for _ in range(0, 1800):
        nval = None
        for tap in range(0, 11):
            if taps[tap + 1]:
                if nval is None:
                    nval = registers[tap]
                else:
                    nval = nval != registers[tap]

        registers.insert(0, nval)
        out.append(registers.pop())

    return int("".join(["1" if x else "0" for x in out]), 2)


def main():
    l1c_o_str_chunks = {}
    for idx in range(0, 63):
        prn = idx + 1
        # for idx in range(0, 63):
        m = S1_POLYNOMIAL_COEFFICIENTS[idx]
        registers0 = INITIAL_BITS[idx]

        seqi = lfsr(registers0, m)
        raw_str = f"{seqi:0450X}"
        chunks = [raw_str[i : i + 73] for i in range(0, len(raw_str), 73)]
        l1c_o_str_chunks[prn] = chunks

    print(
        "constexpr size_t GPS_L1C_OVERLAY_CODE_STR_LENGTH = 450; //!< Does not include the null terminator"
    )
    print(
        f"constexpr char GPS_L1C_OVERLAY_CODE[GPS_L1C_NUMBER_OF_PRN][GPS_L1C_OVERLAY_CODE_STR_LENGTH + 1] = {{"
    )

    for idx in range(0, 63):
        prn = idx + 1
        chunks = l1c_o_str_chunks[prn]
        for chunk in chunks[:-1]:
            print(f'    "{chunk}"')

        print(f'    "{chunks[-1]}", // PRN {prn}')

    print("};")


# Index into the array is [PRN - 1]
S1_POLYNOMIAL_COEFFICIENTS = [
    0o5111,
    0o5421,
    0o5501,
    0o5403,
    0o6417,
    0o6141,
    0o6351,
    0o6501,
    0o6205,
    0o6235,
    0o7751,
    0o6623,
    0o6733,
    0o7627,
    0o5667,
    0o5051,
    0o7665,
    0o6325,
    0o4365,
    0o4745,
    0o7633,
    0o6747,
    0o4475,
    0o4225,
    0o7063,
    0o4423,
    0o6651,
    0o4161,
    0o7237,
    0o4473,
    0o5477,
    0o6163,
    0o7223,
    0o6323,
    0o7125,
    0o7035,
    0o4341,
    0o4353,
    0o4107,
    0o5735,
    0o6741,
    0o7071,
    0o4563,
    0o5755,
    0o6127,
    0o4671,
    0o4511,
    0o4533,
    0o5357,
    0o5607,
    0o6673,
    0o6153,
    0o7565,
    0o7107,
    0o6211,
    0o4321,
    0o7201,
    0o4451,
    0o5411,
    0o5141,
    0o7041,
    0o6637,
    0o4577,
]

INITIAL_BITS = [
    0o3266,
    0o2040,
    0o1527,
    0o3307,
    0o3756,
    0o3026,
    0o0562,
    0o0420,
    0o3415,
    0o0337,
    0o0265,
    0o1230,
    0o2204,
    0o1440,
    0o2412,
    0o3516,
    0o2761,
    0o3750,
    0o2701,
    0o1206,
    0o1544,
    0o1774,
    0o0546,
    0o2213,
    0o3707,
    0o2051,
    0o3650,
    0o1777,
    0o3203,
    0o1762,
    0o2100,
    0o0571,
    0o3710,
    0o3535,
    0o3110,
    0o1426,
    0o0255,
    0o0321,
    0o3124,
    0o0572,
    0o1736,
    0o3306,
    0o1307,
    0o3763,
    0o1604,
    0o1021,
    0o2624,
    0o0406,
    0o0114,
    0o0077,
    0o3477,
    0o1000,
    0o3460,
    0o2607,
    0o2057,
    0o3467,
    0o0706,
    0o2032,
    0o1464,
    0o0520,
    0o1766,
    0o3270,
    0o0341,
]

if __name__ == "__main__":
    main()
