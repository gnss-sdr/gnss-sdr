"""
gps_l1c_toi_codes.py

Generates GPS L1C LSB TOI codes in a format appropriate for direct insertion
into C++ source code. MSB is assumed 0, thus MSB can then be deduced by sign
of correlation.

José Antonio Mayo, 2026. contact@tatjam.eu

-----------------------------------------------------------------------------

GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
This file is part of GNSS-SDR.

Copyright (C) 2022  (see AUTHORS file for a list of contributors)
SPDX-License-Identifier: GPL-3.0-or-later

----------------------------------------------------------------------------
"""


def toi_lfsr(registers0i):
    registers = [int(x, 2) == 1 for x in f"{registers0i:08b}"]
    registers.reverse()

    # Note taps contains 9 bits, not 8
    taps = [1, 1, 0, 0, 1, 1, 1, 1, 1]
    # taps.reverse()

    out = []
    for _ in range(0, 51):
        nval = None
        for tap in range(0, 8):
            if taps[tap + 1]:
                if nval is None:
                    nval = registers[tap]
                else:
                    nval = nval != registers[tap]

        registers.insert(0, nval)
        out.append(registers.pop())

    return int("".join(["1" if x else "0" for x in out]), 2)


def main():
    hypothesis_chunks = {}
    for lsb in range(0, 256):
        seqi = toi_lfsr(lsb)
        raw_str = f"{seqi:051b}"
        chunks = [raw_str[i : i + 73] for i in range(0, len(raw_str), 73)]
        hypothesis_chunks[lsb] = chunks

    print(
        f"constexpr char GPS_L1C_TOI_BCH_LSB_CODES[GPS_L1C_TOI_LSB_VALUES][GPS_L1C_TOI_BCH_BITS] = {{"
    )

    for lsb in range(0, 256):
        chunks = hypothesis_chunks[lsb]
        for chunk in chunks[:-1]:
            print(f'    "{chunk}"')

        print(f'    "{chunks[-1]}", // LSB 0b{lsb:08b}')

    print("};")


if __name__ == "__main__":
    main()

