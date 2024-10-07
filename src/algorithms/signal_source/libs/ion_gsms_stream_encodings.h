/*!
 * \file ion_gsms_stream_encodings.h
 * \brief Implements look up tables for all encodings in the standard
 * \author Víctor Castillo Agüero, 2024. victorcastilloaguero(at)gmail.com
 *
 * These tables are taken from the stardard's official document.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ION_GSMS_STREAM_ENCODINGS_H
#define GNSS_SDR_ION_GSMS_STREAM_ENCODINGS_H

#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */

namespace GnssMetadata
{

using StreamEncoding = unsigned char;

namespace StreamEncodings
{

constexpr unsigned char SIGN = 0;
constexpr unsigned char OB = 1;
constexpr unsigned char SM = 2;
constexpr unsigned char MS = 3;
constexpr unsigned char TC = 4;
constexpr unsigned char OG = 5;
constexpr unsigned char OBA = 6;
constexpr unsigned char SMA = 7;
constexpr unsigned char MSA = 8;
constexpr unsigned char TCA = 9;
constexpr unsigned char OGA = 10;
constexpr unsigned char FP = 11;

}  // namespace StreamEncodings

inline StreamEncoding encoding_from_string(const std::string& str)
{
    if (str == "SIGN")
        {
            return StreamEncodings::SIGN;
        }
    if (str == "OB")
        {
            return StreamEncodings::OB;
        }
    if (str == "SM")
        {
            return StreamEncodings::SM;
        }
    if (str == "MS")
        {
            return StreamEncodings::MS;
        }
    if (str == "TC")
        {
            return StreamEncodings::TC;
        }
    if (str == "OG")
        {
            return StreamEncodings::OG;
        }
    if (str == "OBA")
        {
            return StreamEncodings::OBA;
        }
    if (str == "SMA")
        {
            return StreamEncodings::SMA;
        }
    if (str == "MSA")
        {
            return StreamEncodings::MSA;
        }
    if (str == "TCA")
        {
            return StreamEncodings::TCA;
        }
    if (str == "OGA")
        {
            return StreamEncodings::OGA;
        }
    if (str == "FP")
        {
            return StreamEncodings::FP;
        }
    return 0;
}

template <typename T>
inline T two_bit_look_up[11][4]{
    {},              // [0]
    {-2, -1, 0, 1},  // [1   /*OB*/]
    {0, 1, 0, -1},   // [2   /*SM*/]
    {0, 0, 1, -1},   // [3   /*MS*/]
    {0, 1, -2, -1},  // [4   /*TC*/]
    {-2, -1, 1, 0},  // [5   /*OG*/]
    {-3, -1, 1, 3},  // [6  /*OBA*/]
    {1, 3, -1, -3},  // [7  /*SMA*/]
    {1, -1, 3, -3},  // [8  /*MSA*/]
    {1, 3, -3, -1},  // [9  /*TCA*/]
    {-3, -1, 3, 1},  // [10 /*OGA*/]
};

template <typename T>
inline T three_bit_look_up[11][8]{
    {},                            // [0]
    {-4, -3, -2, -1, 0, 1, 2, 3},  // [1   /*OB*/]
    {0, 1, 2, 3, 0, -1, -2, -3},   // [2   /*SM*/]
    {0, 0, 1, -1, 0, 0, 1, -1},    // [3   /*MS*/]
    {0, 1, 2, 3, -4, -3, -2, -1},  // [4   /*TC*/]
    {-4, -3, -1, -2, 3, 2, 0, 1},  // [5   /*OG*/]
    {-7, -5, -3, -1, 1, 3, 5, 7},  // [6  /*OBA*/]
    {1, 3, 5, 7, -1, -3, -5, -7},  // [7  /*SMA*/]
    {1, -1, 3, -3, 5, -5, 7, -7},  // [8  /*MSA*/]
    {1, 3, 5, 7, -7, -5, -3, -1},  // [9  /*TCA*/]
    {-7, -5, -1, -3, 7, 5, 1, 3},  // [10 /*OGA*/]
};

template <typename T>
inline T four_bit_look_up[11][16]{
    {},                                                              // [0]
    {-8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7},        // [1   /*OB*/]
    {0, 1, 2, 3, 4, 5, 6, 7, 0, -1, -2, -3, -4, -5, -6, -7},         // [2   /*SM*/]
    {0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1},            // [3   /*MS*/]
    {0, 1, 2, 3, 4, 5, 6, 7, -8, -7, -6, -5, -4, -3, -2, -1},        // [4   /*TC*/]
    {-8, -7, -5, -6, -1, -2, -4, -3, 7, 6, 4, 5, 0, 1, 3, 2},        // [5   /*OG*/]
    {-15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15},  // [6  /*OBA*/]
    {1, 3, 5, 7, 9, 11, 13, 15, -1, -3, -5, -7, -9, -11, -13, -15},  // [7  /*SMA*/]
    {1, -1, 3, -3, 5, -5, 7, -7, 9, -9, 11, -11, 13, -13, 15, -15},  // [8  /*MSA*/]
    {1, 3, 5, 7, 9, 11, 13, 15, -15, -13, -11, -9, -7, -5, -3, -1},  // [9  /*TCA*/]
    {-15, -13, -9, -11, -1, -3, -7, -5, 15, 13, 9, 11, 1, 3, 7, 5},  // [10 /*OGA*/]
};

template <typename T>
inline T five_bit_look_up[11][32]{
    {},                                                                                                                                      // [0]
    {-16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},           // [1   /*OB*/]
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -15},             // [2   /*SM*/]
    {0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1},                                // [3   /*MS*/]
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1},           // [4   /*TC*/]
    {-16, -15, -13, -14, -9, -10, -12, -11, -1, -2, -4, -3, -8, -7, -5, -6, 15, 14, 12, 13, 8, 9, 11, 10, 0, 1, 3, 2, 7, 6, 4, 5},           // [5   /*OG*/]
    {-31, -29, -27, -25, -23, -21, -19, -17, -15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31},  // [6  /*OBA*/]
    {1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, -1, -3, -5, -7, -9, -11, -13, -15, -17, -19, -21, -23, -25, -27, -29, -31},  // [7  /*SMA*/]
    {1, -1, 3, -3, 5, -5, 7, -7, 9, -9, 11, -11, 13, -13, 15, -15, 17, -17, 19, -19, 21, -21, 23, -23, 25, -25, 27, -27, 29, -29, 31, -31},  // [8  /*MSA*/]
    {1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, -31, -29, -27, -25, -23, -21, -19, -17, -15, -13, -11, -9, -7, -5, -3, -1},  // [9  /*TCA*/]
    {-31, -29, -25, -27, -17, -19, -23, -21, -1, -3, -7, -5, -15, -13, -9, -11, 31, 29, 25, 27, 17, 19, 23, 21, 1, 3, 7, 5, 15, 13, 9, 11},  // [10 /*OGA*/]
};

}  // namespace GnssMetadata


/** \} */
/** \} */
#endif  // GNSS_SDR_ION_GSMS_STREAM_ENCODINGS_H
