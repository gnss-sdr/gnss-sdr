/*!
 * \file gnss_frequencies.h
 * \brief  GNSS Frequencies
 * \author Carles Fernandez, 2017. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GNSS_FREQUENCIES_H
#define GNSS_SDR_GNSS_FREQUENCIES_H

#include <string>
#include <unordered_map>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


constexpr double FREQ1 = 1.57542e9;       //!<  L1/E1  frequency (Hz)
constexpr double FREQ2 = 1.22760e9;       //!<  L2     frequency (Hz)
constexpr double FREQ5 = 1.17645e9;       //!<  L5/E5a frequency (Hz)
constexpr double FREQ6 = 1.27875e9;       //!<  E6/LEX frequency (Hz)
constexpr double FREQ7 = 1.20714e9;       //!<  E5b    frequency (Hz)
constexpr double FREQ8 = 1.191795e9;      //!<  E5a+b  frequency (Hz)
constexpr double FREQ9 = 2.492028e9;      //!<  S      frequency (Hz)
constexpr double FREQ1_GLO = 1.60200e9;   //!<  GLONASS G1 base frequency (Hz)
constexpr double DFRQ1_GLO = 0.56250e6;   //!<  GLONASS G1 bias frequency (Hz/n)
constexpr double FREQ2_GLO = 1.24600e9;   //!<  GLONASS G2 base frequency (Hz)
constexpr double DFRQ2_GLO = 0.43750e6;   //!<  GLONASS G2 bias frequency (Hz/n)
constexpr double FREQ3_GLO = 1.202025e9;  //!<  GLONASS G3 frequency (Hz)
constexpr double FREQ1_BDS = 1.561098e9;  //!<  BeiDou B1 frequency (Hz)
constexpr double FREQ2_BDS = 1.20714e9;   //!<  BeiDou B2 frequency (Hz)
constexpr double FREQ3_BDS = 1.26852e9;   //!<  BeiDou B3 frequency (Hz)

const std::unordered_map<std::string, double> SIGNAL_FREQ_MAP = {
    {"1C", FREQ1},
    {"2S", FREQ2},
    {"L5", FREQ5},
    {"1B", FREQ1},
    {"5X", FREQ5},
    {"E6", FREQ6},
    {"7X", FREQ7},
    {"1G", FREQ1_GLO},
    {"2G", FREQ2_GLO},
    {"B1", FREQ1_BDS},
    {"B2", FREQ2_BDS},
    {"B3", FREQ3_BDS},
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_FREQUENCIES_H
