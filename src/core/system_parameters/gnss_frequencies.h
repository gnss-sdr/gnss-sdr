/*!
 * \file gnss_frequencies.h
 * \brief  GNSS Frequencies
 * \author Carles Fernandez, 2017. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GNSS_FREQUENCIES_H_
#define GNSS_SDR_GNSS_FREQUENCIES_H_

const double FREQ1 = 1.57542e9;       //!<  L1/E1  frequency (Hz)
const double FREQ2 = 1.22760e9;       //!<  L2     frequency (Hz)
const double FREQ5 = 1.17645e9;       //!<  L5/E5a frequency (Hz)
const double FREQ6 = 1.27875e9;       //!<  E6/LEX frequency (Hz)
const double FREQ7 = 1.20714e9;       //!<  E5b    frequency (Hz)
const double FREQ8 = 1.191795e9;      //!<  E5a+b  frequency (Hz)
const double FREQ9 = 2.492028e9;      //!<  S      frequency (Hz)
const double FREQ1_GLO = 1.60200e9;   //!<  GLONASS G1 base frequency (Hz)
const double DFRQ1_GLO = 0.56250e6;   //!<  GLONASS G1 bias frequency (Hz/n)
const double FREQ2_GLO = 1.24600e9;   //!<  GLONASS G2 base frequency (Hz)
const double DFRQ2_GLO = 0.43750e6;   //!<  GLONASS G2 bias frequency (Hz/n)
const double FREQ3_GLO = 1.202025e9;  //!<  GLONASS G3 frequency (Hz)
const double FREQ1_BDS = 1.561098e9;  //!<  BeiDou B1 frequency (Hz)
const double FREQ2_BDS = 1.20714e9;   //!<  BeiDou B2 frequency (Hz)
const double FREQ3_BDS = 1.26852e9;   //!<  BeiDou B3 frequency (Hz)

#endif
