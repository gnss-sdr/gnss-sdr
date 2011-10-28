/*!
 * \file gps_telemetry.h
 * \brief GPS L1 C/A telemetry processing
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */
#ifndef GPS_TELEMETRY_H_
#define GPS_TELEMETRY_H_

#define _lrotl(X,N)             ((X << N) ^ (X >> (32-N)))      //!< Used in the parity check algorithm

#include "GPS_L1_CA.h"

bool gps_word_parityCheck(unsigned int gpsword);
char bit_mask(int num_bits);
char bit_select(int num_bit);
void print_gps_word_bytes(unsigned int GPS_word);

#endif
