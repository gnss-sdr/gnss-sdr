/*!
 * \file gps_pcode.h
 * \brief Defines functions for dealing with the GPS P COde
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_PCODE_H_
#define GNSS_SDR_GPS_PCODE_H_

#include <vector>

//!Generates the GPS X1A code as a vector of short integers
void gps_x1a_code_gen(std::vector< short > & _dest );

//!Generates the GPS X1B code as a vector of short integers
void gps_x1b_code_gen(std::vector< short > & _dest );

//!Generates the GPS X2A code as a vector of short integers
void gps_x2a_code_gen(std::vector< short > & _dest );

//!Generates the GPS X2B code as a vector of short integers
void gps_x2b_code_gen(std::vector< short > & _dest );

//!Generates the GPS P Code at a given TOW


#endif /* GNSS_SDR_GPS_PCODE_H_ */

