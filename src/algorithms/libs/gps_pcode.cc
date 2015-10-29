/*!
 * \file gps_pcode.cc
 * \brief Implements functions for dealing with the GPS P COde
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

#include "gps_pcode.h"
#include "GPS_P_CODE.h"
#include <gnuradio/digital/lfsr.h>


//!
// From ICD GPS 200C
// Note that the gnuradio lfsr convention is the reverse of theat in the
// ICD - In the ICD the MSB is the output and the new bit is input at
// the LSB, while the opposite holds in gnuradio. Therefore we need to
// reverse the polynomials and seeds in gnuradio.
//
// Example:
// x1 Polynomia;:
//  12    11    8     6
// x  +  x  +  x  +  x  +  1
//
// We need to reverse this polynomial:
//  12    6     4
// x  +  x  +  x  +  x  + 1
//
// And we need to omit the highest order bit ( this is the value being
// computed) to obtain the mask:
//  12        8        4        0 <-- power of x
// [1]  0 0 0 0  0 1 0 1  0 0 1 1 <-- binary representation
//   0x    0        5        3    <-- hex representation
const uint32_t X1A_MASK = 0x053;

//! Similarly the seed value is given in the ICD as:
//  0 0 1 0  0 1 0 0  1 0 0 0
//  So we need to reverse it:
//  0 0 0 1  0 0 1 0   0 1 0 0
//0x   1        2         4
const uint32_t X1A_SEED = 0x124;

const uint32_t X1B_MASK = 0x0C9F;
const uint32_t X1B_SEED = 0x2AA;

const uint32_t X2A_MASK = 0x0BBF;
const uint32_t X2A_SEED = 0xA49;

const uint32_t X2B_MASK = 0x0719;
const uint32_t X2B_SEED = 0x2AA;

const uint32_t XX_REG_LEN = 11;


//!Generates the GPS X1A code as a vector of short integers
void gps_x1a_code_gen(std::vector< short > & _dest )
{
    gr::digital::lfsr sr( X1A_MASK, X1A_SEED, XX_REG_LEN );

    _dest.resize( GPS_X1A_CODE_LENGTH );

    for( unsigned int ii = 0; ii < GPS_X1A_CODE_LENGTH; ++ii )
    {
        _dest[ii] = static_cast< short >( sr.next_bit() );
    }
}

//!Generates the GPS X1B code as a vector of short integers
void gps_x1b_code_gen(std::vector< short > & _dest )
{
    gr::digital::lfsr sr( X1B_MASK, X1B_SEED, XX_REG_LEN );

    _dest.resize( GPS_X1B_CODE_LENGTH );

    for( unsigned int ii = 0; ii < GPS_X1B_CODE_LENGTH; ++ii )
    {
        _dest[ii] = static_cast< short >( sr.next_bit() );
    }
}

//!Generates the GPS X2A code as a vector of short integers
void gps_x2a_code_gen(std::vector< short > & _dest )
{
    gr::digital::lfsr sr( X2A_MASK, X2A_SEED, XX_REG_LEN );

    _dest.resize( GPS_X2A_CODE_LENGTH );

    for( unsigned int ii = 0; ii < GPS_X2A_CODE_LENGTH; ++ii )
    {
        _dest[ii] = static_cast< short >( sr.next_bit() );
    }
}

//!Generates the GPS X2B code as a vector of short integers
void gps_x2b_code_gen(std::vector< short > & _dest )
{
    gr::digital::lfsr sr( X2B_MASK, X2B_SEED, XX_REG_LEN );

    _dest.resize( GPS_X2B_CODE_LENGTH );

    for( unsigned int ii = 0; ii < GPS_X2B_CODE_LENGTH; ++ii )
    {
        _dest[ii] = static_cast< short >( sr.next_bit() );
    }
}





