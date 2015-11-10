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
#include <algorithm>
#include <glog/logging.h>


//!
// From ICD GPS 200C
// Note that the gnuradio lfsr convention is the reverse of that in the
// ICD - In the ICD the MSB is the output and the new bit is input at
// the LSB, while the opposite holds in gnuradio. Therefore we need to
// reverse the polynomials and seeds in gnuradio.
//
// Example:
// x1a Polynomial:
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

GPS_P_Code_Generator::GPS_P_Code_Generator()
{
    gps_x1a_code_gen( d_x1a );

    gps_x1b_code_gen( d_x1b );
    short last_val = d_x1b.back();
    d_x1b.resize( GPS_X1B_CODE_LENGTH + X1B_EXTRA_LENGTH, last_val );

    gps_x2a_code_gen( d_x2a );
    last_val = d_x2a.back();
    d_x2a.resize( GPS_X2A_CODE_LENGTH + X2A_EXTRA_LENGTH, last_val );

    gps_x2b_code_gen( d_x2b );
    last_val = d_x2b.back();
    d_x2b.resize( GPS_X2B_CODE_LENGTH + X2B_EXTRA_LENGTH, last_val );
}

GPS_P_Code_Generator::~GPS_P_Code_Generator()
{
    //Nothing doing...
}


const int64_t GPS_P_Code_Generator::X1_EPOCH_CHIPS = 15345000;
const int64_t GPS_P_Code_Generator::X2_EPOCH_CHIPS = 15345037;

const unsigned int GPS_P_Code_Generator::X1A_EPOCHS_PER_X1_EPOCH = 3750;
const unsigned int GPS_P_Code_Generator::X1B_EPOCHS_PER_X1_EPOCH = 3749;

const unsigned int GPS_P_Code_Generator::X2A_EPOCHS_PER_X2_EPOCH = 3750;
const unsigned int GPS_P_Code_Generator::X2B_EPOCHS_PER_X2_EPOCH = 3749;

const unsigned int GPS_P_Code_Generator::X1_EPOCHS_PER_WEEK = 403200;
const unsigned int GPS_P_Code_Generator::X2_EPOCHS_PER_WEEK = 403200;

const unsigned int GPS_P_Code_Generator::X1B_EXTRA_LENGTH = 343;
const unsigned int GPS_P_Code_Generator::X2A_EXTRA_LENGTH = 1069;
const unsigned int GPS_P_Code_Generator::X2B_EXTRA_LENGTH = 965;

const unsigned int GPS_P_Code_Generator::X2A_EPOCHS_LAST_X2_EPOCH = 104;
const unsigned int GPS_P_Code_Generator::X2B_EPOCHS_LAST_X2_EPOCH = 104;

void GPS_P_Code_Generator::get_chips( int prn, uint64_t first_chip_index, unsigned num_chips,
        std::vector< short > &_dest ) const
{
    // A couple of quick checks:
    if( prn < 1 or prn > 210 )
    {
        LOG(ERROR) << "Invalid SV number: " << prn
                   << " should be in the range 1 to 210 (inclusive)";
    }

    if( first_chip_index > GPS_P_CODE_LENGTH_CHIPS )
    {
        LOG(ERROR) << "Invalid first chip index, should be less than "
                   << GPS_P_CODE_LENGTH_CHIPS;
    }

    // Not going to allow more than one x1 epoch, 1.5 s
    if( num_chips > X1_EPOCH_CHIPS )
    {
        LOG(ERROR) << "Number of chips (" << num_chips
                   << ") is too great. Should be less than "
                   << X1_EPOCH_CHIPS;
    }

    // OK now we can proceed:
    // IS GPS 200H - The first 37 PRN codes are unique, identified by sv
    // below. For PRN values greater than 37 we create the code for
    // (prn-1) % 37 + 1 and delay it by floor( (prn-1)/37 ) days
    int sv = ( prn - 1 ) % 37 + 1;
    int num_days = (prn-1) / 37;
    uint64_t day_shift = num_days*10230000LL*24LL*3600LL;
    first_chip_index = first_chip_index + day_shift;
    first_chip_index %= GPS_P_CODE_LENGTH_CHIPS;

    //Ensure we have space:
    _dest.resize( num_chips );


    // Establish the starting phase:
    // 1) account for the delay of the x2 register
    int64_t first_chip_index_x2 = first_chip_index - sv;
    while( first_chip_index_x2 < 0 )
    {
        first_chip_index_x2 += GPS_P_CODE_LENGTH_CHIPS;
    }

    int64_t x1_epoch = first_chip_index/X1_EPOCH_CHIPS;
    int64_t x2_epoch = first_chip_index_x2/X2_EPOCH_CHIPS;

    int64_t x1a_epoch = (first_chip_index - x1_epoch*X1_EPOCH_CHIPS)/GPS_X1A_CODE_LENGTH;
    int64_t x1b_epoch = (first_chip_index - x1_epoch*X1_EPOCH_CHIPS)/GPS_X1B_CODE_LENGTH;

    int64_t x2a_epoch = (first_chip_index_x2 - x2_epoch*X2_EPOCH_CHIPS)/GPS_X2A_CODE_LENGTH;
    int64_t x2b_epoch = (first_chip_index_x2 - x2_epoch*X2_EPOCH_CHIPS)/GPS_X2B_CODE_LENGTH;

    // Now we deal with the  x1a register, the easiest case
    uint64_t x1a_ind = first_chip_index % GPS_X1A_CODE_LENGTH;
    unsigned dest_ind = 0;

    while( dest_ind < num_chips )
    {
        unsigned chips_to_gen = std::min( num_chips-dest_ind,
                static_cast<unsigned>( GPS_X1A_CODE_LENGTH - x1a_ind ) );

        for( unsigned ii = 0; ii < chips_to_gen; ++ii, ++x1a_ind, ++dest_ind )
        {
            _dest[dest_ind] = d_x1a[ x1a_ind ];
        }
        x1a_ind = x1a_ind % GPS_X1A_CODE_LENGTH;
    }


    // Now we deal with x1b:
    uint64_t x1b_ind = first_chip_index - x1_epoch*X1_EPOCH_CHIPS - x1b_epoch*GPS_X1B_CODE_LENGTH;
    dest_ind = 0;

    uint32_t this_code_period;

    while( dest_ind < num_chips )
    {
        this_code_period = GPS_X1B_CODE_LENGTH;
        if( x1b_epoch >= X1B_EPOCHS_PER_X1_EPOCH - 1 )
        {
            this_code_period = GPS_X1B_CODE_LENGTH + X1B_EXTRA_LENGTH;
            if( x1b_epoch == X1B_EPOCHS_PER_X1_EPOCH )
            {
                x1b_ind += GPS_X1B_CODE_LENGTH;
            }
            x1b_epoch = X1B_EPOCHS_PER_X1_EPOCH - 1;
        }

        unsigned chips_to_gen = std::min(num_chips-dest_ind,
                static_cast< unsigned >( this_code_period - x1b_ind ) );

        for( unsigned int ii = 0; ii < chips_to_gen; ++ii, ++x1b_ind, ++dest_ind )
        {
            _dest[dest_ind] ^= d_x1b[ x1b_ind ];
        }

        x1b_ind %= this_code_period;
        if( x1b_ind == 0 )
        {
            x1b_epoch++;
            x1b_epoch %= X1B_EPOCHS_PER_X1_EPOCH;
        }
    }

    // Now x2a is similar - but we need to account for the last x1 epoch
    // of the week where x2a and x2b are held at their last values
    uint64_t x2a_ind = first_chip_index_x2 - x2_epoch*X2_EPOCH_CHIPS - x2a_epoch*GPS_X2A_CODE_LENGTH;

    uint64_t local_x2_epoch = x2_epoch;
    dest_ind = 0;

    while( dest_ind < num_chips )
    {
        // Three possibilities for the code period:
        // 1) Normal case - use GPS_X2A_CODE_LENGTH
        // 2) At the end of an X2 epoch - extend by 37 chips
        // 3) At the end of the week, only 104 X2A epochs, then hold for 1069 chips
        this_code_period = GPS_X2A_CODE_LENGTH ;

        unsigned int max_num_epochs = X2A_EPOCHS_PER_X2_EPOCH;

        unsigned int extra_code_period = 37; // 37 extra chips at the end of each X2 epoch

        if( local_x2_epoch == X2_EPOCHS_PER_WEEK - 1 )
        {
            max_num_epochs = X2A_EPOCHS_LAST_X2_EPOCH;
            extra_code_period = X2A_EXTRA_LENGTH;
        }

        if( x2a_epoch >= max_num_epochs-1 )
        {
            this_code_period = this_code_period + extra_code_period;
            if( x2a_epoch == max_num_epochs )
            {
                x2a_ind += GPS_X2A_CODE_LENGTH;
            }
            x2a_epoch = max_num_epochs-1;
        }

        unsigned int chips_to_gen = std::min( num_chips - dest_ind,
                static_cast< unsigned>( this_code_period - x2a_ind ) );

        for( unsigned int ii=0; ii < chips_to_gen; ++ii, ++x2a_ind, ++dest_ind )
        {
            _dest[dest_ind] ^= d_x2a[ x2a_ind ];
        }

        x2a_ind %= this_code_period;
        if( x2a_ind == 0 )
        {
            x2a_epoch++;
            x2a_epoch %= max_num_epochs;

            if( x2a_epoch == 0 )
            {
                ++local_x2_epoch;
                local_x2_epoch %= X2_EPOCHS_PER_WEEK;
            }
        }


    }

    // Finally x2b
    uint64_t x2b_ind = first_chip_index_x2 - x2_epoch*X2_EPOCH_CHIPS - x2b_epoch*GPS_X2B_CODE_LENGTH;

    local_x2_epoch = x2_epoch;

    dest_ind = 0;

    while( dest_ind < num_chips )
    {
        this_code_period = GPS_X2B_CODE_LENGTH ;

        unsigned int max_num_epochs = X2B_EPOCHS_PER_X2_EPOCH;

        unsigned int extra_code_period = X1B_EXTRA_LENGTH + 37; // 37 extra chips at the end of each X2 epoch

        if( local_x2_epoch == X2_EPOCHS_PER_WEEK - 1 )
        {
            max_num_epochs = X2B_EPOCHS_LAST_X2_EPOCH;
            extra_code_period = X2B_EXTRA_LENGTH;
        }

        if( x2b_epoch >= max_num_epochs - 1 )
        {
            this_code_period = this_code_period + extra_code_period;
            if( x2b_epoch == max_num_epochs )
            {
                x2b_ind += GPS_X2B_CODE_LENGTH;
            }
            x2b_epoch = max_num_epochs - 1;
        }

        unsigned int chips_to_gen = std::min( num_chips - dest_ind,
                static_cast< unsigned >( this_code_period - x2b_ind ) );

        for( unsigned int ii=0; ii < chips_to_gen; ++ii, ++x2b_ind, ++dest_ind )
        {
            _dest[dest_ind] ^= d_x2b[ x2b_ind ];
        }

        x2b_ind %= this_code_period;
        if( x2b_ind == 0 )
        {
            x2b_epoch++;
            x2b_epoch %= max_num_epochs;

            if( x2b_epoch == 0 )
            {
                ++local_x2_epoch;
                local_x2_epoch %= X2_EPOCHS_PER_WEEK;
            }
        }

    }

    // All good, we should now have filled up dest:
    return;
}



