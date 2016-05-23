/*!
 * \file file_long_code_generator.cc
 * \brief Defines a code generator conforming to the long_code_interface that
 * reads the chips from a file.
 * \author Cillian O'Driscoll, 2016. cillian.odriscoll(at)gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2016  (see AUTHORS file for a list of contributors)
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

#include "file_long_code_generator.h"
#include <algorithm>
#include <glog/logging.h>
#include <iostream>


FileLongCodeGenerator::FileLongCodeGenerator( int sv, std::istream &is )
{
    set_prn( sv );

    initialise( is );
}

bool FileLongCodeGenerator::get_chips( uint64_t first_chip_index, unsigned int num_chips,
        std::vector< short >& dest )
{
    if( first_chip_index < d_first_chip_index ||
            first_chip_index > d_first_chip_index + d_num_chips ||
            first_chip_index + num_chips < d_first_chip_index )
    {
        LOG(INFO) << "Attempted to access chips outside of range. [" << first_chip_index
            << ", " << first_chip_index+num_chips << "] not in [" << d_first_chip_index
            << ", " << d_first_chip_index+d_num_chips << "]";
        return false;
    }

    uint64_t first_chip_to_return = std::max( d_first_chip_index,
            first_chip_index ) - d_first_chip_index;
    uint64_t last_chip_to_return = std::min( first_chip_index + num_chips,
            d_first_chip_index + d_num_chips ) - d_first_chip_index;

    uint64_t num_chips_to_return = last_chip_to_return - first_chip_to_return
        + 1;

    dest.resize( num_chips_to_return );

    LOG(INFO) << "Getting chips: " << first_chip_to_return
        << " to " << last_chip_to_return;

    std::copy( d_the_chips.begin() + first_chip_to_return,
            d_the_chips.begin() + last_chip_to_return + 1,
            dest.begin() );

    return true;
}

void FileLongCodeGenerator::set_prn( int sv )
{
    d_sv = sv;
}

uint64_t FileLongCodeGenerator::get_code_length( void ) const
{
    return d_code_length;
}

void FileLongCodeGenerator::initialise( std::istream &is )
{
    // go to the beginning of the stream:
    is.seekg(0);

    // The file format is:
    // FIRST_CHIP_INDEX
    // CODE_LENGTH
    // CHIP_0
    // CHIP_1
    // ...
    // CHIP_N-1
    // [Blank line]

    // Handy algorithm for counting lines
    uint64_t numLines =
        std::count(std::istreambuf_iterator<char>(is),
                std::istreambuf_iterator<char>(), '\n');

    if( numLines <= 3 )
    {
        LOG(ERROR) << "Invalid file format: expected at least 3 lines";
    }

    d_num_chips = numLines - 3; // First two lines and last blank line

    d_the_chips.resize( d_num_chips );

    is.seekg(0);

    is >> d_first_chip_index;
    is >> d_code_length;

    uint64_t ii = 0;

    while( is >> d_the_chips[ii++] ){};

    if( ii < d_num_chips )
    {
        LOG(ERROR) << "Expected " << d_num_chips << " chips, only read "
            << ii;
    }

}
