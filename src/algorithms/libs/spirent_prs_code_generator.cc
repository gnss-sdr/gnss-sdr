/*!
 * \file spirent_prs_code_generator.cc
 * \brief Implementation of a spirent prs code generator conforming to the long_code_interface
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

#include "spirent_prs_code_generator.h"

SpirentPrsCodeGenerator::SpirentPrsCodeGenerator( int sv, bool is_e1 )
: d_sv( sv ),
    d_downsample_factor( is_e1 ? 4 : 2 ),
    d_offset( sv % 2 )
{
    set_prn( sv );
}

bool SpirentPrsCodeGenerator::get_chips( uint64_t first_chip_index,
        unsigned int num_chips, std::vector< short >& dest )
{
    // First get the pcode and store it locally:
    // For the PRS we have two possibilities:
    // 1) E1 PRS - 1/4 of the rate of the P code
    // 2) E6 PRS - 1/2 the rate of the P code
    //
    // For E1 we have a downsample factor of 4, while the factor is 2
    // for E6. The offset is chosen by spirent as 0 for even PRNS and 1
    // for odd PRNS
    d_code_gen.get_chips( d_sv, d_downsample_factor*first_chip_index + d_offset,
            num_chips*d_downsample_factor, d_pcode_store );

    dest.resize( num_chips );

    // Now downsample:
    for( unsigned int ii = 0, jj=0; ii < num_chips; ii++, jj+=d_downsample_factor )
    {
        dest[ii] = d_pcode_store[jj];
    }

    return true;
}

void SpirentPrsCodeGenerator::set_prn( int sv )
{
    if( sv < 1 || sv > 50 )
    {
        throw "Error: sv out of range, should be 1<= prn <= 50";
    }

    d_sv = sv;

    d_offset = d_sv % 2;
}
