/*!
 * \file gps_pcode_generator.cc
 * \brief Implementation of a pcode generator conforming to the long_code_interface
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

#include "gps_pcode_generator.h"
#include "GPS_P_CODE.h"

GpsPCodeGenerator::GpsPCodeGenerator( int sv )
{
    set_prn( sv );
}

bool GpsPCodeGenerator::get_chips( uint64_t first_chip_index,
        unsigned int num_chips, std::vector< short >& dest )
{

    d_code_gen.get_chips( d_sv, first_chip_index, num_chips, dest );

    return true;

}

void GpsPCodeGenerator::set_prn( int sv )
{
    if( sv < 1 or sv > 210 )
    {
        throw "Error: sv out of range. Should be 1 <= sv <= 210";
    }

    d_sv = sv;
}

uint64_t GpsPCodeGenerator::get_code_length(void) const
{
    return GPS_P_CODE_LENGTH_CHIPS;
}
