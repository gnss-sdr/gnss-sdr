/*!
 * \file pcode_generation_test.cc
 * \brief  This file implements tests for the generation of the GPS P code
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

#include <gtest/gtest.h>
#include <vector>
#include "gps_pcode.h"

TEST(PCodeGenTest, X1ATest)
{
    std::vector< short > dest;

    gps_x1a_code_gen( dest );

    // From Octave:
    std::vector< short > last12Bits = { 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0 };

    for( unsigned int ii = 0; ii < 12; ++ii )
    {
        EXPECT_EQ( dest[ dest.size()-12+ ii ], last12Bits[ii] );
    }

}

TEST(PCodeGenTest, X1BTest)
{
    std::vector< short > dest;

    gps_x1b_code_gen( dest );

    // From Octave:
    std::vector< short > last12Bits = { 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0 };

    for( unsigned int ii = 0; ii < 12; ++ii )
    {
        EXPECT_EQ( dest[ dest.size()-12+ ii ], last12Bits[ii] );
    }

}


TEST(PCodeGenTest, X2ATest)
{
    std::vector< short > dest;

    gps_x2a_code_gen( dest );

    // From Octave:
    std::vector< short > last12Bits = {0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1};

    for( unsigned int ii = 0; ii < 12; ++ii )
    {
        EXPECT_EQ( dest[ dest.size()-12+ ii ], last12Bits[ii] );
    }

}


TEST(PCodeGenTest, X2BTest)
{
    std::vector< short > dest;

    gps_x2b_code_gen( dest );

    // From Octave:
    std::vector< short > last12Bits = {0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0};

    for( unsigned int ii = 0; ii < 12; ++ii )
    {
        EXPECT_EQ( dest[ dest.size()-12+ ii ], last12Bits[ii] );
    }

}


TEST(PCodeGenTest, FirstChipsTest)
{
    std::vector< short > dest;

    std::vector< short > x1a;
    std::vector< short > x1b;
    std::vector< short > x2a;
    std::vector< short > x2b;

    gps_x1a_code_gen( x1a );
    gps_x1b_code_gen( x1b );
    gps_x2a_code_gen( x2a );
    gps_x2b_code_gen( x2b );

    GPS_P_Code_Generator pcode_gen;

    unsigned int num_chips = 24;

    int sv = 1;

    pcode_gen.get_chips( sv, 0, num_chips, dest );

    EXPECT_EQ( dest.size(), num_chips );

    for( unsigned ii = 0; ii < num_chips; ++ii )
    {
        short res = x1a[ii] ^ x1b[ii] ^ x2a[ii] ^ x2b[ii];

        EXPECT_EQ( dest[ii], res );
    }

}

