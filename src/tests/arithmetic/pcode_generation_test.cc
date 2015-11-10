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
#include "GPS_P_CODE.h"
#include "gps_pcode.h"

// Test vectors from IS GPS 200 H
// Note there are some errors in that document as per:
//http://www.gps.gov/technical/icwg/meetings/2015/PIRN-IS-200H-001.pdf
// in particular:
// PRN :  OLD  : NEW
// 66  :  2111 : 6111
// 69  : 4166  : 0166
// 70  : 2251  : 6251

const std::vector< short > first_12_chips_octal = {
    04444, 04000, 04222, 04333, 04377, 04355, 04344, 04340, 04342, 04343,
    04343, 04343, 04343, 04343, 04343, 04343, 04343, 04343, 04343, 04343, 
    04343, 04343, 04343, 04343, 04343, 04343, 04343, 04343, 04343, 04343, 
    04343, 04343, 04343, 04343, 04343, 04343, 04343,  // <-- 37
    03373, 03757, 07545, 05440, 04402, 04023, 00233, 02337, 03375, 03754,
    03544, 07440, 01402, 06423, 01033, 02637, 07135, 05674, 00514, 06064,
    01210, 06726, 01171, 06656, 01105, 06660, // <-- 63
    /* IS GPS 200 H:
    05112, 00667, 02111, 05266, 04711, 04166, 02251, 05306, 04761, 02152,
    05247, 05736, 02575, 03054, 03604, 03520, 05472, 04417, 02025, 03230,
    05736, 04575, 02054, 03204, 03720, 05572, 04457, 04005, 02220, 03332,
    03777, 03555, // <- 95
    03444, 07400, 01422, 02433, 07037, 01635, 06534, 05074, 00614, 06124,
    01270, 02716, 05165, 00650, 06106, 05261, 06752, 05147, 00641, 06102,
    01263, 02713, 03167, 03651, 07506, 05461, 00412, 06027, 01231, 02736, // <-125
    07175, 01654, 06504, 01060, 02612, 07127, 05671, 04516, 04065, 04210,
    04326, 00371, 06356, 05345, 00740, 06142, 01243, 06703, 05163, 04653,
    04107, 04261, 00312, 02525, 07070, 01616, 02525, 07070, 03616, 07525, // <- 155
    05470, 04416, 04025, 04230, 00336, 06375, 01354, 06744, 05140, 04642,
    00103, 06263, 01313, 06767, 01151, 02646, 07101, 05662, 00513, 02067,
    03211, 03726, 03571, 03456, 03405, 03420, 05432, 00437, 06035, 01234, // <- 185
    01067, 06611, 05126, 04671, 00116, 06265, 01310, 06766, 01151, 02646,
    03101, 07662, 05513, 04467, 04011, 04226, 04331, 00376, 06355, 05344,
    00740, 06142, 01243, 06703, 01163 //<- 210 */
    /* PRN IS GPS 200 H 001:*/
    05112, 00667, 06111, 05266, 04711, 00166, 06251, 05306, 00761, 06152,
    01247, 01736, 02575, 03054, 03604, 07520, 05472, 00417, 02025, 07230,
    05736, 00575, 02054, 03204, 07720, 05572, 04457, 00005, 02220, 03332,
    03777, 03555, 03444, 07400, 01422, 02433, 07037, 01635, 06534, 05074,
    00614, 06124, 01270, 06716, 05165, 00650, 06106, 05261, 06752, 05147,
    00641, 06102, 01263, 02713, 03167, 03651, 07506, 05461, 00412, 06027,
    01231, 02736, 07175, 01654, 06504, 01060, 02612, 07127, 05671, 04516,
    04065, 04210, 04326, 00371, 06356, 05345, 00740, 06142, 01243, 06703,
    05163, 04653, 04107, 04261, 00312, 02525, 07070, 01616, 02525, 03070,
    03616, 07525, 05470, 04416, 04025, 04230, 00336, 06375, 01354, 06744,
    05140, 04642, 00103, 06263, 01313, 06767, 01151, 02646, 07101, 05662,
    00513, 02067, 03211, 03726, 03571, 03456, 03405, 07420, 05432, 00437,
    06035, 01234, // <- 185
    01067, 06611, 05126, 04671, 00116, 06265, 01310, 06766, 01151, 02646,
    03101, 07662, 05513, 04467, 04011, 04226, 04331, 00376, 06355, 05344,
    00740, 06142, 01243, 06703, 01163 //<- 210 */
};



TEST(PCodeGenTest, X1ATest)
{
    std::vector< short > dest;

    gps_x1a_code_gen( dest );

    // IS GPS 200 H
    std::vector< short > first_12_chips = { 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0 };

    for( unsigned int ii = 0; ii < 12; ++ii )
    {
        EXPECT_EQ( dest[ii], first_12_chips[ii] ) << "ii: " << ii;
    }

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

    // IS GPS 200 H
    std::vector< short > first_12_chips = { 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0 };

    for( unsigned int ii = 0; ii < 12; ++ii )
    {
        EXPECT_EQ( dest[ii], first_12_chips[ii] ) << "ii: " << ii;
    }
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

    // IS GPS 200 H
    std::vector< short > first_12_chips = { 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1};

    for( unsigned int ii = 0; ii < 12; ++ii )
    {
        EXPECT_EQ( dest[ii], first_12_chips[ii] ) << "ii: " << ii;
    }

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

    // IS GPS 200 H
    std::vector< short > first_12_chips = { 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0 };

    for( unsigned int ii = 0; ii < 12; ++ii )
    {
        EXPECT_EQ( dest[ii], first_12_chips[ii] ) << "ii: " << ii;
    }


    // From Octave :
    std::vector< short > last12Bits = {0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0};

    for( unsigned int ii = 0; ii < 12; ++ii )
    {
        EXPECT_EQ( dest[ dest.size()-12+ ii ], last12Bits[ii] );
    }

}


TEST(PCodeGenTest, FirstChipsTest1)
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
    int first_chip_index = sv;

    pcode_gen.get_chips( sv, first_chip_index, num_chips, dest );

    EXPECT_EQ( dest.size(), num_chips );

    for( unsigned ii = 0, jj = first_chip_index; ii < num_chips; ++ii, ++jj)
    {
        short res = x1a[jj] ^ x1b[jj] ^ x2a[jj-sv] ^ x2b[jj-sv];

        EXPECT_EQ( dest[ii], res );
    }

}

TEST(PCodeGenTest, First12ChipsAll )
{
    std::vector< short > dest;

    GPS_P_Code_Generator pcode_gen;

    unsigned int num_chips = 12;

    int num_err = 0;
    for( int sv = 1; sv <= 210; ++sv )
    {
        pcode_gen.get_chips( sv, 0, num_chips, dest );
        short res = 0;
        for( int ii=0; ii < num_chips; ++ii )
        {
            //short theBit = ( first_12_chips_octal[ sv - 1 ] >> (11-ii) ) & 0x01;
            res = (res<<1) | dest[ii];

            //EXPECT_EQ( dest[ii], theBit ) << " sv " << sv << " bit " << ii;
        }
        short icd = first_12_chips_octal[ sv - 1];
        EXPECT_EQ( res, icd ) << " sv " << sv << ". icd: " << std::oct << icd
            << ". actual: " << std::oct << res << std::dec << ". num_err: " << ++num_err;
    }

    EXPECT_EQ( num_err, 0 );

}

TEST(PCodeGenTest, EndX1EpochTest)
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
    uint64_t end_x1_epoch = 4092LL*3750LL;
    uint64_t start_ind = end_x1_epoch - num_chips;

    int sv = 1;

    pcode_gen.get_chips( sv, start_ind, num_chips, dest );

    EXPECT_EQ( dest.size(), num_chips );

    for( unsigned ii = 0; ii < num_chips; ++ii )
    {
        short res = x1a[4092-num_chips+ii] 
            ^ x1b[4092] ^ x2a[4092-num_chips+ii-sv] 
            ^ x2b[4092];

        EXPECT_EQ( dest[ii], res );
    }
}

TEST(PCodeGenTest, StartSecondX1EpochTest)
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
    uint64_t end_x1_epoch = 4092LL*3750LL;

    int sv = 1;

    uint64_t start_ind = end_x1_epoch;

    pcode_gen.get_chips( sv, start_ind, num_chips, dest );

    EXPECT_EQ( dest.size(), num_chips );

    unsigned ii = 0;
    for( ii = 0; ii < sv && ii < num_chips; ++ii )
    {
        short res = x1a[ii] ^ x1b[ii]
            ^ x2a[4092-sv + ii] ^ x2b[ 4092 ];

        EXPECT_EQ( dest[ii], res );
    }

    for( ; (ii - sv ) < 37 && ii < num_chips; ++ii )
    {
        short res = x1a[ii] ^ x1b[ii]
            ^ x2a[ 4091 ] ^ x2b[ 4092 ];
        EXPECT_EQ( dest[ii], res );
    }

    for( ; ii < num_chips; ++ii )
    {
        short res = x1a[ii] 
            ^ x1b[ii] ^ x2a[ii-37-sv] 
            ^ x2b[ii-37-sv];

        EXPECT_EQ( dest[ii], res );
    }
}

TEST(PCodeGenTest, CrossX1EpochTest)
{
    std::vector< short > dest;

    std::vector< short > dest0;


    GPS_P_Code_Generator pcode_gen;

    uint64_t end_x1_epoch = 4092LL*3750LL;

    int sv = 1;

    unsigned int num_chips_epoch1 = 4092;

    unsigned int num_chips_epoch2 = 124;
    unsigned int num_chips = num_chips_epoch1 + num_chips_epoch2;

    uint64_t start_ind = end_x1_epoch - num_chips_epoch1;

    pcode_gen.get_chips( sv, start_ind, num_chips, dest );

    EXPECT_EQ( dest.size(), num_chips );

    // First check that the 1st x1 epoch chips are good:
    pcode_gen.get_chips( sv, start_ind, num_chips_epoch1, dest0 );

    unsigned ii = 0;

    for( ii = 0; ii < num_chips_epoch1; ++ii )
    {
        EXPECT_EQ( dest[ii], dest0[ii] );
    }

    // Now check the batch from the second x1 epoch:
    pcode_gen.get_chips( sv, end_x1_epoch, num_chips_epoch2, dest0 );

    for( ii = 0; ii < num_chips_epoch2; ++ii )
    {
        EXPECT_EQ( dest[ii+num_chips_epoch1], dest0[ii] );
    }
}

TEST(PCodeGenTest, CrossX2EpochTest)
{
    std::vector< short > dest;

    std::vector< short > dest0;


    GPS_P_Code_Generator pcode_gen;

    uint64_t end_x2_epoch = (4092LL+37LL)*3750LL;

    int sv = 1;

    unsigned int num_chips_epoch1 = 4092;

    unsigned int num_chips_epoch2 = 124;
    unsigned int num_chips = num_chips_epoch1 + num_chips_epoch2;

    uint64_t start_ind = end_x2_epoch - num_chips_epoch1;

    pcode_gen.get_chips( sv, start_ind, num_chips, dest );

    EXPECT_EQ( dest.size(), num_chips );

    // First check that the 1st x1 epoch chips are good:
    pcode_gen.get_chips( sv, start_ind, num_chips_epoch1, dest0 );

    unsigned ii = 0;

    for( ii = 0; ii < num_chips_epoch1; ++ii )
    {
        EXPECT_EQ( dest[ii], dest0[ii] );
    }

    // Now check the batch from the second x1 epoch:
    pcode_gen.get_chips( sv, end_x2_epoch, num_chips_epoch2, dest0 );

    for( ii = 0; ii < num_chips_epoch2; ++ii )
    {
        EXPECT_EQ( dest[ii+num_chips_epoch1], dest0[ii] );
    }
}

TEST(PCodeGenTest, EndOfWeekTest)
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

    unsigned int num_chips = 343;
    uint64_t end_week = GPS_P_CODE_LENGTH_CHIPS;
    uint64_t start_ind = end_week - num_chips;

    int sv = 1;

    pcode_gen.get_chips( sv, start_ind, num_chips, dest );

    EXPECT_EQ( dest.size(), num_chips );

    for( unsigned ii = 0; ii < num_chips; ++ii )
    {
        short res = x1a[4092-num_chips+ii]
            ^ x1b[4092] ^ x2a[4091]
            ^ x2b[4092];

        EXPECT_EQ( dest[ii], res );
    }
}

TEST(PCodeGenTest, Prn37Test)
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

    int sv = 37;
    unsigned int num_chips = 24+sv;
    uint64_t start_ind = 0;


    pcode_gen.get_chips( sv, start_ind, num_chips, dest );

    EXPECT_EQ( dest.size(), num_chips );

    for( unsigned ii = sv; ii < num_chips; ++ii )
    {
        short res = x1a[ii]
            ^ x1b[ii] ^ x2a[ii-sv]
            ^ x2b[ii-sv];

        EXPECT_EQ( dest[ii], res );
    }
}

TEST(PCodeGenTest, NewPrnTest)
{
    std::vector< short > dest;
    std::vector< short > dest0;

    GPS_P_Code_Generator pcode_gen;

    int sv = 137;
    unsigned int num_chips = 24;
    uint64_t start_ind = 0;

    int prn0 = (sv-1)%37 + 1;
    int day_shift = (sv-1)/37;

    uint64_t start_ind0 = start_ind + day_shift*10230000LL*24LL*3600LL;


    pcode_gen.get_chips( sv, start_ind, num_chips, dest );
    pcode_gen.get_chips( prn0, start_ind0, num_chips, dest0 );

    EXPECT_EQ( dest.size(), num_chips );
    EXPECT_EQ( dest0.size(), num_chips );

    for( unsigned ii = 0; ii < num_chips; ++ii )
    {
        EXPECT_EQ( dest[ii], dest0[ii] );
    }

    // From the IS GPS 200H:
    // PRN = 137: first 12 chips =  o0371 (octal)
    // 0 0 0  0 1 1  1 1 1 0 0 1
    std::vector< short > first_12_chips_icd = {
        0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1
        };

    for( unsigned int ii = 0; ii < 12; ++ii )
    {
        EXPECT_EQ( dest[ii], first_12_chips_icd[ii] );
    }
}

