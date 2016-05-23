/*!
 * \file file_long_code_generation_test.cc
 * \brief  This file implements tests for the generation of the long codes
 * whose chips are read from file
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

#include <gtest/gtest.h>
#include <sstream>
#include <fstream>
#include <algorithm>
#include "file_long_code_generator.h"

const uint64_t kFirstChipInd = 204900;
const uint64_t kCodeLength = 1023000*2.5*3600*24*7;
const std::vector< short > kChipVec = { 1, 1, -1, -1, +1, -1, +1, -1, +1, +1 };

class FileLongCodeTestGen : public ::testing::Test
{
    public:
        FileLongCodeTestGen()
            : ios("")
        {
        };

    protected:
        virtual void SetUp()
        {
            ios << kFirstChipInd << std::endl;
            ios << kCodeLength << std::endl;
            for( std::vector< short >::const_iterator it = kChipVec.begin();
                    it != kChipVec.end(); ++it )
            {
                ios << *it << std::endl;
            }

            ios.seekg( 0 );
        }

        std::stringstream ios;

};


TEST_F(FileLongCodeTestGen, CheckFileContents )
{
    uint64_t firstChipIndex;

    ios >> firstChipIndex;

    EXPECT_EQ( kFirstChipInd, firstChipIndex );

    uint64_t code_length;

    ios >> code_length;

    EXPECT_EQ( code_length, kCodeLength );

    // Count the lines in the file:

    ios.seekg( 0 );
    unsigned long numLines =
        std::count(std::istreambuf_iterator<char>(ios),
                std::istreambuf_iterator<char>(), '\n');

    EXPECT_EQ( kChipVec.size() + 2, numLines );

}

TEST_F(FileLongCodeTestGen, GetKnownBits)
{
    FileLongCodeGenerator codeGen( 1, ios );

    std::vector< short > theChips;

    bool res = codeGen.get_chips( kFirstChipInd, kChipVec.size(), theChips );

    EXPECT_TRUE( res );

    EXPECT_EQ( kChipVec.size(), theChips.size() );

    for( unsigned int ii = 0; ii < theChips.size(); ++ii )
    {
        EXPECT_EQ( kChipVec[ii], theChips[ii] ) << "Chip error at index: " << ii;
    }

}

TEST_F(FileLongCodeTestGen, CantGetEarlyBits )
{
    FileLongCodeGenerator codeGen( 1, ios );

    std::vector< short > theChips;

    bool res = codeGen.get_chips( kFirstChipInd - 1, kChipVec.size(), theChips );

    EXPECT_FALSE( res );
}

TEST_F(FileLongCodeTestGen, CanGetSomeBitsAtEnd )
{
    FileLongCodeGenerator codeGen( 1, ios );

    std::vector< short > theChips;

    bool res = codeGen.get_chips( kFirstChipInd + kChipVec.size() - 3,
            kChipVec.size(), theChips );

    EXPECT_TRUE( res );

    EXPECT_EQ( 3, theChips.size() );

    for( unsigned int ii = 0; ii < 3; ++ii )
    {
        EXPECT_EQ( kChipVec[ kChipVec.size()-3+ii ], theChips[ii] );
    }
}

TEST_F( FileLongCodeTestGen, CantGetLateBits )
{
    FileLongCodeGenerator codeGen( 1, ios );

    std::vector< short > theChips;

    bool res = codeGen.get_chips( kFirstChipInd + kChipVec.size(),
            kChipVec.size(), theChips );

    EXPECT_FALSE( res );
}

/*
TEST_F( FileLongCodeTestGen, CanWorkFromFile )
{
    std::ifstream ifs( "sample_file_long_code.txt" );

    ASSERT_TRUE( ifs.is_open() );

    FileLongCodeGenerator codeGen( 1, ifs );

    EXPECT_EQ( 1546776000000, codeGen.get_code_length() );

    uint64_t first_chip_index = 0;

    ifs.clear();
    ifs.seekg( 0 );

    ifs >> first_chip_index;

    EXPECT_EQ( 564426801213, first_chip_index );

    std::vector< short > the_chips;

    bool res = codeGen.get_chips( first_chip_index, 10, the_chips );

    EXPECT_TRUE( res );

    EXPECT_EQ( 10, the_chips.size() );

}
*/
