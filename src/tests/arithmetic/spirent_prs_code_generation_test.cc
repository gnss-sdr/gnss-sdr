/*!
 * \file spirent_prs_code_generation_test.cc
 * \brief  This file implements tests for the generation of the PRS code
 *  as generated in Spirent Simulators.
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
 *
 * Spirent PRS code generation is described here:
 * http://ekb.spirent.com/index?page=content&id=FAQ13323&cat=PROTOCOL_POSIT_CODE&actp=LIST
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
#include "spirent_prs_code_generator.h"
#include "gps_pcode.h"


TEST(SpirentPrsTest, E1FirstChipsTest)
{
    int sv = 1;
    bool is_e1 = true;

    SpirentPrsCodeGenerator code_gen( sv, is_e1 );

    GPS_P_Code_Generator pcode_gen;

    int downsample_factor = 4;

    uint64_t first_chip_index = 0;

    unsigned int num_chips = 24;

    if( ! is_e1 ){
        downsample_factor = 2;
    }

    unsigned int offset = 0;


    for( sv = 1; sv <= 50; ++sv )
    {

        if( sv & 0x01 ){
            offset = 1;
        }
        else{
            offset = 0;
        }

        code_gen.set_prn( sv );


        std::vector< short > prs;

        std::vector< short > pcode;

        code_gen.get_chips( first_chip_index, num_chips, prs );

        uint64_t first_p_chip_index = downsample_factor*first_chip_index;
        if( first_p_chip_index < offset )
        {
            first_p_chip_index = first_p_chip_index + code_gen.get_code_length() - offset;
        }
        else
        {
            first_p_chip_index = first_p_chip_index - offset;
        }

        pcode_gen.get_chips( sv, first_p_chip_index, num_chips*downsample_factor, pcode );

        for( unsigned int ii = 0, jj = 0; ii < num_chips; ++ii, jj+=downsample_factor )
        {
            EXPECT_EQ( prs[ii], pcode[jj] ) << "sv: " << sv << ". ii: " << ii;
        }
    }

}

TEST(SpirentPrsTest, E6FirstChipsTest)
{
    int sv = 1;
    bool is_e1 = false;

    SpirentPrsCodeGenerator code_gen( sv, is_e1 );

    GPS_P_Code_Generator pcode_gen;

    int downsample_factor = 4;

    uint64_t first_chip_index = 0;

    unsigned int num_chips = 24;

    if( ! is_e1 ){
        downsample_factor = 2;
    }

    unsigned int offset = 0;


    for( sv = 1; sv <= 50; ++sv )
    {

        if( sv & 0x01 ){
            offset = 1;
        }
        else{
            offset = 0;
        }

        code_gen.set_prn( sv );


        std::vector< short > prs;

        std::vector< short > pcode;

        code_gen.get_chips( first_chip_index, num_chips, prs );

        uint64_t first_p_chip_index = downsample_factor*first_chip_index;
        if( first_p_chip_index < offset )
        {
            first_p_chip_index = first_p_chip_index + code_gen.get_code_length() - offset;
        }
        else
        {
            first_p_chip_index = first_p_chip_index - offset;
        }

        pcode_gen.get_chips( sv, first_p_chip_index, num_chips*downsample_factor, pcode );

        for( unsigned int ii = 0, jj = 0; ii < num_chips; ++ii, jj+=downsample_factor )
        {
            EXPECT_EQ( prs[ii], pcode[jj] ) << "sv: " << sv << ". ii: " << ii;
        }
    }

}


TEST(SpirentPrsTest, E1LastChipsTest)
{
    int sv = 1;
    bool is_e1 = true;

    SpirentPrsCodeGenerator code_gen( sv, is_e1 );

    GPS_P_Code_Generator pcode_gen;

    int downsample_factor = 4;

    // Get a bunch of chips near the end of the week
    uint64_t first_chip_index = 2557500LL*24LL*3600LL*7LL - 3;

    unsigned int num_chips = 24;

    if( ! is_e1 ){
        downsample_factor = 2;
    }

    unsigned int offset = 0;


    for( sv = 1; sv <= 50; ++sv )
    {

        if( sv & 0x01 ){
            offset = 1;
        }
        else{
            offset = 0;
        }

        code_gen.set_prn( sv );


        std::vector< short > prs;

        std::vector< short > pcode;

        code_gen.get_chips( first_chip_index, num_chips, prs );

        uint64_t first_p_chip_index = downsample_factor*first_chip_index;
        if( first_p_chip_index < offset )
        {
            first_p_chip_index = first_p_chip_index + code_gen.get_code_length() - offset;
        }
        else
        {
            first_p_chip_index = first_p_chip_index - offset;
        }

        pcode_gen.get_chips( sv, first_p_chip_index, num_chips*downsample_factor, pcode );

        for( unsigned int ii = 0, jj = 0; ii < num_chips; ++ii, jj+=downsample_factor )
        {
            EXPECT_EQ( prs[ii], pcode[jj] ) << "sv: " << sv << ". ii: " << ii;
        }
    }

}

TEST(SpirentPrsTest, E6LastChipsTest)
{
    int sv = 1;
    bool is_e1 = false;

    SpirentPrsCodeGenerator code_gen( sv, is_e1 );

    GPS_P_Code_Generator pcode_gen;

    int downsample_factor = 4;

    uint64_t first_chip_index = 5115000LL*24LL*3600LL*7LL - 3;

    unsigned int num_chips = 24;

    if( ! is_e1 ){
        downsample_factor = 2;
    }

    unsigned int offset = 0;


    for( sv = 1; sv <= 50; ++sv )
    {

        if( sv & 0x01 ){
            offset = 1;
        }
        else{
            offset = 0;
        }

        code_gen.set_prn( sv );


        std::vector< short > prs;

        std::vector< short > pcode;

        code_gen.get_chips( first_chip_index, num_chips, prs );

        uint64_t first_p_chip_index = downsample_factor*first_chip_index;
        if( first_p_chip_index < offset )
        {
            first_p_chip_index = first_p_chip_index + code_gen.get_code_length() - offset;
        }
        else
        {
            first_p_chip_index = first_p_chip_index - offset;
        }

        pcode_gen.get_chips( sv, first_p_chip_index, num_chips*downsample_factor, pcode );

        for( unsigned int ii = 0, jj = 0; ii < num_chips; ++ii, jj+=downsample_factor )
        {
            EXPECT_EQ( prs[ii], pcode[jj] ) << "sv: " << sv << ". ii: " << ii;
        }
    }

}
