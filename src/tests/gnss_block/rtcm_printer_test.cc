/*!
 * \file rtcm_printer_test.cc
 * \brief Implements Unit Test for the Rtcm_Printer class.
 * \author Carles Fernandez-Prades, 2013. cfernandez(at)cttc.es
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

#include <fstream>
#include <map>
#include <string>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/nvp.hpp>
#include "rtcm_printer.h"
#include "gps_ephemeris.h"





TEST(Rtcm_Printer_Test, Instantiate)
{
    std::string filename = "hello.rtcm";
    bool flag_rtcm_tty_port = false;
    std::string rtcm_dump_devname = "/dev/pts/4";
    std::unique_ptr<Rtcm_Printer> RTCM_printer(new Rtcm_Printer(filename, flag_rtcm_tty_port, rtcm_dump_devname));
}

TEST(Rtcm_Printer_Test, Instantiate_and_Run)
{
    std::string file_name = "./gps_ephemeris_rx.xml";
    std::map<int,Gps_Ephemeris> gps_ephemeris_map;
    try
    {
            std::ifstream ifs(file_name.c_str(), std::ifstream::binary | std::ifstream::in);
            boost::archive::xml_iarchive xml(ifs);
            gps_ephemeris_map.clear();
            xml >> boost::serialization::make_nvp("GNSS-SDR_ephemeris_map", gps_ephemeris_map);
            ifs.close();
    }
    catch (std::exception& e)
    {
            //LOG(WARNING) << e.what() << "File: " << file_name;
            //std::cout << "File not found" << std::endl;
    }

    std::string filename = "hello.rtcm";
    bool flag_rtcm_tty_port = false;
    std::string rtcm_dump_devname = "/dev/pts/4";

    std::unique_ptr<Rtcm_Printer> RTCM_printer(new Rtcm_Printer(filename, flag_rtcm_tty_port, rtcm_dump_devname));
    std::string reference_msg = "D300133ED7D30202980EDEEF34B4BD62AC0941986F33360B98";
    std::string testing_msg = RTCM_printer->print_M1005_test();

    EXPECT_EQ(reference_msg, testing_msg);
}

TEST(Rtcm_Printer_Test, Bin_to_hex)
{
    auto  rtcm = std::make_shared<Rtcm>();

    std::string test1 = "2A";
    std::string test1_bin = rtcm->hex_to_bin(test1);
    EXPECT_EQ(0, test1_bin.compare("00101010"));

    std::string test2 = "FF";
    std::string test2_bin = rtcm->hex_to_bin(test2);
    EXPECT_EQ(0, test2_bin.compare("11111111"));

    std::string test3 = "ff";
    std::string test3_bin = rtcm->hex_to_bin(test3);
    EXPECT_EQ(0, test3_bin.compare("11111111"));
}



TEST(Rtcm_Printer_Test, Hex_to_int)
{
    auto  rtcm = std::make_shared<Rtcm>();

    std::string test1 = "2A";
    long int test1_int = rtcm->hex_to_int(test1);
    long int expected1 = 42;
    EXPECT_EQ(expected1, test1_int);
}

TEST(Rtcm_Printer_Test, Bin_to_double)
{
    auto  rtcm = std::make_shared<Rtcm>();

    std::bitset<4> test1(5);
    long int test1_int = static_cast<long int>(rtcm->bin_to_double(test1.to_string()));
    long int expected1 = 5;
    EXPECT_EQ(expected1, test1_int);

    std::bitset<4> test2(-5);
    long int test2_int = static_cast<long int>(rtcm->bin_to_double(test2.to_string()));
    long int expected2 = -5;
    EXPECT_EQ(expected2, test2_int);

    std::bitset<65> test3(-5);
    long int test3_int = static_cast<long int>(rtcm->bin_to_double(test3.to_string()));
    long int expected3 = 0;
    EXPECT_EQ(expected3, test3_int);
}


TEST(Rtcm_Printer_Test, Read_M1005)
{
    std::string filename = "hello.rtcm";
    bool flag_rtcm_tty_port = false;
    std::string rtcm_dump_devname = "/dev/pts/4";

    auto rtcm = std::make_shared<Rtcm>();
    auto rtcm_printer = std::make_shared<Rtcm_Printer>(filename, flag_rtcm_tty_port, rtcm_dump_devname);
    std::string reference_msg = rtcm_printer->print_M1005_test();

    unsigned int ref_id;
    double ecef_x;
    double ecef_y;
    double ecef_z;
    bool gps;
    bool glonass;
    bool galileo;

    rtcm->read_M1005(reference_msg, ref_id, ecef_x, ecef_y, ecef_z, gps, glonass, galileo);

    EXPECT_EQ(true, gps);
    EXPECT_EQ(false, glonass);
    EXPECT_EQ(false, galileo);

    EXPECT_EQ(2003, ref_id);
    EXPECT_DOUBLE_EQ(1114104.5999, ecef_x);
    EXPECT_DOUBLE_EQ(-4850729.7108, ecef_y);
    EXPECT_DOUBLE_EQ(3975521.4643, ecef_z);
}
