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
