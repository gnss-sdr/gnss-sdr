/*!
 * \file rtcm_printer_test.cc
 * \brief Implements Unit Test for the Rtcm_Printer class.
 * \author Carles Fernandez-Prades, 2013. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include <string>
#include "rtcm_printer.h"


TEST(RtcmPrinterTest, Instantiate)
{
    std::string filename = "hello.rtcm";
    bool flag_rtcm_tty_port = false;
    std::string rtcm_dump_devname = "/dev/pts/4";
    bool flag_rtcm_server = false;
    unsigned short rtcm_tcp_port = 2101;
    unsigned short rtcm_station_id = 1234;
    std::unique_ptr<Rtcm_Printer> RTCM_printer(new Rtcm_Printer(filename, flag_rtcm_server, flag_rtcm_tty_port, rtcm_tcp_port, rtcm_station_id, rtcm_dump_devname));
}


TEST(RtcmPrinterTest, Run)
{
    std::string filename = "test.rtcm";
    bool flag_rtcm_tty_port = false;
    std::string rtcm_dump_devname = "/dev/pts/4";
    bool flag_rtcm_server = false;
    unsigned short rtcm_tcp_port = 2101;
    unsigned short rtcm_station_id = 1234;

    std::unique_ptr<Rtcm_Printer> RTCM_printer(new Rtcm_Printer(filename, flag_rtcm_server, flag_rtcm_tty_port, rtcm_tcp_port, rtcm_station_id, rtcm_dump_devname));

    std::string reference_msg = "D300133ED7D30202980EDEEF34B4BD62AC0941986F33360B98";

    /* Convert the reference message to binary data */
    std::string reference_msg_binary;
    unsigned char c[1];
    for (unsigned int i = 0; i < reference_msg.length(); i = i + 2)
        {
            unsigned long int n, n2;
            std::istringstream(reference_msg.substr(i, 1)) >> std::hex >> n;
            std::istringstream(reference_msg.substr(i + 1, 1)) >> std::hex >> n2;
            c[0] = static_cast<unsigned char>(n * 16) + static_cast<unsigned char>(n2);
            std::string ret(c, c + 1);
            reference_msg_binary += ret;
        }

    std::string testing_msg = RTCM_printer->print_MT1005_test();

    EXPECT_EQ(0, reference_msg_binary.compare(testing_msg));
}
