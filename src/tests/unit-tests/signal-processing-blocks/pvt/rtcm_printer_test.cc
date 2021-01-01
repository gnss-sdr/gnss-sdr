/*!
 * \file rtcm_printer_test.cc
 * \brief Implements Unit Test for the Rtcm_Printer class.
 * \author Carles Fernandez-Prades, 2013. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "gnss_sdr_make_unique.h"
#include "rtcm_printer.h"
#include <string>


TEST(RtcmPrinterTest, Instantiate)
{
    std::string filename = "hello.rtcm";
    bool flag_rtcm_tty_port = false;
    std::string rtcm_dump_devname = "/dev/pts/4";
    bool flag_rtcm_server = false;
    bool rtcm_file_output_enabled = false;
    unsigned short rtcm_tcp_port = 2101;
    unsigned short rtcm_station_id = 1234;
    auto RTCM_printer = std::make_unique<Rtcm_Printer>(filename, rtcm_file_output_enabled, flag_rtcm_server, flag_rtcm_tty_port, rtcm_tcp_port, rtcm_station_id, rtcm_dump_devname);
}


TEST(RtcmPrinterTest, Run)
{
    std::string filename = "test.rtcm";
    bool flag_rtcm_tty_port = false;
    bool rtcm_file_output_enabled = false;
    std::string rtcm_dump_devname = "/dev/pts/4";
    bool flag_rtcm_server = false;
    unsigned short rtcm_tcp_port = 2101;
    unsigned short rtcm_station_id = 1234;

    auto RTCM_printer = std::make_unique<Rtcm_Printer>(filename, rtcm_file_output_enabled, flag_rtcm_server, flag_rtcm_tty_port, rtcm_tcp_port, rtcm_station_id, rtcm_dump_devname);

    std::string reference_msg = "D300133ED7D30202980EDEEF34B4BD62AC0941986F33360B98";

    /* Convert the reference message to binary data */
    std::string reference_msg_binary;
    unsigned char c[1];
    for (unsigned int i = 0; i < reference_msg.length(); i = i + 2)
        {
            uint64_t n, n2;
            std::istringstream(reference_msg.substr(i, 1)) >> std::hex >> n;
            std::istringstream(reference_msg.substr(i + 1, 1)) >> std::hex >> n2;
            c[0] = static_cast<unsigned char>(n * 16) + static_cast<unsigned char>(n2);
            std::string ret(c, c + 1);
            reference_msg_binary += ret;
        }

    std::string testing_msg = RTCM_printer->print_MT1005_test();

    EXPECT_EQ(0, reference_msg_binary.compare(testing_msg));
}
