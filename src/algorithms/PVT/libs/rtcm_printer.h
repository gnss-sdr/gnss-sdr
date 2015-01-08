/*!
 * \file rtcm_printer.h
 * \brief Interface of a RTCM 3.2 printer for GNSS-SDR
 * This class provides a implementation of a subset of the RTCM Standard 10403.2
 * for Differential GNSS Services
 *
 * \author Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_RTCM_PRINTER_H_
#define GNSS_SDR_RTCM_PRINTER_H_

#include <bitset>   // std::bitset
#include <fstream>  // std::ofstream
#include <iostream> // std::cout
#include <string>   // std::string
#include <vector>
#include <boost/crc.hpp>


/*!
 * \brief This class provides a implementation of a subset of the RTCM Standard 10403.2 messages
 */
class Rtcm_Printer
{
public:
    /*!
     * \brief Default constructor.
     */
    Rtcm_Printer(std::string filename, bool flag_rtcm_tty_port, std::string rtcm_dump_filename);

    /*!
     * \brief Print RTCM 3.2 messages to the initialized device
     */
    //bool Print_Nmea_Line(gps_l1_ca_ls_pvt* position, bool print_average_values);

    /*!
     * \brief Default destructor.
     */
    ~Rtcm_Printer();

    void print_M1001();
    std::string print_M1005_test();
private:
    std::string rtcm_filename; // String with the RTCM log filename
    std::ofstream rtcm_file_descriptor; // Output file stream for RTCM log file
    std::string rtcm_devname;
    int rtcm_dev_descriptor; // RTCM serial device descriptor (i.e. COM port)
    //gps_l1_ca_ls_pvt* d_PVT_data;
    int init_serial (std::string serial_device); //serial port control
    void close_serial ();

    //std::bitset<8> DF001;
    std::bitset<12> DF002;
    std::bitset<12> DF003;
    std::bitset<30> DF004;
    std::bitset<1> DF005;
    std::bitset<5> DF006;
    std::bitset<1> DF007;
    std::bitset<3> DF008;
    std::bitset<6> DF009;
    std::bitset<1> DF010;
    std::bitset<24> DF011;
    std::bitset<20> DF012;
    std::bitset<7> DF013;
    std::bitset<8> DF014;
    std::bitset<8> DF015;


    std::bitset<6> DF021;
    std::bitset<1> DF022;
    std::bitset<1> DF023;
    std::bitset<1> DF024;
    std::bitset<38> DF025;
    std::bitset<38> DF026;
    std::bitset<38> DF027;

    // Contents of GPS Satellite Ephemeris Data, Message Type 1019
    std::bitset<8> DF071;
    std::bitset<10> DF076;
    std::bitset<4> DF077;
    std::bitset<2> DF078;
    std::bitset<14> DF079;
    std::bitset<16> DF081;
    std::bitset<8> DF082;
    std::bitset<16> DF083;
    std::bitset<22> DF084;
    std::bitset<10> DF085;
    std::bitset<16> DF086;
    std::bitset<16> DF087;

    std::bitset<32> DF088;
    std::bitset<16> DF089;
    std::bitset<32> DF090;
    std::bitset<16> DF091;
    std::bitset<32> DF092;
    std::bitset<16> DF093;
    std::bitset<16> DF094;
    std::bitset<32> DF095;
    std::bitset<16> DF096;
    std::bitset<32> DF097;
    std::bitset<16> DF098;
    std::bitset<32> DF099;
    std::bitset<24> DF100;
    std::bitset<8> DF101;
    std::bitset<6> DF102;
    std::bitset<1> DF103;
    std::bitset<1> DF137;


    std::bitset<1> DF141;
    std::bitset<1> DF142;

    // Contents of Galileo F/NAV Satellite Ephemeris Data, Message Type 1045
    std::bitset<6> DF252;
    std::bitset<12> DF289;
    std::bitset<10> DF290;
    std::bitset<8> DF291;
    std::bitset<14> DF292;
    std::bitset<14> DF293;
    std::bitset<6> DF294;
    std::bitset<21> DF295;
    std::bitset<31> DF296;
    std::bitset<16> DF297;
    std::bitset<32> DF298;
    std::bitset<14> DF299;
    std::bitset<16> DF300;
    std::bitset<32> DF301;
    std::bitset<16> DF302;
    std::bitset<32> DF303;
    std::bitset<14> DF304;
    std::bitset<16> DF305;
    std::bitset<32> DF306;
    std::bitset<16> DF307;
    std::bitset<32> DF308;
    std::bitset<16> DF309;
    std::bitset<32> DF310;
    std::bitset<24> DF311;
    std::bitset<10> DF312;
    std::bitset<2> DF314;
    std::bitset<1> DF315;

    std::bitset<2> DF364;

    // Content of message header for MSM1, MSM2, MSM3, MSM4, MSM5, MSM6 and MSM7
    std::bitset<1> DF393;
    std::bitset<1> DF394;
    std::bitset<1> DF395;
    std::bitset<1> DF396; //variable
    std::bitset<1> DF409;
    std::bitset<1> DF411;
    std::bitset<1> DF412;
    std::bitset<1> DF417;
    std::bitset<1> DF418;

    // Content of Satellite data for MSM4 and MSM6
    std::vector<std::bitset<8> > DF397; // 8*NSAT
    std::vector<std::bitset<10> > DF398; // 10*NSAT

    // Content of Satellite data for MSM5 and MSM7
    std::vector<std::bitset<14> > DF399; // 14*NSAT

    // Messages
    std::bitset<64> message1001_header;
    std::bitset<58> message1001_content;
    std::bitset<64> message1002_header;
    std::bitset<74> message1002_content;

    std::bitset<488> message1019_content;

    std::bitset<496> message1045_content;

    std::bitset<169> MSM_header; // 169+X

    std::vector<std::bitset<18> > MSM4_content; // 18 * Nsat

    std::vector<std::bitset<36> > MSM5_content; // 36 * Nsat

    std::bitset<122> get_M1001();
    std::bitset<138> get_M1002();  //  GPS observables
    std::bitset<488> get_M1019();  // GPS ephemeris
    std::bitset<496> get_M1045();  // Galileo ephemeris
    std::bitset<152> get_M1005_test();

    void reset_data_fields ();

    // Transport Layer
    std::bitset<8> preamble;
    std::bitset<6> reserved_field;
    std::bitset<10> message_length;
    std::bitset<24> crc_frame;
    typedef boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> crc_24_q_type;
    crc_24_q_type CRC_RTCM;
    std::string add_CRC(const std::string& m);
    std::string bin_to_hex(const std::string& s);
};

#endif
