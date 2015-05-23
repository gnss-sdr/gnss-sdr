/*!
 * \file rtcm_printer.cc
 * \brief Implementation of a RTCM 3.2 printer for GNSS-SDR
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

#include "rtcm_printer.h"
#include <fcntl.h>    // for O_RDWR
#include <termios.h>  // for tcgetattr
#include <algorithm>  // for std::reverse
#include <sstream>    // for std::stringstream
#include <boost/algorithm/string.hpp>  // for to_upper_copy
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/dynamic_bitset.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>


using google::LogMessage;

//DEFINE_string(RTCM_version, "3.2", "Specifies the RTCM Version");

Rtcm_Printer::Rtcm_Printer(std::string filename, bool flag_rtcm_tty_port, std::string rtcm_dump_devname)
{
    rtcm_filename = filename;
    rtcm_file_descriptor.open(rtcm_filename.c_str(), std::ios::out);
    if (rtcm_file_descriptor.is_open())
        {
            DLOG(INFO) << "RTCM printer writing on " << rtcm_filename.c_str();
        }

    rtcm_devname = rtcm_dump_devname;
    if (flag_rtcm_tty_port == true)
        {
            rtcm_dev_descriptor = init_serial(rtcm_devname.c_str());
            if (rtcm_dev_descriptor != -1)
                {
                    DLOG(INFO) << "RTCM printer writing on " << rtcm_devname.c_str();
                }
        }
    else
        {
            rtcm_dev_descriptor = -1;
        }
    Rtcm_Printer::reset_data_fields();
    preamble = std::bitset<8>("11010011");
    reserved_field = std::bitset<6>("000000");
}




Rtcm_Printer::~Rtcm_Printer()
{
    if (rtcm_file_descriptor.is_open())
        {
            long pos;
            rtcm_file_descriptor.close();
            pos = rtcm_file_descriptor.tellp();
            if (pos == 0)
                {
                    if(remove(rtcm_filename.c_str()) != 0) LOG(INFO) << "Error deleting temporary file";
                }
        }
    close_serial();
}



int Rtcm_Printer::init_serial(std::string serial_device)
{
    /*!
     * Opens the serial device and sets the default baud rate for a NMEA transmission (9600,8,N,1)
     */
    int fd = 0;
    struct termios options;
    long BAUD;
    long DATABITS;
    long STOPBITS;
    long PARITYON;
    long PARITY;

    fd = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return fd;  // failed to open TTY port

    if(fcntl(fd, F_SETFL, 0) == -1) LOG(INFO) << "Error enabling direct I/O";    // clear all flags on descriptor, enable direct I/O
    tcgetattr(fd, &options);  // read serial port options

    BAUD  = B9600;
    //BAUD  =  B38400;
    DATABITS = CS8;
    STOPBITS = 0;
    PARITYON = 0;
    PARITY = 0;

    options.c_cflag = BAUD | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
    // enable receiver, set 8 bit data, ignore control lines
    //options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_iflag = IGNPAR;

    // set the new port options
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}



void Rtcm_Printer::close_serial()
{
    if (rtcm_dev_descriptor != -1)
        {
            close(rtcm_dev_descriptor);
        }
}

void Rtcm_Printer::reset_data_fields()
{
    //DF001.reset();
    DF002.reset();
    DF003.reset();
    DF004.reset();
    DF005.reset();
    DF006.reset();
    DF007.reset();
    DF008.reset();
    DF009.reset();
    DF010.reset();
    DF011.reset();
    DF012.reset();
    DF013.reset();
    DF014.reset();
    DF015.reset();

    // Contents of GPS Satellite Ephemeris Data, Message Type 1019
    DF071.reset();
    DF076.reset();
    DF077.reset();
    DF078.reset();
    DF079.reset();
    DF081.reset();
    DF082.reset();
    DF083.reset();
    DF084.reset();
    DF085.reset();
    DF086.reset();
    DF087.reset();

    DF088.reset();
    DF089.reset();
    DF090.reset();
    DF091.reset();
    DF092.reset();
    DF093.reset();
    DF094.reset();
    DF095.reset();
    DF096.reset();
    DF097.reset();
    DF098.reset();
    DF099.reset();
    DF100.reset();
    DF101.reset();
    DF102.reset();
    DF103.reset();
    DF137.reset();

    // Contents of Galileo F/NAV Satellite Ephemeris Data, Message Type 1045
    DF252.reset();
    DF289.reset();
    DF290.reset();
    DF291.reset();
    DF292.reset();
    DF293.reset();
    DF294.reset();
    DF295.reset();
    DF296.reset();
    DF297.reset();
    DF298.reset();
    DF299.reset();
    DF300.reset();
    DF301.reset();
    DF302.reset();
    DF303.reset();
    DF304.reset();
    DF305.reset();
    DF306.reset();
    DF307.reset();
    DF308.reset();
    DF309.reset();
    DF310.reset();
    DF311.reset();
    DF312.reset();
    DF314.reset();
    DF315.reset();
}



/* Stationary Antenna Reference Point, No Height Information
 * Reference Station Id = 2003
   GPS Service supported, but not GLONASS or Galileo
   ARP ECEF-X = 1114104.5999 meters
   ARP ECEF-Y = -4850729.7108 meters
   ARP ECEF-Z = 3975521.4643 meters
   Expected output: D3 00 13 3E D7 D3 02 02 98 0E DE EF 34 B4 BD 62
                    AC 09 41 98 6F 33 36 0B 98
 */
std::bitset<152> Rtcm_Printer::get_M1005_test ()
{
    unsigned int m1005 = 1005;
    unsigned int reference_station_id = 2003; // Max: 4095
    long long int ECEF_X = 11141045999;       // Resolution 0.0001 m
    long long int ECEF_Y = -48507297108;      // Resolution 0.0001 m
    long long int ECEF_Z = 39755214643;       // Resolution 0.0001 m
    unsigned int itrf_realization_year = 0;   // Reserved
    std::bitset<1> DF001;

    DF002 = std::bitset<12>(m1005);
    DF003 = std::bitset<12>(reference_station_id);
    DF021 = std::bitset<6>(itrf_realization_year);
    DF022 = std::bitset<1>("1");          // GPS
    DF023 = std::bitset<1>("0");          // Glonass
    DF024 = std::bitset<1>("0");          // Galileo
    DF141 = std::bitset<1>("0");          // 0: Real, physical reference station
    DF001 = std::bitset<1>("0");          // Reserved, set to 0
    DF025 = std::bitset<38>(ECEF_X);      // ECEF-X in 0.0001 m
    DF142 = std::bitset<1>("0");          // Single Receiver Oscillator Indicator
    DF026 = std::bitset<38>(ECEF_Y);      // ECEF-Y in 0.0001 m
    DF364 = std::bitset<2>("00");         // Quarter Cycle Indicator
    DF027 = std::bitset<38>(ECEF_Z);      // ECEF-Z in 0.0001 m

    std::string message = DF002.to_string() +
            DF003.to_string() +
            DF021.to_string() +
            DF022.to_string() +
            DF023.to_string() +
            DF024.to_string() +
            DF141.to_string() +
            DF025.to_string() +
            DF142.to_string() +
            DF001.to_string() +
            DF026.to_string() +
            DF364.to_string() +
            DF027.to_string() ;

    std::bitset<152> test_msg(message);
    return test_msg;
}



std::string Rtcm_Printer::print_M1005_test ()
{
    std::bitset<152> m1005 = get_M1005_test();
    unsigned int msg_length_bits = m1005.to_string().length();
    unsigned int msg_length_bytes = std::ceil(static_cast<float>(msg_length_bits) / 8.0);
    message_length = std::bitset<10>(msg_length_bytes);
    unsigned int zeros_to_fill = 8*msg_length_bytes -  msg_length_bits;
    std::string b(zeros_to_fill, '0');
    std::string msg_content = m1005.to_string() + b;
    std::string msg_without_crc = preamble.to_string() +
            reserved_field.to_string() +
            message_length.to_string() +
            msg_content;
    return Rtcm_Printer::add_CRC(msg_without_crc);
}



std::bitset<122> Rtcm_Printer::get_M1001()
{
    unsigned int m1001 = 1001;
    unsigned int reference_station_id = 1234; // Max: 4095
    DF002 = std::bitset<12>(m1001);
    DF003 = std::bitset<12>(reference_station_id);
    //DF004 = std::bitset<30>
    //DF005 = std::bitset<1>
    //DF006 = std::bitset<5>
    DF007 = std::bitset<1>("0");
    //DF008 = std::bitset<3>
    std::bitset<122> fake_msg;
    fake_msg.reset();
    return fake_msg;
}



void Rtcm_Printer::print_M1001 ()
{
    std::bitset<122> m1001 = get_M1001();
    unsigned int msg_length_bits = m1001.to_string().length();
    unsigned int msg_length_bytes = std::ceil(static_cast<float>(msg_length_bits) / 8.0);
    message_length = std::bitset<10>(msg_length_bytes);
    unsigned int zeros_to_fill = 8*msg_length_bytes -  msg_length_bits;
    std::string b(zeros_to_fill, '0');
    message_length = std::bitset<10>(static_cast<int>(msg_length_bytes));
    std::string msg_content = m1001.to_string() + b;
    std::string msg_without_crc = preamble.to_string() +
            reserved_field.to_string() +
            message_length.to_string() +
            msg_content;
    std::string message = Rtcm_Printer::add_CRC(msg_without_crc);
}



std::bitset<138> Rtcm_Printer::get_M1002 ()
{
    std::bitset<138> fake_msg;
    fake_msg.reset();
    return fake_msg;
}

std::bitset<488> Rtcm_Printer::get_M1019 ()
{
    std::bitset<488> fake_msg;
        fake_msg.reset();
        return fake_msg;
}



std::bitset<496> Rtcm_Printer::get_M1045 ()
{
    std::bitset<496> fake_msg;
    fake_msg.reset();
    return fake_msg;
}





std::string Rtcm_Printer::add_CRC (const std::string& message_without_crc)
{
    // ******  Computes Qualcomm CRC-24Q ******
    // 1) Converts the string to a vector of unsigned char:
    boost::dynamic_bitset<unsigned char> frame_bits(message_without_crc);
    std::vector<unsigned char> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(),bytes.end());

    // 2) Computes CRC
    CRC_RTCM.process_bytes(bytes.data(), bytes.size());
    crc_frame = std::bitset<24>(CRC_RTCM.checksum());

    // 3) Builds the complete message
    std::string complete_message = message_without_crc + crc_frame.to_string();
    return bin_to_hex(complete_message);
}


std::string  Rtcm_Printer::bin_to_hex(const std::string& s)
{
    std::string s_aux;
    std::stringstream ss;
    for(int i = 0; i < s.length() - 1; i = i + 32)
        {
            s_aux.assign(s, i, 32);
            std::bitset<32> bs(s_aux);
            unsigned n = bs.to_ulong();
            ss << std::hex << n;
        }
    //return ss.str();
    return boost::to_upper_copy(ss.str());
}


