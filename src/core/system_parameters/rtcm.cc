/*!
 * \file rtcm.cc
 * \brief  Implementation of RTCM 3.2 Standard
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
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

#include "rtcm.h"
#include <algorithm>  // for std::reverse
#include <cmath>      // for std::fmod
#include <cstdlib>    // for strtol
#include <sstream>    // for std::stringstream
#include <boost/algorithm/string.hpp>  // for to_upper_copy
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/dynamic_bitset.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "GPS_L1_CA.h"



using google::LogMessage;

DEFINE_int32(RTCM_Ref_Station_ID, 1234, "Reference Station ID in RTCM messages");



Rtcm::Rtcm()
{
    Rtcm::reset_data_fields();
    preamble = std::bitset<8>("11010011");
    reserved_field = std::bitset<6>("000000");
}




// *****************************************************************************************************
//
//   TRANSPORT LAYER AS DEFINED AT RTCM STANDARD 10403.2
//
// *****************************************************************************************************

std::string Rtcm::add_CRC (const std::string& message_without_crc)
{
    // ******  Computes Qualcomm CRC-24Q ******
    crc_24_q_type CRC_RTCM;
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


bool Rtcm::check_CRC(const std::string & message)
{
    crc_24_q_type CRC_RTCM_CHECK;
    // Convert message to binary
    std::string message_bin = Rtcm::hex_to_bin(message);
    // Check CRC
    std::string crc = message_bin.substr(message_bin.length() - 24, 24);
    std::bitset<24> read_crc =  std::bitset<24>(crc);
    std::string msg_without_crc = message_bin.substr(0, message_bin.length() - 24);

    boost::dynamic_bitset<unsigned char> frame_bits(msg_without_crc);
    std::vector<unsigned char> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(),bytes.end());

    CRC_RTCM_CHECK.process_bytes(bytes.data(), bytes.size());
    std::bitset<24> computed_crc = std::bitset<24>(CRC_RTCM_CHECK.checksum());
    if(read_crc == computed_crc)
        {
            return true;
        }
    else
        {
            return false;
        }
}


std::string Rtcm::bin_to_hex(const std::string& s)
{
    std::string s_aux;
    std::stringstream ss;
    int remainder = static_cast<int>(std::fmod(s.length(), 4));

    if (remainder != 0)
        {
            s_aux.assign(s, 0 , remainder);
            boost::dynamic_bitset<> rembits(s_aux);
            unsigned n = rembits.to_ulong();
            ss << std::hex << n;
        }

    unsigned int start = std::max(remainder, 0);
    for(unsigned int i = start; i < s.length() - 1; i = i + 4)
        {
            s_aux.assign(s, i, 4);
            std::bitset<4> bs(s_aux);
            unsigned n = bs.to_ulong();
            ss << std::hex << n;
        }
    return boost::to_upper_copy(ss.str());
}


std::string Rtcm::hex_to_bin(const std::string& s)
{
    std::string s_aux;
    s_aux.clear();
    std::stringstream ss;
    ss << s;
    std::string s_lower = boost::to_upper_copy(ss.str());
    for(unsigned int i = 0; i < s.length(); i++)
        {
            unsigned long int n;
            std::istringstream(s_lower.substr(i,1)) >> std::hex >> n;
            std::bitset<4> bs(n);
            s_aux += bs.to_string();
        }
    return s_aux;
}


unsigned long int Rtcm::bin_to_uint(const std::string& s)
{
    if(s.length() > 32)
        {
            LOG(WARNING) << "Cannot convert to a unsigned long int";
            return 0;
        }
    unsigned long int reading = strtoul(s.c_str(), NULL, 2);
    return reading;
}


long int Rtcm::bin_to_int(const std::string& s)
{
    if(s.length() > 32)
        {
            LOG(WARNING) << "Cannot convert to a long int";
            return 0;
        }
    long int reading;

    // Handle negative numbers
    if(s.substr(0,1).compare("0"))
        {
            // Computing two's complement
            boost::dynamic_bitset<> original_bitset(s);
            original_bitset.flip();
            reading = - (original_bitset.to_ulong() + 1);
        }
    else
        {
            reading = strtol(s.c_str(), NULL, 2);
        }
    return reading;
}


double Rtcm::bin_to_double(const std::string& s)
{
    double reading;
    if(s.length() > 64)
        {
            LOG(WARNING) << "Cannot convert to a double";
            return 0;
        }

    long long int reading_int;

    // Handle negative numbers
    if(s.substr(0,1).compare("0"))
        {
            // Computing two's complement
            boost::dynamic_bitset<> original_bitset(s);
            original_bitset.flip();
            reading_int = - (original_bitset.to_ulong() + 1);
        }
    else
        {
            reading_int = strtoll(s.c_str(), NULL, 2);
        }

    reading = static_cast<double>(reading_int);
    return reading;
}


unsigned long int Rtcm::hex_to_uint(const std::string& s)
{
    if(s.length() > 32)
        {
            LOG(WARNING) << "Cannot convert to a unsigned long int";
            return 0;
        }
    unsigned long int reading = strtoul(s.c_str(), NULL, 16);
    return reading;
}


long int Rtcm::hex_to_int(const std::string& s)
{
    if(s.length() > 32)
        {
            LOG(WARNING) << "Cannot convert to a long int";
            return 0;
        }
    long int reading = strtol(s.c_str(), NULL, 16);
    return reading;
}


std::string Rtcm::build_message(std::string data)
{
    unsigned int msg_length_bits = data.length();
    unsigned int msg_length_bytes = std::ceil(static_cast<float>(msg_length_bits) / 8.0);
    message_length = std::bitset<10>(msg_length_bytes);
    unsigned int zeros_to_fill = 8 * msg_length_bytes -  msg_length_bits;
    std::string b(zeros_to_fill, '0');
    std::string msg_content = data + b;
    std::string msg_without_crc = preamble.to_string() +
            reserved_field.to_string() +
            message_length.to_string() +
            msg_content;
    return Rtcm::add_CRC(msg_without_crc);
}



// *****************************************************************************************************
//
//   MESSAGES AS DEFINED AT RTCM STANDARD 10403.2
//
// *****************************************************************************************************



// **********************************************
//
//   MESSAGE TYPE 1001 (GPS L1 OBSERVATIONS)
//
// **********************************************

std::bitset<64> Rtcm::get_MT1001_header(const Gps_Ephemeris & gps_eph, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges,
        unsigned int ref_id, unsigned int smooth_int, bool sync_flag, bool divergence_free)
{
    unsigned int m1001 = 1001;
    unsigned int reference_station_id = ref_id; // Max: 4095
    const std::map<int, Gnss_Synchro> pseudoranges_ = pseudoranges;
    bool synchronous_GNSS_flag = sync_flag;
    bool divergence_free_smoothing_indicator = divergence_free;
    unsigned int smoothing_interval = smooth_int;
    Rtcm::set_DF002(m1001);
    Rtcm::set_DF003(reference_station_id);
    Rtcm::set_DF004(gps_eph, obs_time);
    Rtcm::set_DF005(synchronous_GNSS_flag);
    Rtcm::set_DF006(pseudoranges_);
    Rtcm::set_DF007(divergence_free_smoothing_indicator);
    Rtcm::set_DF008(smoothing_interval);

    std::string header = DF002.to_string() +
            DF003.to_string() +
            DF004.to_string() +
            DF005.to_string() +
            DF006.to_string() +
            DF007.to_string() +
            DF008.to_string();

    std::bitset<64> header_msg(header);
    return header_msg;
}


std::bitset<58> Rtcm::get_MT1001_sat_content(const Gnss_Synchro & gnss_synchro)
{
    Gnss_Synchro gnss_synchro_ = gnss_synchro;
    bool code_indicator = false; // code indicator   0: C/A code   1: P(Y) code direct
    Rtcm::set_DF009(gnss_synchro_);
    Rtcm::set_DF010(code_indicator); // code indicator   0: C/A code   1: P(Y) code direct
    Rtcm::set_DF011(gnss_synchro_);

    long int  gps_L1_phaserange_minus_L1_pseudorange;
    long int  phaserange_m = (gnss_synchro.Carrier_phase_rads * GPS_C_m_s) / (GPS_TWO_PI * GPS_L1_FREQ_HZ);
    gps_L1_phaserange_minus_L1_pseudorange  = phaserange_m; // TODO
    DF012 = std::bitset<20>(gps_L1_phaserange_minus_L1_pseudorange);

    unsigned int lock_time_indicator = 0;  // TODO
    DF013 = std::bitset<7>(lock_time_indicator);

    std::string content = DF009.to_string() +
            DF010.to_string() +
            DF011.to_string() +
            DF012.to_string() +
            DF013.to_string();

    std::bitset<58> content_msg(content);
    return content_msg;
}


std::string Rtcm::print_MT1001(const Gps_Ephemeris & gps_eph, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges)
{
    unsigned int ref_id = static_cast<unsigned int>(FLAGS_RTCM_Ref_Station_ID);
    unsigned int smooth_int = 0;
    bool sync_flag = false;
    bool divergence_free = false;

    std::bitset<64> header = Rtcm::get_MT1001_header(gps_eph, obs_time, pseudoranges, ref_id, smooth_int, sync_flag, divergence_free);
    std::string data = header.to_string();

    std::map<int, Gnss_Synchro>::const_iterator pseudoranges_iter;
    for(pseudoranges_iter = pseudoranges.begin();
            pseudoranges_iter != pseudoranges.end();
            pseudoranges_iter++)
        {
            std::bitset<58> content = Rtcm::get_MT1001_sat_content(pseudoranges_iter->second);
            data += content.to_string();
        }

    return Rtcm::build_message(data);
}




// **********************************************
//
//   MESSAGE TYPE 1005 (STATION DESCRIPTION)
//
// **********************************************


/* Stationary Antenna Reference Point, No Height Information
 * Reference Station Id = 2003
   GPS Service supported, but not GLONASS or Galileo
   ARP ECEF-X = 1114104.5999 meters
   ARP ECEF-Y = -4850729.7108 meters
   ARP ECEF-Z = 3975521.4643 meters
   Expected output: D3 00 13 3E D7 D3 02 02 98 0E DE EF 34 B4 BD 62
                    AC 09 41 98 6F 33 36 0B 98
 */
std::bitset<152> Rtcm::get_MT1005_test ()
{
    unsigned int mt1005 = 1005;
    unsigned int reference_station_id = 2003; // Max: 4095
    double ECEF_X = 1114104.5999;             // units: m
    double ECEF_Y = -4850729.7108;            // units: m
    double ECEF_Z = 3975521.4643;             // units: m

    std::bitset<1> DF001_;

    Rtcm::set_DF002(mt1005);
    Rtcm::set_DF003(reference_station_id);
    Rtcm::set_DF021();
    Rtcm::set_DF022(true);                // GPS
    Rtcm::set_DF023(false);               // Glonass
    Rtcm::set_DF024(false);               // Galileo
    DF141 = std::bitset<1>("0");          // 0: Real, physical reference station
    DF001_ = std::bitset<1>("0");         // Reserved, set to 0
    Rtcm::set_DF025(ECEF_X);
    DF142 = std::bitset<1>("0");          // Single Receiver Oscillator Indicator
    Rtcm::set_DF026(ECEF_Y);
    DF364 = std::bitset<2>("00");         // Quarter Cycle Indicator
    Rtcm::set_DF027(ECEF_Z);

    std::string message = DF002.to_string() +
            DF003.to_string() +
            DF021.to_string() +
            DF022.to_string() +
            DF023.to_string() +
            DF024.to_string() +
            DF141.to_string() +
            DF025.to_string() +
            DF142.to_string() +
            DF001_.to_string() +
            DF026.to_string() +
            DF364.to_string() +
            DF027.to_string() ;

    std::bitset<152> test_msg(message);
    return test_msg;
}


std::string Rtcm::print_MT1005( unsigned int ref_id, double ecef_x, double ecef_y, double ecef_z, bool gps, bool glonass, bool galileo, bool non_physical, bool single_oscillator, unsigned int quarter_cycle_indicator)
{
    unsigned int msg_number = 1005;
    std::bitset<1> DF001_;

    Rtcm::set_DF002(msg_number);
    Rtcm::set_DF003(ref_id);
    Rtcm::set_DF021();
    Rtcm::set_DF022(gps);
    Rtcm::set_DF023(glonass);
    Rtcm::set_DF024(galileo);
    DF141 = std::bitset<1>(non_physical);
    DF001_ = std::bitset<1>("0");
    Rtcm::set_DF025(ecef_x);
    DF142 = std::bitset<1>(single_oscillator);
    Rtcm::set_DF026(ecef_y);
    DF364 = std::bitset<2>(quarter_cycle_indicator);
    Rtcm::set_DF027(ecef_z);

    std::string data = DF002.to_string() +
            DF003.to_string() +
            DF021.to_string() +
            DF022.to_string() +
            DF023.to_string() +
            DF024.to_string() +
            DF141.to_string() +
            DF025.to_string() +
            DF142.to_string() +
            DF001_.to_string() +
            DF026.to_string() +
            DF364.to_string() +
            DF027.to_string() ;

    std::string message = build_message(data);
    return message;
}


int Rtcm::read_MT1005(const std::string & message, unsigned int & ref_id, double & ecef_x, double & ecef_y, double & ecef_z, bool & gps, bool & glonass, bool & galileo)
{
    // Convert message to binary
    std::string message_bin = Rtcm::hex_to_bin(message);

    if(!Rtcm::check_CRC(message) )
        {
            LOG(WARNING) << " Bad CRC detected in RTCM message MT1005";
            return 1;
        }

    // Check than the message number is correct
    unsigned int preamble_length = 8;
    unsigned int reserved_field_length = 6;
    unsigned int index = preamble_length + reserved_field_length;

    unsigned int read_message_length = static_cast<unsigned int>(Rtcm::bin_to_uint(message_bin.substr(index, 10)));
    index += 10;
    if (read_message_length != 19)
        {
            LOG(WARNING) << " Message MT1005 with wrong length (19 bytes expected, " << read_message_length << " received)";
            return 1;
        }

    unsigned int msg_number = 1005;
    Rtcm::set_DF002(msg_number);
    std::bitset<12> read_msg_number(message_bin.substr(index, 12));
    index += 12;

    if (DF002 != read_msg_number)
        {
            LOG(WARNING) << " This is not a MT1005 message";
            return 1;
        }


    ref_id = Rtcm::bin_to_uint(message_bin.substr(index, 12));
    index += 12;

    index += 6; // ITRF year
    gps = static_cast<bool>(Rtcm::bin_to_uint(message_bin.substr(index, 1)));
    index += 1;

    glonass = static_cast<bool>(Rtcm::bin_to_uint(message_bin.substr(index, 1)));
    index += 1;

    galileo = static_cast<bool>(Rtcm::bin_to_uint(message_bin.substr(index, 1)));
    index += 1;

    index += 1; // ref_station_indicator

    ecef_x = Rtcm::bin_to_double(message_bin.substr(index, 38)) / 10000.0;
    index += 38;

    index += 1; // single rx oscillator
    index += 1; // reserved

    ecef_y = Rtcm::bin_to_double(message_bin.substr(index, 38)) / 10000.0;
    index += 38;

    index += 2; // quarter cycle indicator
    ecef_z = Rtcm::bin_to_double(message_bin.substr(index, 38)) / 10000.0;

    return 0;
}


std::string Rtcm::print_MT1005_test()
{
    std::bitset<152> mt1005 = get_MT1005_test();
    return Rtcm::build_message(mt1005.to_string());
}



// **********************************************
//
//   MESSAGE TYPE 1019 (GPS EPHEMERIS)
//
// **********************************************

std::string Rtcm::print_MT1019(const Gps_Ephemeris & gps_eph)
{
    unsigned int msg_number = 1019;

    Rtcm::set_DF002(msg_number);
    Rtcm::set_DF009(gps_eph);
    Rtcm::set_DF076(gps_eph);
    Rtcm::set_DF077(gps_eph);
    Rtcm::set_DF078(gps_eph);
    Rtcm::set_DF079(gps_eph);
    Rtcm::set_DF071(gps_eph);
    Rtcm::set_DF081(gps_eph);
    Rtcm::set_DF082(gps_eph);
    Rtcm::set_DF083(gps_eph);
    Rtcm::set_DF084(gps_eph);
    Rtcm::set_DF085(gps_eph);
    Rtcm::set_DF086(gps_eph);
    Rtcm::set_DF087(gps_eph);
    Rtcm::set_DF088(gps_eph);
    Rtcm::set_DF089(gps_eph);
    Rtcm::set_DF090(gps_eph);
    Rtcm::set_DF091(gps_eph);
    Rtcm::set_DF092(gps_eph);
    Rtcm::set_DF093(gps_eph);
    Rtcm::set_DF094(gps_eph);
    Rtcm::set_DF095(gps_eph);
    Rtcm::set_DF096(gps_eph);
    Rtcm::set_DF097(gps_eph);
    Rtcm::set_DF098(gps_eph);
    Rtcm::set_DF099(gps_eph);
    Rtcm::set_DF100(gps_eph);
    Rtcm::set_DF101(gps_eph);
    Rtcm::set_DF102(gps_eph);
    Rtcm::set_DF103(gps_eph);
    Rtcm::set_DF137(gps_eph);

    std::string data;
    data.clear();
    data = DF002.to_string() +
            DF009.to_string() +
            DF076.to_string() +
            DF077.to_string() +
            DF078.to_string() +
            DF079.to_string() +
            DF071.to_string() +
            DF081.to_string() +
            DF082.to_string() +
            DF083.to_string() +
            DF084.to_string() +
            DF085.to_string() +
            DF086.to_string() +
            DF087.to_string() +
            DF088.to_string() +
            DF089.to_string() +
            DF090.to_string() +
            DF091.to_string() +
            DF092.to_string() +
            DF093.to_string() +
            DF094.to_string() +
            DF095.to_string() +
            DF096.to_string() +
            DF097.to_string() +
            DF098.to_string() +
            DF099.to_string() +
            DF100.to_string() +
            DF101.to_string() +
            DF102.to_string() +
            DF103.to_string() +
            DF137.to_string();

    if (data.length() != 488)
        {
            LOG(WARNING) << "Bad-formatted RTCM MT1019 (488 bits expected, found " <<  data.length() << ")";
        }

    message1019_content = std::bitset<488>(data);
    std::string message = build_message(data);
    return message;
}


int Rtcm::read_MT1019(const std::string & message, Gps_Ephemeris & gps_eph)
{
    // Convert message to binary
    std::string message_bin = Rtcm::hex_to_bin(message);

    if(!Rtcm::check_CRC(message) )
        {
            LOG(WARNING) << " Bad CRC detected in RTCM message MT1019";
            return 1;
        }

    unsigned int preamble_length = 8;
    unsigned int reserved_field_length = 6;
    unsigned int index = preamble_length + reserved_field_length;

    unsigned int read_message_length = static_cast<unsigned int>(Rtcm::bin_to_uint(message_bin.substr(index, 10)));
    index += 10;

    if (read_message_length != 61)
        {
            LOG(WARNING) << " Message MT1019 seems too long (61 bytes expected, " << read_message_length << " received)";
            return 1;
        }

    // Check than the message number is correct
    unsigned int read_msg_number = Rtcm::bin_to_uint(message_bin.substr(index, 12));
    index += 12;

    if (1019 != read_msg_number)
        {
            LOG(WARNING) << " This is not a MT1019 message";
            return 1;
        }

    // Fill Gps Ephemeris with message data content
    gps_eph.i_satellite_PRN = static_cast<unsigned int>(Rtcm::bin_to_uint(message_bin.substr(index, 6)));
    index += 6;

    gps_eph.i_GPS_week = static_cast<int>(Rtcm::bin_to_uint(message_bin.substr(index, 10)));
    index += 10;

    gps_eph.i_SV_accuracy = static_cast<int>(Rtcm::bin_to_uint(message_bin.substr(index, 4)));
    index += 4;

    gps_eph.i_code_on_L2 = static_cast<int>(Rtcm::bin_to_uint(message_bin.substr(index, 2)));
    index += 2;

    gps_eph.d_IDOT = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 14))) * I_DOT_LSB;
    index += 14;

    gps_eph.d_IODE_SF2 = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 8)));
    gps_eph.d_IODE_SF3 = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 8)));
    index += 8;

    gps_eph.d_Toc = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 16))) * T_OC_LSB;
    index += 16;

    gps_eph.d_A_f2 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 8))) * A_F2_LSB;
    index += 8;

    gps_eph.d_A_f1 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * A_F1_LSB;
    index += 16;

    gps_eph.d_A_f0 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 22))) * A_F0_LSB;
    index += 22;

    gps_eph.d_IODC = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 10)));
    index += 10;

    gps_eph.d_Crs = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * C_RS_LSB;
    index += 16;

    gps_eph.d_Delta_n = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * DELTA_N_LSB;
    index += 16;

    gps_eph.d_M_0 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 32))) * M_0_LSB;
    index += 32;

    gps_eph.d_Cuc = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * C_UC_LSB;
    index += 16;

    gps_eph.d_e_eccentricity = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 32))) * E_LSB;
    index += 32;

    gps_eph.d_Cus = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * C_US_LSB;
    index += 16;

    gps_eph.d_sqrt_A = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 32))) * SQRT_A_LSB;
    index += 32;

    gps_eph.d_Toe = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 16))) * T_OE_LSB;
    index += 16;

    gps_eph.d_Cic = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * C_IC_LSB;
    index += 16;

    gps_eph.d_OMEGA0 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 32))) * OMEGA_0_LSB;
    index += 32;

    gps_eph.d_Cis = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * C_IS_LSB;
    index += 16;

    gps_eph.d_i_0 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 32))) * I_0_LSB;
    index += 32;

    gps_eph.d_Crc = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * C_RC_LSB;
    index += 16;

    gps_eph.d_OMEGA = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 32))) * OMEGA_LSB;
    index += 32;

    gps_eph.d_OMEGA_DOT = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 24))) * OMEGA_DOT_LSB;
    index += 24;

    gps_eph.d_TGD = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 8))) * T_GD_LSB;
    index += 8;

    gps_eph.i_SV_health = static_cast<int>(Rtcm::bin_to_uint(message_bin.substr(index, 6)));
    index += 6;

    gps_eph.b_L2_P_data_flag = static_cast<bool>(Rtcm::bin_to_uint(message_bin.substr(index, 1)));
    index += 1;

    gps_eph.b_fit_interval_flag = static_cast<bool>(Rtcm::bin_to_uint(message_bin.substr(index, 1)));

    return 0;
}



// **********************************************
//
//   MESSAGE TYPE 1045 (GALILEO EPHEMERIS)
//
// **********************************************

std::string Rtcm::print_MT1045(const Galileo_Ephemeris & gal_eph)
{
    unsigned int msg_number = 1045;

    Rtcm::set_DF002(msg_number);
    Rtcm::set_DF252(gal_eph);
    Rtcm::set_DF289(gal_eph);
    Rtcm::set_DF290(gal_eph);
    Rtcm::set_DF291(gal_eph);
    Rtcm::set_DF293(gal_eph);
    Rtcm::set_DF294(gal_eph);
    Rtcm::set_DF295(gal_eph);
    Rtcm::set_DF296(gal_eph);
    Rtcm::set_DF297(gal_eph);
    Rtcm::set_DF298(gal_eph);
    Rtcm::set_DF299(gal_eph);
    Rtcm::set_DF300(gal_eph);
    Rtcm::set_DF301(gal_eph);
    Rtcm::set_DF302(gal_eph);
    Rtcm::set_DF303(gal_eph);
    Rtcm::set_DF304(gal_eph);
    Rtcm::set_DF305(gal_eph);
    Rtcm::set_DF306(gal_eph);
    Rtcm::set_DF307(gal_eph);
    Rtcm::set_DF308(gal_eph);
    Rtcm::set_DF309(gal_eph);
    Rtcm::set_DF310(gal_eph);
    Rtcm::set_DF311(gal_eph);
    Rtcm::set_DF312(gal_eph);
    Rtcm::set_DF314(gal_eph);
    Rtcm::set_DF315(gal_eph);
    unsigned int seven_zero = 0;
    std::bitset<7> DF001_ = std::bitset<7>(seven_zero);

    std::string data;
    data.clear();
    data = DF002.to_string() +
            DF252.to_string() +
            DF289.to_string() +
            DF290.to_string() +
            DF291.to_string() +
            DF292.to_string() +
            DF293.to_string() +
            DF294.to_string() +
            DF295.to_string() +
            DF296.to_string() +
            DF297.to_string() +
            DF298.to_string() +
            DF299.to_string() +
            DF300.to_string() +
            DF301.to_string() +
            DF302.to_string() +
            DF303.to_string() +
            DF304.to_string() +
            DF305.to_string() +
            DF306.to_string() +
            DF307.to_string() +
            DF308.to_string() +
            DF309.to_string() +
            DF310.to_string() +
            DF311.to_string() +
            DF312.to_string() +
            DF314.to_string() +
            DF315.to_string() +
            DF001_.to_string();

    if (data.length() != 496)
        {
            LOG(WARNING) << "Bad-formatted RTCM MT1045 (496 bits expected, found " <<  data.length() << ")";
        }
    message1045_content = std::bitset<496>(data);
    std::string message = build_message(data);
    return message;
}


int Rtcm::read_MT1045(const std::string & message, Galileo_Ephemeris & gal_eph)
{
    // Convert message to binary
    std::string message_bin = Rtcm::hex_to_bin(message);

    if(!Rtcm::check_CRC(message) )
        {
            LOG(WARNING) << " Bad CRC detected in RTCM message MT1045";
            return 1;
        }

    unsigned int preamble_length = 8;
    unsigned int reserved_field_length = 6;
    unsigned int index = preamble_length + reserved_field_length;

    unsigned int read_message_length = static_cast<unsigned int>(Rtcm::bin_to_uint(message_bin.substr(index, 10)));
    index += 10;

    if (read_message_length != 62)
        {
            LOG(WARNING) << " Message MT1045 seems too long (62 bytes expected, " << read_message_length << " received)";
            return 1;
        }

    // Check than the message number is correct
    unsigned int read_msg_number = Rtcm::bin_to_uint(message_bin.substr(index, 12));
    index += 12;

    if (1045 != read_msg_number)
        {
            LOG(WARNING) << " This is not a MT1045 message";
            return 1;
        }

    // Fill Galileo Ephemeris with message data content
    gal_eph.i_satellite_PRN = static_cast<unsigned int>(Rtcm::bin_to_uint(message_bin.substr(index, 6)));
    index += 6;

    gal_eph.WN_5 = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 12)));
    index += 12;

    gal_eph.IOD_nav_1  = static_cast<int>(Rtcm::bin_to_uint(message_bin.substr(index, 10)));
    index += 10;

    gal_eph.SISA_3  = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 8)));
    index += 8;

    gal_eph.iDot_2 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 14))) * iDot_2_LSB;
    index += 14;

    gal_eph.t0c_4 = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 14))) * t0c_4_LSB;
    index += 14;

    gal_eph.af2_4 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 6))) * af2_4_LSB;
    index += 6;

    gal_eph.af1_4 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 21))) * af1_4_LSB;
    index += 21;

    gal_eph.af0_4 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 31))) * af0_4_LSB;
    index += 31;

    gal_eph.C_rs_3 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * C_rs_3_LSB;
    index += 16;

    gal_eph.delta_n_3 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * delta_n_3_LSB;
    index += 16;

    gal_eph.M0_1 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 32))) * M0_1_LSB;
    index += 32;

    gal_eph.C_uc_3 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * C_uc_3_LSB;
    index += 16;

    gal_eph.e_1 = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 32))) * e_1_LSB;
    index += 32;

    gal_eph.C_us_3 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * C_us_3_LSB;
    index += 16;

    gal_eph.A_1 = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 32))) * A_1_LSB_gal;
    index += 32;

    gal_eph.t0e_1 = static_cast<double>(Rtcm::bin_to_uint(message_bin.substr(index, 14))) * t0e_1_LSB;
    index += 14;

    gal_eph.C_ic_4 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * C_ic_4_LSB;
    index += 16;

    gal_eph.OMEGA_0_2 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 32))) * OMEGA_0_2_LSB;
    index += 32;

    gal_eph.C_is_4 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16)))  * C_is_4_LSB;
    index += 16;

    gal_eph.i_0_2 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 32))) * i_0_2_LSB;
    index += 32;

    gal_eph.C_rc_3 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 16))) * C_rc_3_LSB;
    index += 16;

    gal_eph.omega_2 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 32))) * omega_2_LSB;
    index += 32;

    gal_eph.OMEGA_dot_3 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 24))) * OMEGA_dot_3_LSB;
    index += 24;

    gal_eph.BGD_E1E5a_5 = static_cast<double>(Rtcm::bin_to_int(message_bin.substr(index, 10)));
    index += 10;

    gal_eph.E5a_HS = static_cast<unsigned int>(Rtcm::bin_to_uint(message_bin.substr(index, 2)));
    index += 2;

    gal_eph.E5a_DVS = static_cast<bool>(Rtcm::bin_to_uint(message_bin.substr(index, 1)));

    return 0;
}


// *****************************************************************************************************
//
//   DATA FIELDS AS DEFINED AT RTCM STANDARD 10403.2
//
// *****************************************************************************************************

int Rtcm::reset_data_fields()
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

    DF364.reset();

    DF393.reset();
    DF394.reset();
    DF395.reset();

    DF409.reset();

    DF411.reset();
    DF412.reset();
    DF417.reset();
    DF418.reset();

    return 0;
}


int Rtcm::set_DF002(unsigned int message_number)
{
    if (message_number > 4095)
        {
            LOG(WARNING) << "RTCM message number must be between 0 and 4095, but it has been set to " << message_number;
        }
    DF002 = std::bitset<12>(message_number);
    return 0;
}


int Rtcm::set_DF003(unsigned int ref_station_ID)
{
    //unsigned int station_ID = ref_station_ID;
    if (ref_station_ID > 4095)
        {
            LOG(WARNING) << "RTCM reference station ID must be between 0 and 4095, but it has been set to " << ref_station_ID;
        }
    DF003 = std::bitset<12>(ref_station_ID);
    return 0;
}


int Rtcm::set_DF004(const Gps_Ephemeris & gps_eph, double obs_time)
{
    // TOW in milliseconds from the beginning of the GPS week, measured in GPS time
    unsigned long int tow = static_cast<unsigned long int>(std::round((obs_time + 604800 * static_cast<double>(gps_eph.i_GPS_week % 1024)) * 1000));
    if(tow > 604799999)
        {
            LOG(WARNING) << "To large TOW! Set to the last millisecond of the week";
            tow = 604799999;
        }
    DF004 = std::bitset<30>(tow);
    return 0;
}


int Rtcm::set_DF005(bool sync_flag)
{
    // 0 - No further GNSS observables referenced to the same Epoch Time will be transmitted. This enables the receiver to begin processing
    //     the data immediately after decoding the message.
    // 1 - The next message will contain observables of another GNSS source referenced to the same Epoch Time.
    DF005 = std::bitset<1>(sync_flag);
    return 0;
}


int Rtcm::set_DF006(const std::map<int, Gnss_Synchro> & pseudoranges)
{
    //Number of satellites observed in current epoch
    unsigned short int nsats = 0;
    std::map<int, Gnss_Synchro>::const_iterator pseudoranges_iter;
    for(pseudoranges_iter = pseudoranges.begin();
            pseudoranges_iter != pseudoranges.end();
            pseudoranges_iter++)
        {
            nsats++;
        }
    if (nsats > 31)
        {
            LOG(WARNING) << "The number of processed GPS satellites must be between 0 and 31, but it seems that you are processing " << nsats;
            nsats = 31;
        }
    DF006 = std::bitset<5>(nsats);
    return 0;
}


int Rtcm::set_DF007(bool divergence_free_smoothing_indicator)
{
    // 0 - Divergence-free smoothing not used 1 - Divergence-free smoothing used
    DF007 = std::bitset<1>(divergence_free_smoothing_indicator);
    return 0;
}


int Rtcm::set_DF008(short int smoothing_interval)
{
    DF008 = std::bitset<3>(smoothing_interval);
    return 0;
}


int Rtcm::set_DF009(const Gnss_Synchro & gnss_synchro)
{
    unsigned int prn_ = gnss_synchro.PRN;
    if(prn_ > 31)
        {
            LOG(WARNING) << "GPS satellite ID must be between 0 and 31, but PRN " << prn_ << " was found";
        }
    DF009 = std::bitset<6>(prn_);
    return 0;
}


int Rtcm::set_DF009(const Gps_Ephemeris & gps_eph)
{
    unsigned int prn_ = gps_eph.i_satellite_PRN;
    if(prn_ > 31)
        {
            LOG(WARNING) << "GPS satellite ID must be between 0 and 31, but PRN " << prn_ << " was found";
        }
    DF009 = std::bitset<6>(prn_);
    return 0;
}


int Rtcm::set_DF010(bool code_indicator)
{
    DF010 = std::bitset<1>(code_indicator);
    return 0;
}


int Rtcm::set_DF011(const Gnss_Synchro & gnss_synchro)
{
    unsigned long int gps_L1_pseudorange = static_cast<long unsigned int>(std::round(std::fmod(gnss_synchro.Pseudorange_m, 299792.458) / 0.02 ));
    DF011 = std::bitset<24>(gps_L1_pseudorange);
    return 0;
}


int Rtcm::set_DF012(const Gnss_Synchro & gnss_synchro)
{
    double L1_pseudorange = gnss_synchro.Pseudorange_m;
    //double L1_pseudorange_integers = std::floor(L1_pseudorange / 299792.458);
    double L1_pseudorange_field = std::fmod(L1_pseudorange, 299792.458);
    double L1_phaserange_m = (gnss_synchro.Carrier_phase_rads / GPS_TWO_PI) * GPS_C_m_s / GPS_L1_FREQ_HZ;

    long int gps_L1_phaserange_minus_L1_pseudorange =  static_cast<long int>((0 - L1_pseudorange_field) / 0.0005); ///////////////////////
    DF012 = std::bitset<20>(gps_L1_phaserange_minus_L1_pseudorange);
    return 0;
}


int Rtcm::set_DF014(const Gnss_Synchro & gnss_synchro)
{
    unsigned int gps_L1_pseudorange_ambiguity = static_cast<unsigned int>(std::floor(gnss_synchro.Pseudorange_m / 299792.458));
    DF014 = std::bitset<8>(gps_L1_pseudorange_ambiguity);
    return 0;
}


int Rtcm::set_DF015(const Gnss_Synchro & gnss_synchro)
{
    double CN0_dB_Hz_est = gnss_synchro.CN0_dB_hz;
    if (CN0_dB_Hz_est > 63.75)
        {
            CN0_dB_Hz_est = 63.75;
        }
    unsigned int CN0_dB_Hz = static_cast<unsigned int>(std::round(CN0_dB_Hz_est / 0.25 ));
    DF015 = std::bitset<8>(CN0_dB_Hz);
    return 0;
}


int Rtcm::set_DF021()
{
    unsigned short int itfr_year = 0;
    DF021 = std::bitset<6>(itfr_year);
    return 0;
}


int Rtcm::set_DF022(bool gps_indicator)
{
    DF022 = std::bitset<1>(gps_indicator);
    return 0;
}


int Rtcm::set_DF023(bool glonass_indicator)
{
    DF023 = std::bitset<1>(glonass_indicator);
    return 0;
}


int Rtcm::set_DF024(bool galileo_indicator)
{
    DF024 = std::bitset<1>(galileo_indicator);
    return 0;
}


int Rtcm::set_DF025(double antenna_ECEF_X_m)
{
    long long int ant_ref_x = static_cast<long long int>(std::round( antenna_ECEF_X_m * 10000));
    DF025 = std::bitset<38>(ant_ref_x);
    return 0;
}


int Rtcm::set_DF026(double antenna_ECEF_Y_m)
{
    long long int ant_ref_y = static_cast<long long int>(std::round( antenna_ECEF_Y_m * 10000));
    DF026 = std::bitset<38>(ant_ref_y);
    return 0;
}


int  Rtcm::set_DF027(double antenna_ECEF_Z_m)
{
    long long int ant_ref_z = static_cast<long long int>(std::round( antenna_ECEF_Z_m * 10000));
    DF027 = std::bitset<38>(ant_ref_z);
    return 0;
}


int Rtcm::set_DF071(const Gps_Ephemeris & gps_eph)
{
    unsigned int iode = static_cast<unsigned int>(gps_eph.d_IODE_SF2);
    DF071 = std::bitset<8>(iode);
    return 0;
}


int Rtcm::set_DF076(const Gps_Ephemeris & gps_eph)
{
    unsigned int week_number = static_cast<unsigned int>(gps_eph.i_GPS_week);
    DF076 = std::bitset<10>(week_number);
    return 0;
}


int Rtcm::set_DF077(const Gps_Ephemeris & gps_eph)
{
    unsigned short int ura = static_cast<unsigned short int>(gps_eph.i_SV_accuracy);
    DF077 = std::bitset<4>(ura);
    return 0;
}


int Rtcm::set_DF078(const Gps_Ephemeris & gps_eph)
{
    unsigned short int code_on_L2 = static_cast<unsigned short int>(gps_eph.i_code_on_L2);
    DF078 = std::bitset<2>(code_on_L2);
    return 0;
}


int Rtcm::set_DF079(const Gps_Ephemeris & gps_eph)
{
    unsigned int idot = static_cast<unsigned int>(std::round(gps_eph.d_IDOT / I_DOT_LSB ));
    DF079 = std::bitset<14>(idot);
    return 0;
}


int Rtcm::set_DF080(const Gps_Ephemeris & gps_eph)
{
    unsigned short int iode = static_cast<unsigned short int>(gps_eph.d_IODE_SF2);
    DF080 = std::bitset<8>(iode);
    return 0;
}


int Rtcm::set_DF081(const Gps_Ephemeris & gps_eph)
{
    unsigned int toc = static_cast<unsigned int>(std::round(gps_eph.d_Toc / T_OC_LSB ));
    DF081 = std::bitset<16>(toc);
    return 0;
}


int Rtcm::set_DF082(const Gps_Ephemeris & gps_eph)
{
    short int af2 = static_cast<short int>(std::round(gps_eph.d_A_f2 / A_F2_LSB ));
    DF082 = std::bitset<8>(af2);
    return 0;
}


int Rtcm::set_DF083(const Gps_Ephemeris & gps_eph)
{
    int af1 = static_cast<int>(std::round(gps_eph.d_A_f1 / A_F1_LSB ));
    DF083 = std::bitset<16>(af1);
    return 0;
}


int Rtcm::set_DF084(const Gps_Ephemeris & gps_eph)
{
    long int af0 = static_cast<long int>(std::round(gps_eph.d_A_f0 / A_F0_LSB ));
    DF084 = std::bitset<22>(af0);
    return 0;
}


int Rtcm::set_DF085(const Gps_Ephemeris & gps_eph)
{
    unsigned int iodc = static_cast<unsigned int>(gps_eph.d_IODC);
    DF085 = std::bitset<10>(iodc);
    return 0;
}


int Rtcm::set_DF086(const Gps_Ephemeris & gps_eph)
{
    int crs = static_cast<int>(std::round(gps_eph.d_Crs / C_RS_LSB ));
    DF086 = std::bitset<16>(crs);
    return 0;
}


int Rtcm::set_DF087(const Gps_Ephemeris & gps_eph)
{
    int delta_n = static_cast<int>(std::round(gps_eph.d_Delta_n / DELTA_N_LSB ));
    DF087 = std::bitset<16>(delta_n);
    return 0;
}


int Rtcm::set_DF088(const Gps_Ephemeris & gps_eph)
{
    long int m0 = static_cast<long int>(std::round(gps_eph.d_M_0 / M_0_LSB ));
    DF088 = std::bitset<32>(m0);
    return 0;
}


int Rtcm::set_DF089(const Gps_Ephemeris & gps_eph)
{
    int cuc = static_cast<int>(std::round(gps_eph.d_Cuc / C_UC_LSB ));
    DF089 = std::bitset<16>(cuc);
    return 0;
}

int Rtcm::set_DF090(const Gps_Ephemeris & gps_eph)
{
    unsigned long int ecc = static_cast<unsigned long int>(std::round(gps_eph.d_e_eccentricity / E_LSB ));
    DF090 = std::bitset<32>(ecc);
    return 0;
}


int Rtcm::set_DF091(const Gps_Ephemeris & gps_eph)
{
    int cus = static_cast<int>(std::round(gps_eph.d_Cus / C_US_LSB ));
    DF091 = std::bitset<16>(cus);
    return 0;
}


int Rtcm::set_DF092(const Gps_Ephemeris & gps_eph)
{
    unsigned long int sqr_a = static_cast<unsigned long int>(std::round(gps_eph.d_sqrt_A / SQRT_A_LSB ));
    DF092 = std::bitset<32>(sqr_a);
    return 0;
}


int Rtcm::set_DF093(const Gps_Ephemeris & gps_eph)
{
    unsigned int toe = static_cast<unsigned int>(std::round(gps_eph.d_Toe / T_OE_LSB ));
    DF093 = std::bitset<16>(toe);
    return 0;
}


int Rtcm::set_DF094(const Gps_Ephemeris & gps_eph)
{
    int cic = static_cast<int>(std::round(gps_eph.d_Cic / C_IC_LSB ));
    DF094 = std::bitset<16>(cic);
    return 0;
}


int Rtcm::set_DF095(const Gps_Ephemeris & gps_eph)
{
    long int Omega0 = static_cast<long int>(std::round(gps_eph.d_OMEGA0 / OMEGA_0_LSB ));
    DF095 = std::bitset<32>(Omega0);
    return 0;
}


int Rtcm::set_DF096(const Gps_Ephemeris & gps_eph)
{
    int cis = static_cast<int>(std::round(gps_eph.d_Cis / C_IS_LSB ));
    DF096 = std::bitset<16>(cis);
    return 0;
}


int Rtcm::set_DF097(const Gps_Ephemeris & gps_eph)
{
    long int i0 = static_cast<long int>(std::round(gps_eph.d_i_0 / I_0_LSB ));
    DF097 = std::bitset<32>(i0);
    return 0;
}


int Rtcm::set_DF098(const Gps_Ephemeris & gps_eph)
{
    int crc = static_cast<int>(std::round(gps_eph.d_Crc / C_RC_LSB ));
    DF098 = std::bitset<16>(crc);
    return 0;
}


int Rtcm::set_DF099(const Gps_Ephemeris & gps_eph)
{
    long int omega = static_cast<long int>(std::round(gps_eph.d_OMEGA / OMEGA_LSB ));
    DF099 = std::bitset<32>(omega);
    return 0;
}


int Rtcm::set_DF100(const Gps_Ephemeris & gps_eph)
{
    long int omegadot = static_cast<long int>(std::round(gps_eph.d_OMEGA_DOT / OMEGA_DOT_LSB ));
    DF100 = std::bitset<24>(omegadot);
    return 0;
}


int Rtcm::set_DF101(const Gps_Ephemeris & gps_eph)
{
    short int tgd = static_cast<short int>(std::round(gps_eph.d_TGD / T_GD_LSB ));
    DF101 = std::bitset<8>(tgd);
    return 0;
}


int Rtcm::set_DF102(const Gps_Ephemeris & gps_eph)
{
    unsigned short int sv_heath = static_cast<unsigned short int>(gps_eph.i_SV_health);
    DF102 = std::bitset<6>(sv_heath);
    return 0;
}


int Rtcm::set_DF103(const Gps_Ephemeris & gps_eph)
{
    DF103 = std::bitset<1>(gps_eph.b_L2_P_data_flag);
    return 0;
}




int Rtcm::set_DF137(const Gps_Ephemeris & gps_eph)
{
    DF137 = std::bitset<1>(gps_eph.b_fit_interval_flag);
    return 0;
}



int Rtcm::set_DF252(const Galileo_Ephemeris & gal_eph)
{
    unsigned int prn_ = gal_eph.i_satellite_PRN;
    if(prn_ > 63)
        {
            LOG(WARNING) << "Galileo satellite ID must be between 0 and 63, but PRN " << prn_ << " was found";
        }
    DF252 = std::bitset<6>(prn_);
    return 0;
}


int Rtcm::set_DF289(const Galileo_Ephemeris & gal_eph)
{
    unsigned int galileo_week_number = static_cast<unsigned int>(gal_eph.WN_5);
    if(galileo_week_number > 4095)
        {
            LOG(WARNING) << "Error decoding Galileo week number (it has a 4096 roll-off, but " << galileo_week_number << " was detected)";
        }
    DF289 = std::bitset<12>(galileo_week_number);
    return 0;
}


int Rtcm::set_DF290(const Galileo_Ephemeris & gal_eph)
{
    unsigned int iod_nav = static_cast<unsigned int>(gal_eph.IOD_nav_1);
    if(iod_nav > 1023)
        {
            LOG(WARNING) << "Error decoding Galileo IODnav (it has a max of 1023, but " << iod_nav << " was detected)";
        }
    DF290 = std::bitset<10>(iod_nav);
    return 0;
}


int Rtcm::set_DF291(const Galileo_Ephemeris & gal_eph)
{
    unsigned short int SISA = static_cast<unsigned short int>(gal_eph.SISA_3);
    //SISA = 0; // SIS Accuracy, data content definition not given in Galileo OS SIS ICD, Issue 1.1, Sept 2010
    DF291 = std::bitset<8>(SISA);
    return 0;
}


int Rtcm::set_DF292(const Galileo_Ephemeris & gal_eph)
{

    int idot = static_cast<int>(std::round(gal_eph.iDot_2 / FNAV_idot_2_LSB));
    DF292 = std::bitset<14>(idot);
    return 0;
}

int Rtcm::set_DF293(const Galileo_Ephemeris & gal_eph)
{

    unsigned int toc = static_cast<unsigned int>(gal_eph.t0c_4);
    if(toc > 604740)
        {
            LOG(WARNING) << "Error decoding Galileo ephemeris time (max of 604740, but " << toc << " was detected)";
        }
    DF293 = std::bitset<14>(toc);
    return 0;
}


int Rtcm::set_DF294(const Galileo_Ephemeris & gal_eph)
{
    short int af2 = static_cast<short int>(std::round(gal_eph.af2_4 / FNAV_af2_1_LSB));
    DF294 = std::bitset<6>(af2);
    return 0;
}


int Rtcm::set_DF295(const Galileo_Ephemeris & gal_eph)
{
    long int af1 = static_cast<long int>(std::round(gal_eph.af1_4 / FNAV_af1_1_LSB));
    DF295 = std::bitset<21>(af1);
    return 0;
}


int Rtcm::set_DF296(const Galileo_Ephemeris & gal_eph)
{
    long int af0 = static_cast<unsigned int>(std::round(gal_eph.af0_4 / FNAV_af0_1_LSB));
    DF296 = std::bitset<31>(af0);
    return 0;
}


int Rtcm::set_DF297(const Galileo_Ephemeris & gal_eph)
{
    int crs = static_cast<int>(std::round(gal_eph.C_rs_3 / FNAV_Crs_3_LSB));
    DF297 = std::bitset<16>(crs);
    return 0;
}


int Rtcm::set_DF298(const Galileo_Ephemeris & gal_eph)
{
    int delta_n = static_cast<int>(std::round(gal_eph.delta_n_3 / FNAV_deltan_3_LSB));
    DF298 = std::bitset<16>(delta_n);
    return 0;
}


int Rtcm::set_DF299(const Galileo_Ephemeris & gal_eph)
{
    long int m0 = static_cast<long int>(std::round(gal_eph.M0_1 / FNAV_M0_2_LSB));
    DF299 = std::bitset<32>(m0);
    return 0;
}


int Rtcm::set_DF300(const Galileo_Ephemeris & gal_eph)
{
    int cuc = static_cast<unsigned int>(std::round(gal_eph.C_uc_3 / FNAV_Cuc_3_LSB));
    DF300 = std::bitset<16>(cuc);
    return 0;
}

int Rtcm::set_DF301(const Galileo_Ephemeris & gal_eph)
{
    unsigned long int ecc = static_cast<unsigned long int>(std::round(gal_eph.e_1 / FNAV_e_2_LSB));
    DF301 = std::bitset<32>(ecc);
    return 0;
}


int Rtcm::set_DF302(const Galileo_Ephemeris & gal_eph)
{
    int cus = static_cast<int>(std::round(gal_eph.C_us_3 / FNAV_Cus_3_LSB));
    DF302 = std::bitset<16>(cus);
    return 0;
}


int Rtcm::set_DF303(const Galileo_Ephemeris & gal_eph)
{
    unsigned long int sqr_a = static_cast<unsigned long int>(std::round(gal_eph.A_1 / FNAV_a12_2_LSB));
    DF303 = std::bitset<32>(sqr_a);
    return 0;
}



int Rtcm::set_DF304(const Galileo_Ephemeris & gal_eph)
{
    unsigned int toe = static_cast<unsigned int>(std::round(gal_eph.t0e_1 / FNAV_t0e_3_LSB));
    DF304 = std::bitset<14>(toe);
    return 0;
}


int Rtcm::set_DF305(const Galileo_Ephemeris & gal_eph)
{
    int cic = static_cast<int>(std::round(gal_eph.C_ic_4 / FNAV_Cic_4_LSB));
    DF305 = std::bitset<16>(cic);
    return 0;
}


int Rtcm::set_DF306(const Galileo_Ephemeris & gal_eph)
{
    long int Omega0 = static_cast<long int>(std::round(gal_eph.OMEGA_0_2 / FNAV_omega0_2_LSB));
    DF306 = std::bitset<32>(Omega0);
    return 0;
}


int Rtcm::set_DF307(const Galileo_Ephemeris & gal_eph)
{
    int cis = static_cast<int>(std::round(gal_eph.C_is_4 / FNAV_Cis_4_LSB));
    DF307 = std::bitset<16>(cis);
    return 0;
}


int Rtcm::set_DF308(const Galileo_Ephemeris & gal_eph)
{
    long int i0 = static_cast<long int>(std::round(gal_eph.i_0_2 / FNAV_i0_3_LSB));
    DF308 = std::bitset<32>(i0);
    return 0;
}


int Rtcm::set_DF309(const Galileo_Ephemeris & gal_eph)
{
    int crc = static_cast<unsigned int>(std::round(gal_eph.C_rc_3 / FNAV_Crc_3_LSB));
    DF309 = std::bitset<16>(crc);
    return 0;
}


int Rtcm::set_DF310(const Galileo_Ephemeris & gal_eph)
{
    int omega = static_cast<int>(std::round(gal_eph.omega_2 / FNAV_omega0_2_LSB));
    DF310 = std::bitset<32>(omega);
    return 0;
}


int Rtcm::set_DF311(const Galileo_Ephemeris & gal_eph)
{
    long int Omegadot = static_cast<long int>(std::round(gal_eph.OMEGA_dot_3 / FNAV_omegadot_2_LSB));
    DF311 = std::bitset<24>(Omegadot);
    return 0;
}


int Rtcm::set_DF312(const Galileo_Ephemeris & gal_eph)
{
    int bdg_E1_E5a = static_cast<int>(std::round(gal_eph.BGD_E1E5a_5 / FNAV_BGD_1_LSB));
    DF312 = std::bitset<10>(bdg_E1_E5a);
    return 0;
}


int Rtcm::set_DF313(const Galileo_Ephemeris & gal_eph)
{
    unsigned int bdg_E5b_E1 = static_cast<unsigned int>(std::round(gal_eph.BGD_E1E5b_5 ));
    //bdg_E5b_E1 = 0; //reserved
    DF313 = std::bitset<10>(bdg_E5b_E1);
    return 0;
}


int Rtcm::set_DF314(const Galileo_Ephemeris & gal_eph)
{
    DF314 = std::bitset<2>(gal_eph.E5a_HS);
    return 0;
}


int Rtcm::set_DF315(const Galileo_Ephemeris & gal_eph)
{
    DF315 = std::bitset<1>(gal_eph.E5a_DVS);
    return 0;
}



int Rtcm::set_DF393(bool more_messages)
{
    DF393 = std::bitset<1>(more_messages);
    return 0;
}


int Rtcm::set_DF394(const std::map<int, Gnss_Synchro> & gnss_synchro)
{
    DF394.reset();
    std::map<int, Gnss_Synchro>::const_iterator gnss_synchro_iter;
    unsigned int mask_position;
    for(gnss_synchro_iter = gnss_synchro.begin();
            gnss_synchro_iter != gnss_synchro.end();
            gnss_synchro_iter++)
        {
            mask_position = 65 - gnss_synchro_iter->second.PRN;
            DF394.set(mask_position, true);
        }
    return 0;
}


int Rtcm::set_DF395(const std::map<int, Gnss_Synchro> & gnss_synchro)
{
    DF395.reset();
    std::map<int, Gnss_Synchro>::const_iterator gnss_synchro_iter;
    std::string sig;
    unsigned int mask_position;
    for(gnss_synchro_iter = gnss_synchro.begin();
            gnss_synchro_iter != gnss_synchro.end();
            gnss_synchro_iter++)
        {
            std::string sig_(gnss_synchro_iter->second.Signal);
            sig = sig_.substr(0,2);

            std::string sys(gnss_synchro_iter->second.System, 1);

            if ((sig.compare("1C") == 0) && (sys.compare("G") == 0 ) )
                {
                    mask_position = 33 - 2;
                    DF395.set(mask_position, true);
                }
            if ((sig.compare("2S") == 0) && (sys.compare("G") == 0 ) )
                {
                    mask_position = 33 - 15;
                    DF395.set(mask_position, true);
                }

            if ((sig.compare("5X") == 0) && (sys.compare("G") == 0 ) )
                {
                    mask_position = 33 - 24;
                    DF395.set(mask_position, true);
                }
            if ((sig.compare("1B") == 0) && (sys.compare("E") == 0 ) )
                {
                    mask_position = 33 - 4;
                    DF395.set(mask_position, true);
                }

            if ((sig.compare("5X") == 0) && (sys.compare("E") == 0 ) )
                {
                    mask_position = 33 - 24;
                    DF395.set(mask_position, true);
                }
            if ((sig.compare("7X") == 0) && (sys.compare("E") == 0 ) )
                {
                    mask_position = 33 - 16;
                    DF395.set(mask_position, true);
                }
        }
    return 0;
}


int Rtcm::set_DF409(unsigned int iods)
{
    DF409 = std::bitset<3>(iods);
    return 0;
}


int Rtcm::set_DF417(bool using_divergence_free_smoothing)
{
    DF417 = std::bitset<1>(using_divergence_free_smoothing);
    return 0;
}
