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
#include <chrono>     // std::chrono::seconds
#include <cmath>      // for std::fmod
#include <cstdlib>    // for strtol
#include <sstream>    // for std::stringstream
#include <thread>
#include <boost/algorithm/string.hpp>  // for to_upper_copy
#include <boost/crc.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/dynamic_bitset.hpp>
#include <glog/logging.h>
#include "Galileo_E1.h"
#include "GPS_L2C.h"

using google::LogMessage;


Rtcm::Rtcm(unsigned short port)
{
    RTCM_port = port;
    preamble = std::bitset<8>("11010011");
    reserved_field = std::bitset<6>("000000");
    rtcm_message_queue = std::make_shared< concurrent_queue<std::string> >();
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), RTCM_port);
    servers.emplace_back(io_service, endpoint);
    server_is_running = false;
}


Rtcm::~Rtcm()
{
    if(server_is_running)
        {
            try
            {
                    stop_server();
            }
            catch(const boost::exception & e)
            {
                    LOG(WARNING) << "Boost exception: " << boost::diagnostic_information(e);
            }
            catch(const std::exception & ex)
            {
                    LOG(WARNING) << "STD exception: " << ex.what();
            }
        }
}



// *****************************************************************************************************
//
//   TCP Server helper classes
//
// *****************************************************************************************************
void Rtcm::run_server()
{
    std::cout << "Starting a TCP Server on port " << RTCM_port << std::endl;
    try
    {
            std::thread tq([&]{ std::make_shared<Queue_Reader>(io_service, rtcm_message_queue, RTCM_port)->do_read_queue(); });
            tq.detach();

            std::thread t([&]{ io_service.run(); });
            server_is_running = true;
            t.detach();
    }
    catch (const std::exception & e)
    {
            std::cerr << "Exception: " << e.what() << "\n";
    }
}


void Rtcm::stop_service()
{
    io_service.stop();
}


void Rtcm::stop_server()
{
    std::cout << "Stopping TCP Server on port " << RTCM_port << std::endl;
    rtcm_message_queue->push("Goodbye"); // this terminates tq
    Rtcm::stop_service();
    servers.front().close_server();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    server_is_running = false;
}


void Rtcm::send_message(const std::string & msg)
{
    rtcm_message_queue->push(msg);
}


bool Rtcm::is_server_running() const
{
    return server_is_running;
}


// *****************************************************************************************************
//
//   TRANSPORT LAYER AS DEFINED AT RTCM STANDARD 10403.2
//
// *****************************************************************************************************

std::string Rtcm::add_CRC (const std::string & message_without_crc) const
{
    // ******  Computes Qualcomm CRC-24Q ******
    boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> CRC_RTCM;
    // 1) Converts the string to a vector of unsigned char:
    boost::dynamic_bitset<unsigned char> frame_bits(message_without_crc);
    std::vector<unsigned char> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(), bytes.end());

    // 2) Computes CRC
    CRC_RTCM.process_bytes(bytes.data(), bytes.size());
    std::bitset<24> crc_frame = std::bitset<24>(CRC_RTCM.checksum());

    // 3) Builds the complete message
    std::string complete_message = message_without_crc + crc_frame.to_string();
    return bin_to_binary_data(complete_message);
}


bool Rtcm::check_CRC(const std::string & message) const
{
    boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> CRC_RTCM_CHECK;
    // Convert message to binary
    std::string message_bin = Rtcm::binary_data_to_bin(message);
    // Check CRC
    std::string crc = message_bin.substr(message_bin.length() - 24, 24);
    std::bitset<24> read_crc =  std::bitset<24>(crc);
    std::string msg_without_crc = message_bin.substr(0, message_bin.length() - 24);

    boost::dynamic_bitset<unsigned char> frame_bits(msg_without_crc);
    std::vector<unsigned char> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(), bytes.end());

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


std::string Rtcm::bin_to_binary_data(const std::string& s) const
{
    std::string s_aux;
    int remainder = static_cast<int>(std::fmod(s.length(), 8));
    unsigned char c[s.length()];
    unsigned int k = 0;
    if (remainder != 0)
        {
            s_aux.assign(s, 0 , remainder);
            boost::dynamic_bitset<> rembits(s_aux);
            unsigned long int n = rembits.to_ulong();
            c[0] = static_cast<unsigned char>(n);
            k++;
        }

    unsigned int start = std::max(remainder, 0);
    for(unsigned int i = start; i < s.length() - 1; i = i + 8)
        {
            s_aux.assign(s, i, 4);
            std::bitset<4> bs(s_aux);
            unsigned n = bs.to_ulong();
            s_aux.assign(s, i + 4 , 4);
            std::bitset<4> bs2(s_aux);
            unsigned n2 = bs2.to_ulong();
            c[k] = static_cast<unsigned char>(n * 16) + static_cast<unsigned char>(n2);
            k++;
        }

    std::string ret(c, c + k / sizeof(c[0]));
    return ret;
}


std::string Rtcm::binary_data_to_bin(const std::string& s) const
{
    std::string s_aux;
    std::stringstream ss;

    for(unsigned int i = 0; i < s.length(); i++)
        {
            unsigned char val = static_cast<unsigned char>(s.at(i));
            std::bitset<8> bs(val);
            ss << bs;
        }

    s_aux = ss.str();
    return s_aux;
}


std::string Rtcm::bin_to_hex(const std::string& s) const
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


std::string Rtcm::hex_to_bin(const std::string& s) const
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


unsigned long int Rtcm::bin_to_uint(const std::string& s) const
{
    if(s.length() > 32)
        {
            LOG(WARNING) << "Cannot convert to a unsigned long int";
            return 0;
        }
    unsigned long int reading = strtoul(s.c_str(), NULL, 2);
    return reading;
}


long int Rtcm::bin_to_int(const std::string& s) const
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


double Rtcm::bin_to_double(const std::string& s) const
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
            std::string aux;
            to_string(original_bitset, aux);
            reading_int = - (strtoll(aux.c_str(), NULL, 2) + 1);
        }
    else
        {
            reading_int = strtoll(s.c_str(), NULL, 2);
        }

    reading = static_cast<double>(reading_int);
    return reading;
}


unsigned long int Rtcm::hex_to_uint(const std::string& s) const
{
    if(s.length() > 32)
        {
            LOG(WARNING) << "Cannot convert to a unsigned long int";
            return 0;
        }
    unsigned long int reading = strtoul(s.c_str(), NULL, 16);
    return reading;
}


long int Rtcm::hex_to_int(const std::string& s) const
{
    if(s.length() > 32)
        {
            LOG(WARNING) << "Cannot convert to a long int";
            return 0;
        }
    long int reading = strtol(s.c_str(), NULL, 16);
    return reading;
}


std::string Rtcm::build_message(const std::string & data) const
{
    unsigned int msg_length_bits = data.length();
    unsigned int msg_length_bytes = std::ceil(static_cast<float>(msg_length_bits) / 8.0);
    std::bitset<10> message_length = std::bitset<10>(msg_length_bytes);
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



// ********************************************************
//
//   MESSAGE TYPE 1001 (GPS L1 OBSERVATIONS)
//
// ********************************************************

std::bitset<64> Rtcm::get_MT1001_4_header(unsigned int msg_number, double obs_time, const std::map<int, Gnss_Synchro> & observables,
        unsigned int ref_id, unsigned int smooth_int, bool sync_flag, bool divergence_free)
{
    unsigned int reference_station_id = ref_id; // Max: 4095
    const std::map<int, Gnss_Synchro> observables_ = observables;
    bool synchronous_GNSS_flag = sync_flag;
    bool divergence_free_smoothing_indicator = divergence_free;
    unsigned int smoothing_interval = smooth_int;
    Rtcm::set_DF002(msg_number);
    Rtcm::set_DF003(reference_station_id);
    Rtcm::set_DF004(obs_time);
    Rtcm::set_DF005(synchronous_GNSS_flag);
    Rtcm::set_DF006(observables_);
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


std::bitset<58> Rtcm::get_MT1001_sat_content(const Gps_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro)
{
    bool code_indicator = false; // code indicator   0: C/A code   1: P(Y) code direct
    Rtcm::set_DF009(gnss_synchro);
    Rtcm::set_DF010(code_indicator); // code indicator   0: C/A code   1: P(Y) code direct
    Rtcm::set_DF011(gnss_synchro);
    Rtcm::set_DF012(gnss_synchro);
    Rtcm::set_DF013(eph, obs_time, gnss_synchro);

    std::string content = DF009.to_string() +
            DF010.to_string() +
            DF011.to_string() +
            DF012.to_string() +
            DF013.to_string();

    std::bitset<58> content_msg(content);
    return content_msg;
}


std::string Rtcm::print_MT1001(const Gps_Ephemeris & gps_eph, double obs_time, const std::map<int, Gnss_Synchro> & observables, unsigned short station_id)
{
    unsigned int ref_id = static_cast<unsigned int>(station_id);
    unsigned int smooth_int = 0;
    bool sync_flag = false;
    bool divergence_free = false;

    //Get a map with GPS L1 only observations
    std::map<int, Gnss_Synchro> observablesL1;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;

    for(observables_iter = observables.cbegin();
            observables_iter != observables.cend();
            observables_iter++)
        {
            std::string system_(&observables_iter->second.System, 1);
            std::string sig_(observables_iter->second.Signal);
            if((system_.compare("G") == 0) && (sig_.compare("1C") == 0))
                {
                    observablesL1.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::bitset<64> header = Rtcm::get_MT1001_4_header(1001, obs_time, observablesL1, ref_id, smooth_int, sync_flag, divergence_free);
    std::string data = header.to_string();

    for(observables_iter = observablesL1.cbegin();
            observables_iter != observablesL1.cend();
            observables_iter++)
        {
            std::bitset<58> content = Rtcm::get_MT1001_sat_content(gps_eph, obs_time, observables_iter->second);
            data += content.to_string();
        }

    std::string msg = build_message(data);
    if(server_is_running)
        {
            rtcm_message_queue->push(msg);
        }
    return msg;
}



// ********************************************************
//
//   MESSAGE TYPE 1002 (EXTENDED GPS L1 OBSERVATIONS)
//
// ********************************************************

std::string Rtcm::print_MT1002(const Gps_Ephemeris & gps_eph, double obs_time, const std::map<int, Gnss_Synchro> & observables, unsigned short station_id)
{
    unsigned int ref_id = static_cast<unsigned int>(station_id);
    unsigned int smooth_int = 0;
    bool sync_flag = false;
    bool divergence_free = false;

    //Get a map with GPS L1 only observations
    std::map<int, Gnss_Synchro> observablesL1;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;

    for(observables_iter = observables.cbegin();
            observables_iter != observables.cend();
            observables_iter++)
        {
            std::string system_(&observables_iter->second.System, 1);
            std::string sig_(observables_iter->second.Signal);
            if((system_.compare("G") == 0) && (sig_.compare("1C") == 0))
                {
                    observablesL1.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    std::bitset<64> header = Rtcm::get_MT1001_4_header(1002, obs_time, observablesL1, ref_id, smooth_int, sync_flag, divergence_free);
    std::string data = header.to_string();

    for(observables_iter = observablesL1.cbegin();
            observables_iter != observablesL1.cend();
            observables_iter++)
        {
            std::bitset<74> content = Rtcm::get_MT1002_sat_content(gps_eph, obs_time, observables_iter->second);
            data += content.to_string();
        }

    std::string msg = build_message(data);
    if(server_is_running)
        {
            rtcm_message_queue->push(msg);
        }
    return msg;
}


std::bitset<74> Rtcm::get_MT1002_sat_content(const Gps_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro)
{
    bool code_indicator = false; // code indicator   0: C/A code   1: P(Y) code direct
    Rtcm::set_DF009(gnss_synchro);
    Rtcm::set_DF010(code_indicator); // code indicator   0: C/A code   1: P(Y) code direct
    Rtcm::set_DF011(gnss_synchro);
    Rtcm::set_DF012(gnss_synchro);
    Rtcm::set_DF013(eph, obs_time, gnss_synchro);

    std::string content = DF009.to_string() +
            DF010.to_string() +
            DF011.to_string() +
            DF012.to_string() +
            DF013.to_string() +
            DF014.to_string() +
            DF015.to_string();

    std::bitset<74> content_msg(content);
    return content_msg;
}



// ********************************************************
//
//   MESSAGE TYPE 1003 (GPS L1 & L2 OBSERVATIONS)
//
// ********************************************************

std::string Rtcm::print_MT1003(const Gps_Ephemeris & ephL1, const Gps_CNAV_Ephemeris & ephL2, double obs_time, const std::map<int, Gnss_Synchro> & observables, unsigned short station_id)
{
    unsigned int ref_id = static_cast<unsigned int>(station_id);
    unsigned int smooth_int = 0;
    bool sync_flag = false;
    bool divergence_free = false;

    //Get maps with GPS L1 and L2 observations
    std::map<int, Gnss_Synchro> observablesL1;
    std::map<int, Gnss_Synchro> observablesL2;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter2;

    for(observables_iter = observables.cbegin();
            observables_iter != observables.cend();
            observables_iter++)
        {
            std::string system_(&observables_iter->second.System, 1);
            std::string sig_(observables_iter->second.Signal);
            if((system_.compare("G") == 0) && (sig_.compare("1C") == 0))
                {
                    observablesL1.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("G") == 0) && (sig_.compare("2S") == 0))
                {
                    observablesL2.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    // Get common observables
    std::vector< std::pair< Gnss_Synchro, Gnss_Synchro > >  common_observables;
    std::vector< std::pair< Gnss_Synchro, Gnss_Synchro > >::const_iterator common_observables_iter;
    std::map<int, Gnss_Synchro> observablesL1_with_L2;

    for(observables_iter = observablesL1.cbegin();
            observables_iter != observablesL1.cend();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            for(observables_iter2 = observablesL2.cbegin();
                    observables_iter2 != observablesL2.cend();
                    observables_iter2++)
                {
                    if(observables_iter2->second.PRN == prn_)
                        {
                            std::pair<Gnss_Synchro, Gnss_Synchro> p;
                            Gnss_Synchro pr1 = observables_iter->second;
                            Gnss_Synchro pr2 = observables_iter2->second;
                            p = std::make_pair(pr1, pr2);
                            common_observables.push_back(p);
                            observablesL1_with_L2.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                        }
                }
        }

    std::bitset<64> header = Rtcm::get_MT1001_4_header(1003, obs_time, observablesL1_with_L2, ref_id, smooth_int, sync_flag, divergence_free);
    std::string data = header.to_string();

    for(common_observables_iter = common_observables.cbegin();
            common_observables_iter != common_observables.cend();
            common_observables_iter++)
        {
            std::bitset<101> content = Rtcm::get_MT1003_sat_content(ephL1, ephL2, obs_time, common_observables_iter->first, common_observables_iter->second);
            data += content.to_string();
        }

    std::string msg = build_message(data);
    if(server_is_running)
        {
            rtcm_message_queue->push(msg);
        }
    return msg;
}


std::bitset<101> Rtcm::get_MT1003_sat_content(const Gps_Ephemeris & ephL1, const Gps_CNAV_Ephemeris & ephL2, double obs_time, const Gnss_Synchro & gnss_synchroL1, const Gnss_Synchro & gnss_synchroL2)
{
    bool code_indicator = false; // code indicator   0: C/A code   1: P(Y) code direct
    Rtcm::set_DF009(gnss_synchroL1);
    Rtcm::set_DF010(code_indicator); // code indicator   0: C/A code   1: P(Y) code direct
    Rtcm::set_DF011(gnss_synchroL1);
    Rtcm::set_DF012(gnss_synchroL1);
    Rtcm::set_DF013(ephL1, obs_time, gnss_synchroL1);
    std::bitset<2> DF016_ = std::bitset<2>(0); // code indicator   0: C/A or L2C code   1: P(Y) code direct  2:P(Y) code cross-correlated    3: Correlated P/Y
    Rtcm::set_DF017(gnss_synchroL1, gnss_synchroL2);
    Rtcm::set_DF018(gnss_synchroL1, gnss_synchroL2);
    Rtcm::set_DF019(ephL2, obs_time, gnss_synchroL2);

    std::string content = DF009.to_string() +
            DF010.to_string() +
            DF011.to_string() +
            DF012.to_string() +
            DF013.to_string() +
            DF016_.to_string() +
            DF017.to_string() +
            DF018.to_string() +
            DF019.to_string();

    std::bitset<101> content_msg(content);
    return content_msg;
}



// ******************************************************************
//
//   MESSAGE TYPE 1004 (EXTENDED GPS L1 & L2 OBSERVATIONS)
//
// ******************************************************************

std::string Rtcm::print_MT1004(const Gps_Ephemeris & ephL1, const Gps_CNAV_Ephemeris & ephL2, double obs_time, const std::map<int, Gnss_Synchro> & observables, unsigned short station_id)
{
    unsigned int ref_id = static_cast<unsigned int>(station_id);
    unsigned int smooth_int = 0;
    bool sync_flag = false;
    bool divergence_free = false;

    //Get maps with GPS L1 and L2 observations
    std::map<int, Gnss_Synchro> observablesL1;
    std::map<int, Gnss_Synchro> observablesL2;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter2;

    for(observables_iter = observables.cbegin();
            observables_iter != observables.cend();
            observables_iter++)
        {
            std::string system_(&observables_iter->second.System, 1);
            std::string sig_(observables_iter->second.Signal);
            if((system_.compare("G") == 0) && (sig_.compare("1C") == 0))
                {
                    observablesL1.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
            if((system_.compare("G") == 0) && (sig_.compare("2S") == 0))
                {
                    observablesL2.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                }
        }

    // Get common observables
    std::vector< std::pair< Gnss_Synchro, Gnss_Synchro > >  common_observables;
    std::vector< std::pair< Gnss_Synchro, Gnss_Synchro > >::const_iterator common_observables_iter;
    std::map<int, Gnss_Synchro> observablesL1_with_L2;

    for(observables_iter = observablesL1.cbegin();
            observables_iter != observablesL1.cend();
            observables_iter++)
        {
            unsigned int prn_ = observables_iter->second.PRN;
            for(observables_iter2 = observablesL2.cbegin();
                    observables_iter2 != observablesL2.cend();
                    observables_iter2++)
                {
                    if(observables_iter2->second.PRN == prn_)
                        {
                            std::pair<Gnss_Synchro, Gnss_Synchro> p;
                            Gnss_Synchro pr1 = observables_iter->second;
                            Gnss_Synchro pr2 = observables_iter2->second;
                            p = std::make_pair(pr1, pr2);
                            common_observables.push_back(p);
                            observablesL1_with_L2.insert(std::pair<int, Gnss_Synchro>(observables_iter->first, observables_iter->second));
                        }
                }
        }

    std::bitset<64> header = Rtcm::get_MT1001_4_header(1004, obs_time, observablesL1_with_L2, ref_id, smooth_int, sync_flag, divergence_free);
    std::string data = header.to_string();

    for(common_observables_iter = common_observables.cbegin();
            common_observables_iter != common_observables.cend();
            common_observables_iter++)
        {
            std::bitset<125> content = Rtcm::get_MT1004_sat_content(ephL1, ephL2, obs_time, common_observables_iter->first, common_observables_iter->second);
            data += content.to_string();
        }

    std::string msg = build_message(data);
    if(server_is_running)
        {
            rtcm_message_queue->push(msg);
        }
    return msg;
}


std::bitset<125> Rtcm::get_MT1004_sat_content(const Gps_Ephemeris & ephL1, const Gps_CNAV_Ephemeris & ephL2, double obs_time, const Gnss_Synchro & gnss_synchroL1, const Gnss_Synchro & gnss_synchroL2)
{
    bool code_indicator = false; // code indicator   0: C/A code   1: P(Y) code direct
    Rtcm::set_DF009(gnss_synchroL1);
    Rtcm::set_DF010(code_indicator); // code indicator   0: C/A code   1: P(Y) code direct
    Rtcm::set_DF011(gnss_synchroL1);
    Rtcm::set_DF012(gnss_synchroL1);
    Rtcm::set_DF013(ephL1, obs_time, gnss_synchroL1);
    Rtcm::set_DF014(gnss_synchroL1);
    Rtcm::set_DF015(gnss_synchroL1);
    std::bitset<2> DF016_ = std::bitset<2>(0); // code indicator   0: C/A or L2C code   1: P(Y) code direct  2:P(Y) code cross-correlated    3: Correlated P/Y
    Rtcm::set_DF017(gnss_synchroL1, gnss_synchroL2);
    Rtcm::set_DF018(gnss_synchroL1, gnss_synchroL2);
    Rtcm::set_DF019(ephL2, obs_time, gnss_synchroL2);
    Rtcm::set_DF020(gnss_synchroL2);

    std::string content = DF009.to_string() +
            DF010.to_string() +
            DF011.to_string() +
            DF012.to_string() +
            DF013.to_string() +
            DF014.to_string() +
            DF015.to_string() +
            DF016_.to_string() +
            DF017.to_string() +
            DF018.to_string() +
            DF019.to_string() +
            DF020.to_string();

    std::bitset<125> content_msg(content);
    return content_msg;
}



// ********************************************************
//
//   MESSAGE TYPE 1005 (STATION DESCRIPTION)
//
// ********************************************************


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

    std::string msg = build_message(data);
    if(server_is_running)
        {
            rtcm_message_queue->push(msg);
        }
    return msg;
}


int Rtcm::read_MT1005(const std::string & message, unsigned int & ref_id, double & ecef_x, double & ecef_y, double & ecef_z, bool & gps, bool & glonass, bool & galileo)
{
    // Convert message to binary
    std::string message_bin = Rtcm::binary_data_to_bin(message);

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

// ********************************************************
//
//   MESSAGE TYPE 1006 (STATION DESCRIPTION PLUS HEIGHT INFORMATION)
//
// ********************************************************

std::string Rtcm::print_MT1006(unsigned int ref_id, double ecef_x, double ecef_y, double ecef_z, bool gps, bool glonass, bool galileo, bool non_physical, bool single_oscillator, unsigned int quarter_cycle_indicator, double height)
{
    unsigned int msg_number = 1006;
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
    Rtcm::set_DF028(height);

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
            DF027.to_string() +
            DF028.to_string();

    std::string msg = build_message(data);
    if(server_is_running)
        {
            rtcm_message_queue->push(msg);
        }
    return msg;
}


// ********************************************************
//
//   MESSAGE TYPE 1008 (ANTENNA DESCRIPTOR & SERIAL NUMBER)
//
// ********************************************************
std::string Rtcm::print_MT1008(unsigned int ref_id, const std::string & antenna_descriptor, unsigned int antenna_setup_id, const std::string & antenna_serial_number)
{
    unsigned int msg_number = 1008;
    std::bitset<12> DF002_ = std::bitset<12>(msg_number);
    Rtcm::set_DF003(ref_id);
    std::string ant_descriptor = antenna_descriptor;
    unsigned int len = ant_descriptor.length();
    if (len > 31)
        {
            ant_descriptor = ant_descriptor.substr(0, 31);
            len = 31;
        }
    DF029 = std::bitset<8>(len);

    std::string DF030_str_;
    for(auto it = ant_descriptor.cbegin(); it != ant_descriptor.cend(); it++)
        {
            char c = *it;
            std::bitset<8> character = std::bitset<8>(c);
            DF030_str_ += character.to_string();
        }

    Rtcm::set_DF031(antenna_setup_id);

    std::string ant_sn(antenna_serial_number);
    unsigned int len2 = ant_sn.length();
    if (len2 > 31)
        {
            ant_sn = ant_sn.substr(0, 31);
            len2 = 31;
        }
    DF032 = std::bitset<8>(len2);

    std::string DF033_str_;
    for(auto it = ant_sn.cbegin(); it != ant_sn.cend(); it++)
        {
            char c = *it;
            std::bitset<8> character = std::bitset<8>(c);
            DF033_str_ += character.to_string();
        }

    std::string data = DF002_.to_string() +
            DF003.to_string() +
            DF029.to_string() +
            DF030_str_ +
            DF031.to_string() +
            DF032.to_string() +
            DF033_str_;

    std::string msg = build_message(data);
    if(server_is_running)
        {
            rtcm_message_queue->push(msg);
        }
    return msg;
}


// ********************************************************
//
//   MESSAGE TYPE 1019 (GPS EPHEMERIS)
//
// ********************************************************

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

    std::string msg = build_message(data);
    if(server_is_running)
        {
            rtcm_message_queue->push(msg);
        }
    return msg;
}


int Rtcm::read_MT1019(const std::string & message, Gps_Ephemeris & gps_eph)
{
    // Convert message to binary
    std::string message_bin = Rtcm::binary_data_to_bin(message);

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


// ********************************************************
//
//   MESSAGE TYPE 1029 (UNICODE TEXT STRING)
//
// ********************************************************

std::string Rtcm::print_MT1029(unsigned int ref_id, const Gps_Ephemeris & gps_eph, double obs_time, const std::string & message)
{
    unsigned int msg_number = 1029;

    Rtcm::set_DF002(msg_number);
    Rtcm::set_DF003(ref_id);
    Rtcm::set_DF051(gps_eph, obs_time);
    Rtcm::set_DF052(gps_eph, obs_time);

    unsigned int i = 0;
    bool first = true;
    std::string text_binary;
    for(auto it = message.cbegin(); it != message.cend(); it++)
        {
            char c = *it;
            if(isgraph(c))
                {
                    i++;
                    first = true;
                }
            else if(c == ' ')
                {
                    i++;
                    first = true;
                }
            else
                {
                    if(!first)
                        {
                            i++;
                            first = true;
                        }
                    else
                        {
                            first = false;
                        }
                }
            std::bitset<8> character = std::bitset<8>(c);
            text_binary += character.to_string();
        }

    std::bitset<7> DF138_ = std::bitset<7>(i);
    std::bitset<8> DF139_ = std::bitset<8>(message.length());

    std::string data = DF002.to_string() +
            DF003.to_string() +
            DF051.to_string() +
            DF052.to_string() +
            DF138_.to_string() +
            DF139_.to_string() +
            text_binary;

    std::string msg = build_message(data);
    if(server_is_running)
        {
            rtcm_message_queue->push(msg);
        }
    return msg;
}


// ********************************************************
//
//   MESSAGE TYPE 1045 (GALILEO EPHEMERIS)
//
// ********************************************************

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

    std::string msg = build_message(data);
    if(server_is_running)
        {
            rtcm_message_queue->push(msg);
        }
    return msg;
}


int Rtcm::read_MT1045(const std::string & message, Galileo_Ephemeris & gal_eph)
{
    // Convert message to binary
    std::string message_bin = Rtcm::binary_data_to_bin(message);

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




// **********************************************************************************************
//
//   MESSAGE TYPE MSM1 (COMPACT observables)
//
// **********************************************************************************************

std::string Rtcm::print_MSM_1( const Gps_Ephemeris & gps_eph,
        const Gps_CNAV_Ephemeris & gps_cnav_eph,
        const Galileo_Ephemeris & gal_eph,
        double obs_time,
        const std::map<int, Gnss_Synchro> & observables,
        unsigned int ref_id,
        unsigned int clock_steering_indicator,
        unsigned int external_clock_indicator,
        int smooth_int,
        bool divergence_free,
        bool more_messages)
{
    unsigned int msg_number = 0;
    if(gps_eph.i_satellite_PRN != 0) msg_number = 1071;
    if(gps_cnav_eph.i_satellite_PRN != 0) msg_number = 1071;
    if(gal_eph.i_satellite_PRN != 0) msg_number = 1091;
    if(((gps_eph.i_satellite_PRN != 0) ||(gps_cnav_eph.i_satellite_PRN != 0) ) && (gal_eph.i_satellite_PRN != 0))
        {
            LOG(WARNING) << "MSM messages for observables from different systems are not defined"; //print two messages?
        }
    if(msg_number == 0)
        {
            LOG(WARNING) << "Invalid ephemeris provided";
            msg_number = 1071;
        }

    std::string header = Rtcm::get_MSM_header(msg_number,
             obs_time,
             observables,
             ref_id,
             clock_steering_indicator,
             external_clock_indicator,
             smooth_int,
             divergence_free,
             more_messages);

    std::string sat_data = Rtcm::get_MSM_1_content_sat_data(observables);

    std::string signal_data = Rtcm::get_MSM_1_content_signal_data(observables);

    std::string message = build_message(header + sat_data + signal_data);

    if(server_is_running)
        {
            rtcm_message_queue->push(message);
        }

    return message;
}


std::string Rtcm::get_MSM_header(unsigned int msg_number,
        double obs_time,
        const std::map<int, Gnss_Synchro> & observables,
        unsigned int ref_id,
        unsigned int clock_steering_indicator,
        unsigned int external_clock_indicator,
        int smooth_int,
        bool divergence_free,
        bool more_messages)
{
    Rtcm::set_DF002(msg_number);
    Rtcm::set_DF003(ref_id);
    Rtcm::set_DF004(obs_time);
    Rtcm::set_DF393(more_messages);
    Rtcm::set_DF409(0); // Issue of Data Station. 0: not utilized
    std::bitset<7> DF001_ = std::bitset<7>("0000000");
    Rtcm::set_DF411(clock_steering_indicator);
    Rtcm::set_DF412(external_clock_indicator);
    Rtcm::set_DF417(divergence_free);
    Rtcm::set_DF418(smooth_int);

    Rtcm::set_DF394(observables);
    Rtcm::set_DF395(observables);

    std::string header = DF002.to_string() + DF003.to_string();
    header += DF004.to_string();
    header = header + DF393.to_string() +
            DF409.to_string() +
            DF001_.to_string() +
            DF411.to_string() +
            DF417.to_string() +
            DF412.to_string() +
            DF418.to_string() +
            DF394.to_string() +
            DF395.to_string() +
            Rtcm::set_DF396(observables);

    return header;
}


std::string Rtcm::get_MSM_1_content_sat_data(const std::map<int, Gnss_Synchro> & observables)
{
    std::string sat_data;
    sat_data.clear();

    Rtcm::set_DF394(observables);
    unsigned int num_satellites = DF394.count();

    std::vector<std::pair<int, Gnss_Synchro> > observables_vector;
    std::map<int, Gnss_Synchro>::const_iterator gnss_synchro_iter;
    std::vector<unsigned int> pos;
    std::vector<unsigned int>::iterator it;

    for(gnss_synchro_iter = observables.cbegin();
            gnss_synchro_iter != observables.cend();
            gnss_synchro_iter++)
        {
            it = std::find(pos.begin(), pos.end(), 65 - gnss_synchro_iter->second.PRN);
            if(it == pos.end())
                {
                    pos.push_back(65 - gnss_synchro_iter->second.PRN);
                    observables_vector.push_back(*gnss_synchro_iter);
                }
        }

    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_PRN_pos = Rtcm::sort_by_PRN_mask(observables_vector);

    for(unsigned int nsat = 0; nsat < num_satellites; nsat++)
        {
            Rtcm::set_DF398( ordered_by_PRN_pos.at(nsat).second );
            sat_data += DF398.to_string();
        }

    return sat_data;
}


std::string Rtcm::get_MSM_1_content_signal_data(const std::map<int, Gnss_Synchro> & observables)
{
    std::string signal_data;
    signal_data.clear();
    unsigned int Ncells = observables.size();

    std::vector<std::pair<int, Gnss_Synchro> > observables_vector;
    std::map<int, Gnss_Synchro>::const_iterator map_iter;

    for(map_iter = observables.cbegin();
            map_iter != observables.cend();
            map_iter++)
        {
            observables_vector.push_back(*map_iter);
        }

    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_signal = Rtcm::sort_by_signal(observables_vector);
    std::reverse(ordered_by_signal.begin(), ordered_by_signal.end());
    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_PRN_pos = Rtcm::sort_by_PRN_mask(ordered_by_signal);

    for(unsigned int cell = 0; cell < Ncells ; cell++)
           {
               Rtcm::set_DF400(ordered_by_PRN_pos.at( cell ).second);
               signal_data += DF400.to_string();
           }

    return signal_data;
}


// **********************************************************************************************
//
//   MESSAGE TYPE MSM2 (COMPACT PHASERANGES)
//
// **********************************************************************************************

std::string Rtcm::print_MSM_2( const Gps_Ephemeris & gps_eph,
        const Gps_CNAV_Ephemeris & gps_cnav_eph,
        const Galileo_Ephemeris & gal_eph,
        double obs_time,
        const std::map<int, Gnss_Synchro> & observables,
        unsigned int ref_id,
        unsigned int clock_steering_indicator,
        unsigned int external_clock_indicator,
        int smooth_int,
        bool divergence_free,
        bool more_messages)
{
    unsigned int msg_number = 0;
    if(gps_eph.i_satellite_PRN != 0) msg_number = 1072;
    if(gps_cnav_eph.i_satellite_PRN != 0) msg_number = 1072;
    if(gal_eph.i_satellite_PRN != 0) msg_number = 1092;
    if(((gps_eph.i_satellite_PRN != 0) ||(gps_cnav_eph.i_satellite_PRN != 0) ) && (gal_eph.i_satellite_PRN != 0))
        {
            LOG(WARNING) << "MSM messages for observables from different systems are not defined"; //print two messages?
        }
    if(msg_number == 0)
        {
            LOG(WARNING) << "Invalid ephemeris provided";
            msg_number = 1072;
        }

    std::string header = Rtcm::get_MSM_header(msg_number,
             obs_time,
             observables,
             ref_id,
             clock_steering_indicator,
             external_clock_indicator,
             smooth_int,
             divergence_free,
             more_messages);

    std::string sat_data = Rtcm::get_MSM_1_content_sat_data(observables);

    std::string signal_data = Rtcm::get_MSM_2_content_signal_data(gps_eph, gps_cnav_eph, gal_eph, obs_time, observables);

    std::string message = build_message(header + sat_data + signal_data);
    if(server_is_running)
        {
            rtcm_message_queue->push(message);
        }

    return message;
}


std::string Rtcm::get_MSM_2_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & observables)
{
    std::string signal_data;
    std::string first_data_type;
    std::string second_data_type;
    std::string third_data_type;

    unsigned int Ncells = observables.size();

    std::vector<std::pair<int, Gnss_Synchro> > observables_vector;
    std::map<int, Gnss_Synchro>::const_iterator map_iter;

    for(map_iter = observables.cbegin();
            map_iter != observables.cend();
            map_iter++)
        {
            observables_vector.push_back(*map_iter);
        }

    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_signal = Rtcm::sort_by_signal(observables_vector);
    std::reverse(ordered_by_signal.begin(), ordered_by_signal.end());
    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_PRN_pos = Rtcm::sort_by_PRN_mask(ordered_by_signal);

    for(unsigned int cell = 0; cell < Ncells ; cell++)
        {
            Rtcm::set_DF401(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF402(ephNAV, ephCNAV, ephFNAV, obs_time, ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF420(ordered_by_PRN_pos.at( cell ).second);
            first_data_type += DF401.to_string();
            second_data_type += DF402.to_string();
            third_data_type += DF420.to_string();
        }

    signal_data = first_data_type + second_data_type + third_data_type;
    return signal_data;
}



// **********************************************************************************************
//
//   MESSAGE TYPE MSM3 (COMPACT PSEUDORANGES AND PHASERANGES)
//
// **********************************************************************************************

std::string Rtcm::print_MSM_3( const Gps_Ephemeris & gps_eph,
        const Gps_CNAV_Ephemeris & gps_cnav_eph,
        const Galileo_Ephemeris & gal_eph,
        double obs_time,
        const std::map<int, Gnss_Synchro> & observables,
        unsigned int ref_id,
        unsigned int clock_steering_indicator,
        unsigned int external_clock_indicator,
        int smooth_int,
        bool divergence_free,
        bool more_messages)
{
    unsigned int msg_number = 0;
    if(gps_eph.i_satellite_PRN != 0) msg_number = 1073;
    if(gps_cnav_eph.i_satellite_PRN != 0) msg_number = 1073;
    if(gal_eph.i_satellite_PRN != 0) msg_number = 1093;
    if(((gps_eph.i_satellite_PRN != 0) ||(gps_cnav_eph.i_satellite_PRN != 0) ) && (gal_eph.i_satellite_PRN != 0))
        {
            LOG(WARNING) << "MSM messages for observables from different systems are not defined"; //print two messages?
        }
    if(msg_number == 0)
        {
            LOG(WARNING) << "Invalid ephemeris provided";
            msg_number = 1073;
        }

    std::string header = Rtcm::get_MSM_header(msg_number,
             obs_time,
             observables,
             ref_id,
             clock_steering_indicator,
             external_clock_indicator,
             smooth_int,
             divergence_free,
             more_messages);

    std::string sat_data = Rtcm::get_MSM_1_content_sat_data(observables);

    std::string signal_data = Rtcm::get_MSM_3_content_signal_data(gps_eph, gps_cnav_eph, gal_eph, obs_time, observables);

    std::string message = build_message(header + sat_data + signal_data);
    if(server_is_running)
        {
            rtcm_message_queue->push(message);
        }

    return message;
}


std::string Rtcm::get_MSM_3_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & observables)
{
    std::string signal_data;
    std::string first_data_type;
    std::string second_data_type;
    std::string third_data_type;
    std::string fourth_data_type;

    unsigned int Ncells = observables.size();

    std::vector<std::pair<int, Gnss_Synchro> > observables_vector;
    std::map<int, Gnss_Synchro>::const_iterator map_iter;

    for(map_iter = observables.cbegin();
            map_iter != observables.cend();
            map_iter++)
        {
            observables_vector.push_back(*map_iter);
        }

    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_signal = Rtcm::sort_by_signal(observables_vector);
    std::reverse(ordered_by_signal.begin(), ordered_by_signal.end());
    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_PRN_pos = Rtcm::sort_by_PRN_mask(ordered_by_signal);

    for(unsigned int cell = 0; cell < Ncells ; cell++)
        {
            Rtcm::set_DF400(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF401(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF402(ephNAV, ephCNAV, ephFNAV, obs_time, ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF420(ordered_by_PRN_pos.at( cell ).second);
            first_data_type += DF400.to_string();
            second_data_type += DF401.to_string();
            third_data_type += DF402.to_string();
            fourth_data_type += DF420.to_string();
        }

    signal_data = first_data_type + second_data_type + third_data_type + fourth_data_type;
    return signal_data;
}


// **********************************************************************************************
//
//   MESSAGE TYPE MSM4 (FULL PSEUDORANGES AND PHASERANGES PLUS CNR)
//
// **********************************************************************************************

std::string Rtcm::print_MSM_4( const Gps_Ephemeris & gps_eph,
        const Gps_CNAV_Ephemeris & gps_cnav_eph,
        const Galileo_Ephemeris & gal_eph,
        double obs_time,
        const std::map<int, Gnss_Synchro> & observables,
        unsigned int ref_id,
        unsigned int clock_steering_indicator,
        unsigned int external_clock_indicator,
        int smooth_int,
        bool divergence_free,
        bool more_messages)
{
    unsigned int msg_number = 0;
    if(gps_eph.i_satellite_PRN != 0) msg_number = 1074;
    if(gps_cnav_eph.i_satellite_PRN != 0) msg_number = 1074;
    if(gal_eph.i_satellite_PRN != 0) msg_number = 1094;
    if(((gps_eph.i_satellite_PRN != 0) ||(gps_cnav_eph.i_satellite_PRN != 0) ) && (gal_eph.i_satellite_PRN != 0))
        {
            LOG(WARNING) << "MSM messages for observables from different systems are not defined"; //print two messages?
        }
    if(msg_number == 0)
        {
            LOG(WARNING) << "Invalid ephemeris provided";
            msg_number = 1074;
        }

    std::string header = Rtcm::get_MSM_header(msg_number,
             obs_time,
             observables,
             ref_id,
             clock_steering_indicator,
             external_clock_indicator,
             smooth_int,
             divergence_free,
             more_messages);

    std::string sat_data = Rtcm::get_MSM_4_content_sat_data(observables);

    std::string signal_data = Rtcm::get_MSM_4_content_signal_data(gps_eph, gps_cnav_eph, gal_eph, obs_time, observables);

    std::string message = build_message(header + sat_data + signal_data);
    if(server_is_running)
        {
            rtcm_message_queue->push(message);
        }

    return message;
}


std::string Rtcm::get_MSM_4_content_sat_data(const std::map<int, Gnss_Synchro> & observables)
{
    std::string sat_data;
    std::string first_data_type;
    std::string second_data_type;

    Rtcm::set_DF394(observables);
    unsigned int num_satellites = DF394.count();

    std::vector<std::pair<int, Gnss_Synchro> > observables_vector;
    std::map<int, Gnss_Synchro>::const_iterator gnss_synchro_iter;
    std::vector<unsigned int> pos;
    std::vector<unsigned int>::iterator it;

    for(gnss_synchro_iter = observables.cbegin();
            gnss_synchro_iter != observables.cend();
            gnss_synchro_iter++)
        {
            it = std::find(pos.begin(), pos.end(), 65 - gnss_synchro_iter->second.PRN);
            if(it == pos.end())
                {
                    pos.push_back(65 - gnss_synchro_iter->second.PRN);
                    observables_vector.push_back(*gnss_synchro_iter);
                }
        }

    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_PRN_pos = Rtcm::sort_by_PRN_mask(observables_vector);

    for(unsigned int nsat = 0; nsat < num_satellites; nsat++)
        {
            Rtcm::set_DF397( ordered_by_PRN_pos.at(nsat).second );
            Rtcm::set_DF398( ordered_by_PRN_pos.at(nsat).second );
            first_data_type += DF397.to_string();
            second_data_type += DF398.to_string();
        }
    sat_data = first_data_type + second_data_type;
    return sat_data;
}


std::string Rtcm::get_MSM_4_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & observables)
{
    std::string signal_data;
    std::string first_data_type;
    std::string second_data_type;
    std::string third_data_type;
    std::string fourth_data_type;
    std::string fifth_data_type;

    unsigned int Ncells = observables.size();

    std::vector<std::pair<int, Gnss_Synchro> > observables_vector;
    std::map<int, Gnss_Synchro>::const_iterator map_iter;

    for(map_iter = observables.cbegin();
            map_iter != observables.cend();
            map_iter++)
        {
            observables_vector.push_back(*map_iter);
        }

    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_signal = Rtcm::sort_by_signal(observables_vector);
    std::reverse(ordered_by_signal.begin(), ordered_by_signal.end());
    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_PRN_pos = Rtcm::sort_by_PRN_mask(ordered_by_signal);

    for(unsigned int cell = 0; cell < Ncells ; cell++)
        {
            Rtcm::set_DF400(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF401(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF402(ephNAV, ephCNAV, ephFNAV, obs_time, ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF420(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF403(ordered_by_PRN_pos.at( cell ).second);
            first_data_type += DF400.to_string();
            second_data_type += DF401.to_string();
            third_data_type += DF402.to_string();
            fourth_data_type += DF420.to_string();
            fifth_data_type += DF403.to_string();
        }

    signal_data = first_data_type + second_data_type + third_data_type + fourth_data_type + fifth_data_type;
    return signal_data;
}


// **********************************************************************************************
//
//   MESSAGE TYPE MSM5 (FULL PSEUDORANGES, PHASERANGES, PHASERANGERATE PLUS CNR)
//
// **********************************************************************************************

std::string Rtcm::print_MSM_5( const Gps_Ephemeris & gps_eph,
        const Gps_CNAV_Ephemeris & gps_cnav_eph,
        const Galileo_Ephemeris & gal_eph,
        double obs_time,
        const std::map<int, Gnss_Synchro> & observables,
        unsigned int ref_id,
        unsigned int clock_steering_indicator,
        unsigned int external_clock_indicator,
        int smooth_int,
        bool divergence_free,
        bool more_messages)
{
    unsigned int msg_number = 0;
    if(gps_eph.i_satellite_PRN != 0) msg_number = 1075;
    if(gps_cnav_eph.i_satellite_PRN != 0) msg_number = 1075;
    if(gal_eph.i_satellite_PRN != 0) msg_number = 1095;
    if(((gps_eph.i_satellite_PRN != 0) ||(gps_cnav_eph.i_satellite_PRN != 0) ) && (gal_eph.i_satellite_PRN != 0))
        {
            LOG(WARNING) << "MSM messages for observables from different systems are not defined"; //print two messages?
        }
    if(msg_number == 0)
        {
            LOG(WARNING) << "Invalid ephemeris provided";
            msg_number = 1075;
        }

    std::string header = Rtcm::get_MSM_header(msg_number,
             obs_time,
             observables,
             ref_id,
             clock_steering_indicator,
             external_clock_indicator,
             smooth_int,
             divergence_free,
             more_messages);

    std::string sat_data = Rtcm::get_MSM_5_content_sat_data(observables);

    std::string signal_data = Rtcm::get_MSM_5_content_signal_data(gps_eph, gps_cnav_eph, gal_eph, obs_time, observables);

    std::string message = build_message(header + sat_data + signal_data);
    if(server_is_running)
        {
            rtcm_message_queue->push(message);
        }

    return message;
}


std::string Rtcm::get_MSM_5_content_sat_data(const std::map<int, Gnss_Synchro> & observables)
{
    std::string sat_data;
    std::string first_data_type;
    std::string second_data_type;
    std::string third_data_type;
    std::string fourth_data_type;

    Rtcm::set_DF394(observables);
    unsigned int num_satellites = DF394.count();

    std::vector<std::pair<int, Gnss_Synchro> > observables_vector;
    std::map<int, Gnss_Synchro>::const_iterator gnss_synchro_iter;
    std::vector<unsigned int> pos;
    std::vector<unsigned int>::iterator it;

    for(gnss_synchro_iter = observables.cbegin();
            gnss_synchro_iter != observables.cend();
            gnss_synchro_iter++)
        {
            it = std::find(pos.begin(), pos.end(), 65 - gnss_synchro_iter->second.PRN);
            if(it == pos.end())
                {
                    pos.push_back(65 - gnss_synchro_iter->second.PRN);
                    observables_vector.push_back(*gnss_synchro_iter);
                }
        }

    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_PRN_pos = Rtcm::sort_by_PRN_mask(observables_vector);

    for(unsigned int nsat = 0; nsat < num_satellites; nsat++)
        {
            Rtcm::set_DF397( ordered_by_PRN_pos.at(nsat).second );
            Rtcm::set_DF398( ordered_by_PRN_pos.at(nsat).second );
            Rtcm::set_DF399( ordered_by_PRN_pos.at(nsat).second );
            std::bitset<4> reserved = std::bitset<4>("0000");
            first_data_type += DF397.to_string();
            second_data_type += reserved.to_string();
            third_data_type += DF398.to_string();
            fourth_data_type += DF399.to_string();
        }
    sat_data = first_data_type + second_data_type + third_data_type + fourth_data_type;
    return sat_data;
}


std::string Rtcm::get_MSM_5_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & observables)
{
    std::string signal_data;
    std::string first_data_type;
    std::string second_data_type;
    std::string third_data_type;
    std::string fourth_data_type;
    std::string fifth_data_type;
    std::string sixth_data_type;

    unsigned int Ncells = observables.size();

    std::vector<std::pair<int, Gnss_Synchro> > observables_vector;
    std::map<int, Gnss_Synchro>::const_iterator map_iter;

    for(map_iter = observables.cbegin();
            map_iter != observables.cend();
            map_iter++)
        {
            observables_vector.push_back(*map_iter);
        }

    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_signal = Rtcm::sort_by_signal(observables_vector);
    std::reverse(ordered_by_signal.begin(), ordered_by_signal.end());
    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_PRN_pos = Rtcm::sort_by_PRN_mask(ordered_by_signal);

    for(unsigned int cell = 0; cell < Ncells ; cell++)
        {
            Rtcm::set_DF400(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF401(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF402(ephNAV, ephCNAV, ephFNAV, obs_time, ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF420(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF403(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF404(ordered_by_PRN_pos.at( cell ).second);
            first_data_type += DF400.to_string();
            second_data_type += DF401.to_string();
            third_data_type += DF402.to_string();
            fourth_data_type += DF420.to_string();
            fifth_data_type += DF403.to_string();
            sixth_data_type += DF404.to_string();
        }

    signal_data = first_data_type + second_data_type + third_data_type + fourth_data_type + fifth_data_type + sixth_data_type;
    return signal_data;
}



// **********************************************************************************************
//
//   MESSAGE TYPE MSM6 (FULL PSEUDORANGES AND PHASERANGES PLUS CNR, HIGH RESOLUTION)
//
// **********************************************************************************************

std::string Rtcm::print_MSM_6( const Gps_Ephemeris & gps_eph,
        const Gps_CNAV_Ephemeris & gps_cnav_eph,
        const Galileo_Ephemeris & gal_eph,
        double obs_time,
        const std::map<int, Gnss_Synchro> & observables,
        unsigned int ref_id,
        unsigned int clock_steering_indicator,
        unsigned int external_clock_indicator,
        int smooth_int,
        bool divergence_free,
        bool more_messages)
{
    unsigned int msg_number = 0;
    if(gps_eph.i_satellite_PRN != 0) msg_number = 1076;
    if(gps_cnav_eph.i_satellite_PRN != 0) msg_number = 1076;
    if(gal_eph.i_satellite_PRN != 0) msg_number = 1096;
    if(((gps_eph.i_satellite_PRN != 0) ||(gps_cnav_eph.i_satellite_PRN != 0) ) && (gal_eph.i_satellite_PRN != 0))
        {
            LOG(WARNING) << "MSM messages for observables from different systems are not defined"; //print two messages?
        }
    if(msg_number == 0)
        {
            LOG(WARNING) << "Invalid ephemeris provided";
            msg_number = 1076;
        }

    std::string header = Rtcm::get_MSM_header(msg_number,
             obs_time,
             observables,
             ref_id,
             clock_steering_indicator,
             external_clock_indicator,
             smooth_int,
             divergence_free,
             more_messages);

    std::string sat_data = Rtcm::get_MSM_4_content_sat_data(observables);

    std::string signal_data = Rtcm::get_MSM_6_content_signal_data(gps_eph, gps_cnav_eph, gal_eph, obs_time, observables);

    std::string message = build_message(header + sat_data + signal_data);
    if(server_is_running)
        {
            rtcm_message_queue->push(message);
        }

    return message;
}


std::string Rtcm::get_MSM_6_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & observables)
{
    std::string signal_data;
    std::string first_data_type;
    std::string second_data_type;
    std::string third_data_type;
    std::string fourth_data_type;
    std::string fifth_data_type;

    unsigned int Ncells = observables.size();

    std::vector<std::pair<int, Gnss_Synchro> > observables_vector;
    std::map<int, Gnss_Synchro>::const_iterator map_iter;

    for(map_iter = observables.cbegin();
            map_iter != observables.cend();
            map_iter++)
        {
            observables_vector.push_back(*map_iter);
        }

    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_signal = Rtcm::sort_by_signal(observables_vector);
    std::reverse(ordered_by_signal.begin(), ordered_by_signal.end());
    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_PRN_pos = Rtcm::sort_by_PRN_mask(ordered_by_signal);

    for(unsigned int cell = 0; cell < Ncells ; cell++)
        {
            Rtcm::set_DF405(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF406(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF407(ephNAV, ephCNAV, ephFNAV, obs_time, ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF420(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF408(ordered_by_PRN_pos.at( cell ).second);
            first_data_type += DF405.to_string();
            second_data_type += DF406.to_string();
            third_data_type += DF407.to_string();
            fourth_data_type += DF420.to_string();
            fifth_data_type += DF408.to_string();
        }

    signal_data = first_data_type + second_data_type + third_data_type + fourth_data_type + fifth_data_type;
    return signal_data;
}



// **********************************************************************************************
//
//   MESSAGE TYPE MSM7 (FULL PSEUDORANGES, PHASERANGES, PHASERANGERATE AND CNR, HIGH RESOLUTION)
//
// **********************************************************************************************

std::string Rtcm::print_MSM_7( const Gps_Ephemeris & gps_eph,
        const Gps_CNAV_Ephemeris & gps_cnav_eph,
        const Galileo_Ephemeris & gal_eph,
        double obs_time,
        const std::map<int, Gnss_Synchro> & observables,
        unsigned int ref_id,
        unsigned int clock_steering_indicator,
        unsigned int external_clock_indicator,
        int smooth_int,
        bool divergence_free,
        bool more_messages)
{
    unsigned int msg_number = 0;
    if(gps_eph.i_satellite_PRN != 0) msg_number = 1077;
    if(gps_cnav_eph.i_satellite_PRN != 0) msg_number = 1077;
    if(gal_eph.i_satellite_PRN != 0) msg_number = 1097;
    if(((gps_eph.i_satellite_PRN != 0) || (gps_cnav_eph.i_satellite_PRN != 0) ) && (gal_eph.i_satellite_PRN != 0))
        {
            LOG(WARNING) << "MSM messages for observables from different systems are not defined"; //print two messages?
        }
    if(msg_number == 0)
        {
            LOG(WARNING) << "Invalid ephemeris provided";
            msg_number = 1076;
        }

    std::string header = Rtcm::get_MSM_header(msg_number,
             obs_time,
             observables,
             ref_id,
             clock_steering_indicator,
             external_clock_indicator,
             smooth_int,
             divergence_free,
             more_messages);

    std::string sat_data = Rtcm::get_MSM_5_content_sat_data(observables);

    std::string signal_data = Rtcm::get_MSM_7_content_signal_data(gps_eph, gps_cnav_eph, gal_eph, obs_time, observables);

    std::string message = build_message(header + sat_data + signal_data);
    if(server_is_running)
        {
            rtcm_message_queue->push(message);
        }

    return message;
}


std::string Rtcm::get_MSM_7_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & observables)
{
    std::string signal_data;
    std::string first_data_type;
    std::string second_data_type;
    std::string third_data_type;
    std::string fourth_data_type;
    std::string fifth_data_type;
    std::string sixth_data_type;

    unsigned int Ncells = observables.size();

    std::vector<std::pair<int, Gnss_Synchro> > observables_vector;
    std::map<int, Gnss_Synchro>::const_iterator map_iter;

    for(map_iter = observables.cbegin();
            map_iter != observables.cend();
            map_iter++)
        {
            observables_vector.push_back(*map_iter);
        }

    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_signal = Rtcm::sort_by_signal(observables_vector);
    std::reverse(ordered_by_signal.begin(), ordered_by_signal.end());
    std::vector<std::pair<int, Gnss_Synchro> > ordered_by_PRN_pos = Rtcm::sort_by_PRN_mask(ordered_by_signal);

    for(unsigned int cell = 0; cell < Ncells ; cell++)
        {
            Rtcm::set_DF405(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF406(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF407(ephNAV, ephCNAV, ephFNAV, obs_time, ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF420(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF408(ordered_by_PRN_pos.at( cell ).second);
            Rtcm::set_DF404(ordered_by_PRN_pos.at( cell ).second);
            first_data_type += DF405.to_string();
            second_data_type += DF406.to_string();
            third_data_type += DF407.to_string();
            fourth_data_type += DF420.to_string();
            fifth_data_type += DF408.to_string();
            sixth_data_type += DF404.to_string();
        }

    signal_data = first_data_type + second_data_type + third_data_type + fourth_data_type + fifth_data_type + sixth_data_type;
    return signal_data;
}



// *****************************************************************************************************
// Some utilities
// *****************************************************************************************************

std::vector<std::pair<int, Gnss_Synchro> > Rtcm::sort_by_PRN_mask(const std::vector<std::pair<int, Gnss_Synchro> >  & synchro_map) const
{
    std::vector<std::pair<int, Gnss_Synchro> >::const_iterator synchro_map_iter;
    std::vector<std::pair<int, Gnss_Synchro> > my_vec;
    struct {
        bool operator()(const std::pair<int, Gnss_Synchro> & a, const std::pair<int, Gnss_Synchro> & b)
        {
            unsigned int value_a = 64 - a.second.PRN;
            unsigned int value_b = 64 - b.second.PRN;
            return value_a < value_b;
        }
    } has_lower_pos;

    for(synchro_map_iter = synchro_map.cbegin();
            synchro_map_iter != synchro_map.cend();
            synchro_map_iter++)

        {
            std::pair<int, Gnss_Synchro> p(synchro_map_iter->first, synchro_map_iter->second);
            my_vec.push_back(p);
        }

    std::sort(my_vec.begin(), my_vec.end(), has_lower_pos);
    std::reverse(my_vec.begin(), my_vec.end());
    return my_vec;
}


std::vector<std::pair<int, Gnss_Synchro> > Rtcm::sort_by_signal(const std::vector<std::pair<int, Gnss_Synchro> >  & synchro_map) const
{
    std::vector<std::pair<int, Gnss_Synchro> >::const_iterator synchro_map_iter;
    std::vector<std::pair<int, Gnss_Synchro> > my_vec;

    struct {
        bool operator()(const std::pair<int, Gnss_Synchro> & a, const std::pair<int, Gnss_Synchro> & b)
        {
            unsigned int value_a = 0;
            unsigned int value_b = 0;
            std::string system_a(&a.second.System, 1);
            std::string system_b(&b.second.System, 1);
            std::string sig_a_(a.second.Signal);
            std::string sig_a = sig_a_.substr(0,2);
            std::string sig_b_(b.second.Signal);
            std::string sig_b = sig_b_.substr(0,2);

            if(system_a.compare("G") == 0)
                {
                    value_a = gps_signal_map.at(sig_a);
                }

            if(system_a.compare("E") == 0)
                {
                    value_a = galileo_signal_map.at(sig_a);
                }

            if(system_b.compare("G") == 0)
                {
                    value_b = gps_signal_map.at(sig_b);
                }

            if(system_b.compare("E") == 0)
                {
                    value_b = galileo_signal_map.at(sig_b);
                }

            return value_a < value_b;
        }
    } has_lower_signalID;


    for(synchro_map_iter = synchro_map.cbegin();
            synchro_map_iter != synchro_map.cend();
            synchro_map_iter++)

        {
            std::pair<int, Gnss_Synchro> p(synchro_map_iter->first, synchro_map_iter->second);
            my_vec.push_back(p);
        }

    std::sort(my_vec.begin(), my_vec.end(), has_lower_signalID);
    return my_vec;
}


std::map<std::string, int> Rtcm::gps_signal_map = []
{
    std::map<std::string, int> gps_signal_map_;
    // Table 3.5-91
    gps_signal_map_["1C"] = 2;
    gps_signal_map_["1P"] = 3;
    gps_signal_map_["1W"] = 4;
    gps_signal_map_["2C"] = 8;
    gps_signal_map_["2P"] = 9;
    gps_signal_map_["2W"] = 10;
    gps_signal_map_["2S"] = 15;
    gps_signal_map_["2L"] = 16;
    gps_signal_map_["2X"] = 17;
    gps_signal_map_["5I"] = 22;
    gps_signal_map_["5Q"] = 23;
    gps_signal_map_["5X"] = 24;
    return gps_signal_map_;
}();


std::map<std::string, int> Rtcm::galileo_signal_map = []
{
    std::map<std::string, int> galileo_signal_map_;
    // Table 3.5-100
    galileo_signal_map_["1C"] = 2;
    galileo_signal_map_["1A"] = 3;
    galileo_signal_map_["1B"] = 4;
    galileo_signal_map_["1X"] = 5;
    galileo_signal_map_["1Z"] = 6;
    galileo_signal_map_["6C"] = 8;
    galileo_signal_map_["6A"] = 9;
    galileo_signal_map_["6B"] = 10;
    galileo_signal_map_["6X"] = 11;
    galileo_signal_map_["6Z"] = 12;
    galileo_signal_map_["7I"] = 14;
    galileo_signal_map_["7Q"] = 15;
    galileo_signal_map_["7X"] = 16;
    galileo_signal_map_["8I"] = 18;
    galileo_signal_map_["8Q"] = 19;
    galileo_signal_map_["8X"] = 20;
    galileo_signal_map_["5I"] = 22;
    galileo_signal_map_["5Q"] = 23;
    galileo_signal_map_["5X"] = 24;
    return galileo_signal_map_;
}();


boost::posix_time::ptime Rtcm::compute_GPS_time(const Gps_Ephemeris & eph, double obs_time) const
{
    const double gps_t = obs_time;
    boost::posix_time::time_duration t = boost::posix_time::millisec((gps_t + 604800 * static_cast<double>(eph.i_GPS_week % 1024)) * 1000);
    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
    return p_time;
}


boost::posix_time::ptime Rtcm::compute_GPS_time(const Gps_CNAV_Ephemeris & eph, double obs_time) const
{
    const double gps_t = obs_time;
    boost::posix_time::time_duration t = boost::posix_time::millisec((gps_t + 604800 * static_cast<double>(eph.i_GPS_week % 1024)) * 1000);
    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
    return p_time;
}


boost::posix_time::ptime Rtcm::compute_Galileo_time(const Galileo_Ephemeris & eph, double obs_time) const
{
    double galileo_t = obs_time;
    boost::posix_time::time_duration t = boost::posix_time::millisec((galileo_t + 604800 * static_cast<double>(eph.WN_5)) * 1000);
    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
    return p_time;
}


unsigned int Rtcm::lock_time(const Gps_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro)
{
    unsigned int lock_time_in_seconds;
    boost::posix_time::ptime current_time = Rtcm::compute_GPS_time(eph, obs_time);
    boost::posix_time::ptime last_lock_time = Rtcm::gps_L1_last_lock_time[65 - gnss_synchro.PRN];
    if(last_lock_time.is_not_a_date_time() )// || CHECK LLI!!......)
        {
            Rtcm::gps_L1_last_lock_time[65 - gnss_synchro.PRN] = current_time;
        }
    boost::posix_time::time_duration lock_duration = current_time - Rtcm::gps_L1_last_lock_time[65 - gnss_synchro.PRN];
    lock_time_in_seconds = static_cast<unsigned int>(lock_duration.total_seconds());
    // Debug:
    // std::cout << "lock time PRN " << gnss_synchro.PRN << ": " << lock_time_in_seconds <<  "  current time: " << current_time << std::endl;
    return lock_time_in_seconds;
}


unsigned int Rtcm::lock_time(const Gps_CNAV_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro)
{
    unsigned int lock_time_in_seconds;
    boost::posix_time::ptime current_time = Rtcm::compute_GPS_time(eph, obs_time);
    boost::posix_time::ptime last_lock_time = Rtcm::gps_L2_last_lock_time[65 - gnss_synchro.PRN];
    if(last_lock_time.is_not_a_date_time() )// || CHECK LLI!!......)
        {
            Rtcm::gps_L2_last_lock_time[65 - gnss_synchro.PRN] = current_time;
        }
    boost::posix_time::time_duration lock_duration = current_time - Rtcm::gps_L2_last_lock_time[65 - gnss_synchro.PRN];
    lock_time_in_seconds = static_cast<unsigned int>(lock_duration.total_seconds());
    return lock_time_in_seconds;
}


unsigned int Rtcm::lock_time(const Galileo_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro)
{
    unsigned int lock_time_in_seconds;
    boost::posix_time::ptime current_time = Rtcm::compute_Galileo_time(eph, obs_time);

    boost::posix_time::ptime last_lock_time;
    std::string sig_(gnss_synchro.Signal);
    if(sig_.compare("1B") == 0)
        {
            last_lock_time = Rtcm::gal_E1_last_lock_time[65 - gnss_synchro.PRN];
        }
    if((sig_.compare("5X") == 0) || (sig_.compare("8X") == 0) || (sig_.compare("7X") == 0) )
        {
            last_lock_time = Rtcm::gal_E5_last_lock_time[65 - gnss_synchro.PRN];
        }

    if(last_lock_time.is_not_a_date_time() )// || CHECK LLI!!......)
        {
            if(sig_.compare("1B") == 0)
                {
                    Rtcm::gal_E1_last_lock_time[65 - gnss_synchro.PRN] = current_time;
                }
            if((sig_.compare("5X") == 0) || (sig_.compare("8X") == 0) || (sig_.compare("7X") == 0) )
                {
                    Rtcm::gal_E5_last_lock_time[65 - gnss_synchro.PRN] = current_time;
                }
        }

    boost::posix_time::time_duration lock_duration = current_time - current_time;
    if(sig_.compare("1B") == 0)
        {
            lock_duration = current_time - Rtcm::gal_E1_last_lock_time[65 - gnss_synchro.PRN];
        }
    if((sig_.compare("5X") == 0) || (sig_.compare("8X") == 0) || (sig_.compare("7X") == 0) )
        {
            lock_duration = current_time - Rtcm::gal_E5_last_lock_time[65 - gnss_synchro.PRN];
        }

    lock_time_in_seconds = static_cast<unsigned int>(lock_duration.total_seconds());
    return lock_time_in_seconds;
}


unsigned int Rtcm::lock_time_indicator(unsigned int lock_time_period_s)
{
    // Table 3.4-2
    if(lock_time_period_s <= 0 ) return 0;
    if(lock_time_period_s < 24 ) return lock_time_period_s;
    if(lock_time_period_s < 72 ) return (lock_time_period_s + 24  ) / 2;
    if(lock_time_period_s < 168) return (lock_time_period_s + 120 ) / 4;
    if(lock_time_period_s < 360) return (lock_time_period_s + 408 ) / 8;
    if(lock_time_period_s < 744) return (lock_time_period_s + 1176) / 16;
    if(lock_time_period_s < 937) return (lock_time_period_s + 3096) / 32;
    return 127;
}


unsigned int Rtcm::msm_lock_time_indicator(unsigned int lock_time_period_s)
{
    // Table 3.5-74
    if(lock_time_period_s < 32    ) return 0;
    if(lock_time_period_s < 64    ) return 1;
    if(lock_time_period_s < 128   ) return 2;
    if(lock_time_period_s < 256   ) return 3;
    if(lock_time_period_s < 512   ) return 4;
    if(lock_time_period_s < 1024  ) return 5;
    if(lock_time_period_s < 2048  ) return 6;
    if(lock_time_period_s < 4096  ) return 7;
    if(lock_time_period_s < 8192  ) return 8;
    if(lock_time_period_s < 16384 ) return 9;
    if(lock_time_period_s < 32768 ) return 10;
    if(lock_time_period_s < 65536 ) return 11;
    if(lock_time_period_s < 131072) return 12;
    if(lock_time_period_s < 262144) return 13;
    if(lock_time_period_s < 524288) return 14;
    return 15;
}


unsigned int Rtcm::msm_extended_lock_time_indicator(unsigned int lock_time_period_s)
{
    // Table 3.5-75
    if(                                   lock_time_period_s < 64       ) return (       lock_time_period_s                      );
    if(       64 <= lock_time_period_s && lock_time_period_s < 128      ) return ( 64 + (lock_time_period_s - 64      ) / 2      );
    if(      128 <= lock_time_period_s && lock_time_period_s < 256      ) return ( 96 + (lock_time_period_s - 128     ) / 4      );
    if(      256 <= lock_time_period_s && lock_time_period_s < 512      ) return (128 + (lock_time_period_s - 256     ) / 8      );
    if(      512 <= lock_time_period_s && lock_time_period_s < 1024     ) return (160 + (lock_time_period_s - 512     ) / 16     );
    if(     1024 <= lock_time_period_s && lock_time_period_s < 2048     ) return (192 + (lock_time_period_s - 1024    ) / 32     );
    if(     2048 <= lock_time_period_s && lock_time_period_s < 4096     ) return (224 + (lock_time_period_s - 2048    ) / 64     );
    if(     4096 <= lock_time_period_s && lock_time_period_s < 8192     ) return (256 + (lock_time_period_s - 4096    ) / 128    );
    if(     8192 <= lock_time_period_s && lock_time_period_s < 16384    ) return (288 + (lock_time_period_s - 8192    ) / 256    );
    if(    16384 <= lock_time_period_s && lock_time_period_s < 32768    ) return (320 + (lock_time_period_s - 16384   ) / 512    );
    if(    32768 <= lock_time_period_s && lock_time_period_s < 65536    ) return (352 + (lock_time_period_s - 32768   ) / 1024   );
    if(    65536 <= lock_time_period_s && lock_time_period_s < 131072   ) return (384 + (lock_time_period_s - 65536   ) / 2048   );
    if(   131072 <= lock_time_period_s && lock_time_period_s < 262144   ) return (416 + (lock_time_period_s - 131072  ) / 4096   );
    if(   262144 <= lock_time_period_s && lock_time_period_s < 524288   ) return (448 + (lock_time_period_s - 262144  ) / 8192   );
    if(   524288 <= lock_time_period_s && lock_time_period_s < 1048576  ) return (480 + (lock_time_period_s - 524288  ) / 16384  );
    if(  1048576 <= lock_time_period_s && lock_time_period_s < 2097152  ) return (512 + (lock_time_period_s - 1048576 ) / 32768  );
    if(  2097152 <= lock_time_period_s && lock_time_period_s < 4194304  ) return (544 + (lock_time_period_s - 2097152 ) / 65536  );
    if(  4194304 <= lock_time_period_s && lock_time_period_s < 8388608  ) return (576 + (lock_time_period_s - 4194304 ) / 131072 );
    if(  8388608 <= lock_time_period_s && lock_time_period_s < 16777216 ) return (608 + (lock_time_period_s - 8388608 ) / 262144 );
    if( 16777216 <= lock_time_period_s && lock_time_period_s < 33554432 ) return (640 + (lock_time_period_s - 16777216) / 524288 );
    if( 33554432 <= lock_time_period_s && lock_time_period_s < 67108864 ) return (672 + (lock_time_period_s - 33554432) / 1048576);
    if( 67108864 <= lock_time_period_s                                  ) return (704                                            );
    return 1023; // will never happen
}


// *****************************************************************************************************
//
//   DATA FIELDS AS DEFINED AT RTCM STANDARD 10403.2
//
// *****************************************************************************************************

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


int Rtcm::set_DF004(double obs_time)
{
    // TOW in milliseconds from the beginning of the GPS week, measured in GPS time
    unsigned long int tow = static_cast<unsigned long int>(std::round(obs_time * 1000));
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


int Rtcm::set_DF006(const std::map<int, Gnss_Synchro> & observables)
{
    //Number of satellites observed in current epoch
    unsigned short int nsats = 0;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;
    for(observables_iter = observables.cbegin();
            observables_iter != observables.cend();
            observables_iter++)
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
    double ambiguity = std::floor( gnss_synchro.Pseudorange_m / 299792.458 );
    unsigned long int gps_L1_pseudorange = static_cast<unsigned long int>(std::round(( gnss_synchro.Pseudorange_m - ambiguity * 299792.458) / 0.02 ));
    DF011 = std::bitset<24>(gps_L1_pseudorange);
    return 0;
}


int Rtcm::set_DF012(const Gnss_Synchro & gnss_synchro)
{
    const double lambda = GPS_C_m_s / GPS_L1_FREQ_HZ;
    double ambiguity = std::floor( gnss_synchro.Pseudorange_m / 299792.458 );
    double gps_L1_pseudorange = std::round(( gnss_synchro.Pseudorange_m - ambiguity * 299792.458) / 0.02 );
    double gps_L1_pseudorange_c = gps_L1_pseudorange * 0.02 + ambiguity * 299792.458;
    double L1_phaserange_c = gnss_synchro.Carrier_phase_rads / GPS_TWO_PI;
    double L1_phaserange_c_r = std::fmod(L1_phaserange_c - gps_L1_pseudorange_c / lambda + 1500.0, 3000.0) - 1500.0;
    long int gps_L1_phaserange_minus_L1_pseudorange = static_cast<long int>(std::round(L1_phaserange_c_r * lambda / 0.0005 ));
    DF012 = std::bitset<20>(gps_L1_phaserange_minus_L1_pseudorange);
    return 0;
}


int Rtcm::set_DF013(const Gps_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro)
{
    unsigned int lock_time_indicator;
    unsigned int lock_time_period_s = Rtcm::lock_time(eph, obs_time, gnss_synchro);
    lock_time_indicator = Rtcm::lock_time_indicator(lock_time_period_s);
    DF013 = std::bitset<7>(lock_time_indicator);
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


int Rtcm::set_DF017(const Gnss_Synchro & gnss_synchroL1, const Gnss_Synchro & gnss_synchroL2)
{
    double ambiguity = std::floor( gnss_synchroL1.Pseudorange_m / 299792.458 );
    double gps_L1_pseudorange = std::round(( gnss_synchroL1.Pseudorange_m - ambiguity * 299792.458) / 0.02 );
    double gps_L1_pseudorange_c = gps_L1_pseudorange * 0.02 + ambiguity * 299792.458;

    double l2_l1_pseudorange = gnss_synchroL2.Pseudorange_m - gps_L1_pseudorange_c;
    int pseudorange_difference = 0xFFFFE000; // invalid value;
    if(std::fabs(l2_l1_pseudorange) <= 163.82)
        {
            pseudorange_difference = static_cast<int>(std::round(l2_l1_pseudorange / 0.02));
        }
    DF017 = std::bitset<14>(pseudorange_difference);
    return 0;
}


int Rtcm::set_DF018(const Gnss_Synchro & gnss_synchroL1, const Gnss_Synchro & gnss_synchroL2)
{
    const double lambda2 = GPS_C_m_s / GPS_L2_FREQ_HZ;
    int l2_phaserange_minus_l1_pseudorange = 0xFFF80000;
    double ambiguity = std::floor( gnss_synchroL1.Pseudorange_m / 299792.458 );
    double gps_L1_pseudorange = std::round(( gnss_synchroL1.Pseudorange_m - ambiguity * 299792.458) / 0.02 );
    double gps_L1_pseudorange_c = gps_L1_pseudorange * 0.02 + ambiguity * 299792.458;
    double L2_phaserange_c = gnss_synchroL2.Carrier_phase_rads / GPS_TWO_PI;
    double L1_phaserange_c_r = std::fmod(L2_phaserange_c - gps_L1_pseudorange_c / lambda2 + 1500.0, 3000.0) - 1500.0;

    if (std::fabs(L1_phaserange_c_r * lambda2) <= 262.1435 )
        {
            l2_phaserange_minus_l1_pseudorange = static_cast<int>(std::round(L1_phaserange_c_r * lambda2 / 0.0005));
        }

    DF018 = std::bitset<20>(l2_phaserange_minus_l1_pseudorange);
    return 0;
}


int Rtcm::set_DF019(const Gps_CNAV_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro)
{
    unsigned int lock_time_indicator;
    unsigned int lock_time_period_s = Rtcm::lock_time(eph, obs_time, gnss_synchro);
    lock_time_indicator = Rtcm::lock_time_indicator(lock_time_period_s);
    DF019 = std::bitset<7>(lock_time_indicator);
    return 0;
}


int Rtcm::set_DF020(const Gnss_Synchro & gnss_synchro)
{
    double CN0_dB_Hz_est = gnss_synchro.CN0_dB_hz;
    if (CN0_dB_Hz_est > 63.75)
        {
            CN0_dB_Hz_est = 63.75;
        }
    unsigned int CN0_dB_Hz = static_cast<unsigned int>(std::round(CN0_dB_Hz_est / 0.25 ));
    DF020 = std::bitset<8>(CN0_dB_Hz);
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


int Rtcm::set_DF027(double antenna_ECEF_Z_m)
{
    long long int ant_ref_z = static_cast<long long int>(std::round( antenna_ECEF_Z_m * 10000));
    DF027 = std::bitset<38>(ant_ref_z);
    return 0;
}


int Rtcm::set_DF028(double height)
{
    unsigned int h_ = static_cast<unsigned int>(std::round( height * 10000));
    DF028 = std::bitset<16>(h_);
    return 0;
}


int Rtcm::set_DF031(unsigned int antenna_setup_id)
{
    DF031 = std::bitset<8>(antenna_setup_id);
    return 0;
}


int Rtcm::set_DF051(const Gps_Ephemeris & gps_eph, double obs_time)
{
    const double gps_t = obs_time;
    boost::posix_time::time_duration t = boost::posix_time::millisec((gps_t + 604800 * static_cast<double>(gps_eph.i_GPS_week % 1024)) * 1000);
    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
    std::string now_ptime = to_iso_string(p_time);
    std::string today_ptime = now_ptime.substr(0, 8);
    boost::gregorian::date d(boost::gregorian::from_undelimited_string(today_ptime));
    unsigned int mjd = d.modjulian_day();
    DF051 = std::bitset<16>(mjd);
    return 0;
}


int Rtcm::set_DF052(const Gps_Ephemeris & gps_eph, double obs_time)
{
    const double gps_t = obs_time;
    boost::posix_time::time_duration t = boost::posix_time::millisec((gps_t + 604800 * static_cast<double>(gps_eph.i_GPS_week % 1024)) * 1000);
    boost::posix_time::ptime p_time(boost::gregorian::date(1999, 8, 22), t);
    std::string now_ptime = to_iso_string(p_time);
    std::string hours = now_ptime.substr(9, 2);
    std::string minutes = now_ptime.substr(11, 2);
    std::string seconds = now_ptime.substr(13, 8);
    //boost::gregorian::date d(boost::gregorian::from_undelimited_string(today_ptime));
    long unsigned int seconds_of_day = boost::lexical_cast<unsigned int>(hours) * 60 * 60 + boost::lexical_cast<unsigned int>(minutes) * 60 + boost::lexical_cast<unsigned int>(seconds);
    DF052 = std::bitset<17>(seconds_of_day);
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


int Rtcm::set_DF248(double obs_time)
{
    // TOW in milliseconds from the beginning of the Galileo week, measured in Galileo time
    unsigned long int tow = static_cast<unsigned long int>(std::round(obs_time * 1000));
    if(tow > 604799999)
        {
            LOG(WARNING) << "To large TOW! Set to the last millisecond of the week";
            tow = 604799999;
        }
    DF248 = std::bitset<30>(tow);
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
    for(gnss_synchro_iter = gnss_synchro.cbegin();
            gnss_synchro_iter != gnss_synchro.cend();
            gnss_synchro_iter++)
        {
            mask_position = 64 - gnss_synchro_iter->second.PRN;
            DF394.set(mask_position, true);
        }
    return 0;
}


int Rtcm::set_DF395(const std::map<int, Gnss_Synchro> & gnss_synchro)
{
    DF395.reset();
    if(gnss_synchro.size() == 0)
        {
            return 1;
        }
    std::map<int, Gnss_Synchro>::const_iterator gnss_synchro_iter;
    std::string sig;
    unsigned int mask_position;
    for(gnss_synchro_iter = gnss_synchro.cbegin();
            gnss_synchro_iter != gnss_synchro.cend();
            gnss_synchro_iter++)
        {
            std::string sig_(gnss_synchro_iter->second.Signal);
            sig = sig_.substr(0,2);

            std::string sys(&gnss_synchro_iter->second.System, 1);

            if ((sig.compare("1C") == 0) && (sys.compare("G") == 0 ) )
                {
                    mask_position = 32 - 2;
                    DF395.set(mask_position, true);
                }
            if ((sig.compare("2S") == 0) && (sys.compare("G") == 0 ) )
                {
                    mask_position = 32 - 15;
                    DF395.set(mask_position, true);
                }

            if ((sig.compare("5X") == 0) && (sys.compare("G") == 0 ) )
                {
                    mask_position = 32 - 24;
                    DF395.set(mask_position, true);
                }
            if ((sig.compare("1B") == 0) && (sys.compare("E") == 0 ) )
                {
                    mask_position = 32 - 4;
                    DF395.set(mask_position, true);
                }

            if ((sig.compare("5X") == 0) && (sys.compare("E") == 0 ) )
                {
                    mask_position = 32 - 24;
                    DF395.set(mask_position, true);
                }
            if ((sig.compare("7X") == 0) && (sys.compare("E") == 0 ) )
                {
                    mask_position = 32 - 16;
                    DF395.set(mask_position, true);
                }
        }

    return 0;
}


std::string Rtcm::set_DF396(const std::map<int, Gnss_Synchro> & observables)
{
    std::string DF396;
    std::map<int, Gnss_Synchro>::const_iterator observables_iter;
    Rtcm::set_DF394(observables);
    Rtcm::set_DF395(observables);
    unsigned int num_signals = DF395.count();
    unsigned int num_satellites = DF394.count();

    if ((num_signals == 0) || (num_satellites == 0))
        {
            std::string s("");
            return s;
        }
    std::vector<std::vector<bool> > matrix(num_signals, std::vector<bool>());

    std::string sig;
    std::vector<unsigned int> list_of_sats;
    std::vector<int> list_of_signals;

    for(observables_iter = observables.cbegin();
            observables_iter != observables.cend();
            observables_iter++)
        {
            list_of_sats.push_back(observables_iter->second.PRN);

            std::string sig_(observables_iter->second.Signal);
            sig = sig_.substr(0,2);

            std::string sys(&observables_iter->second.System, 1);

            if ((sig.compare("1C") == 0) && (sys.compare("G") == 0 ) )
                {
                    list_of_signals.push_back(32 - 2);
                }
            if ((sig.compare("2S") == 0) && (sys.compare("G") == 0 ) )
                {
                    list_of_signals.push_back(32 - 15);
                }

            if ((sig.compare("5X") == 0) && (sys.compare("G") == 0 ) )
                {
                    list_of_signals.push_back(32 - 24);
                }
            if ((sig.compare("1B") == 0) && (sys.compare("E") == 0 ) )
                {
                    list_of_signals.push_back(32 - 4);
                }

            if ((sig.compare("5X") == 0) && (sys.compare("E") == 0 ) )
                {
                    list_of_signals.push_back(32 - 24);
                }
            if ((sig.compare("7X") == 0) && (sys.compare("E") == 0 ) )
                {
                    list_of_signals.push_back(32 - 16);
                }
        }

    std::sort( list_of_sats.begin(), list_of_sats.end() );
    list_of_sats.erase( std::unique( list_of_sats.begin(), list_of_sats.end() ), list_of_sats.end() );

    std::sort( list_of_signals.begin(), list_of_signals.end() );
    std::reverse(list_of_signals.begin(), list_of_signals.end());
    list_of_signals.erase( std::unique( list_of_signals.begin(), list_of_signals.end() ), list_of_signals.end() );

    // fill the matrix
    bool value;

    for(unsigned int row = 0; row < num_signals; row++)
        {
            for(unsigned int sat = 0; sat < num_satellites; sat++)
                {
                    value = false;
                    for(observables_iter = observables.cbegin();
                            observables_iter != observables.cend();
                            observables_iter++)
                        {
                            std::string sig_(observables_iter->second.Signal);
                            sig = sig_.substr(0,2);
                            std::string sys(&observables_iter->second.System, 1);

                            if ((sig.compare("1C") == 0) && (sys.compare("G") == 0 ) && (list_of_signals.at(row) == 32 - 2) && (observables_iter->second.PRN == list_of_sats.at(sat) ) )
                                {
                                    value = true;
                                }

                            if ((sig.compare("2S") == 0) && (sys.compare("G") == 0 ) && (list_of_signals.at(row) == 32 - 15) && (observables_iter->second.PRN == list_of_sats.at(sat) ) )
                                {
                                    value = true;
                                }

                            if ((sig.compare("5X") == 0) && (sys.compare("G") == 0 ) && (list_of_signals.at(row) == 32 - 24) && (observables_iter->second.PRN == list_of_sats.at(sat) ) )
                                {
                                    value = true;
                                }

                            if ((sig.compare("1B") == 0) && (sys.compare("E") == 0 ) && (list_of_signals.at(row) == 32 - 4) && (observables_iter->second.PRN == list_of_sats.at(sat) ) )
                                {
                                    value = true;
                                }

                            if ((sig.compare("5X") == 0) && (sys.compare("E") == 0 ) && (list_of_signals.at(row) == 32 - 24) && (observables_iter->second.PRN == list_of_sats.at(sat) ) )
                                {
                                    value = true;
                                }

                            if ((sig.compare("7X") == 0) && (sys.compare("E") == 0 ) && (list_of_signals.at(row) == 32 - 16) && (observables_iter->second.PRN == list_of_sats.at(sat) ) )
                                {
                                    value = true;
                                }
                        }
                    matrix[row].push_back(value);
                }
        }

    // write the matrix column-wise
    DF396.clear();
    for(unsigned int col = 0; col < num_satellites; col++)
        {
            for(unsigned int row = 0; row < num_signals; row++)
                {
                    std::string ss;
                    if(matrix[row].at(col))
                        {
                            ss = "1";
                        }
                    else
                        {
                            ss = "0";
                        }
                    DF396 += ss;
                }
        }
    return DF396;
}


int Rtcm::set_DF397(const Gnss_Synchro & gnss_synchro)
{
    double meters_to_miliseconds = GPS_C_m_s * 0.001;
    double rough_range_s = std::round(gnss_synchro.Pseudorange_m / meters_to_miliseconds / TWO_N10) * meters_to_miliseconds * TWO_N10;

    unsigned int int_ms = 0;

    if (rough_range_s == 0.0)
        {
            int_ms = 255;
        }
    else if((rough_range_s < 0.0) || (rough_range_s > meters_to_miliseconds * 255.0))
        {
            int_ms = 255;
        }
    else
        {
            int_ms = static_cast<unsigned int>(std::floor(rough_range_s / meters_to_miliseconds / TWO_N10) + 0.5) >> 10;
        }

    DF397 = std::bitset<8>(int_ms);
    return 0;
}


int Rtcm::set_DF398(const Gnss_Synchro & gnss_synchro)
{
    double meters_to_miliseconds = GPS_C_m_s * 0.001;
    double rough_range_m = std::round(gnss_synchro.Pseudorange_m / meters_to_miliseconds / TWO_N10) * meters_to_miliseconds * TWO_N10;
    unsigned int rr_mod_ms;
    if((rough_range_m <= 0.0) || (rough_range_m > meters_to_miliseconds * 255.0))
        {
            rr_mod_ms = 0;
        }
    else
        {
            rr_mod_ms = static_cast<unsigned int>(std::floor(rough_range_m / meters_to_miliseconds / TWO_N10) + 0.5) & 0x3FFu;
        }
    DF398 = std::bitset<10>(rr_mod_ms);
    return 0;
}


int Rtcm::set_DF399(const Gnss_Synchro & gnss_synchro)
{
    double lambda = 0.0;
    std::string sig_(gnss_synchro.Signal);
    std::string sig = sig_.substr(0,2);

    if (sig.compare("1C") == 0 )
        {
            lambda = GPS_C_m_s / GPS_L1_FREQ_HZ;
        }
    if (sig.compare("2S") == 0 )
        {
            lambda = GPS_C_m_s / GPS_L2_FREQ_HZ;
        }
    if (sig.compare("5X") == 0 )
        {
            lambda = GPS_C_m_s / Galileo_E5a_FREQ_HZ;
        }
    if (sig.compare("1B") == 0 )
        {
            lambda = GPS_C_m_s / Galileo_E1_FREQ_HZ;
        }
    if (sig.compare("7X") == 0 )
        {
            lambda = GPS_C_m_s / 1.207140e9; // Galileo_E1b_FREQ_HZ;
        }

    double rough_phase_range_rate_ms = std::round(- gnss_synchro.Carrier_Doppler_hz * lambda );
    if(rough_phase_range_rate_ms < - 8191) rough_phase_range_rate_ms = -8192;
    if(rough_phase_range_rate_ms > 8191) rough_phase_range_rate_ms = -8192;

    DF399 = std::bitset<14>(static_cast<int>(rough_phase_range_rate_ms));
    return 0;
}


int Rtcm::set_DF400(const Gnss_Synchro & gnss_synchro)
{
    double meters_to_miliseconds = GPS_C_m_s * 0.001;
    double rough_range_m = std::round(gnss_synchro.Pseudorange_m / meters_to_miliseconds / TWO_N10) * meters_to_miliseconds * TWO_N10;
    double psrng_s;
    int fine_pseudorange;

    psrng_s = gnss_synchro.Pseudorange_m - rough_range_m;

    if (psrng_s == 0)
        {
            fine_pseudorange = -16384;
        }
    else if(std::fabs(psrng_s) > 292.7)
        {
            fine_pseudorange = -16384; // 4000h: invalid value
        }
    else
        {
            fine_pseudorange = static_cast<int>(std::round(psrng_s / meters_to_miliseconds / TWO_N24));
        }

    DF400 = std::bitset<15>(fine_pseudorange);
    return 0;
}


int Rtcm::set_DF401(const Gnss_Synchro & gnss_synchro)
{
    double meters_to_miliseconds = GPS_C_m_s * 0.001;
    double rough_range_m = std::round(gnss_synchro.Pseudorange_m / meters_to_miliseconds / TWO_N10) * meters_to_miliseconds * TWO_N10;
    double phrng_m;
    long int fine_phaserange;

    double lambda = 0.0;
    std::string sig_(gnss_synchro.Signal);
    std::string sig = sig_.substr(0,2);

    if (sig.compare("1C") == 0 )
        {
            lambda = GPS_C_m_s / GPS_L1_FREQ_HZ;
        }
    if (sig.compare("2S") == 0 )
        {
            lambda = GPS_C_m_s / GPS_L2_FREQ_HZ;
        }
    if (sig.compare("5X") == 0 )
        {
            lambda = GPS_C_m_s / Galileo_E5a_FREQ_HZ;
        }
    if (sig.compare("1B") == 0 )
        {
            lambda = GPS_C_m_s / Galileo_E1_FREQ_HZ;
        }
    if (sig.compare("7X") == 0 )
        {
            lambda = GPS_C_m_s / 1.207140e9; // Galileo_E1b_FREQ_HZ;
        }

    phrng_m = (gnss_synchro.Carrier_phase_rads / GPS_TWO_PI ) * lambda - rough_range_m;

    /* Substract phase - pseudorange integer cycle offset */
    /* TODO: check LLI! */
    double cp = gnss_synchro.Carrier_phase_rads / GPS_TWO_PI; // ?
    if(std::fabs(phrng_m - cp) > 1171.0)
        {
            cp = std::round(phrng_m / lambda) * lambda;
        }
    phrng_m -= cp;

    if(phrng_m == 0.0)
        {
            fine_phaserange = - 2097152;
        }
    else if(std::fabs(phrng_m) > 1171.0)
        {
            fine_phaserange = - 2097152;
        }
    else
        {
            fine_phaserange = static_cast<long int>(std::round(phrng_m / meters_to_miliseconds / TWO_N29));
        }

    DF401 = std::bitset<22>(fine_phaserange);
    return 0;
}


int Rtcm::set_DF402(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const Gnss_Synchro & gnss_synchro)
{
    unsigned int lock_time_period_s = 0;
    unsigned int lock_time_indicator;
    std::string sig_(gnss_synchro.Signal);
    if(sig_.compare("1C"))
        {
            lock_time_period_s = Rtcm::lock_time(ephNAV, obs_time, gnss_synchro);
        }
    if(sig_.compare("2S"))
        {
            lock_time_period_s = Rtcm::lock_time(ephCNAV, obs_time, gnss_synchro);
        }
    if(sig_.compare("1B") || sig_.compare("5X") || sig_.compare("7X") || sig_.compare("8X"))
        {
            lock_time_period_s = Rtcm::lock_time(ephFNAV, obs_time, gnss_synchro);
        }
    lock_time_indicator = Rtcm::msm_lock_time_indicator(lock_time_period_s);
    DF402 = std::bitset<4>(lock_time_indicator);
    return 0;
}


int Rtcm::set_DF403(const Gnss_Synchro & gnss_synchro)
{
    unsigned int cnr_dB_Hz;
    cnr_dB_Hz = static_cast<unsigned int>(std::round(gnss_synchro.CN0_dB_hz));
    DF403 = std::bitset<6>(cnr_dB_Hz);
    return 0;
}


int Rtcm::set_DF404(const Gnss_Synchro & gnss_synchro)
{
    double lambda = 0.0;
    std::string sig_(gnss_synchro.Signal);
    std::string sig = sig_.substr(0,2);
    int fine_phaserange_rate;

    if (sig.compare("1C") == 0 )
        {
            lambda = GPS_C_m_s / GPS_L1_FREQ_HZ;
        }
    if (sig.compare("2S") == 0 )
        {
            lambda = GPS_C_m_s / GPS_L2_FREQ_HZ;
        }
    if (sig.compare("5X") == 0 )
        {
            lambda = GPS_C_m_s / Galileo_E5a_FREQ_HZ;
        }
    if (sig.compare("1B") == 0 )
        {
            lambda = GPS_C_m_s / Galileo_E1_FREQ_HZ;
        }
    if (sig.compare("7X") == 0 )
        {
            lambda = GPS_C_m_s / 1.207140e9; // Galileo_E1b_FREQ_HZ;
        }

    double rough_phase_range_rate = std::round(- gnss_synchro.Carrier_Doppler_hz * lambda );
    double phrr = (- gnss_synchro.Carrier_Doppler_hz * lambda - rough_phase_range_rate);

    if(phrr == 0.0)
          {
              fine_phaserange_rate = -16384;
          }
      else if(std::fabs(phrr) > 1.6384)
          {
              fine_phaserange_rate = -16384;
          }
      else
          {
              fine_phaserange_rate = static_cast<int>(std::round(phrr / 0.0001));
          }

    DF404 = std::bitset<15>(fine_phaserange_rate);
    return 0;
}


int Rtcm::set_DF405(const Gnss_Synchro & gnss_synchro)
{
    double meters_to_miliseconds = GPS_C_m_s * 0.001;
    double rough_range_m = std::round(gnss_synchro.Pseudorange_m / meters_to_miliseconds / TWO_N10) * meters_to_miliseconds * TWO_N10;
    double psrng_s;
    long int fine_pseudorange;

    psrng_s = gnss_synchro.Pseudorange_m - rough_range_m;

    if(psrng_s == 0.0)
        {
            fine_pseudorange = - 524288;
        }
    else if(std::fabs(psrng_s) > 292.7)
        {
            fine_pseudorange = - 524288;
        }
    else
        {
            fine_pseudorange = static_cast<long int>(std::round(psrng_s / meters_to_miliseconds / TWO_N29));
        }
    DF405 = std::bitset<20>(fine_pseudorange);
    return 0;

}


int Rtcm::set_DF406(const Gnss_Synchro & gnss_synchro)
{
    long int fine_phaserange_ex;
    double meters_to_miliseconds = GPS_C_m_s * 0.001;
    double rough_range_m = std::round(gnss_synchro.Pseudorange_m / meters_to_miliseconds / TWO_N10) * meters_to_miliseconds * TWO_N10;
    double phrng_m;
    double lambda = 0.0;
    std::string sig_(gnss_synchro.Signal);
    std::string sig = sig_.substr(0,2);

    if (sig.compare("1C") == 0 )
        {
            lambda = GPS_C_m_s / GPS_L1_FREQ_HZ;
        }
    if (sig.compare("2S") == 0 )
        {
            lambda = GPS_C_m_s / GPS_L2_FREQ_HZ;
        }
    if (sig.compare("5X") == 0 )
        {
            lambda = GPS_C_m_s / Galileo_E5a_FREQ_HZ;
        }
    if (sig.compare("1B") == 0 )
        {
            lambda = GPS_C_m_s / Galileo_E1_FREQ_HZ;
        }
    if (sig.compare("7X") == 0 )
        {
            lambda = GPS_C_m_s / 1.207140e9; // Galileo_E1b_FREQ_HZ;
        }

    phrng_m = (gnss_synchro.Carrier_phase_rads / GPS_TWO_PI ) * lambda - rough_range_m;

    /* Substract phase - pseudorange integer cycle offset */
    /* TODO: check LLI! */
    double cp = gnss_synchro.Carrier_phase_rads / GPS_TWO_PI; // ?
    if(std::fabs(phrng_m - cp) > 1171.0)
        {
            cp = std::round(phrng_m / lambda) * lambda;
        }
    phrng_m -= cp;

    if(phrng_m == 0.0)
        {
            fine_phaserange_ex = - 8388608;
        }
    else if(std::fabs(phrng_m) > 1171.0)
        {
            fine_phaserange_ex = - 8388608;
        }
    else
        {
            fine_phaserange_ex = static_cast<long int>(std::round(phrng_m / meters_to_miliseconds / TWO_N31));
        }

    DF406 = std::bitset<24>(fine_phaserange_ex);
    return 0;
}


int Rtcm::set_DF407(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const Gnss_Synchro & gnss_synchro)
{
    unsigned int lock_time_indicator;
    unsigned int lock_time_period_s = 0;

    std::string sig_(gnss_synchro.Signal);
    if(sig_.compare("1C"))
        {
            lock_time_period_s = Rtcm::lock_time(ephNAV, obs_time, gnss_synchro);
        }
    if(sig_.compare("2S"))
        {
            lock_time_period_s = Rtcm::lock_time(ephCNAV, obs_time, gnss_synchro);
        }
    if(sig_.compare("1B") || sig_.compare("5X") || sig_.compare("7X") || sig_.compare("8X"))
        {
            lock_time_period_s = Rtcm::lock_time(ephFNAV, obs_time, gnss_synchro);
        }
    lock_time_indicator = Rtcm::msm_extended_lock_time_indicator(lock_time_period_s);
    DF407 = std::bitset<10>(lock_time_indicator);
    return 0;
}


int Rtcm::set_DF408(const Gnss_Synchro & gnss_synchro)
{
    unsigned int cnr_dB_Hz;
    cnr_dB_Hz = static_cast<unsigned int>(std::round(gnss_synchro.CN0_dB_hz / 0.0625));
    DF408 = std::bitset<10>(cnr_dB_Hz);
    return 0;
}


int Rtcm::set_DF409(unsigned int iods)
{
    DF409 = std::bitset<3>(iods);
    return 0;
}


int Rtcm::set_DF411(unsigned int clock_steering_indicator)
{
    DF411 = std::bitset<2>(clock_steering_indicator);
    return 0;
}


int Rtcm::set_DF412(unsigned int external_clock_indicator)
{
    DF412 = std::bitset<2>(external_clock_indicator);
    return 0;
}


int Rtcm::set_DF417(bool using_divergence_free_smoothing)
{
    DF417 = std::bitset<1>(using_divergence_free_smoothing);
    return 0;
}


int Rtcm::set_DF418(int carrier_smoothing_interval_s)
{
    if(carrier_smoothing_interval_s < 0)
        {
            DF418 = std::bitset<3>("111");
        }
    else
        {
            if(carrier_smoothing_interval_s == 0)
                {
                    DF418 = std::bitset<3>("000");
                }
            else if(carrier_smoothing_interval_s < 30)
                {
                    DF418 = std::bitset<3>("001");
                }
            else if(carrier_smoothing_interval_s < 60)
                {
                    DF418 = std::bitset<3>("010");
                }
            else if(carrier_smoothing_interval_s < 120)
                {
                    DF418 = std::bitset<3>("011");
                }
            else if(carrier_smoothing_interval_s < 240)
                {
                    DF418 = std::bitset<3>("100");
                }
            else if(carrier_smoothing_interval_s < 480)
                {
                    DF418 = std::bitset<3>("101");
                }
            else
                {
                    DF418 = std::bitset<3>("110");
                }
        }
    return 0;
}


int Rtcm::set_DF420(const Gnss_Synchro & gnss_synchro  __attribute__((unused)))
{
    // todo: read the value from gnss_synchro
    bool half_cycle_ambiguity_indicator = true;
    DF420 = std::bitset<1>(half_cycle_ambiguity_indicator);
    return 0;
}
