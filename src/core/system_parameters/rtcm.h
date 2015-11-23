/*!
 * \file rtcm.h
 * \brief  Interface for the RTCM 3.2 Standard
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


#ifndef GNSS_SDR_RTCM_H_
#define GNSS_SDR_RTCM_H_


#include <bitset>
#include <map>
#include <string>
#include <vector>
#include <boost/crc.hpp>
#include "gnss_synchro.h"
#include "galileo_fnav_message.h"
#include "gps_navigation_message.h"

/*!
 * \brief This class implements the generation and reading of some Message Types
 * defined in the RTCM 3.2 Standard.
 */
class Rtcm
{
public:
    Rtcm(); //<! Default constructor

    std::string print_MT1001(const Gps_Ephemeris& gps_eph, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges);

    /*!
     * \brief Prints message type 1005 (Stationary Antenna Reference Point)
     */
    std::string print_MT1005(unsigned int ref_id, double ecef_x, double ecef_y, double ecef_z, bool gps, bool glonass, bool galileo, bool non_physical, bool single_oscillator, unsigned int quarter_cycle_indicator);

    /*!
     * \brief Verifies and reads messages of type 1005 (Stationary Antenna Reference Point). Returns 1 if anything goes wrong, 0 otherwise.
     */
    int read_MT1005(const std::string & message, unsigned int & ref_id, double & ecef_x, double & ecef_y, double & ecef_z, bool & gps, bool & glonass, bool & galileo);

    /*!
     * \brief Prints message type 1019 (GPS Ephemeris), should be broadcast in the event that
     * the IODC does not match the IODE, and every 2 minutes.
     */
    std::string print_MT1019(const Gps_Ephemeris & gps_eph);

    /*!
     * \brief Verifies and reads messages of type 1019 (GPS Ephemeris). Returns 1 if anything goes wrong, 0 otherwise.
     */
    int read_MT1019(const std::string & message, Gps_Ephemeris & gps_eph);

    /*!
     * \brief Prints message type 1045 (Galileo Ephemeris), should be broadcast every 2 minutes
     */
    std::string print_MT1045(const Galileo_Ephemeris & gal_eph);

    /*!
     * \brief Verifies and reads messages of type 1045 (Galileo Ephemeris). Returns 1 if anything goes wrong, 0 otherwise.
     */
    int read_MT1045(const std::string & message, Galileo_Ephemeris & gal_eph);

    std::string bin_to_hex(const std::string& s); //<! Returns a string of hexadecimal symbols from a string of binary symbols
    std::string hex_to_bin(const std::string& s); //<! Returns a string of binary symbols from a string of hexadecimal symbols

    unsigned long int bin_to_uint(const std::string& s); //<! Returns an unsigned long int from a string of binary symbols
    long int bin_to_int(const std::string& s);           //<! Returns a long int from a string of binary symbols

    unsigned long int hex_to_uint(const std::string& s); //<! Returns an unsigned long int from a string of hexadecimal symbols
    long int hex_to_int(const std::string& s);           //<! Returns a long int from a string of hexadecimal symbols

    double bin_to_double(const std::string& s);          //<! Returns double from a string of binary symbols
    std::string print_MT1005_test();                     //<! For testing purposes

    bool check_CRC(const std::string & message);         //<! Checks that the CRC of a RTCM package is correct
private:
    //
    // Messages
    //
    std::bitset<64> message1001_header;
    std::bitset<58> message1001_content;
    std::bitset<64> message1002_header;
    std::bitset<74> message1002_content;
    std::bitset<488> message1019_content;
    std::bitset<496> message1045_content;
    std::bitset<169> MSM_header; // 169+X
    std::vector<std::bitset<18> > MSM4_content; // 18 * Nsat
    std::vector<std::bitset<36> > MSM5_content; // 36 * Nsat

    std::bitset<64> get_MT1001_header(const Gps_Ephemeris & gps_eph,
            double obs_time,
            const std::map<int, Gnss_Synchro> & pseudoranges,
            unsigned int ref_id,
            unsigned int smooth_int,
            bool sync_flag,
            bool divergence_free);

    std::bitset<58> get_MT1001_sat_content(const Gnss_Synchro & gnss_synchro);

    std::bitset<152> get_MT1005_test();

    //
    // Transport Layer
    //
    std::bitset<8> preamble;
    std::bitset<6> reserved_field;
    std::bitset<10> message_length;
    std::bitset<24> crc_frame;
    typedef boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> crc_24_q_type;
    std::string add_CRC(const std::string& m);
    std::string build_message(std::string data); // adds 0s to complete a byte and adds the CRC

    //
    // Data Fields
    //
    int reset_data_fields();

    std::bitset<12> DF002;
    int set_DF002(unsigned int message_number);

    std::bitset<12> DF003;
    int set_DF003(unsigned int ref_station_ID);

    std::bitset<30> DF004;
    int set_DF004(const Gps_Ephemeris & gps_eph, double obs_time);

    std::bitset<1> DF005;
    int set_DF005(bool sync_flag);

    std::bitset<5> DF006;
    int set_DF006(const std::map<int, Gnss_Synchro> & pseudoranges);

    std::bitset<1> DF007;
    int set_DF007(bool divergence_free_smoothing_indicator); // 0 - Divergence-free smoothing not used 1 - Divergence-free smoothing used

    std::bitset<3> DF008;
    int set_DF008(short int smoothing_interval);

    std::bitset<6> DF009;
    int set_DF009(const Gnss_Synchro & gnss_synchro);
    int set_DF009(const Gps_Ephemeris & gps_eph);

    std::bitset<1> DF010;
    int set_DF010(bool code_indicator);

    std::bitset<24> DF011;
    int set_DF011(const Gnss_Synchro & gnss_synchro);

    std::bitset<20> DF012;
    int set_DF012(const Gnss_Synchro & gnss_synchro);

    std::bitset<7> DF013;
    std::bitset<8> DF014;
    int set_DF014(const Gnss_Synchro & gnss_synchro);

    std::bitset<8> DF015;
    int set_DF015(const Gnss_Synchro & gnss_synchro);


    std::bitset<6> DF021;
    int set_DF021();

    std::bitset<1> DF022;
    int set_DF022(bool gps_indicator);

    std::bitset<1> DF023;
    int set_DF023(bool glonass_indicator);

    std::bitset<1> DF024;
    int set_DF024(bool galileo_indicator);

    std::bitset<38> DF025;
    int set_DF025(double antenna_ECEF_X_m);

    std::bitset<38> DF026;
    int set_DF026(double antenna_ECEF_Y_m);

    std::bitset<38> DF027;
    int set_DF027(double antenna_ECEF_Z_m);

    // Contents of GPS Satellite Ephemeris Data, Message Type 1019
    std::bitset<8> DF071;
    int set_DF071(const Gps_Ephemeris & gps_eph);

    std::bitset<10> DF076;
    int set_DF076(const Gps_Ephemeris & gps_eph);

    std::bitset<4> DF077;
    int set_DF077(const Gps_Ephemeris & gps_eph);

    std::bitset<2> DF078;
    int set_DF078(const Gps_Ephemeris & gps_eph);

    std::bitset<14> DF079;
    int set_DF079(const Gps_Ephemeris & gps_eph);

    std::bitset<8> DF080;
    int set_DF080(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF081;
    int set_DF081(const Gps_Ephemeris & gps_eph);

    std::bitset<8> DF082;
    int set_DF082(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF083;
    int set_DF083(const Gps_Ephemeris & gps_eph);

    std::bitset<22> DF084;
    int set_DF084(const Gps_Ephemeris & gps_eph);

    std::bitset<10> DF085;
    int set_DF085(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF086;
    int set_DF086(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF087;
    int set_DF087(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF088;
    int set_DF088(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF089;
    int set_DF089(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF090;
    int set_DF090(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF091;
    int set_DF091(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF092;
    int set_DF092(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF093;
    int set_DF093(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF094;
    int set_DF094(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF095;
    int set_DF095(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF096;
    int set_DF096(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF097;
    int set_DF097(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF098;
    int set_DF098(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF099;
    int set_DF099(const Gps_Ephemeris & gps_eph);

    std::bitset<24> DF100;
    int set_DF100(const Gps_Ephemeris & gps_eph);

    std::bitset<8> DF101;
    int set_DF101(const Gps_Ephemeris & gps_eph);

    std::bitset<6> DF102;
    int set_DF102(const Gps_Ephemeris & gps_eph);

    std::bitset<1> DF103;
    int set_DF103(const Gps_Ephemeris & gps_eph);

    std::bitset<1> DF137;
    int set_DF137(const Gps_Ephemeris & gps_eph);



    std::bitset<1> DF141;
    int set_DF141(const Gps_Ephemeris & gps_eph);

    std::bitset<1> DF142;
    int set_DF142(const Gps_Ephemeris & gps_eph);

    // Contents of Galileo F/NAV Satellite Ephemeris Data, Message Type 1045
    std::bitset<6> DF252;
    int set_DF252(const Galileo_Ephemeris & gal_eph);

    std::bitset<12> DF289;
    int set_DF289(const Galileo_Ephemeris & gal_eph);

    std::bitset<10> DF290;
    int set_DF290(const Galileo_Ephemeris & gal_eph);

    std::bitset<8> DF291;
    int set_DF291(const Galileo_Ephemeris & gal_eph);

    std::bitset<14> DF292;
    int set_DF292(const Galileo_Ephemeris & gal_eph);

    std::bitset<14> DF293;
    int set_DF293(const Galileo_Ephemeris & gal_eph);

    std::bitset<6> DF294;
    int set_DF294(const Galileo_Ephemeris & gal_eph);

    std::bitset<21> DF295;
    int set_DF295(const Galileo_Ephemeris & gal_eph);

    std::bitset<31> DF296;
    int set_DF296(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF297;
    int set_DF297(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF298;
    int set_DF298(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF299;
    int set_DF299(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF300;
    int set_DF300(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF301;
    int set_DF301(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF302;
    int set_DF302(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF303;
    int set_DF303(const Galileo_Ephemeris & gal_eph);

    std::bitset<14> DF304;
    int set_DF304(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF305;
    int set_DF305(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF306;
    int set_DF306(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF307;
    int set_DF307(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF308;
    int set_DF308(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF309;
    int set_DF309(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF310;
    int set_DF310(const Galileo_Ephemeris & gal_eph);

    std::bitset<24> DF311;
    int set_DF311(const Galileo_Ephemeris & gal_eph);

    std::bitset<10> DF312;
    int set_DF312(const Galileo_Ephemeris & gal_eph);

    std::bitset<10> DF313;
    int set_DF313(const Galileo_Ephemeris & gal_eph);

    std::bitset<2> DF314;
    int set_DF314(const Galileo_Ephemeris & gal_eph);

    std::bitset<1> DF315;
    int set_DF315(const Galileo_Ephemeris & gal_eph);

    std::bitset<2> DF364;

    // Content of message header for MSM1, MSM2, MSM3, MSM4, MSM5, MSM6 and MSM7
    std::bitset<1> DF393;
    int set_DF393(bool more_messages); //1 indicates that more MSMs follow for given physical time and reference station ID

    std::bitset<64> DF394;
    int set_DF394(const std::map<int, Gnss_Synchro> & gnss_synchro);

    std::bitset<32> DF395;
    int set_DF395(const std::map<int, Gnss_Synchro> & gnss_synchro);

    //std::bitset<1> DF396; //variable
    std::bitset<3> DF409;
    int set_DF409(unsigned int iods);

    std::bitset<2> DF411;
    std::bitset<2> DF412;
    std::bitset<1> DF417;
    int set_DF417(bool using_divergence_free_smoothing);

    std::bitset<3> DF418;

    // Content of Satellite data for MSM4 and MSM6
    std::vector<std::bitset<8> > DF397; // 8*NSAT
    std::vector<std::bitset<10> > DF398; // 10*NSAT

    // Content of Satellite data for MSM5 and MSM7
    std::vector<std::bitset<14> > DF399; // 14*NSAT
};

#endif
