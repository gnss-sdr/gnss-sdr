/*!
 * \file beidou_cnav2_navigation_message.cc
 * \brief  Implementation of a beidou cnav2 Data message decoder as described in BEIDOU ICD
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">BEIDOU ICD</a>
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

#include "beidou_cnav2_navigation_message.h"
#include "gnss_satellite.h"
#include <boost/crc.hpp>  // for boost::crc_basic, boost::crc_optimal
#include <boost/dynamic_bitset.hpp>
#include <glog/logging.h>
#include <iostream>

typedef boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> Crc_Beidou_Cnav2_type;


void Beidou_Cnav2_Navigation_Message::reset()
{
    // Satellite Identification
    i_satellite_PRN = 0;
    i_alm_satellite_PRN = 0;
    // Ephemeris Flags
    flag_all_ephemeris = false;
    flag_ephemeris_mes_type_10 = false;
    flag_ephemeris_mes_type_11 = false;

    // Almanac Flags
    flag_almanac_mes_type_31 = false;
    flag_almanac_mes_type_33 = false;
    flag_almanac_mes_type_40 = false;

    // UTC and System Clocks Flags
    flag_utc_model_valid = false;        //!< If set, it indicates that the UTC model parameters are filled
    flag_utc_model_mes_type_32 = false;  //!< Clock info send in Type 32
    flag_utc_model_mes_type_33 = false;  //!< Clock info send in Type 33
    flag_utc_model_mes_type_34 = false;  //!< Clock info send in Type 34

    // broadcast orbit 1
    flag_TOW_set = false;
    flag_TOW_new = false;

    flag_crc_test = false;
    i_frame_mes_type = 0;
    i_channel_ID = 0;

    // Clock terms
    d_satClkCorr = 0.0;
    d_dtr = 0.0;
    d_satClkDrift = 0.0;

    // Data update information
    d_previous_tb = 0.0;
    for (uint32_t i = 0; i < BEIDOU_CNAV2_NBR_SATS; i++)
        d_previous_Na[i] = 0.0;

    std::map<int32_t, std::string> satelliteBlock;  //!< Map that stores to which block the PRN belongs http://www.navcen.uscg.gov/?Do=constellationStatus

    auto gnss_sat = Gnss_Satellite();
    std::string _system("BEIDOU");
}


Beidou_Cnav2_Navigation_Message::Beidou_Cnav2_Navigation_Message()
{
    reset();
}


bool Beidou_Cnav2_Navigation_Message::read_navigation_bool(std::bitset<BEIDOU_CNAV2_DATA_BITS> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter)
{
    bool value;

    // bitset::any() (bits.any())

    if (bits[BEIDOU_CNAV2_DATA_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


uint64_t Beidou_Cnav2_Navigation_Message::read_navigation_unsigned(std::bitset<BEIDOU_CNAV2_DATA_BITS> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter)
{
    uint64_t value = 0;
    int32_t num_of_slices = parameter.size();
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;  //shift left
                    if (bits[BEIDOU_CNAV2_DATA_BITS - parameter[i].first - j] == 1)
                        {
                            value |= 1;  // insert the bit
                        }
                }
        }
    return value;
}


int64_t Beidou_Cnav2_Navigation_Message::read_navigation_signed(std::bitset<BEIDOU_CNAV2_DATA_BITS> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter)
{
    int64_t value = 0;
    int64_t sign = 0;
    int32_t num_of_slices = parameter.size();
    // read the MSB and perform the sign extension
    if (bits[BEIDOU_CNAV2_DATA_BITS - parameter[0].first] == 1)
        {
            sign = -1;
        }
    else
        {
            sign = 1;
        }
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 1; j < parameter[i].second; j++)
                {
                    value <<= 1;  //shift left
                    if (bits[BEIDOU_CNAV2_DATA_BITS - parameter[i].first - j] == 1)
                        {
                            value |= 1;  // insert the bit
                        }
                }
        }
    return (sign * value);
}


bool Beidou_Cnav2_Navigation_Message::crc_test(std::bitset<BEIDOU_CNAV2_DATA_BITS> bits, uint32_t crc_decoded)
{
    Crc_Beidou_Cnav2_type Crc_Beidou_Cnav2;

    uint32_t crc_computed;

    boost::dynamic_bitset<uint8_t> frame_bits(std::string(bits.to_string()));
    std::vector<uint8_t> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(), bytes.end());

    // Only include data without 3 lsb from crc
    Crc_Beidou_Cnav2.process_bytes(bytes.data(), BEIDOU_CNAV2_DATA_BYTES - 3);
    crc_computed = Crc_Beidou_Cnav2.checksum();

    if (crc_decoded == crc_computed)
        {
            return true;
        }
    else
        {
            return false;
        }
}


int32_t Beidou_Cnav2_Navigation_Message::frame_decoder(std::string const &frame_string)
{
    // Gets the message data
    std::bitset<BEIDOU_CNAV2_DATA_BITS> frame_bits(frame_string);

    // Gets the crc data for comparison, last 24 bits from 288 bit data frame
    std::bitset<BEIDOU_CNAV2_CRC_BITS> checksum(frame_string.substr(264, 24));

    // Perform data verification and exit code if error in bit sequence
    flag_crc_test = crc_test(frame_bits, checksum.to_ulong());

    if (flag_crc_test == false)
        return 0;

    i_frame_mes_type = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, MesType));

    // Decode all 8 string messages
    switch (i_frame_mes_type)
        {
        case 10:
            //--- It is Type 10 -----------------------------------------------
            cnav2_ephemeris.i_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN));
            //cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(frame_bits, MesType));
            cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(frame_bits, SOW)) * 3;       //[s] effective range 0~604797
            cnav2_ephemeris.i_BDS_week = static_cast<double>(read_navigation_unsigned(frame_bits, WN_10));  //[week] effective range 0~8191
            cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_10));
            cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_10));
            cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_10));
            cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(frame_bits, SISMAI_10));
            cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_B1C_10));
            cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_B1C_10));
            cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_B1C_10));
            cnav2_ephemeris.IODE = static_cast<double>(read_navigation_unsigned(frame_bits, IODE_10));

            // Ephemeris I Start
            cnav2_ephemeris.t_oe = static_cast<double>(read_navigation_unsigned(frame_bits, t_oe_10)) * 300;                              //[s] effective range 0~604500
            cnav2_ephemeris.SatType = static_cast<double>(read_navigation_unsigned(frame_bits, SatType_10));                              //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
            cnav2_ephemeris.dA = static_cast<double>(read_navigation_signed(frame_bits, dA_10)) * TWO_N9;                                 //[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
            cnav2_ephemeris.A_dot = static_cast<double>(read_navigation_signed(frame_bits, A_dot_10)) * TWO_N21;                          //[m/s]
            cnav2_ephemeris.dn_0 = static_cast<double>(read_navigation_signed(frame_bits, dn_0_10)) * BEIDOU_CNAV2_PI * TWO_N44;          //[pi/s]
            cnav2_ephemeris.dn_0_dot = static_cast<double>(read_navigation_signed(frame_bits, dn_0_dot_10)) * BEIDOU_CNAV2_PI * TWO_N57;  //[pi/s^2]
            cnav2_ephemeris.M_0 = static_cast<double>(read_navigation_signed(frame_bits, M_0_10)) * BEIDOU_CNAV2_PI * TWO_N32;            //[pi]
            cnav2_ephemeris.e = static_cast<double>(read_navigation_unsigned(frame_bits, e_10)) * TWO_N34;                                //[dimensionless]
            cnav2_ephemeris.omega = static_cast<double>(read_navigation_signed(frame_bits, omega_10)) * BEIDOU_CNAV2_PI * TWO_N32;        //[pi]
            // Ephemeris I End

            // Set flags relative to time and message
            flag_TOW_set = true;
            flag_TOW_10 = true;
            flag_ephemeris_mes_type_10 = true;

            break;

        case 11:
            //--- It is Type 11 -----------------------------------------------
            cnav2_ephemeris.i_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN));
            //cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(frame_bits, MesType));
            cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(frame_bits, SOW)) * 3;  //[s] effective range 0~604797
            cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(frame_bits, HS_11));
            cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_11));
            cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_11));
            cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_11));
            cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(frame_bits, SISMAI_11));
            cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_B1C_11));
            cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_B1C_11));
            cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_B1C_11));

            // Ephemeris II Start
            cnav2_ephemeris.Omega_0 = static_cast<double>(read_navigation_signed(frame_bits, Omega_0_11)) * BEIDOU_CNAV2_PI * TWO_N32;      //[pi]
            cnav2_ephemeris.i_0 = static_cast<double>(read_navigation_signed(frame_bits, i_0_11)) * BEIDOU_CNAV2_PI * TWO_N32;              //[pi]
            cnav2_ephemeris.Omega_dot = static_cast<double>(read_navigation_signed(frame_bits, Omega_dot_11)) * BEIDOU_CNAV2_PI * TWO_N44;  //[pi/s]
            cnav2_ephemeris.i_0_dot = static_cast<double>(read_navigation_signed(frame_bits, i_0_dot_11)) * BEIDOU_CNAV2_PI * TWO_N44;      //[pi/s]
            cnav2_ephemeris.C_IS = static_cast<double>(read_navigation_signed(frame_bits, C_IS_11)) * TWO_N30;                              //[rad]
            cnav2_ephemeris.C_IC = static_cast<double>(read_navigation_signed(frame_bits, C_IC_11)) * TWO_N30;                              //[rad]
            cnav2_ephemeris.C_RS = static_cast<double>(read_navigation_signed(frame_bits, C_RS_11)) * TWO_N8;                               //[m]
            cnav2_ephemeris.C_RC = static_cast<double>(read_navigation_signed(frame_bits, C_RC_11)) * TWO_N8;                               //[m]
            cnav2_ephemeris.C_US = static_cast<double>(read_navigation_signed(frame_bits, C_US_11)) * TWO_N30;                              //[rad]
            cnav2_ephemeris.C_UC = static_cast<double>(read_navigation_signed(frame_bits, C_UC_11)) * TWO_N30;                              //[rad]
            // Ephemeris II End

            // Set flags relative to time and message
            flag_TOW_set = true;
            flag_TOW_11 = true;
            flag_ephemeris_mes_type_11 = true;

            break;

        case 30:
            // --- It is Type 30 ----------------------------------------------
            cnav2_ephemeris.i_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN));
            //cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(frame_bits, MesType));
            cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(frame_bits, SOW)) * 3;  //[s] effective range 0~604797
            cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(frame_bits, HS_30));
            cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_30));
            cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_30));
            cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_30));
            cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(frame_bits, SISMAI_30));
            cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_B1C_30));
            cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_B1C_30));
            cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_B1C_30));

            // Clock Correction Parameters (69 bits)
            cnav2_ephemeris.t_oc = static_cast<double>(read_navigation_unsigned(frame_bits, t_oc_30)) * 300;  //[s]
            cnav2_ephemeris.a_0 = static_cast<double>(read_navigation_signed(frame_bits, a_0_30)) * TWO_N34;  //[s]
            cnav2_ephemeris.a_1 = static_cast<double>(read_navigation_signed(frame_bits, a_1_30)) * TWO_N50;  //[s/s]
            cnav2_ephemeris.a_2 = static_cast<double>(read_navigation_signed(frame_bits, a_2_30)) * TWO_N66;  //[s/s^2]
            // Clock Correction Parameters End

            cnav2_ephemeris.IODC = static_cast<double>(read_navigation_unsigned(frame_bits, IODC_30));
            cnav2_ephemeris.T_GDB2ap = static_cast<double>(read_navigation_signed(frame_bits, T_GDB2ap_30)) * TWO_N34;    //[s]
            cnav2_ephemeris.ISC_B2ad = static_cast<double>(read_navigation_unsigned(frame_bits, ISC_B2ad_30)) * TWO_N34;  //[s]

            // Ionospheric Delay Correction Model Parameters (74 bits)
            cnav2_iono.alpha1 = static_cast<double>(read_navigation_unsigned(frame_bits, alpha_1_30)) * TWO_N3;         //[TECu]
            cnav2_iono.alpha2 = static_cast<double>(read_navigation_signed(frame_bits, alpha_2_30)) * TWO_N3;           //[TECu]
            cnav2_iono.alpha3 = static_cast<double>(read_navigation_unsigned(frame_bits, alpha_3_30)) * TWO_N3;         //[TECu]
            cnav2_iono.alpha4 = static_cast<double>(read_navigation_unsigned(frame_bits, alpha_4_30)) * TWO_N3;         //[TECu]
            cnav2_iono.alpha5 = static_cast<double>(read_navigation_unsigned(frame_bits, alpha_5_30)) * TWO_N3 * (-1);  //[TECu]
            cnav2_iono.alpha6 = static_cast<double>(read_navigation_signed(frame_bits, alpha_6_30)) * TWO_N3;           //[TECu]
            cnav2_iono.alpha7 = static_cast<double>(read_navigation_signed(frame_bits, alpha_7_30)) * TWO_N3;           //[TECu]
            cnav2_iono.alpha8 = static_cast<double>(read_navigation_signed(frame_bits, alpha_8_30)) * TWO_N3;           //[TECu]
            cnav2_iono.alpha9 = static_cast<double>(read_navigation_signed(frame_bits, alpha_9_30)) * TWO_N3;           //[TECu]
            // Ionospheric Delay Correction Model Parameters End

            cnav2_ephemeris.T_GDB1Cp = static_cast<double>(read_navigation_signed(frame_bits, T_GDB1Cp_30)) * TWO_N34;  //[s]
            cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(frame_bits, Rev_30));

            // Set flags relative to time and message
            flag_TOW_set = true;
            flag_TOW_30 = true;
            flag_ephemeris_mes_type_30 = true;
            flag_iono_mes_type_30 = true;
            break;

        case 31:
            // --- It is Type 31 ----------------------------------------------
            cnav2_ephemeris.i_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN));
            //cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(frame_bits, MesType));
            cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(frame_bits, SOW)) * 3;  //[s] effective range 0~604797
            cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(frame_bits, HS_31));
            cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_31));
            cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_31));
            cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_31));
            cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(frame_bits, SISMAI_31));
            cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_B1C_31));
            cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_B1C_31));
            cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_B1C_31));

            // Clock Correction Parameters (69 bits)
            cnav2_ephemeris.t_oc = static_cast<double>(read_navigation_unsigned(frame_bits, t_oc_31)) * 300;  //[s]
            cnav2_ephemeris.a_0 = static_cast<double>(read_navigation_signed(frame_bits, a_0_31)) * TWO_N34;  //[s]
            cnav2_ephemeris.a_1 = static_cast<double>(read_navigation_signed(frame_bits, a_1_31)) * TWO_N50;  //[s/s]
            cnav2_ephemeris.a_2 = static_cast<double>(read_navigation_signed(frame_bits, a_2_31)) * TWO_N66;  //[s/s^2]

            // Clock Correction Parameters End
            cnav2_ephemeris.IODC = static_cast<double>(read_navigation_unsigned(frame_bits, IODC_31));

            // Reduced Almanac Parameters Sat 1(38 bits)
            i_alm_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN_a1_31));  //[dimensionless] effective range 1~63
            cnav2_almanac[i_alm_satellite_PRN - 1].i_satellite_PRN = i_alm_satellite_PRN;
            cnav2_almanac[i_alm_satellite_PRN - 1].i_BDS_week = static_cast<int32_t>(read_navigation_unsigned(frame_bits, WN_a_31));                           //[week] effective range 0~8191
            cnav2_almanac[i_alm_satellite_PRN - 1].t_oa = static_cast<double>(read_navigation_unsigned(frame_bits, t_oa_31)) * TWO_P12;                        //[s] effective range 0~602112
            cnav2_almanac[i_alm_satellite_PRN - 1].SatType = static_cast<double>(read_navigation_unsigned(frame_bits, SatType1_31));                           //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
            cnav2_almanac[i_alm_satellite_PRN - 1].delta_A = static_cast<double>(read_navigation_signed(frame_bits, delta_A1_31)) * TWO_P9;                    //[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
            cnav2_almanac[i_alm_satellite_PRN - 1].Omega_0 = static_cast<double>(read_navigation_signed(frame_bits, Omega_01_31)) * BEIDOU_CNAV2_PI * TWO_N6;  //[pi]
            cnav2_almanac[i_alm_satellite_PRN - 1].Phi_0 = static_cast<double>(read_navigation_signed(frame_bits, Phi_01_31)) * BEIDOU_CNAV2_PI * TWO_N6;      //[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
            cnav2_almanac[i_alm_satellite_PRN - 1].Health = static_cast<double>(read_navigation_unsigned(frame_bits, Health1_31));                             //[dimensionless]
            // Reduced Almanac Parameters End

            // Reduced Almanac Parameters Sat 2(38 bits)
            i_alm_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN_a2_31));  //[dimensionless] effective range 1~63
            cnav2_almanac[i_alm_satellite_PRN - 1].i_satellite_PRN = i_alm_satellite_PRN;
            cnav2_almanac[i_alm_satellite_PRN - 1].i_BDS_week = static_cast<int32_t>(read_navigation_unsigned(frame_bits, WN_a_31));                           //[week] effective range 0~8191
            cnav2_almanac[i_alm_satellite_PRN - 1].t_oa = static_cast<double>(read_navigation_unsigned(frame_bits, t_oa_31)) * TWO_P12;                        //[s] effective range 0~602112
            cnav2_almanac[i_alm_satellite_PRN - 1].SatType = static_cast<double>(read_navigation_unsigned(frame_bits, SatType2_31));                           //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
            cnav2_almanac[i_alm_satellite_PRN - 1].delta_A = static_cast<double>(read_navigation_signed(frame_bits, delta_A2_31)) * TWO_P9;                    //[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
            cnav2_almanac[i_alm_satellite_PRN - 1].Omega_0 = static_cast<double>(read_navigation_signed(frame_bits, Omega_02_31)) * BEIDOU_CNAV2_PI * TWO_N6;  //[pi]
            cnav2_almanac[i_alm_satellite_PRN - 1].Phi_0 = static_cast<double>(read_navigation_signed(frame_bits, Phi_02_31)) * BEIDOU_CNAV2_PI * TWO_N6;      //[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
            cnav2_almanac[i_alm_satellite_PRN - 1].Health = static_cast<double>(read_navigation_unsigned(frame_bits, Health2_31));                             //[dimensionless]
            // Reduced Almanac Parameters End

            // Reduced Almanac Parameters Sat 3(38 bits)
            i_alm_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN_a3_31));  //[dimensionless] effective range 1~63
            cnav2_almanac[i_alm_satellite_PRN - 1].i_satellite_PRN = i_alm_satellite_PRN;
            cnav2_almanac[i_alm_satellite_PRN - 1].i_BDS_week = static_cast<int32_t>(read_navigation_unsigned(frame_bits, WN_a_31));                           //[week] effective range 0~8191
            cnav2_almanac[i_alm_satellite_PRN - 1].t_oa = static_cast<double>(read_navigation_unsigned(frame_bits, t_oa_31)) * TWO_P12;                        //[s] effective range 0~602112
            cnav2_almanac[i_alm_satellite_PRN - 1].SatType = static_cast<double>(read_navigation_unsigned(frame_bits, SatType3_31));                           //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
            cnav2_almanac[i_alm_satellite_PRN - 1].delta_A = static_cast<double>(read_navigation_signed(frame_bits, delta_A3_31)) * TWO_P9;                    //[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
            cnav2_almanac[i_alm_satellite_PRN - 1].Omega_0 = static_cast<double>(read_navigation_signed(frame_bits, Omega_03_31)) * BEIDOU_CNAV2_PI * TWO_N6;  //[pi]
            cnav2_almanac[i_alm_satellite_PRN - 1].Phi_0 = static_cast<double>(read_navigation_signed(frame_bits, Phi_03_31)) * BEIDOU_CNAV2_PI * TWO_N6;      //[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
            cnav2_almanac[i_alm_satellite_PRN - 1].Health = static_cast<double>(read_navigation_unsigned(frame_bits, Health3_31));                             //[dimensionless]

            // Reduced Almanac Parameters End
            cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(frame_bits, Rev_31));

            // Set flags relative to time and message
            flag_TOW_set = true;
            flag_TOW_31 = true;
            flag_ephemeris_mes_type_31 = true;
            flag_almanac_mes_type_31 = true;
            break;

        case 32:
            // --- It is Type 32 ----------------------------------------------
            cnav2_ephemeris.i_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN));
            //cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(frame_bits, MesType));
            cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(frame_bits, SOW)) * 3;  //[s] effective range 0~604797
            cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(frame_bits, HS_32));
            cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_32));
            cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_32));
            cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_32));
            cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(frame_bits, SISMAI_32));
            cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_B1C_32));
            cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_B1C_32));
            cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_B1C_32));

            // Clock Correction Parameters (69 bits)
            cnav2_ephemeris.t_oc = static_cast<double>(read_navigation_unsigned(frame_bits, t_oc_32)) * 300;  //[s]
            cnav2_ephemeris.a_0 = static_cast<double>(read_navigation_signed(frame_bits, a_0_32)) * TWO_N34;  //[s]
            cnav2_ephemeris.a_1 = static_cast<double>(read_navigation_signed(frame_bits, a_1_32)) * TWO_N50;  //[s/s]
            cnav2_ephemeris.a_2 = static_cast<double>(read_navigation_signed(frame_bits, a_2_32)) * TWO_N66;  //[s/s^2]

            // Clock Correction Parameters End
            cnav2_ephemeris.IODC = static_cast<double>(read_navigation_unsigned(frame_bits, IODC_32));

            // EOP Parameters (138 bits)
            cnav2_ephemeris.t_EOP = static_cast<double>(read_navigation_unsigned(frame_bits, t_EOP_32)) * TWO_P4;       //[s] effective range 0~604784
            cnav2_ephemeris.PM_X = static_cast<double>(read_navigation_signed(frame_bits, PM_X_32)) * TWO_N20;          //[arc-seconds]
            cnav2_ephemeris.PM_X_dot = static_cast<double>(read_navigation_signed(frame_bits, PM_X_dot_32)) * TWO_N21;  //[arc-seconds/day]
            cnav2_ephemeris.PM_Y = static_cast<double>(read_navigation_signed(frame_bits, PM_Y_32)) * TWO_N20;          //[arc-seconds]
            cnav2_ephemeris.PM_Y_dot = static_cast<double>(read_navigation_signed(frame_bits, PM_Y_dot_32)) * TWO_N21;  //[arc-seconds/day]
            cnav2_ephemeris.dUT1 = static_cast<double>(read_navigation_signed(frame_bits, dUT1_32)) * TWO_N24;          //[s]
            cnav2_ephemeris.dUT1_dot = static_cast<double>(read_navigation_signed(frame_bits, dUT1_dot_32)) * TWO_N25;  //[s/day]

            // EOP Parameters End
            cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(frame_bits, Rev_32));

            // Set flags relative to time and message
            flag_TOW_set = true;
            flag_TOW_32 = true;
            flag_ephemeris_mes_type_32 = true;
            flag_utc_model_mes_type_32 = true;
            break;

        case 33:
            // --- It is Type 33 ----------------------------------------------
            cnav2_ephemeris.i_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN));
            //cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(frame_bits, MesType));
            cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(frame_bits, SOW)) * 3;  //[s] effective range 0~604797
            cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(frame_bits, HS_33));
            cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_33));
            cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_33));
            cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_33));
            cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(frame_bits, SISMAI_33));
            cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_B1C_33));
            cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_B1C_33));
            cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_B1C_33));

            // Clock Correction Parameters (69 bits)
            cnav2_ephemeris.t_oc = static_cast<double>(read_navigation_unsigned(frame_bits, t_oc_33)) * 300;  //[s]
            cnav2_ephemeris.a_0 = static_cast<double>(read_navigation_signed(frame_bits, a_0_33)) * TWO_N34;  //[s]
            cnav2_ephemeris.a_1 = static_cast<double>(read_navigation_signed(frame_bits, a_1_33)) * TWO_N50;  //[s/s]
            cnav2_ephemeris.a_2 = static_cast<double>(read_navigation_signed(frame_bits, a_2_33)) * TWO_N66;  //[s/s^2]
                                                                                                              // Clock Correction Parameters End

            // BGTO Parameters (68 bits)
            cnav2_utc_model.GNSS_ID = static_cast<double>(read_navigation_unsigned(frame_bits, GNSS_ID_33));            //[dimensionless]
            cnav2_utc_model.WN_0BGTO = static_cast<double>(read_navigation_unsigned(frame_bits, WN_0BGTO_33));          //[week]
            cnav2_utc_model.t_0BGTO = static_cast<double>(read_navigation_unsigned(frame_bits, t_0BGTO_33)) * TWO_P4;   //[s] effective range 0~604784
            cnav2_utc_model.A_0BGTO = static_cast<double>(read_navigation_unsigned(frame_bits, A_0BGTO_33)) * TWO_N35;  //[s]
            cnav2_utc_model.A_1BGTO = static_cast<double>(read_navigation_unsigned(frame_bits, A_1BGTO_33)) * TWO_N51;  //[s/s]
            cnav2_utc_model.A_2BGTO = static_cast<double>(read_navigation_unsigned(frame_bits, A_2BGTO_33)) * TWO_N68;  //[s/s^2]
                                                                                                                        // BGTO Parameters End

            // Reduced Almanac Parameters (38 bits)
            i_alm_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN_ALM_33));  //[dimensionless] effective range 1~63
            cnav2_almanac[i_alm_satellite_PRN - 1].i_satellite_PRN = i_alm_satellite_PRN;
            cnav2_almanac[i_alm_satellite_PRN - 1].SatType = static_cast<double>(read_navigation_unsigned(frame_bits, SatType_33));                           //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
            cnav2_almanac[i_alm_satellite_PRN - 1].delta_A = static_cast<double>(read_navigation_signed(frame_bits, delta_A_33)) * TWO_P9;                    //[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
            cnav2_almanac[i_alm_satellite_PRN - 1].Omega_0 = static_cast<double>(read_navigation_signed(frame_bits, Omega_0_33)) * BEIDOU_CNAV2_PI * TWO_N6;  //[pi]
            cnav2_almanac[i_alm_satellite_PRN - 1].Phi_0 = static_cast<double>(read_navigation_signed(frame_bits, Phi_0_33)) * BEIDOU_CNAV2_PI * TWO_N6;      //[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
            cnav2_almanac[i_alm_satellite_PRN - 1].Health = static_cast<double>(read_navigation_unsigned(frame_bits, Health_33));                             //[dimensionless]
                                                                                                                                                              // Reduced Almanac Parameters End

            cnav2_ephemeris.IODC = static_cast<double>(read_navigation_unsigned(frame_bits, IODC_33));
            cnav2_almanac[i_alm_satellite_PRN - 1].i_BDS_week = static_cast<int32_t>(read_navigation_unsigned(frame_bits, WN_ALM_33));   //[week] effective range 0~8191
            cnav2_almanac[i_alm_satellite_PRN - 1].t_oa = static_cast<double>(read_navigation_unsigned(frame_bits, t_oa_33)) * TWO_P12;  //[s] effective range 0~602112
            cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(frame_bits, Rev_33));

            // Set flags relative to time and message
            flag_TOW_set = true;
            flag_TOW_33 = true;
            flag_almanac_mes_type_33 = true;
            flag_utc_model_mes_type_33 = true;
            break;

        case 34:
            // --- It is Type 34 ----------------------------------------------

            cnav2_ephemeris.i_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN));
            //cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(frame_bits, MesType));
            cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(frame_bits, SOW)) * 3;  //[s] effective range 0~604797
            cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(frame_bits, HS_34));
            cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_34));
            cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_34));
            cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_34));
            cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(frame_bits, SISMAI_34));
            cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_B1C_34));
            cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_B1C_34));
            cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_B1C_34));

            // SISAI_OC (22 bits)
            cnav2_ephemeris.t_op = static_cast<double>(read_navigation_unsigned(frame_bits, t_op_34));
            cnav2_ephemeris.SISAI_ocb = static_cast<double>(read_navigation_unsigned(frame_bits, SISAI_ocb_34));
            cnav2_ephemeris.SISAI_oc1 = static_cast<double>(read_navigation_unsigned(frame_bits, SISAI_oc1_34));
            cnav2_ephemeris.SISAI_oc2 = static_cast<double>(read_navigation_unsigned(frame_bits, SISAI_oc2_34));
            // SISAI_OC End

            // Clock Correction Parameters (69 bits)
            cnav2_ephemeris.t_oc = static_cast<double>(read_navigation_unsigned(frame_bits, t_oc_34)) * 300;  //[s]
            cnav2_ephemeris.a_0 = static_cast<double>(read_navigation_signed(frame_bits, a_0_34)) * TWO_N34;  //[s]
            cnav2_ephemeris.a_1 = static_cast<double>(read_navigation_signed(frame_bits, a_1_34)) * TWO_N50;  //[s/s]
            cnav2_ephemeris.a_2 = static_cast<double>(read_navigation_signed(frame_bits, a_2_34)) * TWO_N66;  //[s/s^2]
                                                                                                              // Clock Correction Parameters End

            cnav2_ephemeris.IODC = static_cast<double>(read_navigation_unsigned(frame_bits, IODC_34));

            // BDT-UTC Time Offset Parameters (97 bits)
            cnav2_utc_model.A_0UTC = static_cast<double>(read_navigation_signed(frame_bits, A_0UTC_34)) * TWO_N35;  //[s]
            cnav2_utc_model.A_1UTC = static_cast<double>(read_navigation_signed(frame_bits, A_1UTC_34)) * TWO_N51;  //[s/s]
            cnav2_utc_model.A_2UTC = static_cast<double>(read_navigation_signed(frame_bits, A_2UTC_34)) * TWO_N68;  //[s/s^2]
            cnav2_utc_model.dt_LS = static_cast<double>(read_navigation_signed(frame_bits, dt_LS_34));              //[s]
            cnav2_utc_model.t_ot = static_cast<double>(read_navigation_unsigned(frame_bits, t_ot_34)) * TWO_P4;     //[s] effective range 0~604784
            cnav2_utc_model.WN_ot = static_cast<double>(read_navigation_unsigned(frame_bits, WN_ot_34));            //[week]
            cnav2_utc_model.WN_LSF = static_cast<double>(read_navigation_unsigned(frame_bits, WN_LSF_34));          //[week]
            cnav2_utc_model.DN = static_cast<double>(read_navigation_unsigned(frame_bits, DN_34));                  //[day] effective range 0~6
            cnav2_utc_model.dt_LSF = static_cast<double>(read_navigation_signed(frame_bits, dt_LSF_34));            //[s]
            // BDT-UTC Time Offset Parameters End

            cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(frame_bits, Rev_34));

            // Set flags relative to time and message
            flag_TOW_set = true;
            flag_TOW_34 = true;
            flag_ephemeris_mes_type_34 = true;
            flag_utc_model_mes_type_34 = true;
            break;

        case 40:
            // --- It is Type 40 ----------------------------------------------

            cnav2_ephemeris.i_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN));
            //cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(frame_bits, MesType));
            cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(frame_bits, SOW)) * 3;  //[s] effective range 0~604797
            cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(frame_bits, HS_40));
            cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_40));
            cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_40));
            cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_40));
            cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(frame_bits, SISMAI_40));
            cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, DIF_B1C_40));
            cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, SIF_B1C_40));
            cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(frame_bits, AIF_B1C_40));
            cnav2_ephemeris.SISAI_OE = static_cast<double>(read_navigation_unsigned(frame_bits, SISAI_OE_40));

            // SISAI_OC (22 bits)
            cnav2_ephemeris.t_op = static_cast<double>(read_navigation_unsigned(frame_bits, t_op_40));
            cnav2_ephemeris.SISAI_ocb = static_cast<double>(read_navigation_unsigned(frame_bits, SISAI_ocb_40));
            cnav2_ephemeris.SISAI_oc1 = static_cast<double>(read_navigation_unsigned(frame_bits, SISAI_oc1_40));
            cnav2_ephemeris.SISAI_oc2 = static_cast<double>(read_navigation_unsigned(frame_bits, SISAI_oc2_40));
            // SISAI_OC End

            // Midi Almanac Parameters (156 bits)
            i_alm_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(frame_bits, PRN_a_40));  //[dimensionless] effective range 1~63
            cnav2_almanac[i_alm_satellite_PRN - 1].i_satellite_PRN = i_alm_satellite_PRN;
            cnav2_almanac[i_alm_satellite_PRN - 1].SatType = static_cast<double>(read_navigation_unsigned(frame_bits, SatType_40));                                //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
            cnav2_almanac[i_alm_satellite_PRN - 1].i_BDS_week = static_cast<int32_t>(read_navigation_unsigned(frame_bits, WN_a_40));                               //[week] effective range 0~8191
            cnav2_almanac[i_alm_satellite_PRN - 1].t_oa = static_cast<double>(read_navigation_unsigned(frame_bits, t_oa_40)) * TWO_P12;                            //[s] effective range 0~602112
            cnav2_almanac[i_alm_satellite_PRN - 1].e = static_cast<double>(read_navigation_unsigned(frame_bits, e_40)) * TWO_N16;                                  //[dimensionless]
            cnav2_almanac[i_alm_satellite_PRN - 1].delta_i = static_cast<double>(read_navigation_signed(frame_bits, delta_i_40)) * BEIDOU_CNAV2_PI * TWO_N14;      //[pi]
            cnav2_almanac[i_alm_satellite_PRN - 1].sqrt_A = static_cast<double>(read_navigation_unsigned(frame_bits, sqrt_A_40)) * TWO_N4;                         //[m^0.5]
            cnav2_almanac[i_alm_satellite_PRN - 1].Omega_0 = static_cast<double>(read_navigation_signed(frame_bits, Omega_0_40)) * BEIDOU_CNAV2_PI * TWO_N15;      //[pi]
            cnav2_almanac[i_alm_satellite_PRN - 1].Omega_dot = static_cast<double>(read_navigation_signed(frame_bits, Omega_dot_40)) * BEIDOU_CNAV2_PI * TWO_N33;  //[pi/s]
            cnav2_almanac[i_alm_satellite_PRN - 1].omega = static_cast<double>(read_navigation_signed(frame_bits, omega_40)) * BEIDOU_CNAV2_PI * TWO_N15;          //[pi]
            cnav2_almanac[i_alm_satellite_PRN - 1].M_0 = static_cast<double>(read_navigation_signed(frame_bits, M_0_40)) * BEIDOU_CNAV2_PI * TWO_N15;              //[pi]
            cnav2_almanac[i_alm_satellite_PRN - 1].a_f0 = static_cast<double>(read_navigation_signed(frame_bits, a_f0_40)) * TWO_N20;                              //[s]
            cnav2_almanac[i_alm_satellite_PRN - 1].a_f1 = static_cast<double>(read_navigation_signed(frame_bits, a_f1_40)) * TWO_N37;                              //[s/s]
            cnav2_almanac[i_alm_satellite_PRN - 1].Health = static_cast<double>(read_navigation_unsigned(frame_bits, Health_40));                                  //[dimensionless] 8th(MSB):Satellite, 7th:B1C, 6th:B2a, 5th~1st:reserve, 0:normal/health, 1:abnormal
            // Midi Almanac Parameters End

            cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(frame_bits, Rev_40));

            // Set flags relative to time and message
            flag_TOW_set = true;
            flag_TOW_40 = true;
            flag_almanac_mes_type_40 = true;
            flag_ephemeris_mes_type_40 = true;
            break;


        default:
            LOG(INFO) << "BEIDOU CNAV2: Invalid String ID of received. Received " << i_frame_mes_type
                      << ", but acceptable values are 10, 11, 30, 31, 32, 33, 34, and 40";

            break;
        }  // switch string ID

    return i_frame_mes_type;
}


Beidou_Cnav2_Ephemeris Beidou_Cnav2_Navigation_Message::get_ephemeris()
{
    return cnav2_ephemeris;
}


Beidou_Cnav2_Utc_Model Beidou_Cnav2_Navigation_Message::get_utc_model()
{
    return cnav2_utc_model;
}


Beidou_Cnav2_Iono Beidou_Cnav2_Navigation_Message::get_iono()
{
    return cnav2_iono;
}


Beidou_Cnav2_Almanac Beidou_Cnav2_Navigation_Message::get_almanac(uint32_t satellite_slot_number)
{
    return cnav2_almanac[satellite_slot_number - 1];
}


bool Beidou_Cnav2_Navigation_Message::have_new_ephemeris()  //Check if we have a new ephemeris stored in the galileo navigation class
{
    bool new_eph = false;
    // We need to make sure we have received the ephemeris info plus the time info
    if ((flag_ephemeris_mes_type_10 == true) and (flag_ephemeris_mes_type_11 == true))
        {
            if (d_previous_tb != cnav2_ephemeris.IODE)
                {
                    flag_ephemeris_mes_type_10 = false;  // clear the flag
                    flag_ephemeris_mes_type_11 = false;  // clear the flag
                    flag_all_ephemeris = true;
                    // Update the time of ephemeris information
                    d_previous_tb = cnav2_ephemeris.IODE;
                    DLOG(INFO) << "Beidou Cnav2 Ephemeris (1, 2) have been received and belong to the same batch" << std::endl;
                    new_eph = true;
                }
        }

    return new_eph;
}


bool Beidou_Cnav2_Navigation_Message::have_new_utc_model()  // Check if we have a new utc data set stored in the beidou navigation class
{
    if ((flag_utc_model_mes_type_32 == true) and (flag_utc_model_mes_type_33 == true) and (flag_utc_model_mes_type_34 == true))
        {
            flag_utc_model_mes_type_32 = false;  // clear the flag
            flag_utc_model_mes_type_33 = false;  // clear the flag
            flag_utc_model_mes_type_34 = false;  // clear the flag
            return true;
        }
    else
        return false;
}


bool Beidou_Cnav2_Navigation_Message::have_new_iono()  // Check if we have a new utc data set stored in the beidou navigation class
{
    if (flag_iono_mes_type_30 == true)
        {
            flag_iono_mes_type_30 = false;  // clear the flag
            return true;
        }
    else
        return false;
}

bool Beidou_Cnav2_Navigation_Message::have_new_almanac()  //Check if we have a new almanac data set stored in the beidou navigation class
{
    if (flag_almanac_mes_type_31 == true)
        {
            if (d_previous_Na[i_alm_satellite_PRN] != cnav2_almanac[i_alm_satellite_PRN - 1].t_oa)
                {
                    //All almanac have been received for this satellite
                    flag_almanac_mes_type_31 = false;
                    return true;
                }
        }
    if (flag_almanac_mes_type_33 == true)
        {
            if (d_previous_Na[i_alm_satellite_PRN] != cnav2_almanac[i_alm_satellite_PRN - 1].t_oa)
                {
                    //All almanac have been received for this satellite
                    flag_almanac_mes_type_33 = false;
                    return true;
                }
        }
    if (flag_almanac_mes_type_40 == true)
        {
            if (d_previous_Na[i_alm_satellite_PRN] != cnav2_almanac[i_alm_satellite_PRN - 1].t_oa)
                {
                    //All almanac have been received for this satellite
                    flag_almanac_mes_type_40 = false;
                    return true;
                }
        }
    return false;
}