/*!
 * \file gps_cnav_navigation_message.cc
 * \brief Implementation of a GPS CNAV Data message decoder as described in IS-GPS-200H
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200H.pdf Appendix III
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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

#include "gps_cnav_navigation_message.h"
#include "gnss_satellite.h"


void Gps_CNAV_Navigation_Message::reset()
{
    b_flag_ephemeris_1 = false;
    b_flag_ephemeris_2 = false;
    b_flag_iono_valid = false;
    b_flag_utc_valid = false;

    // satellite positions
    d_satpos_X = 0;
    d_satpos_Y = 0;
    d_satpos_Z = 0;

    // info
    i_channel_ID = 0;
    i_satellite_PRN = 0;

    // Satellite velocity
    d_satvel_X = 0;
    d_satvel_Y = 0;
    d_satvel_Z = 0;

    d_TOW = 0.0;
}


Gps_CNAV_Navigation_Message::Gps_CNAV_Navigation_Message()
{
    reset();
    Gnss_Satellite gnss_satellite_ = Gnss_Satellite();
    for(unsigned int prn_ = 1; prn_ < 33; prn_++)
        {
            satelliteBlock[prn_] = gnss_satellite_.what_block("GPS", prn_);
        }
    b_flag_iono_valid = false;
    b_flag_utc_valid = false;
}



bool Gps_CNAV_Navigation_Message::read_navigation_bool(std::bitset<GPS_CNAV_DATA_PAGE_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    bool value;

    if (bits[GPS_CNAV_DATA_PAGE_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


unsigned long int Gps_CNAV_Navigation_Message::read_navigation_unsigned(std::bitset<GPS_CNAV_DATA_PAGE_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    unsigned long int value = 0;
    int num_of_slices = parameter.size();
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1; //shift left
                    if (bits[GPS_CNAV_DATA_PAGE_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1; // insert the bit
                        }
                }
        }
    return value;
}


signed long int Gps_CNAV_Navigation_Message::read_navigation_signed(std::bitset<GPS_CNAV_DATA_PAGE_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    signed long int value = 0;
    int num_of_slices = parameter.size();
    // Discriminate between 64 bits and 32 bits compiler
    int long_int_size_bytes = sizeof(signed long int);
    if (long_int_size_bytes == 8) // if a long int takes 8 bytes, we are in a 64 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GPS_CNAV_DATA_PAGE_BITS - parameter[0].first] == 1)
                {
                    value ^= 0xFFFFFFFFFFFFFFFF; //64 bits variable
                }
            else
                {
                    value &= 0;
                }

            for (int i = 0; i < num_of_slices; i++)
                {
                    for (int j = 0; j < parameter[i].second; j++)
                        {
                            value <<= 1; //shift left
                            value &= 0xFFFFFFFFFFFFFFFE; //reset the corresponding bit (for the 64 bits variable)
                            if (bits[GPS_CNAV_DATA_PAGE_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1; // insert the bit
                                }
                        }
                }
        }
    else  // we assume we are in a 32 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GPS_CNAV_DATA_PAGE_BITS - parameter[0].first] == 1)
                {
                    value ^= 0xFFFFFFFF;
                }
            else
                {
                    value &= 0;
                }

            for (int i = 0; i < num_of_slices; i++)
                {
                    for (int j = 0; j < parameter[i].second; j++)
                        {
                            value <<= 1; //shift left
                            value &= 0xFFFFFFFE; //reset the corresponding bit
                            if (bits[GPS_CNAV_DATA_PAGE_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1; // insert the bit
                                }
                        }
                }
        }
    return value;
}


void Gps_CNAV_Navigation_Message::decode_page(std::bitset<GPS_CNAV_DATA_PAGE_BITS> data_bits)
{
    int PRN;
    int page_type;

    bool alert_flag;

    // common to all messages
    PRN = static_cast<int>(read_navigation_unsigned(data_bits, CNAV_PRN));
    ephemeris_record.i_satellite_PRN = PRN;

    d_TOW = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_TOW));
    d_TOW = d_TOW * CNAV_TOW_LSB;
    ephemeris_record.d_TOW = d_TOW;

    alert_flag = static_cast<bool>(read_navigation_bool(data_bits, CNAV_ALERT_FLAG));
    ephemeris_record.b_alert_flag = alert_flag;

    page_type = static_cast<int>(read_navigation_unsigned(data_bits, CNAV_MSG_TYPE));

    switch(page_type)
    {
    case 10: // Ephemeris 1/2
        ephemeris_record.i_GPS_week = static_cast<int>(read_navigation_unsigned(data_bits, CNAV_WN));
        ephemeris_record.i_signal_health = static_cast<int>(read_navigation_unsigned(data_bits, CNAV_HEALTH));
        ephemeris_record.d_Top = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_TOP1));
        ephemeris_record.d_Top = ephemeris_record.d_Top * CNAV_TOP1_LSB;
        ephemeris_record.d_URA0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_URA));
        ephemeris_record.d_Toe1 = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_TOE1));
        ephemeris_record.d_Toe1 = ephemeris_record.d_Toe1 * CNAV_TOE1_LSB;
        ephemeris_record.d_DELTA_A = static_cast<double>(read_navigation_signed(data_bits, CNAV_DELTA_A));
        ephemeris_record.d_DELTA_A = ephemeris_record.d_DELTA_A * CNAV_DELTA_A_LSB;
        ephemeris_record.d_A_DOT = static_cast<double>(read_navigation_signed(data_bits, CNAV_A_DOT));
        ephemeris_record.d_A_DOT = ephemeris_record.d_A_DOT * CNAV_A_DOT_LSB;
        ephemeris_record.d_Delta_n = static_cast<double>(read_navigation_signed(data_bits, CNAV_DELTA_N0));
        ephemeris_record.d_Delta_n = ephemeris_record.d_Delta_n * CNAV_DELTA_N0_LSB;
        ephemeris_record.d_DELTA_DOT_N = static_cast<double>(read_navigation_signed(data_bits, CNAV_DELTA_N0_DOT));
        ephemeris_record.d_DELTA_DOT_N = ephemeris_record.d_DELTA_DOT_N * CNAV_DELTA_N0_DOT_LSB;
        ephemeris_record.d_M_0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_M0));
        ephemeris_record.d_M_0 = ephemeris_record.d_M_0 * CNAV_M0_LSB;
        ephemeris_record.d_e_eccentricity = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_E_ECCENTRICITY));
        ephemeris_record.d_e_eccentricity = ephemeris_record.d_e_eccentricity * CNAV_E_ECCENTRICITY_LSB;
        ephemeris_record.d_OMEGA = static_cast<double>(read_navigation_signed(data_bits, CNAV_OMEGA));
        ephemeris_record.d_OMEGA = ephemeris_record.d_OMEGA * CNAV_OMEGA_LSB;

        ephemeris_record.b_integrity_status_flag = static_cast<bool>(read_navigation_bool(data_bits, CNAV_INTEGRITY_FLAG));
        ephemeris_record.b_l2c_phasing_flag = static_cast<bool>(read_navigation_bool(data_bits, CNAV_L2_PHASING_FLAG));

        b_flag_ephemeris_1 = true;
        break;
    case 11: // Ephemeris 2/2
        ephemeris_record.d_Toe2 = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_TOE2));
        ephemeris_record.d_Toe2 = ephemeris_record.d_Toe2 * CNAV_TOE2_LSB;
        ephemeris_record.d_OMEGA0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_OMEGA0));
        ephemeris_record.d_OMEGA0 = ephemeris_record.d_OMEGA0 * CNAV_OMEGA0_LSB;
        ephemeris_record.d_DELTA_OMEGA_DOT = static_cast<double>(read_navigation_signed(data_bits, CNAV_DELTA_OMEGA_DOT));
        ephemeris_record.d_DELTA_OMEGA_DOT = ephemeris_record.d_DELTA_OMEGA_DOT * CNAV_DELTA_OMEGA_DOT_LSB;
        ephemeris_record.d_i_0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_I0));
        ephemeris_record.d_i_0 = ephemeris_record.d_i_0 * CNAV_I0_LSB;
        ephemeris_record.d_IDOT = static_cast<double>(read_navigation_signed(data_bits, CNAV_I0_DOT));
        ephemeris_record.d_IDOT = ephemeris_record.d_IDOT * CNAV_I0_DOT_LSB;
        ephemeris_record.d_Cis = static_cast<double>(read_navigation_signed(data_bits, CNAV_CIS));
        ephemeris_record.d_Cis = ephemeris_record.d_Cis * CNAV_CIS_LSB;
        ephemeris_record.d_Cic = static_cast<double>(read_navigation_signed(data_bits, CNAV_CIC));
        ephemeris_record.d_Cic = ephemeris_record.d_Cic * CNAV_CIC_LSB;
        ephemeris_record.d_Crs = static_cast<double>(read_navigation_signed(data_bits, CNAV_CRS));
        ephemeris_record.d_Crs = ephemeris_record.d_Crs * CNAV_CRS_LSB;
        ephemeris_record.d_Crc = static_cast<double>(read_navigation_signed(data_bits, CNAV_CRC));
        ephemeris_record.d_Cic = ephemeris_record.d_Cic * CNAV_CRC_LSB;
        ephemeris_record.d_Cus = static_cast<double>(read_navigation_signed(data_bits, CNAV_CUS));
        ephemeris_record.d_Cus = ephemeris_record.d_Cus * CNAV_CUS_LSB;
        ephemeris_record.d_Cuc = static_cast<double>(read_navigation_signed(data_bits, CNAV_CUC));
        ephemeris_record.d_Cuc = ephemeris_record.d_Cuc * CNAV_CUS_LSB;
        b_flag_ephemeris_2 = true;
        break;
    case 30: // (CLOCK, IONO, GRUP DELAY)
        //clock
        ephemeris_record.d_Toc = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_TOC));
        ephemeris_record.d_Toc = ephemeris_record.d_Toc * CNAV_TOC_LSB;
        ephemeris_record.d_URA0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_URA_NED0));
        ephemeris_record.d_URA1 = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_URA_NED1));
        ephemeris_record.d_URA2 = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_URA_NED2));
        ephemeris_record.d_A_f0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF0));
        ephemeris_record.d_A_f0 = ephemeris_record.d_A_f0 * CNAV_AF0_LSB;
        ephemeris_record.d_A_f1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF1));
        ephemeris_record.d_A_f1 = ephemeris_record.d_A_f1 * CNAV_AF1_LSB;
        ephemeris_record.d_A_f2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF2));
        ephemeris_record.d_A_f2 = ephemeris_record.d_A_f2 * CNAV_AF2_LSB;
        //group delays
        ephemeris_record.d_TGD = static_cast<double>(read_navigation_signed(data_bits, CNAV_TGD));
        ephemeris_record.d_TGD = ephemeris_record.d_TGD * CNAV_TGD_LSB;
        ephemeris_record.d_ISCL1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ISCL1));
        ephemeris_record.d_ISCL1 = ephemeris_record.d_ISCL1 * CNAV_ISCL1_LSB;
        ephemeris_record.d_ISCL2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ISCL2));
        ephemeris_record.d_ISCL2 = ephemeris_record.d_ISCL2 * CNAV_ISCL2_LSB;
        ephemeris_record.d_ISCL5I = static_cast<double>(read_navigation_signed(data_bits, CNAV_ISCL5I));
        ephemeris_record.d_ISCL5I = ephemeris_record.d_ISCL5I * CNAV_ISCL5I_LSB;
        ephemeris_record.d_ISCL5Q = static_cast<double>(read_navigation_signed(data_bits, CNAV_ISCL5Q));
        ephemeris_record.d_ISCL5Q = ephemeris_record.d_ISCL5Q * CNAV_ISCL5Q_LSB;
        //iono
        iono_record.d_alpha0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ALPHA0));
        iono_record.d_alpha0 = iono_record.d_alpha0 * CNAV_ALPHA0_LSB;
        iono_record.d_alpha1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ALPHA1));
        iono_record.d_alpha1 = iono_record.d_alpha1 * CNAV_ALPHA1_LSB;
        iono_record.d_alpha2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ALPHA2));
        iono_record.d_alpha2 = iono_record.d_alpha2 * CNAV_ALPHA2_LSB;
        iono_record.d_alpha3 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ALPHA3));
        iono_record.d_alpha3 = iono_record.d_alpha3 * CNAV_ALPHA3_LSB;
        iono_record.d_beta0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_BETA0));
        iono_record.d_beta0 = iono_record.d_beta0 * CNAV_BETA0_LSB;
        iono_record.d_beta1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_BETA1));
        iono_record.d_beta1 = iono_record.d_beta1 * CNAV_BETA1_LSB;
        iono_record.d_beta2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_BETA2));
        iono_record.d_beta2 = iono_record.d_beta2 * CNAV_BETA2_LSB;
        iono_record.d_beta3 = static_cast<double>(read_navigation_signed(data_bits, CNAV_BETA3));
        iono_record.d_beta3 = iono_record.d_beta3 * CNAV_BETA3_LSB;
        b_flag_iono_valid = true;
        break;
    case 33: // (CLOCK & UTC)
        ephemeris_record.d_Top = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_TOP1));
        ephemeris_record.d_Top = ephemeris_record.d_Top * CNAV_TOP1_LSB;
        ephemeris_record.d_Toc = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_TOC));
        ephemeris_record.d_Toc = ephemeris_record.d_Toc * CNAV_TOC_LSB;
        ephemeris_record.d_A_f0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF0));
        ephemeris_record.d_A_f0 = ephemeris_record.d_A_f0 * CNAV_AF0_LSB;
        ephemeris_record.d_A_f1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF1));
        ephemeris_record.d_A_f1 = ephemeris_record.d_A_f1 * CNAV_AF1_LSB;
        ephemeris_record.d_A_f2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF2));
        ephemeris_record.d_A_f2 = ephemeris_record.d_A_f2 * CNAV_AF2_LSB;


        utc_model_record.d_A0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_A0));
        utc_model_record.d_A0 = utc_model_record.d_A0 * CNAV_A0_LSB;
        utc_model_record.d_A1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_A1));
        utc_model_record.d_A1 = utc_model_record.d_A1 * CNAV_A1_LSB;
        utc_model_record.d_A2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_A2));
        utc_model_record.d_A2 = utc_model_record.d_A2 * CNAV_A2_LSB;


        utc_model_record.d_DeltaT_LS = static_cast<double>(read_navigation_signed(data_bits, CNAV_DELTA_TLS));
        utc_model_record.d_DeltaT_LS = utc_model_record.d_DeltaT_LS * CNAV_DELTA_TLS_LSB;

        utc_model_record.d_t_OT = static_cast<double>(read_navigation_signed(data_bits, CNAV_TOT));
        utc_model_record.d_t_OT = utc_model_record.d_t_OT * CNAV_TOT_LSB;

        utc_model_record.i_WN_T = static_cast<double>(read_navigation_signed(data_bits, CNAV_WN_OT));
        utc_model_record.i_WN_T = utc_model_record.i_WN_T * CNAV_WN_OT_LSB;

        utc_model_record.i_WN_LSF = static_cast<double>(read_navigation_signed(data_bits, CNAV_WN_LSF));
        utc_model_record.i_WN_LSF = utc_model_record.i_WN_LSF * CNAV_WN_LSF_LSB;

        utc_model_record.i_DN = static_cast<double>(read_navigation_signed(data_bits, CNAV_DN));
        utc_model_record.i_DN = utc_model_record.i_DN * CNAV_DN_LSB;

        utc_model_record.d_DeltaT_LSF = static_cast<double>(read_navigation_signed(data_bits, CNAV_DELTA_TLSF));
        utc_model_record.d_DeltaT_LSF = utc_model_record.d_DeltaT_LSF * CNAV_DELTA_TLSF_LSB;
        b_flag_utc_valid = true;
        break;
    default:
        break;
    }
}


bool Gps_CNAV_Navigation_Message::have_new_ephemeris() //Check if we have a new ephemeris stored in the galileo navigation class
{
    if (b_flag_ephemeris_1 == true and b_flag_ephemeris_2 == true)
        {
            if (ephemeris_record.d_Toe1 == ephemeris_record.d_Toe2)// and ephemeris_record.d_Toe1==ephemeris_record.d_Toc)
                {
                    //if all ephemeris pages have the same TOE, then they belong to the same block
                    // std::cout << "Ephemeris (1, 2) have been received and belong to the same batch" << std::endl;
                    b_flag_ephemeris_1 = false;// clear the flag
                    b_flag_ephemeris_2 = false;// clear the flag
                    return true;
                }
            else
                {
                    return false;
                }
        }
    else
        {
            return false;
        }
}


Gps_CNAV_Ephemeris Gps_CNAV_Navigation_Message::get_ephemeris()
{
    return ephemeris_record;
}


bool Gps_CNAV_Navigation_Message::have_new_iono() //Check if we have a new iono data stored in the galileo navigation class
{
    if (b_flag_iono_valid == true)
        {
            b_flag_iono_valid = false;// clear the flag
            return true;
        }
    else
        {
            return false;
        }
}


Gps_CNAV_Iono Gps_CNAV_Navigation_Message::get_iono()
{
    return iono_record;
}


bool Gps_CNAV_Navigation_Message::have_new_utc_model() //Check if we have a new iono data stored in the galileo navigation class
{
    if (b_flag_utc_valid == true)
        {
            b_flag_utc_valid = false;// clear the flag
            return true;
        }
    else
        {
            return false;
        }
}

Gps_CNAV_Utc_Model Gps_CNAV_Navigation_Message::get_utc_model()
{
    return utc_model_record;
}
