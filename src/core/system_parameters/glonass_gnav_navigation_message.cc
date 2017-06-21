/*!
m * \file glonass_gnav_navigation_message.cc
 * \brief  Implementation of a GLONASS GNAV Data message decoder as described in GLONASS ICD (Edition 5.1)
 * See http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
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

#include "glonass_gnav_navigation_message.h"
#include <cmath>
#include <iostream>
#include <gnss_satellite.h>


void Glonass_Gnav_Navigation_Message::reset()
{
    b_valid_ephemeris_set_flag = false;
    double d_TOW; //!< Time of GPS Week of the ephemeris set (taken from subframes TOW) [s]
    d_TOW_SF1 = 0;            //!< Time of GPS Week from HOW word of Subframe 1 [s]
    d_TOW_SF2 = 0;            //!< Time of GPS Week from HOW word of Subframe 2 [s]
    d_TOW_SF3 = 0;            //!< Time of GPS Week from HOW word of Subframe 3 [s]
    d_TOW_SF4 = 0;            //!< Time of GPS Week from HOW word of Subframe 4 [s]
    d_TOW_SF5 = 0;            //!< Time of GPS Week from HOW word of Subframe 5 [s]

    d_m = 0.0;               //!< String number within frame [dimensionless]
    d_t_k = 0.0;             //!< Time referenced to the beginning of the frame within the current day [hours, minutes, seconds]
    d_t_b = 0.0;             //!< Index of a time interval within current day according to UTC(SU) + 03 hours 00 min. [minutes]
    d_M = 0.0;               //!< Type of satellite transmitting navigation signal [dimensionless]
    d_gamma_n = 0.0;         //!< Relative deviation of predicted carrier frequency value of n- satellite from nominal value at the instant tb [dimensionless]
    d_tau_n = 0.0;           //!< Correction to the nth satellite time (tn) relative to GLONASS time (te),
    d_B_n = 0.0;             //!< Health flag [dimensionless]
    d_P = 0.0;               //!< Technological parameter of control segment, indication the satellite operation mode in respect of time parameters [dimensionless]
    d_N_T = 0.0;             //!< Current date, calendar number of day within four-year interval starting from the 1-st of January in a leap year [days]
    d_F_T = 0.0;             //!< Parameter that provides the predicted satellite user range accuracy at time tb [dimensionless]
    d_n = 0.0;               //!< Index of the satellite transmitting given navigation signal. It corresponds to a slot number within GLONASS constellation
    d_Delta_tau_n = 0.0;     //!< Time difference between navigation RF signal transmitted in L2 sub- band and aviation RF signal transmitted in L1 sub-band by nth satellite. [dimensionless]
    d_E_n = 0.0;             //!< Characterises "age" of a current information [days]
    d_P_1 = 0.0;             //!< Flag of the immediate data updating.
    d_P_2 = 0.0;             //!< Flag of oddness ("1") or evenness ("0") of the value of (tb) [dimensionless]
    d_P_3 = 0.0;             //!< Flag indicating a number of satellites for which almanac is transmitted within given frame: "1" corresponds to 5 satellites and "0" corresponds to 4 satellites [dimensionless]
    d_P_4 = 0.0;             //!< Flag to show that ephemeris parameters are present. "1" indicates that updated ephemeris or frequency/time parameters have been uploaded by the control segment [dimensionless]
    d_l_n = 0.0;             //!< Health flag for nth satellite; ln = 0 indicates the n-th satellite is helthy, ln = 1 indicates malfunction of this nth satellite [dimensionless]

    // Almanac and Not Inmediate Information
    d_tau_c = 0.0;             //!< GLONASS time scale correction to UTC(SU) time. [s]
    d_tau_gps = 0.0;           //!< Correction to GPS time to GLONASS time [day]
    d_N_4 = 0.0;               //!< Four year interval number starting from 1996 [4 year interval]
    d_N_A = 0.0;               //!< Calendar day number within the four-year period beginning since the leap year [days]
    d_n_A = 0.0;               //!< Conventional number of satellite within GLONASS space segment [dimensionless]
    d_H_n_A = 0.0;             //!< Carrier frequency number of navigation RF signal transmitted by d_nA satellite [dimensionless]
    d_lambda_n_A = 0.0;        //!< Longitude of the first (within the d_NA day) ascending node of d_nA  [semi-circles]
    d_t_lambda_n_A = 0.0;      //!< Time of first ascending node passage [s]
    d_Delta_i_n_A = 0.0;        //!< Correction of the mean value of inclination of d_n_A satellite at instant t_lambda_n_A [semi-circles]
    d_Delta_T_n_A = 0.0;       //!< Correction to the mean value of Draconian period of d_n_A satellite at instant t_lambda_n_A[s / orbital period]
    d_Delta_T_n_A_dot = 0.0;   //!< Rate of change of Draconian period of d_n_A satellite at instant t_lambda_n_A [s / orbital period^2]
    d_epsilon_n_A = 0.0;       //!< Eccentricity of d_n_A satellite at instant t_lambda_n_A [dimensionless]
    d_omega_n_A = 0.0;         //!< Argument of preigree of d_n_A satellite at instant t_lambdan_A [semi-circles]
    d_M_n_A = 0.0;             //!< Type of satellite n_A [dimensionless]
    d_B1 = 0.0;                //!< Coefficient  to  determine DeltaUT1 [s]
    d_B2 = 0.0;                //!< Coefficient  to  determine DeltaUT1 [s/msd]
    d_KP = 0.0;                //!< Notification on forthcoming leap second correction of UTC [dimensionless]
    d_tau_n_A = 0.0;           //!< Coarse value of d_n_A satellite time correction to GLONASS time at instant  t_lambdan_A[s]
    d_C_n_A = 0.0;             //!< Generalized “unhealthy flag” of n_A satellite at instant of almanac upload [dimensionless]

    std::map<int,std::string> satelliteBlock; //!< Map that stores to which block the PRN belongs http://www.navcen.uscg.gov/?Do=constellationStatus


    /*! \brief If true, enhanced level of integrity assurance.
     *
     *  If false, indicates that the conveying signal is provided with the legacy level of integrity assurance.
     *  That is, the probability that the instantaneous URE of the conveying signal exceeds 4.42 times the upper bound
     *  value of the current broadcast URA index, for more than 5.2 seconds, without an accompanying alert, is less
     *  than 1E-5 per hour. If true, indicates that the conveying signal is provided with an enhanced level of
     *  integrity assurance. That is, the probability that the instantaneous URE of the conveying signal exceeds 5.73
     *  times the upper bound value of the current broadcast URA index, for more than 5.2 seconds, without an
     *  accompanying alert, is less than 1E-8 per hour.
     */
    b_integrity_status_flag = false;
    b_alert_flag = false;      //!< If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    b_antispoofing_flag = false;  //!<  If true, the AntiSpoofing mode is ON in that SV

    // Clock terms
    d_satClkCorr = 0.0;     // Satellite clock error
    d_dtr = 0.0;            // Relativistic clock correction term
    d_satClkDrift = 0.0;    // Satellite clock drift

    // satellite identification info
    int i_channel_ID = 0;
    int i_satellite_freq_channel = 0; //!< SV PRN NUMBER

    // time synchro
    d_subframe_timestamp_ms = 0; //[ms]

    // UTC parameters
    bool flag_utc_model_valid = false; //!< If set, it indicates that the UTC model parameters are filled

    // satellite positions
    d_satpos_X = 0.0;        //!< Earth-fixed coordinate x of the satellite in PZ-90.02 coordinate system [km].
    d_satpos_Y = 0.0;        //!< Earth-fixed coordinate y of the satellite in PZ-90.02 coordinate system [km]
    d_satpos_Z = 0.0;        //!< Earth-fixed coordinate z of the satellite in PZ-90.02 coordinate system [km]
    // Satellite velocity
    d_satvel_X = 0.0;        //!< Earth-fixed velocity coordinate x of the satellite in PZ-90.02 coordinate system [km/s]
    d_satvel_Y = 0.0;        //!< Earth-fixed velocity coordinate y of the satellite in PZ-90.02 coordinate system [km/s]
    d_satvel_Z = 0.0;        //!< Earth-fixed velocity coordinate z of the satellite in PZ-90.02 coordinate system [km/s]
    // Satellite acceleration
    d_satacc_X = 0.0;        //!< Earth-fixed acceleration coordinate x of the satellite in PZ-90.02 coordinate system [km/s^2]
    d_satacc_Y = 0.0;        //!< Earth-fixed acceleration coordinate y of the satellite in PZ-90.02 coordinate system [km/s^2]
    d_satacc_Z = 0.0;        //!< Earth-fixed acceleration coordinate z of the satellite in PZ-90.02 coordinate system [km/s^2]

    auto gnss_sat = Gnss_Satellite();
    std::string _system ("GLONASS");
    //TODO SHould number of channels be hardcoded?
    for(unsigned int i = 1; i < 14; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
}


Glonass_Gnav_Navigation_Message::Glonass_Gnav_Navigation_Message()
{
    reset();
}


bool Glonass_Gnav_Navigation_Message::read_navigation_bool(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    bool value;

    if (bits[GLONASS_GNAV_STRING_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


unsigned long int Glonass_Gnav_Navigation_Message::read_navigation_unsigned(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    unsigned long int value = 0;
    int num_of_slices = parameter.size();
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1; //shift left
                    if (bits[GLONASS_GNAV_STRING_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1; // insert the bit
                        }
                }
        }
    return value;
}


signed long int Glonass_Gnav_Navigation_Message::read_navigation_signed(std::bitset<GLONASS_GNAV_STRING_BITS> bits, const std::vector<std::pair<int,int>> parameter)
{
    signed long int value = 0;
    int num_of_slices = parameter.size();
    // Discriminate between 64 bits and 32 bits compiler
    int long_int_size_bytes = sizeof(signed long int);
    if (long_int_size_bytes == 8) // if a long int takes 8 bytes, we are in a 64 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GLONASS_GNAV_STRING_BITS - parameter[0].first] == 1)
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
                            if (bits[GLONASS_GNAV_STRING_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1; // insert the bit
                                }
                        }
                }
        }
    else  // we assume we are in a 32 bits system
        {
            // read the MSB and perform the sign extension
            if (bits[GLONASS_GNAV_STRING_BITS - parameter[0].first] == 1)
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
                            if (bits[GPS_SUBFRAME_BITS - parameter[i].first - j] == 1)
                                {
                                    value += 1; // insert the bit
                                }
                        }
                }
        }
    return value;
}

int Glonass_Gnav_Navigation_Message::string_decoder(char * frame_string, int frame_ID)
{
    int string_ID = 0;

    // UNPACK BYTES TO BITS AND REMOVE THE CRC REDUNDANCE
    std::bitset<GLONASS_GNAV_STRING_BITS> string_bits(std::string(frame_string));

    string_ID = static_cast<int>(read_navigation_unsigned(string_bits, STRING_ID));

    // Decode all 15 string messages
    switch (string_ID)
        {
        case 1:
            //--- It is string 1 -----------------------------------------------
            gnav_ephemeris.d_P_1 = static_cast<double>(read_navigation_unsigned(string_bits, P1));
            gnav_ephemeris.d_t_k = static_cast<double>(read_navigation_unsigned(string_bits, T_K));
            gnav_ephemeris.d_VXn = static_cast<double>(read_navigation_signed(string_bits, X_N_DOT));
            gnav_ephemeris.d_AXn = static_cast<double>(read_navigation_signed(string_bits, X_N_DOT_DOT));
            gnav_ephemeris.d_Xn = static_cast<double>(read_navigation_signed(string_bits, X_N));

            break;

        case 2:
            //--- It is string 2 -----------------------------------------------
            gnav_ephemeris.d_B_n = static_cast<double>(read_navigation_unsigned(string_bits, B_N));
            gnav_ephemeris.d_P_2 = static_cast<double>(read_navigation_unsigned(string_bits, P2));
            gnav_ephemeris.d_t_b = static_cast<double>(read_navigation_unsigned(string_bits, T_B));
            gnav_ephemeris.d_VYn = static_cast<double>(read_navigation_signed(string_bits, Y_N_DOT));
            gnav_ephemeris.d_AYn = static_cast<double>(read_navigation_signed(string_bits, Y_N_DOT_DOT));
            gnav_ephemeris.d_Yn = static_cast<double>(read_navigation_signed(string_bits, Y_N));

            break;

        case 3:
            // --- It is string 3 ----------------------------------------------
            gnav_ephemeris.d_P_3 = static_cast<double>(read_navigation_unsigned(string_bits, P3));
            gnav_ephemeris.d_gamma_n = static_cast<double>(read_navigation_signed(string_bits, GAMMA_N));
            gnav_ephemeris.d_P = static_cast<double>(read_navigation_unsigned(string_bits, P));
            gnav_ephemeris.d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, EPH_L_N));
            gnav_ephemeris.d_VZn = static_cast<double>(read_navigation_signed(string_bits, Z_N_DOT));
            gnav_ephemeris.d_AZn = static_cast<double>(read_navigation_signed(string_bits, Z_N_DOT_DOT));
            gnav_ephemeris.d_Zn = static_cast<double>(read_navigation_signed(string_bits, Z_N));

            break;

        case 4:
            // --- It is subframe 4 --------------------------------------------
            // TODO signed vs unsigned reading from datasheet
            gnav_ephemeris.d_tau_n = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N));
            gnav_ephemeris.d_Delta_tau_n = static_cast<double>(read_navigation_signed(string_bits, DELTA_TAU_N));
            gnav_ephemeris.d_E_n = static_cast<double>(read_navigation_unsigned(string_bits, E_N));
            gnav_ephemeris.d_P_4 = static_cast<double>(read_navigation_unsigned(string_bits, P4));
            gnav_ephemeris.d_F_T = static_cast<double>(read_navigation_unsigned(string_bits, F_T));
            gnav_ephemeris.d_N_T = static_cast<double>(read_navigation_unsigned(string_bits, N_T));
            gnav_ephemeris.d_n = static_cast<double>(read_navigation_unsigned(string_bits, N));
            gnav_ephemeris.d_M = static_cast<double>(read_navigation_unsigned(string_bits, M));

            break;

        case 5:
            // --- It is string 5 ----------------------------------------------
            // TODO signed vs unsigned reading from datasheet
            d_N_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            d_tau_c = static_cast<double>(read_navigation_unsigned(string_bits, TAU_C));
            d_N_4 = static_cast<double>(read_navigation_unsigned(string_bits, N_4));
            d_tau_c = static_cast<double>(read_navigation_unsigned(string_bits, TAU_C));
            d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

            break;

        case 6:
            // --- It is string 6 ----------------------------------------------
            // TODO signed vs unsigned reading from datasheet
            i_satellite_slot_number = = static_cast<double>(read_navigation_unsigned(string_bits, N_A));

            gnav_almanac[i_satellite_slot_number - 1].d_C_N = static_cast<double>(read_navigation_unsigned(string_bits, C_N));
            gnav_almanac[i_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, LAMBDA_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_I_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A));

            flag_almanac_str_6 = true;

            break;

        case 7:
            // --- It is string 7 ----------------------------------------------
            if (flag_almanac_str_6 == true)
              {
                // TODO signed vs unsigned reading from datasheet
                gnav_almanac[i_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_unsigned(string_bits, OMEGA_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_T_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_T_DOT_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

                flag_almanac_str_7 = true;
              }


            break;
        case 8:
            // --- It is string 8 ----------------------------------------------
            // TODO signed vs unsigned reading from datasheet
            i_satellite_slot_number = = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_C_N = static_cast<double>(read_navigation_unsigned(string_bits, C_N));
            gnav_almanac[i_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, LAMBDA_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_I_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A));

            flag_almanac_str_8 = true;

            break;

        case 9:
            // --- It is string 9 ----------------------------------------------
            if (flag_almanac_str_8 == true)
              {
                // TODO signed vs unsigned reading from datasheet
                gnav_almanac[i_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_unsigned(string_bits, OMEGA_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_T_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_T_DOT_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

                flag_almanac_str_9 = true;
              }
        case 10:
            // --- It is string 8 ----------------------------------------------
            // TODO signed vs unsigned reading from datasheet
            i_satellite_slot_number = = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_C_N = static_cast<double>(read_navigation_unsigned(string_bits, C_N));
            gnav_almanac[i_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, LAMBDA_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_I_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A));

            flag_almanac_str_10 = true;

            break;

        case 11:
            // --- It is string 9 ----------------------------------------------
            if (flag_almanac_str_10 == true)
              {
                // TODO signed vs unsigned reading from datasheet
                gnav_almanac[i_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_unsigned(string_bits, OMEGA_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_T_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_T_DOT_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

                flag_almanac_str_11 = true;
              }
            break;
        case 12:
            // --- It is string 8 ----------------------------------------------
            // TODO signed vs unsigned reading from datasheet
            i_satellite_slot_number = = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_C_N = static_cast<double>(read_navigation_unsigned(string_bits, C_N));
            gnav_almanac[i_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, LAMBDA_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_I_N_A));
            gnav_almanac[i_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A));

            flag_almanac_str_12 = true;

            break;

        case 13:
            // --- It is string 9 ----------------------------------------------
            if (flag_almanac_str_12 == true)
              {
                // TODO signed vs unsigned reading from datasheet
                gnav_almanac[i_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_unsigned(string_bits, OMEGA_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_T_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_T_DOT_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

                flag_almanac_str_13 = true;
              }
        case 14:
            // --- It is string 8 ----------------------------------------------
            if( frame_number == 5)
              {
                gnav_utc_model.B1 = static_cast<double>(read_navigation_unsigned(string_bits, B1));
                gnav_utc_model.B2 = static_cast<double>(read_navigation_unsigned(string_bits, B2));
              }
            else
              {
                // TODO signed vs unsigned reading from datasheet
                i_satellite_slot_number = = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_C_N = static_cast<double>(read_navigation_unsigned(string_bits, C_N));
                gnav_almanac[i_satellite_slot_number - 1].d_M_n_A = static_cast<double>(read_navigation_unsigned(string_bits, M_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_n_A = static_cast<double>(read_navigation_unsigned(string_bits, N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_tau_n_A = static_cast<double>(read_navigation_unsigned(string_bits, TAU_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, LAMBDA_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_i_n_A = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_I_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_epsilon_n_A = static_cast<double>(read_navigation_unsigned(string_bits, EPSILON_N_A));

                flag_almanac_str_14 = true;
              }


            break;

        case 15:
            // --- It is string 9 ----------------------------------------------
            if (frame_number != 5 and flag_almanac_str_14 = true )
              {
                // TODO signed vs unsigned reading from datasheet
                gnav_almanac[i_satellite_slot_number - 1].d_omega_n_A = static_cast<double>(read_navigation_unsigned(string_bits, OMEGA_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_t_lambda_n_A = static_cast<double>(read_navigation_unsigned(string_bits, T_LAMBDA_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_T_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_Delta_T_n_A_dot = static_cast<double>(read_navigation_unsigned(string_bits, DELTA_T_DOT_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_H_n_A = static_cast<double>(read_navigation_unsigned(string_bits, H_N_A));
                gnav_almanac[i_satellite_slot_number - 1].d_l_n = static_cast<double>(read_navigation_unsigned(string_bits, ALM_L_N));

                flag_almanac_str_15 = true;
              }
        default:
            break;
        } // switch subframeID ...

    return subframe_ID;
}




double Glonass_Gnav_Navigation_Message::utc_time(const double glonass_time_corrected) const
{
    double t_utc;

    t_utc = glonass_time_corrected + 3*3600 + d_tau_c;
    return t_utc;
}



Glonass_Gnav_Ephemeris Glonass_Gnav_Navigation_Message::get_ephemeris()
{
    Glonass_Gnav_Ephemeris ephemeris;

    ephemeris.i_satellite_freq_channel = i_satellite_freq_channel;
    ephemeris.d_m = d_m;
    ephemeris.d_t_k = d_t_k;
    ephemeris.d_t_b = d_t_b;
    ephemeris.d_M = d_M;
    ephemeris.d_gamma_n = d_gamma_n;
    ephemeris.d_tau_n = d_tau_n;
    // satellite positions
    ephemeris.d_satpos_X = d_satpos_X;
    ephemeris.d_satpos_Y = d_satpos_Y;
    ephemeris.d_satpos_Z = d_satpos_Z;
    // Satellite velocity
    ephemeris.d_satvel_X = d_satvel_X;
    ephemeris.d_satvel_Y = d_satvel_Y;
    ephemeris.d_satvel_Z = d_satvel_Z;
    // Satellite acceleration
    ephemeris.d_satacc_X = d_satacc_X;
    ephemeris.d_satacc_Y = d_satacc_Y;
    ephemeris.d_satacc_Z = d_satacc_Z;
    ephemeris.d_B_n = d_B_n;
    ephemeris.d_P = d_P;
    ephemeris.d_N_T = d_N_T;
    ephemeris.d_F_T = d_F_T;
    ephemeris.d_n = d_n;
    ephemeris.d_Delta_tau_n = d_Delta_tau_n;
    ephemeris.d_E_n = d_E_n;
    ephemeris.d_P_1 = d_P_1;
    ephemeris.d_P_2 = d_P_2;
    ephemeris.d_P_3 = d_P_3;
    ephemeris.d_P_4 = d_P_4;
    ephemeris.d_l_n = d_l_n;

    // clock terms derived from ephemeris data
    ephemeris.d_satClkDrift = d_satClkDrift;
    ephemeris.d_dtr = d_dtr;

    return ephemeris;
}


Glonass_Gnav_Utc_Model Glonass_Gnav_Navigation_Message::get_utc_model()
{
    Gps_Utc_Model utc_model;
    utc_model.valid = flag_utc_model_valid;
    // UTC parameters
    utc_model.d_A1 = d_A1;
    utc_model.d_A0 = d_A0;
    utc_model.d_t_OT = d_t_OT;
    utc_model.i_WN_T = i_WN_T;
    utc_model.d_DeltaT_LS = d_DeltaT_LS;
    utc_model.i_WN_LSF = i_WN_LSF;
    utc_model.i_DN = i_DN;
    utc_model.d_DeltaT_LSF = d_DeltaT_LSF;
    // warning: We clear flag_utc_model_valid in order to not re-send the same information to the ionospheric parameters queue
    flag_utc_model_valid = false;
    return utc_model;
}


bool Glonass_Gnav_Navigation_Message::satellite_validation()
{
    bool flag_data_valid = false;
    b_valid_ephemeris_set_flag = false;

    // First Step:
    // check Issue Of Ephemeris Data (IODE IODC..) to find a possible interrupted reception
    // and check if the data have been filled (!=0)
    if (d_TOW_SF1 != 0 and d_TOW_SF2 != 0 and d_TOW_SF3 != 0)
        {
            if (d_IODE_SF2 == d_IODE_SF3 and d_IODC == d_IODE_SF2 and d_IODC!= -1)
                {
                    flag_data_valid = true;
                    b_valid_ephemeris_set_flag = true;
                }
        }
    return flag_data_valid;
}
