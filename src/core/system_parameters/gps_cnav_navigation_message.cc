/*!
 * \file gps_cnav_navigation_message.cc
 * \brief Implementation of a GPS CNAV Data message decoder as described in IS-GPS-200M
 *
 * See https://www.gps.gov/technical/icwg/IS-GPS-200M.pdf Appendix III
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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

#include "gps_cnav_navigation_message.h"
#include "gnss_satellite.h"
#include <cmath>   // for std::sqrt
#include <limits>  // for std::numeric_limits


Gps_CNAV_Navigation_Message::Gps_CNAV_Navigation_Message()
{
    Gnss_Satellite gnss_satellite_ = Gnss_Satellite();
    for (uint32_t prn_ = 1; prn_ < 33; prn_++)
        {
            satelliteBlock[prn_] = gnss_satellite_.what_block("GPS", prn_);
        }
    b_flag_iono_valid = false;
    b_flag_utc_valid = false;
}


bool Gps_CNAV_Navigation_Message::read_navigation_bool(const std::bitset<GPS_CNAV_DATA_PAGE_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const
{
    bool value = bits[GPS_CNAV_DATA_PAGE_BITS - parameter[0].first];
    return value;
}


uint64_t Gps_CNAV_Navigation_Message::read_navigation_unsigned(const std::bitset<GPS_CNAV_DATA_PAGE_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const
{
    uint64_t value = 0ULL;
    for (const auto& p : parameter)
        {
            for (int32_t j = 0; j < p.second; j++)
                {
                    value = (value << 1) | static_cast<uint64_t>(bits[GPS_CNAV_DATA_PAGE_BITS - p.first - j]);
                }
        }
    return value;
}


int64_t Gps_CNAV_Navigation_Message::read_navigation_signed(const std::bitset<GPS_CNAV_DATA_PAGE_BITS>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter) const
{
    int64_t value = (bits[GPS_CNAV_DATA_PAGE_BITS - parameter[0].first] == 1) ? -1LL : 0LL;
    for (const auto& p : parameter)
        {
            for (int32_t j = 0; j < p.second; j++)
                {
                    value = (value << 1) | static_cast<int64_t>(bits[GPS_CNAV_DATA_PAGE_BITS - p.first - j]);
                }
        }
    return value;
}


void Gps_CNAV_Navigation_Message::decode_page(const std::bitset<GPS_CNAV_DATA_PAGE_BITS>& data_bits)
{
    int32_t page_type;
    bool alert_flag;

    // common to all messages
    const auto PRN = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV_PRN));
    ephemeris_record.PRN = PRN;

    d_TOW = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV_TOW));
    d_TOW *= CNAV_TOW_LSB;
    ephemeris_record.tow = d_TOW;

    alert_flag = read_navigation_bool(data_bits, CNAV_ALERT_FLAG);
    ephemeris_record.alert_flag = alert_flag;

    page_type = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV_MSG_TYPE));


    switch (page_type)
        {
        case 10:  // Ephemeris 1/2
            ephemeris_record.WN = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV_WN));
            ephemeris_record.signal_health = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV_HEALTH));
            ephemeris_record.top = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV_TOP1));
            ephemeris_record.top *= CNAV_TOP1_LSB;
            ephemeris_record.URA0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_URA));
            ephemeris_record.toe1 = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV_TOE1));
            ephemeris_record.toe1 *= CNAV_TOE1_LSB;
            ephemeris_record.delta_A = static_cast<double>(read_navigation_signed(data_bits, CNAV_DELTA_A));
            ephemeris_record.delta_A *= CNAV_DELTA_A_LSB;
            ephemeris_record.Adot = static_cast<double>(read_navigation_signed(data_bits, CNAV_A_DOT));
            ephemeris_record.Adot *= CNAV_A_DOT_LSB;
            ephemeris_record.sqrtA = std::sqrt(CNAV_A_REF + ephemeris_record.delta_A);
            ephemeris_record.delta_n = static_cast<double>(read_navigation_signed(data_bits, CNAV_DELTA_N0));
            ephemeris_record.delta_n *= CNAV_DELTA_N0_LSB;
            ephemeris_record.delta_ndot = static_cast<double>(read_navigation_signed(data_bits, CNAV_DELTA_N0_DOT));
            ephemeris_record.delta_ndot *= CNAV_DELTA_N0_DOT_LSB;
            ephemeris_record.M_0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_M0));
            ephemeris_record.M_0 *= CNAV_M0_LSB;
            ephemeris_record.ecc = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_E_ECCENTRICITY));
            ephemeris_record.ecc *= CNAV_E_ECCENTRICITY_LSB;
            ephemeris_record.omega = static_cast<double>(read_navigation_signed(data_bits, CNAV_OMEGA));
            ephemeris_record.omega *= CNAV_OMEGA_LSB;

            ephemeris_record.integrity_status_flag = read_navigation_bool(data_bits, CNAV_INTEGRITY_FLAG);
            ephemeris_record.l2c_phasing_flag = read_navigation_bool(data_bits, CNAV_L2_PHASING_FLAG);

            b_flag_ephemeris_1 = true;
            break;
        case 11:  // Ephemeris 2/2
            ephemeris_record.toe2 = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV_TOE2));
            ephemeris_record.toe2 *= CNAV_TOE2_LSB;
            ephemeris_record.OMEGA_0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_OMEGA0));
            ephemeris_record.OMEGA_0 *= CNAV_OMEGA0_LSB;
            ephemeris_record.delta_OMEGAdot = static_cast<double>(read_navigation_signed(data_bits, CNAV_DELTA_OMEGA_DOT));
            ephemeris_record.delta_OMEGAdot *= CNAV_DELTA_OMEGA_DOT_LSB;
            ephemeris_record.OMEGAdot = CNAV_OMEGA_DOT_REF * GNSS_PI + ephemeris_record.delta_OMEGAdot;
            ephemeris_record.i_0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_I0));
            ephemeris_record.i_0 *= CNAV_I0_LSB;
            ephemeris_record.idot = static_cast<double>(read_navigation_signed(data_bits, CNAV_I0_DOT));
            ephemeris_record.idot *= CNAV_I0_DOT_LSB;
            ephemeris_record.Cis = static_cast<double>(read_navigation_signed(data_bits, CNAV_CIS));
            ephemeris_record.Cis *= CNAV_CIS_LSB;
            ephemeris_record.Cic = static_cast<double>(read_navigation_signed(data_bits, CNAV_CIC));
            ephemeris_record.Cic *= CNAV_CIC_LSB;
            ephemeris_record.Crs = static_cast<double>(read_navigation_signed(data_bits, CNAV_CRS));
            ephemeris_record.Crs *= CNAV_CRS_LSB;
            ephemeris_record.Crc = static_cast<double>(read_navigation_signed(data_bits, CNAV_CRC));
            ephemeris_record.Crc *= CNAV_CRC_LSB;
            ephemeris_record.Cus = static_cast<double>(read_navigation_signed(data_bits, CNAV_CUS));
            ephemeris_record.Cus *= CNAV_CUS_LSB;
            ephemeris_record.Cuc = static_cast<double>(read_navigation_signed(data_bits, CNAV_CUC));
            ephemeris_record.Cuc *= CNAV_CUC_LSB;
            b_flag_ephemeris_2 = true;
            break;
        case 30:  // (CLOCK, IONO, GRUP DELAY)
            // clock
            ephemeris_record.toc = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV_TOC));
            ephemeris_record.toc *= CNAV_TOC_LSB;
            ephemeris_record.URA0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_URA_NED0));
            ephemeris_record.URA1 = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_URA_NED1));
            ephemeris_record.URA2 = static_cast<double>(read_navigation_unsigned(data_bits, CNAV_URA_NED2));
            ephemeris_record.af0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF0));
            ephemeris_record.af0 *= CNAV_AF0_LSB;
            ephemeris_record.af1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF1));
            ephemeris_record.af1 *= CNAV_AF1_LSB;
            ephemeris_record.af2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF2));
            ephemeris_record.af2 *= CNAV_AF2_LSB;
            // group delays
            // Check if the grup delay values are not available. See IS-GPS-200, Table 30-IV.
            // Bit string "1000000000000" is -4096 in 2 complement
            ephemeris_record.TGD = static_cast<double>(read_navigation_signed(data_bits, CNAV_TGD));
            if (ephemeris_record.TGD < -4095.9)
                {
                    ephemeris_record.TGD = 0.0;
                }
            ephemeris_record.TGD *= CNAV_TGD_LSB;

            ephemeris_record.ISCL1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ISCL1));
            if (ephemeris_record.ISCL1 < -4095.9)
                {
                    ephemeris_record.ISCL1 = 0.0;
                }
            ephemeris_record.ISCL1 *= CNAV_ISCL1_LSB;

            ephemeris_record.ISCL2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ISCL2));
            if (ephemeris_record.ISCL2 < -4095.9)
                {
                    ephemeris_record.ISCL2 = 0.0;
                }
            ephemeris_record.ISCL2 *= CNAV_ISCL2_LSB;

            ephemeris_record.ISCL5I = static_cast<double>(read_navigation_signed(data_bits, CNAV_ISCL5I));
            if (ephemeris_record.ISCL5I < -4095.9)
                {
                    ephemeris_record.ISCL5I = 0.0;
                }
            ephemeris_record.ISCL5I *= CNAV_ISCL5I_LSB;

            ephemeris_record.ISCL5Q = static_cast<double>(read_navigation_signed(data_bits, CNAV_ISCL5Q));
            if (ephemeris_record.ISCL5Q < -4095.9)
                {
                    ephemeris_record.ISCL5Q = 0.0;
                }
            ephemeris_record.ISCL5Q *= CNAV_ISCL5Q_LSB;
            // iono
            iono_record.alpha0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ALPHA0));
            iono_record.alpha0 = iono_record.alpha0 * CNAV_ALPHA0_LSB;
            iono_record.alpha1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ALPHA1));
            iono_record.alpha1 = iono_record.alpha1 * CNAV_ALPHA1_LSB;
            iono_record.alpha2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ALPHA2));
            iono_record.alpha2 = iono_record.alpha2 * CNAV_ALPHA2_LSB;
            iono_record.alpha3 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ALPHA3));
            iono_record.alpha3 = iono_record.alpha3 * CNAV_ALPHA3_LSB;
            iono_record.beta0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_BETA0));
            iono_record.beta0 = iono_record.beta0 * CNAV_BETA0_LSB;
            iono_record.beta1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_BETA1));
            iono_record.beta1 = iono_record.beta1 * CNAV_BETA1_LSB;
            iono_record.beta2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_BETA2));
            iono_record.beta2 = iono_record.beta2 * CNAV_BETA2_LSB;
            iono_record.beta3 = static_cast<double>(read_navigation_signed(data_bits, CNAV_BETA3));
            iono_record.beta3 = iono_record.beta3 * CNAV_BETA3_LSB;
            b_flag_iono_valid = true;
            break;
        case 33:  // (CLOCK & UTC)
            ephemeris_record.top = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV_TOP1));
            ephemeris_record.top = ephemeris_record.top * CNAV_TOP1_LSB;
            ephemeris_record.toc = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV_TOC));
            ephemeris_record.toc = ephemeris_record.toc * CNAV_TOC_LSB;
            ephemeris_record.af0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF0));
            ephemeris_record.af0 = ephemeris_record.af0 * CNAV_AF0_LSB;
            ephemeris_record.af1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF1));
            ephemeris_record.af1 = ephemeris_record.af1 * CNAV_AF1_LSB;
            ephemeris_record.af2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_AF2));
            ephemeris_record.af2 = ephemeris_record.af2 * CNAV_AF2_LSB;

            utc_model_record.A0 = static_cast<double>(read_navigation_signed(data_bits, CNAV_A0));
            utc_model_record.A0 = utc_model_record.A0 * CNAV_A0_LSB;
            utc_model_record.A1 = static_cast<double>(read_navigation_signed(data_bits, CNAV_A1));
            utc_model_record.A1 = utc_model_record.A1 * CNAV_A1_LSB;
            utc_model_record.A2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_A2));
            utc_model_record.A2 = utc_model_record.A2 * CNAV_A2_LSB;

            utc_model_record.DeltaT_LS = static_cast<int32_t>(read_navigation_signed(data_bits, CNAV_DELTA_TLS));
            utc_model_record.DeltaT_LS = utc_model_record.DeltaT_LS * CNAV_DELTA_TLS_LSB;

            utc_model_record.tot = static_cast<int32_t>(read_navigation_signed(data_bits, CNAV_TOT));
            utc_model_record.tot = utc_model_record.tot * CNAV_TOT_LSB;

            utc_model_record.WN_T = static_cast<int32_t>(read_navigation_signed(data_bits, CNAV_WN_OT));
            utc_model_record.WN_T = utc_model_record.WN_T * CNAV_WN_OT_LSB;

            utc_model_record.WN_LSF = static_cast<int32_t>(read_navigation_signed(data_bits, CNAV_WN_LSF));
            utc_model_record.WN_LSF = utc_model_record.WN_LSF * CNAV_WN_LSF_LSB;

            utc_model_record.DN = static_cast<int32_t>(read_navigation_signed(data_bits, CNAV_DN));
            utc_model_record.DN = utc_model_record.DN * CNAV_DN_LSB;

            utc_model_record.DeltaT_LSF = static_cast<int32_t>(read_navigation_signed(data_bits, CNAV_DELTA_TLSF));
            utc_model_record.DeltaT_LSF = utc_model_record.DeltaT_LSF * CNAV_DELTA_TLSF_LSB;
            b_flag_utc_valid = true;
            break;
        default:
            break;
        }
}


bool Gps_CNAV_Navigation_Message::have_new_ephemeris()  // Check if we have a new ephemeris stored in the galileo navigation class
{
    if (b_flag_ephemeris_1 == true and b_flag_ephemeris_2 == true)
        {
            if (ephemeris_record.toe1 == ephemeris_record.toe2)  // and ephemeris_record.toe1==ephemeris_record.d_Toc)
                {
                    // if all ephemeris pages have the same TOE, then they belong to the same block
                    // std::cout << "Ephemeris (1, 2) have been received and belong to the same batch\n";
                    b_flag_ephemeris_1 = false;  // clear the flag
                    b_flag_ephemeris_2 = false;  // clear the flag
                    return true;
                }
        }
    return false;
}


Gps_CNAV_Ephemeris Gps_CNAV_Navigation_Message::get_ephemeris() const
{
    return ephemeris_record;
}


bool Gps_CNAV_Navigation_Message::have_new_iono()  // Check if we have a new iono data stored in the galileo navigation class
{
    if (b_flag_iono_valid == true)
        {
            b_flag_iono_valid = false;  // clear the flag
            return true;
        }
    return false;
}


Gps_CNAV_Iono Gps_CNAV_Navigation_Message::get_iono() const
{
    return iono_record;
}


bool Gps_CNAV_Navigation_Message::have_new_utc_model()  // Check if we have a new iono data stored in the galileo navigation class
{
    if (b_flag_utc_valid == true)
        {
            b_flag_utc_valid = false;  // clear the flag
            return true;
        }
    return false;
}


Gps_CNAV_Utc_Model Gps_CNAV_Navigation_Message::get_utc_model()
{
    utc_model_record.valid = true;
    return utc_model_record;
}
