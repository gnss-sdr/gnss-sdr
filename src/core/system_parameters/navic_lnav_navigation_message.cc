/*!
 * \file navic_lnav_navigation_message.cc
 * \brief  Implementation of a NavIC LNAV Data message decoder as described
 * in IRNSS SIS ICD for SPS, Version 1.1 (ISRO-IRNSS-ICD-SPS-1.1)
 *
 * \author Pradyumna Krishna, 2026. pradyumnakrishna(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "navic_lnav_navigation_message.h"
#include "NAVIC_LNAV.h"
#include <cstdint>
#include <string>


Navic_Lnav_Navigation_Message::Navic_Lnav_Navigation_Message()
{
    flag_sf1_decoded = false;
    flag_sf2_decoded = false;
    flag_sf3_decoded = false;
    flag_sf4_decoded = false;
    flag_new_ephemeris = false;
    flag_new_almanac = false;
    flag_new_utc_model = false;
    flag_new_iono = false;
    flag_crc_test = false;
    i_channel_ID = 0;
    i_satellite_PRN = 0U;
    d_sf1_iodec = -1;
    d_sf2_iodec = -1;
    d_previous_iodec = -1;
}


uint64_t Navic_Lnav_Navigation_Message::read_navigation_unsigned(
    const std::string& bits,
    const std::vector<std::pair<int32_t, int32_t>>& field_def) const
{
    uint64_t value = 0ULL;
    for (const auto& [start, len] : field_def)
        {
            for (int32_t i = 0; i < len; i++)
                {
                    value <<= 1U;
                    if (bits[start - 1 + i] == '1')
                        {
                            value |= 1ULL;
                        }
                }
        }
    return value;
}


int64_t Navic_Lnav_Navigation_Message::read_navigation_signed(
    const std::string& bits,
    const std::vector<std::pair<int32_t, int32_t>>& field_def) const
{
    uint64_t value = 0ULL;
    int32_t total_bits = 0;
    for (const auto& [start, len] : field_def)
        {
            for (int32_t i = 0; i < len; i++)
                {
                    value <<= 1U;
                    if (bits[start - 1 + i] == '1')
                        {
                            value |= 1ULL;
                        }
                }
            total_bits += len;
        }
    // Sign extend if MSB is set (2's complement)
    if ((value >> (total_bits - 1)) & 1ULL)
        {
            value |= ~((1ULL << total_bits) - 1ULL);
        }
    return static_cast<int64_t>(value);
}


bool Navic_Lnav_Navigation_Message::crc24q_check(const std::string& bits) const
{
    // CRC-24Q polynomial used by NavIC (same as GPS CNAV):
    // g(X) = X^24 + X^23 + X^18 + X^17 + X^14 + X^11 + X^10 +
    //        X^7 + X^6 + X^5 + X^4 + X^3 + X^1 + X^0
    // In hex: 0x1864CFB
    constexpr uint32_t CRC24Q_POLY = 0x1864CFBU;

    // Compute CRC over bits 1-262 (indices 0..261 in 0-based string)
    uint32_t crc = 0U;
    for (int32_t i = 0; i < 262; i++)
        {
            uint32_t bit = (bits[i] == '1') ? 1U : 0U;
            crc ^= (bit << 23U);
            if (crc & 0x800000U)
                {
                    crc = ((crc << 1U) ^ CRC24Q_POLY) & 0xFFFFFFU;
                }
            else
                {
                    crc = (crc << 1U) & 0xFFFFFFU;
                }
        }

    // Extract transmitted CRC from bits 263-286 (indices 262..285 in 0-based)
    uint32_t transmitted_crc = 0U;
    for (int32_t i = 262; i < 286; i++)
        {
            transmitted_crc <<= 1U;
            if (bits[i] == '1')
                {
                    transmitted_crc |= 1U;
                }
        }

    return (crc == transmitted_crc);
}


void Navic_Lnav_Navigation_Message::decode_subframe(const std::string& subframe_bits)
{
    if (subframe_bits.size() < 292)
        {
            return;
        }

    // Verify CRC-24Q
    flag_crc_test = crc24q_check(subframe_bits);
    if (!flag_crc_test)
        {
            return;
        }

    // --- Extract common header fields ---
    // TOWC: bits 9-25, 17 bits unsigned, represents TOW count in units of 12 seconds
    const auto towc = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_TOWC));

    // Alert flag: bit 26
    const auto alert_flag = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_ALERT));

    // AutoNav flag: bit 27
    const auto autonav_flag = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_AUTONAV));

    // Subframe ID: bits 28-29 (2 bits): 00=SF1, 01=SF2, 10=SF3, 11=SF4
    const auto subframe_id = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_SUBFRAME_ID));

    // Store common fields in ephemeris
    navic_ephemeris.tow = towc * 12;  // TOWC is in units of 12 seconds

    switch (subframe_id)
        {
        case 0:  // --- Subframe 1: Clock and partial ephemeris parameters ---
            {
                // Week Number (10 bits, unsigned)
                navic_ephemeris.WN = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_WN));

                // Clock bias af0 (22 bits, signed)
                navic_ephemeris.af0 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_AF0)) * NAVIC_LNAV_AF0_LSB;

                // Clock drift af1 (16 bits, signed)
                navic_ephemeris.af1 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_AF1)) * NAVIC_LNAV_AF1_LSB;

                // Clock drift rate af2 (8 bits, signed)
                navic_ephemeris.af2 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_AF2)) * NAVIC_LNAV_AF2_LSB;

                // User Range Accuracy (4 bits, unsigned)
                navic_ephemeris.URA = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_URA));

                // Time of clock (16 bits, unsigned, scaled by 16)
                navic_ephemeris.toc = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_TOC) * NAVIC_LNAV_TOC_LSB);

                // Total Group Delay (8 bits, signed)
                navic_ephemeris.TGD = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_TGD)) * NAVIC_LNAV_TGD_LSB;

                // Mean Motion Difference (22 bits, signed)
                navic_ephemeris.delta_n = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_DELTA_N)) * NAVIC_LNAV_DELTA_N_LSB;

                // Issue of Data, Ephemeris and Clock (8 bits, unsigned)
                d_sf1_iodec = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_IODEC));
                navic_ephemeris.IODEC = d_sf1_iodec;

                // L5 flag (1 bit)
                navic_ephemeris.L5_flag = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_L5_FLAG));

                // S flag (1 bit)
                navic_ephemeris.S_flag = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_S_FLAG));

                // Cuc (15 bits, signed)
                navic_ephemeris.Cuc = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_CUC)) * NAVIC_LNAV_CUC_LSB;

                // Cus (15 bits, signed)
                navic_ephemeris.Cus = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_CUS)) * NAVIC_LNAV_CUS_LSB;

                // Cic (15 bits, signed)
                navic_ephemeris.Cic = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_CIC)) * NAVIC_LNAV_CIC_LSB;

                // Cis (15 bits, signed)
                navic_ephemeris.Cis = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_CIS)) * NAVIC_LNAV_CIS_LSB;

                // Crc (15 bits, signed)
                navic_ephemeris.Crc = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_CRC)) * NAVIC_LNAV_CRC_LSB;

                // Crs (15 bits, signed)
                navic_ephemeris.Crs = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_CRS)) * NAVIC_LNAV_CRS_LSB;

                // IDOT (14 bits, signed)
                navic_ephemeris.idot = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_IDOT)) * NAVIC_LNAV_IDOT_LSB;

                // Check IODEC consistency - if new epoch, invalidate cached subframe 2
                if (d_sf1_iodec != d_previous_iodec)
                    {
                        // New IODEC epoch - invalidate cached subframe 2 data
                        flag_sf2_decoded = false;
                        d_previous_iodec = d_sf1_iodec;
                    }

                // Set system flags
                flag_sf1_decoded = true;

                break;
            }

        case 1:  // --- Subframe 2: Remaining ephemeris parameters ---
            {
                // Mean anomaly at reference time M0 (32 bits, signed)
                navic_ephemeris.M_0 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_M0)) * NAVIC_LNAV_M0_LSB;

                // Time of Ephemeris (16 bits, unsigned, scaled by 16)
                navic_ephemeris.toe = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_TOE) * NAVIC_LNAV_TOE_LSB);

                // Eccentricity (32 bits, unsigned)
                navic_ephemeris.ecc = static_cast<double>(read_navigation_unsigned(subframe_bits, NAVIC_ECCENTRICITY)) * NAVIC_LNAV_E_LSB;

                // Square root of semi-major axis (32 bits, unsigned)
                navic_ephemeris.sqrtA = static_cast<double>(read_navigation_unsigned(subframe_bits, NAVIC_SQRT_A)) * NAVIC_LNAV_SQRT_A_LSB;

                // Longitude of ascending node Omega0 (32 bits, signed)
                navic_ephemeris.OMEGA_0 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_OMEGA0)) * NAVIC_LNAV_OMEGA0_LSB;

                // Argument of perigee omega (32 bits, signed)
                navic_ephemeris.omega = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_OMEGA)) * NAVIC_LNAV_OMEGA_LSB;

                // Rate of right ascension Omega_dot (22 bits, signed)
                navic_ephemeris.OMEGAdot = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_OMEGA_DOT)) * NAVIC_LNAV_OMEGA_DOT_LSB;

                // Inclination angle at reference time i0 (32 bits, signed)
                navic_ephemeris.i_0 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_I0)) * NAVIC_LNAV_I0_LSB;

                // Set system flags
                flag_sf2_decoded = true;

                // Check if we have a complete ephemeris set
                if (flag_sf1_decoded && flag_sf2_decoded)
                    {
                        flag_new_ephemeris = true;
                    }

                break;
            }

        case 2:  // --- Subframe 3: Secondary parameters (message-type based) ---
        case 3:  // --- Subframe 4: Secondary parameters (message-type based) ---
            {
                // Message ID (6 bits, unsigned)
                const auto message_id = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MSG_ID));

                switch (message_id)
                    {
                    case NAVIC_MSG_TYPE_ALMANAC:  // MT 7: Almanac
                        {
                            // Week Number of Almanac (10 bits, unsigned)
                            navic_almanac.WNa = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT7_WNA));

                            // Eccentricity (16 bits, unsigned)
                            navic_almanac.ecc = static_cast<double>(read_navigation_unsigned(subframe_bits, NAVIC_MT7_E)) * NAVIC_LNAV_ALM_E_LSB;

                            // Time of Almanac (16 bits, unsigned, scaled by 16)
                            navic_almanac.toa = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT7_TOA) * NAVIC_LNAV_ALM_TOA_LSB);

                            // Inclination (24 bits, signed)
                            navic_almanac.delta_i = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT7_I0)) * NAVIC_LNAV_ALM_I0_LSB;

                            // Rate of RAAN (16 bits, signed)
                            navic_almanac.OMEGAdot = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT7_OMEGA_DOT)) * NAVIC_LNAV_ALM_OMEGA_DOT_LSB;

                            // Square root of semi-major axis (24 bits, unsigned)
                            navic_almanac.sqrtA = static_cast<double>(read_navigation_unsigned(subframe_bits, NAVIC_MT7_SQRT_A)) * NAVIC_LNAV_ALM_SQRT_A_LSB;

                            // Longitude of ascending node (24 bits, signed)
                            navic_almanac.OMEGA_0 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT7_OMEGA0)) * NAVIC_LNAV_ALM_OMEGA0_LSB;

                            // Argument of perigee (24 bits, signed)
                            navic_almanac.omega = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT7_OMEGA)) * NAVIC_LNAV_ALM_OMEGA_LSB;

                            // Mean anomaly (24 bits, signed)
                            navic_almanac.M_0 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT7_M0)) * NAVIC_LNAV_ALM_M0_LSB;

                            // Clock bias (11 bits, signed)
                            navic_almanac.af0 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT7_AF0)) * NAVIC_LNAV_ALM_AF0_LSB;

                            // Clock drift (11 bits, signed)
                            navic_almanac.af1 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT7_AF1)) * NAVIC_LNAV_ALM_AF1_LSB;

                            // PRN ID of almanac satellite (6 bits, unsigned)
                            navic_almanac.PRN = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT7_PRN_ID_AL));

                            // Set flags
                            flag_new_almanac = true;
                            if (subframe_id == 2)
                                {
                                    flag_sf3_decoded = true;
                                }
                            else
                                {
                                    flag_sf4_decoded = true;
                                }

                            break;
                        }

                    case NAVIC_MSG_TYPE_UTC_GPS:  // MT 9: UTC and GPS Time Parameters
                        {
                            // A0_UTC (16 bits, signed)
                            navic_utc_model.A0_UTC = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT9_A0UTC)) * NAVIC_LNAV_A0UTC_LSB;

                            // A1_UTC (13 bits, signed)
                            navic_utc_model.A1_UTC = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT9_A1UTC)) * NAVIC_LNAV_A1UTC_LSB;

                            // A2_UTC (7 bits, signed)
                            navic_utc_model.A2_UTC = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT9_A2UTC)) * NAVIC_LNAV_A2UTC_LSB;

                            // DeltaT_LS (8 bits, signed)
                            navic_utc_model.DeltaT_LS = static_cast<int32_t>(read_navigation_signed(subframe_bits, NAVIC_MT9_DELTA_TLS));

                            // t_OT_UTC (16 bits, unsigned, scaled by 16)
                            navic_utc_model.t_OT_UTC = static_cast<double>(read_navigation_unsigned(subframe_bits, NAVIC_MT9_TOUTC)) * NAVIC_LNAV_TOUTC_LSB;

                            // WN_OT_UTC (10 bits, unsigned)
                            navic_utc_model.WN_OT_UTC = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT9_WNUTC));

                            // WN_LSF (10 bits, unsigned)
                            navic_utc_model.WN_LSF = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT9_WNLSF));

                            // DN (4 bits, unsigned)
                            navic_utc_model.DN = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT9_DN));

                            // DeltaT_LSF (8 bits, signed)
                            navic_utc_model.DeltaT_LSF = static_cast<int32_t>(read_navigation_signed(subframe_bits, NAVIC_MT9_DELTA_TLSF));

                            // IRNSS-GPS time offset: A0, A1, A2
                            navic_utc_model.A0 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT9_A0)) * NAVIC_LNAV_A0_LSB;
                            navic_utc_model.A1 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT9_A1)) * NAVIC_LNAV_A1_LSB;
                            navic_utc_model.A2 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT9_A2)) * NAVIC_LNAV_A2_LSB;

                            // t_OT (16 bits, unsigned, scaled by 16)
                            navic_utc_model.t_OT = static_cast<double>(read_navigation_unsigned(subframe_bits, NAVIC_MT9_TOT)) * NAVIC_LNAV_TOT_LSB;

                            // WN_OT (10 bits, unsigned)
                            navic_utc_model.WN_OT = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT9_WNOT));

                            // GNSS_ID (3 bits, unsigned)
                            navic_utc_model.GNSS_ID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT9_GNSS_ID));

                            navic_utc_model.valid = true;
                            flag_new_utc_model = true;
                            if (subframe_id == 2)
                                {
                                    flag_sf3_decoded = true;
                                }
                            else
                                {
                                    flag_sf4_decoded = true;
                                }

                            break;
                        }

                    case NAVIC_MSG_TYPE_UTC_GNSS:  // MT 26: UTC, UTC(NPLI) and Other GNSS Time
                        {
                            // Same layout as MT9 for the UTC section
                            navic_utc_model.A0_UTC = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT26_A0UTC)) * NAVIC_LNAV_A0UTC_LSB;
                            navic_utc_model.A1_UTC = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT26_A1UTC)) * NAVIC_LNAV_A1UTC_LSB;
                            navic_utc_model.A2_UTC = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT26_A2UTC)) * NAVIC_LNAV_A2UTC_LSB;
                            navic_utc_model.DeltaT_LS = static_cast<int32_t>(read_navigation_signed(subframe_bits, NAVIC_MT26_DELTA_TLS));
                            navic_utc_model.t_OT_UTC = static_cast<double>(read_navigation_unsigned(subframe_bits, NAVIC_MT26_TOUTC)) * NAVIC_LNAV_TOUTC_LSB;
                            navic_utc_model.WN_OT_UTC = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT26_WNUTC));
                            navic_utc_model.WN_LSF = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT26_WNLSF));
                            navic_utc_model.DN = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT26_DN));
                            navic_utc_model.DeltaT_LSF = static_cast<int32_t>(read_navigation_signed(subframe_bits, NAVIC_MT26_DELTA_TLSF));

                            // IRNSS-GNSS time offset
                            navic_utc_model.A0 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT26_A0)) * NAVIC_LNAV_A0_LSB;
                            navic_utc_model.A1 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT26_A1)) * NAVIC_LNAV_A1_LSB;
                            navic_utc_model.A2 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT26_A2)) * NAVIC_LNAV_A2_LSB;
                            navic_utc_model.t_OT = static_cast<double>(read_navigation_unsigned(subframe_bits, NAVIC_MT26_TOT)) * NAVIC_LNAV_TOT_LSB;
                            navic_utc_model.WN_OT = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT26_WNOT));
                            navic_utc_model.GNSS_ID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, NAVIC_MT26_GNSS_ID));

                            navic_utc_model.valid = true;
                            flag_new_utc_model = true;
                            if (subframe_id == 2)
                                {
                                    flag_sf3_decoded = true;
                                }
                            else
                                {
                                    flag_sf4_decoded = true;
                                }

                            break;
                        }

                    case NAVIC_MSG_TYPE_EOP_IONO:  // MT 11: EOP and Ionosphere Coefficients
                        {
                            // Ionospheric coefficients
                            navic_iono.alpha0 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT11_ALPHA0)) * NAVIC_LNAV_ALPHA0_LSB;
                            navic_iono.alpha1 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT11_ALPHA1)) * NAVIC_LNAV_ALPHA1_LSB;
                            navic_iono.alpha2 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT11_ALPHA2)) * NAVIC_LNAV_ALPHA2_LSB;
                            navic_iono.alpha3 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT11_ALPHA3)) * NAVIC_LNAV_ALPHA3_LSB;
                            navic_iono.beta0 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT11_BETA0)) * NAVIC_LNAV_BETA0_LSB;
                            navic_iono.beta1 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT11_BETA1)) * NAVIC_LNAV_BETA1_LSB;
                            navic_iono.beta2 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT11_BETA2)) * NAVIC_LNAV_BETA2_LSB;
                            navic_iono.beta3 = static_cast<double>(read_navigation_signed(subframe_bits, NAVIC_MT11_BETA3)) * NAVIC_LNAV_BETA3_LSB;

                            navic_iono.valid = true;
                            flag_new_iono = true;
                            if (subframe_id == 2)
                                {
                                    flag_sf3_decoded = true;
                                }
                            else
                                {
                                    flag_sf4_decoded = true;
                                }

                            break;
                        }

                    case NAVIC_MSG_TYPE_DIFF_CORR:  // MT 14: Differential Corrections
                        {
                            // Parse but do not process differential corrections for now
                            if (subframe_id == 2)
                                {
                                    flag_sf3_decoded = true;
                                }
                            else
                                {
                                    flag_sf4_decoded = true;
                                }

                            break;
                        }

                    case NAVIC_MSG_TYPE_TEXT:  // MT 18: Text Message
                        {
                            // Parse but do not process text messages for now
                            if (subframe_id == 2)
                                {
                                    flag_sf3_decoded = true;
                                }
                            else
                                {
                                    flag_sf4_decoded = true;
                                }

                            break;
                        }

                    case NAVIC_MSG_TYPE_NULL:  // MT 0: Null message
                        {
                            // Nothing to do
                            break;
                        }

                    default:
                        break;
                    }

                break;
            }

        default:
            break;
        }
}


Navic_Lnav_Ephemeris Navic_Lnav_Navigation_Message::get_ephemeris() const
{
    Navic_Lnav_Ephemeris eph = navic_ephemeris;
    eph.PRN = i_satellite_PRN;
    return eph;
}


Navic_Lnav_Almanac Navic_Lnav_Navigation_Message::get_almanac() const
{
    return navic_almanac;
}


Navic_Lnav_Iono Navic_Lnav_Navigation_Message::get_iono() const
{
    return navic_iono;
}


Navic_Lnav_Utc_Model Navic_Lnav_Navigation_Message::get_utc_model() const
{
    return navic_utc_model;
}


bool Navic_Lnav_Navigation_Message::have_new_ephemeris()
{
    if (flag_new_ephemeris)
        {
            flag_new_ephemeris = false;
            return true;
        }
    return false;
}


bool Navic_Lnav_Navigation_Message::have_new_almanac()
{
    if (flag_new_almanac)
        {
            flag_new_almanac = false;
            return true;
        }
    return false;
}


bool Navic_Lnav_Navigation_Message::have_new_iono()
{
    if (flag_new_iono)
        {
            flag_new_iono = false;
            return true;
        }
    return false;
}


bool Navic_Lnav_Navigation_Message::have_new_utc_model()
{
    if (flag_new_utc_model)
        {
            flag_new_utc_model = false;
            return true;
        }
    return false;
}
