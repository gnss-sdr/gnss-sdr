/*!
 * \file gps_cnav2_navigation_message.h
 * \brief  Interface of a GPS CNAV2 Data message decoder
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
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

#include "GPS_CNAV.h"
#include "GPS_CNAV2.h"
#include "gnss_satellite.h"
#include "gps_cnav2_navigation_message.h"
#include "read_navigation.h"
#include <cmath>   // for std::sqrt
#include <limits>  // for std::numeric_limits

Gps_CNAV2_Navigation_Message::Gps_CNAV2_Navigation_Message(uint32_t prn, CnavSystem system)
    : d_system(system)
{
    ephemeris_record.PRN = prn;
}

void Gps_CNAV2_Navigation_Message::decode_sf2(uint16_t toi, const std::bitset<GPS_L1C_SF_2_DATA_BITS>& data_bits)
{
    uint16_t this_sf_toi = toi - 1;
    if (toi == 0)
        {
            this_sf_toi = 399;
        }

    // TODO: Create constants for CNAV2, even if they are the same as CNAV
    b_flag_ephemeris = true;
    ephemeris_record.WN = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV2_WN));

    ephemeris_record.tow = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV2_ITOW));
    ephemeris_record.tow *= 2 * 3600;
    ephemeris_record.tow += this_sf_toi * 18;

    ephemeris_record.top = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV2_TOP));
    ephemeris_record.l1c_signal_health = read_navigation_bool(data_bits, CNAV2_L1C_HEALTH);
    ephemeris_record.URAED = static_cast<int32_t>(read_navigation_signed(data_bits, CNAV2_URAED));
    // We set toe1 = toe2 = toe, as toe1 and toe2 no longer exist in CNAV2
    ephemeris_record.toe1 = static_cast<uint32_t>(read_navigation_unsigned(data_bits, CNAV2_TOE));
    ephemeris_record.toe1 *= CNAV_TOE1_LSB;
    ephemeris_record.toe2 = ephemeris_record.toe1;
    ephemeris_record.toe = ephemeris_record.toe1;
    // toc is also toe
    ephemeris_record.toc = ephemeris_record.toe1;

    ephemeris_record.delta_A = static_cast<double>(read_navigation_signed(data_bits, CNAV2_DELTA_A));
    ephemeris_record.delta_A *= CNAV_DELTA_A_LSB;

    if (d_system == CnavSystem::GPS)
        {
            ephemeris_record.sqrtA = std::sqrt(CNAV_A_REF + ephemeris_record.delta_A);
        }
    else
        {
            ephemeris_record.sqrtA = std::sqrt(CNAV_QZSS_A_REF + ephemeris_record.delta_A);
        }


    ephemeris_record.Adot = static_cast<double>(read_navigation_signed(data_bits, CNAV2_DOT_A));
    ephemeris_record.Adot *= CNAV_A_DOT_LSB;
    ephemeris_record.delta_n = static_cast<double>(read_navigation_signed(data_bits, CNAV2_DELTA_N));
    ephemeris_record.delta_n *= CNAV_DELTA_N0_LSB;
    ephemeris_record.delta_ndot = static_cast<double>(read_navigation_signed(data_bits, CNAV2_DELTA_DOT_N));
    ephemeris_record.delta_ndot *= CNAV_DELTA_N0_DOT_LSB;
    ephemeris_record.M_0 = static_cast<double>(read_navigation_signed(data_bits, CNAV2_M0));
    ephemeris_record.M_0 *= CNAV_M0_LSB;
    ephemeris_record.ecc = static_cast<double>(read_navigation_unsigned(data_bits, CNAV2_E));
    ephemeris_record.ecc *= CNAV_E_ECCENTRICITY_LSB;
    ephemeris_record.omega = static_cast<double>(read_navigation_signed(data_bits, CNAV2_OMEGA));
    ephemeris_record.omega *= CNAV_OMEGA_LSB;
    ephemeris_record.OMEGA_0 = static_cast<double>(read_navigation_signed(data_bits, CNAV2_OMEGA0));
    ephemeris_record.OMEGA_0 *= CNAV_OMEGA0_LSB;
    ephemeris_record.delta_OMEGAdot = static_cast<double>(read_navigation_signed(data_bits, CNAV2_DELTA_OMEGA_DOT));
    ephemeris_record.delta_OMEGAdot *= CNAV_DELTA_OMEGA_DOT_LSB;
    ephemeris_record.OMEGAdot = CNAV_OMEGA_DOT_REF * GNSS_PI + ephemeris_record.delta_OMEGAdot;

    ephemeris_record.i_0 = static_cast<double>(read_navigation_signed(data_bits, CNAV2_I0));
    ephemeris_record.i_0 *= CNAV_I0_LSB;
    ephemeris_record.idot = static_cast<double>(read_navigation_signed(data_bits, CNAV2_IDOT));
    ephemeris_record.idot *= CNAV_I0_DOT_LSB;
    ephemeris_record.Cis = static_cast<double>(read_navigation_signed(data_bits, CNAV2_CIS));
    ephemeris_record.Cis *= CNAV_CIS_LSB;
    ephemeris_record.Cic = static_cast<double>(read_navigation_signed(data_bits, CNAV2_CIC));
    ephemeris_record.Cic *= CNAV_CIC_LSB;
    ephemeris_record.Crs = static_cast<double>(read_navigation_signed(data_bits, CNAV2_CRS));
    ephemeris_record.Crs *= CNAV_CRS_LSB;
    ephemeris_record.Crc = static_cast<double>(read_navigation_signed(data_bits, CNAV2_CRC));
    ephemeris_record.Crc *= CNAV_CRC_LSB;
    ephemeris_record.Cus = static_cast<double>(read_navigation_signed(data_bits, CNAV2_CUS));
    ephemeris_record.Cus *= CNAV_CUS_LSB;
    ephemeris_record.Cuc = static_cast<double>(read_navigation_signed(data_bits, CNAV2_CUC));
    ephemeris_record.Cuc *= CNAV_CUC_LSB;
    ephemeris_record.URANED0 = static_cast<int32_t>(read_navigation_signed(data_bits, CNAV2_URA_NED0));
    ephemeris_record.URANED1 = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV2_URA_NED1));
    ephemeris_record.URANED2 = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV2_URA_NED2));
    ephemeris_record.af0 = static_cast<double>(read_navigation_signed(data_bits, CNAV2_AF0));
    ephemeris_record.af0 *= CNAV_AF0_LSB;
    ephemeris_record.af1 = static_cast<double>(read_navigation_signed(data_bits, CNAV2_AF1));
    ephemeris_record.af1 *= CNAV_AF1_LSB;
    ephemeris_record.af2 = static_cast<double>(read_navigation_signed(data_bits, CNAV2_AF2));
    ephemeris_record.af2 *= CNAV_AF2_LSB;
    ephemeris_record.TGD = static_cast<double>(read_navigation_signed(data_bits, CNAV2_TGD));
    // TODO: Check invalid vaue
    ephemeris_record.TGD *= CNAV_TGD_LSB;

    // TODO
    // ephemeris_record.ISCL1CP= static_cast<double>(read_navigation_signed(data_bits, CNAV2_ISC_L1));
    // ephemeris_record.ISCL2 = static_cast<double>(read_navigation_signed(data_bits, CNAV_ISCL2));
    // ephemeris_record.ISCL2 *= CNAV_ISCL2_LSB;
    ephemeris_record.WNop = static_cast<int32_t>(read_navigation_unsigned(data_bits, CNAV2_WNOP));
}

void Gps_CNAV2_Navigation_Message::decode_sf3(uint16_t toi, const std::bitset<GPS_L1C_SF_3_DATA_BITS>& data_bits)
{
}

Gps_CNAV2_Ephemeris Gps_CNAV2_Navigation_Message::get_ephemeris() const
{
    return ephemeris_record;
}

bool Gps_CNAV2_Navigation_Message::have_new_ephemeris()
{
    if (b_flag_ephemeris)
        {
            b_flag_ephemeris = false;
            return true;
        }

    return false;
}
