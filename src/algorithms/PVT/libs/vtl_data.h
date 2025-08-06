/*!
 * \file vtl_data.h
 * \brief Class that exchange information to and from the Vector Tracking Loop (VTL)
 * \author Pedro Pereira, 2025. pereirapedrocp@gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_VTL_DATA_H
#define GNSS_SDR_VTL_DATA_H
// constants definition
constexpr double Lambda_GPS_L1 = 0.1902936728;
constexpr double Lambda_GPS_L5 = 0.2548280488;
constexpr double L1E1_CODE_FREQ = 1023000;
constexpr double L5E5_CODE_FREQ = 10230000;
constexpr double RANGE_TO_FREQ_L1E1_FACTOR = 0.0034123607;  // L1E1_CODE_FREQ / SPEED_OF_LIGHT_M_S
constexpr double RANGE_TO_FREQ_L5E5_FACTOR = 0.0341236069;  // L5E5_CODE_FREQ / SPEED_OF_LIGHT_M_S

#include <armadillo>
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


class Vtl_Data
{
public:
    Vtl_Data();
    void init_storage(int N_sv);
    void clear_storage();

    arma::mat rx_ch;   // receiver channel
    arma::mat rx_ch2;  // receiver channel - second frequency
    arma::mat rx_p;    // receiver position [m]
    arma::mat rx_v;    // receiver velocity [m/s]
    arma::mat rx_clk;  // receiver clock bias and drift [s,m/s]

    arma::mat sv_id;          // satellite ID
    arma::mat sv_p;           // satellite position [m]
    arma::mat sv_v;           // satellite velocity [m/s]
    arma::mat sv_clk;         // satellite clock bias and drift [s,m/s]
    arma::colvec sv_elev;     // satellite elevation [rad]
    arma::colvec tropo_bias;  // troposphere bias [m]
    arma::colvec iono_bias;   // ionosphere bias [m]
    arma::colvec code_bias;   // code bias [m]
    arma::colvec band;        // frequency band
    arma::colvec code_freq;
    arma::colvec CN0_dB_hz;
    arma::colvec PVT_sample_counter;
    arma::colvec ch2_sample_counter;
    int ionoopt;  // ionosphere option

    arma::colvec obs_pr;   // observed pseudorange [m]
    arma::colvec obs_prr;  // observed pseudorange rate [m/s]

    arma::colvec active_ch;       // active channels
    arma::colvec past_active_ch;  // past active channels
    arma::colvec new_ch;          // new channels
    arma::colvec old_ch;          // old channels
    u_int8_t N_sv;                // number of satellites for pvt
    u_int8_t active_N_gps_ch;     // active gps channels
    u_int8_t active_N_gal_ch;     // active gps channels

    arma::colvec enable_VDLL;  // vtl VDLL closure
    arma::colvec enable_VPLL;  // vtl VDLL closure

    double rx_time;
    double dt_s;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_VTL_DATA_H
