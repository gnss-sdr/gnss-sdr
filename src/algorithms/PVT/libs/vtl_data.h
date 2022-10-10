/*!
 * \file vtl_data.h
 * \brief Class that exchange information to and from the Vector Tracking Loop (VTL) Kalman filter engine
 * \author Javier Arribas, 2022. jarribas(at)cttc.es
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

#include <armadillo>
#include <cstdint>
#include <string>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


class Vtl_Data
{
public:
    Vtl_Data();
    void init_storage(int n_sats);

    arma::mat sat_p;            // Satellite ECEF Position [m]
    arma::mat sat_v;            // Satellite Velocity [m/s]
    arma::mat sat_dts;          // Satellite clock bias and drift [s,s/s]
    arma::colvec sat_var;          // sat position and clock error variance [m^2]
    arma::colvec sat_health_flag;  // sat health flag (0 is ok)
    arma::colvec sat_CN0_dB_hz;     // sat CN0 in dB-Hz
    int sat_number;             // on-view sat number
    
    arma::colvec pr_m;                // Satellite Code pseudoranges [m]
    arma::colvec doppler_hz;          // satellite Carrier Dopplers [Hz]
    arma::colvec carrier_phase_rads;  // satellite accumulated carrier phases [rads]

    arma::mat rx_p;            // Receiver ENU Position [m]
    arma::mat rx_v;            // Receiver Velocity [m/s]
    arma::mat rx_pvt_var;      // Receiver position, velocity and time VARIANCE [m/s]
    arma::mat rx_dts;          // Receiver clock bias and drift [s,s/s]
    arma::colvec rx_var;       // Receiver position and clock error variance [m^2]
    arma::colvec kf_state;     // KF STATE
    // time handling
    double epoch_tow_s;       // current observation RX time [s]
    uint64_t sample_counter;  // current sample counter associated with RX time [samples from start]
    void debug_print();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_VTL_DATA_H
