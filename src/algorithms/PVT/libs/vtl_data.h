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
#include <map>
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

    arma::mat sat_p;            //Satellite ECEF Position [m]
    arma::mat sat_v;            //Satellite Velocity [m/s]
    arma::mat sat_dts;          //Satellite clock bias and drift [s,s/s]
    arma::vec sat_var;          //sat position and clock error variance [m^2]
    arma::vec sat_health_flag;  //sat health flag (0 is ok)

    arma::vec pr_m;                //Satellite Code pseudoranges [m]
    arma::vec doppler_hz;          //satellite Carrier Dopplers [Hz]
    arma::vec carrier_phase_rads;  //satellite accumulated carrier phases [rads]

    double epoch_tow_s;  //current observation RX time [s]
    void debug_print();
};


/** \} */
/** \} */
#endif  // GNSS_SDR_VTL_DATA_H
