/*!
 * \file vtl_data.c
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


#include "vtl_data.h"

Vtl_Data::Vtl_Data()
{
    epoch_tow_s = 0;
}

void Vtl_Data::init_storage(int n_sats)
{
    sat_p = arma::mat(n_sats, 3);
    sat_v = arma::mat(n_sats, 3);
    sat_dts = arma::mat(n_sats, 2);
    sat_var = arma::vec(n_sats);
    sat_health_flag = arma::vec(n_sats);
    pr_m = arma::vec(n_sats);
    doppler_hz = arma::vec(n_sats);
    carrier_phase_rads = arma::vec(n_sats);
    epoch_tow_s = 0;
}

void Vtl_Data::debug_print()
{
    std::cout << "vtl_data debug print at TOW: " << epoch_tow_s << "\n ";
    sat_p.print("VTL Sat Positions");
    sat_v.print("VTL Sat Velocities");
    sat_dts.print("VTL Sat clocks");
    sat_var.print("VTL Sat clock variances");
    sat_health_flag.print("VTL Sat health");

    pr_m.print("Satellite Code pseudoranges [m]");
    doppler_hz.print("satellite Carrier Dopplers [Hz]");
    carrier_phase_rads.print("satellite accumulated carrier phases [rads]");
}
