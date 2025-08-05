/*!
 * \file vtl_data.c
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


#include "vtl_data.h"
#include "armadillo"
#include "vector"

Vtl_Data::Vtl_Data() = default;

void Vtl_Data::init_storage(int N_sv)
{
    rx_ch = arma::vec(N_sv);
    rx_ch2 = arma::vec(N_sv);
    rx_p = arma::mat(1, 3);
    rx_v = arma::mat(1, 3);
    rx_clk = arma::mat(1, 3);

    sv_id = arma::vec(N_sv);
    sv_p = arma::mat(N_sv, 3);
    sv_v = arma::mat(N_sv, 3);
    sv_clk = arma::mat(N_sv, 2);
    sv_elev = arma::vec(N_sv);
    tropo_bias = arma::vec(N_sv);
    iono_bias = arma::vec(N_sv);
    code_bias = arma::vec(N_sv);
    band = arma::vec(N_sv);
    code_freq = arma::vec(N_sv);
    CN0_dB_hz = arma::vec(N_sv);
    ch_sample_counter = arma::vec(N_sv);
    ch2_sample_counter = arma::vec(N_sv);
    ionoopt = 0;

    obs_pr = arma::vec(N_sv);
    obs_prr = arma::vec(N_sv);

    active_ch = arma::vec(N_sv);
    past_active_ch = arma::vec(N_sv);
    new_ch = arma::vec(N_sv);
    old_ch = arma::vec(N_sv);

    enable_VDLL = arma::vec(N_sv);
    enable_VPLL = arma::vec(N_sv);

    rx_time = 0.0;
    dt_s = 0.0;
}

void Vtl_Data::clear_storage()
{
    active_ch.zeros();
    new_ch.zeros();
    old_ch.zeros();
    enable_VDLL.zeros();
    enable_VPLL.zeros();
}