/*!
 * \file pvt_kf.h
 * \brief Kalman Filter for Position and Velocity
 * \author Javier Arribas, 2023. jarribas(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_PVT_KF_H
#define GNSS_SDR_PVT_KF_H

#include <armadillo>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


/*!
 * \brief Kalman Filter for Position and Velocity
 *
 */
class Pvt_Kf
{
public:
    Pvt_Kf() = default;
    virtual ~Pvt_Kf() = default;
    void init_Kf(const arma::vec& p,
        const arma::vec& v,
        double update_interval_s,
        double measures_ecef_pos_sd_m,
        double measures_ecef_vel_sd_ms,
        double system_ecef_pos_sd_m,
        double system_ecef_vel_sd_ms);
    bool is_initialized() const;
    void run_Kf(const arma::vec& p, const arma::vec& v);
    void get_pv_Kf(arma::vec& p, arma::vec& v) const;
    void reset_Kf();

private:
    // Kalman Filter class variables
    arma::mat d_F;
    arma::mat d_H;
    arma::mat d_R;
    arma::mat d_Q;
    arma::mat d_P_old_old;
    arma::mat d_P_new_old;
    arma::mat d_P_new_new;
    arma::vec d_x_old_old;
    arma::vec d_x_new_old;
    arma::vec d_x_new_new;
    bool d_initialized{false};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_Pvt_Kf_H
