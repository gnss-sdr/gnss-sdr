/*!
 * \file ls_pvt.h
 * \brief Interface of a base class for Least Squares PVT solutions
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_LS_PVT_H
#define GNSS_SDR_LS_PVT_H

#if ARMA_NO_BOUND_CHECKING
#define ARMA_NO_DEBUG 1
#endif

#include "pvt_solution.h"
#include <armadillo>
#include <array>

/*!
 * \brief Base class for the Least Squares PVT solution
 *
 */
class Ls_Pvt : public Pvt_Solution
{
public:
    Ls_Pvt() = default;

    /*!
     * \brief Computes the initial position solution based on the Bancroft algorithm
     */
    arma::vec bancroftPos(const arma::mat& satpos, const arma::vec& obs);

    /*!
     * \brief Computes the Weighted Least Squares position solution
     */
    arma::vec leastSquarePos(const arma::mat& satpos, const arma::vec& obs, const arma::vec& w_vec);

    double get_hdop() const override;
    double get_vdop() const override;
    double get_pdop() const override;
    double get_gdop() const override;

private:
    /*
     * Computes the Lorentz inner product between two vectors
     */
    double lorentz(const arma::vec& x, const arma::vec& y);

    /*
     * Tropospheric correction
     *
     *  \param[in] sinel     - sin of elevation angle of satellite
     *  \param[in] hsta_km   - height of station in km
     *  \param[in] p_mb      - atmospheric pressure in mb at height hp_km
     *  \param[in] t_kel     - surface temperature in degrees Kelvin at height htkel_km
     *  \param[in] hum       - humidity in % at height hhum_km
     *  \param[in] hp_km     - height of pressure measurement in km
     *  \param[in] htkel_km  - height of temperature measurement in km
     *  \param[in] hhum_km   - height of humidity measurement in km
     *
     *  \param[out] ddr_m     - range correction (meters)
     *
     *
     * Reference:
     * Goad, C.C. & Goodman, L. (1974) A Modified Hopfield Tropospheric
     *   Refraction Correction Model. Paper presented at the
     *   American Geophysical Union Annual Fall Meeting, San
     *   Francisco, December 12-17
     *
     * Translated to C++ by Carles Fernandez from a Matlab implementation by Kai Borre
     */
    int tropo(double* ddr_m, double sinel, double hsta_km, double p_mb, double t_kel, double hum, double hp_km, double htkel_km, double hhum_km);

    std::array<double, 3> rotateSatellite(double traveltime, const std::array<double, 3>& X_sat);
};

#endif
