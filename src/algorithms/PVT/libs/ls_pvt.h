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


#include "pvt_solution.h"

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

private:
    /*!
     * \brief Computes the Lorentz inner product between two vectors
     */
    double lorentz(const arma::vec& x, const arma::vec& y);
};

#endif
