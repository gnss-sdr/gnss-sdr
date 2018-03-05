/*!
 * \file ls_pvt.h
 * \brief Interface of a base class for Least Squares PVT solutions
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_LS_PVT_H_
#define GNSS_SDR_LS_PVT_H_


#include "pvt_solution.h"

/*!
 * \brief Base class for the Least Squares PVT solution
 *
 */
class Ls_Pvt : public Pvt_Solution
{
private:
    /*!
     * \brief Computes the Lorentz inner product between two vectors
     */
    double lorentz(const arma::vec& x, const arma::vec& y);

public:
    Ls_Pvt();

    /*!
     * \brief Computes the initial position solution based on the Bancroft algorithm
     */
    arma::vec bancroftPos(const arma::mat& satpos, const arma::vec& obs);

    /*!
     * \brief Computes the Weighted Least Squares position solution
     */
    arma::vec leastSquarePos(const arma::mat& satpos, const arma::vec& obs, const arma::vec& w_vec);
};

#endif
