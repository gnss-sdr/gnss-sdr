/*!
 * \file ls_pvt.cc
 * \brief Implementation of a base class for Least Squares PVT solutions
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

#include "ls_pvt.h"
#include <exception>
#include "GPS_L1_CA.h"
#include <gflags/gflags.h>
#include <glog/logging.h>


using google::LogMessage;


Ls_Pvt::Ls_Pvt() : Pvt_Solution()
{
    d_x_m = 0.0;
    d_y_m = 0.0;
    d_z_m = 0.0;
}

arma::vec Ls_Pvt::leastSquarePos(const arma::mat & satpos, const arma::vec & obs, const arma::mat & w)
{
    /* Computes the Least Squares Solution.
     *   Inputs:
     *       satpos      - Satellites positions in ECEF system: [X; Y; Z;]
     *       obs         - Observations - the pseudorange measurements to each satellite
     *       w           - weigths vector
     *
     *   Returns:
     *       pos         - receiver position and receiver clock error
     *                   (in ECEF system: [X, Y, Z, dt])
     */

    //=== Initialization =======================================================
    int nmbOfIterations = 10; // TODO: include in config
    int nmbOfSatellites;
    nmbOfSatellites = satpos.n_cols;    //Armadillo
    arma::vec pos = "0.0 0.0 0.0 0.0";
    arma::mat A;
    arma::mat omc;
    arma::mat az;
    arma::mat el;
    A = arma::zeros(nmbOfSatellites, 4);
    omc = arma::zeros(nmbOfSatellites, 1);
    az = arma::zeros(1, nmbOfSatellites);
    el = arma::zeros(1, nmbOfSatellites);
    arma::mat X = satpos;
    arma::vec Rot_X;
    double rho2;
    double traveltime;
    double trop = 0.0;
    double dlambda;
    double dphi;
    double h;
    arma::mat mat_tmp;
    arma::vec x;

    //=== Iteratively find receiver position ===================================
    for (int iter = 0; iter < nmbOfIterations; iter++)
        {
            for (int i = 0; i < nmbOfSatellites; i++)
                {
                    if (iter == 0)
                        {
                            //--- Initialize variables at the first iteration --------------
                            Rot_X = X.col(i); //Armadillo
                            trop = 0.0;
                        }
                    else
                        {
                            //--- Update equations -----------------------------------------
                            rho2 = (X(0, i) - pos(0)) *
                                   (X(0, i) - pos(0)) + (X(1, i) - pos(1)) *
                                   (X(1, i) - pos(1)) + (X(2, i) - pos(2)) *
                                   (X(2, i) - pos(2));
                            traveltime = sqrt(rho2) / GPS_C_m_s;

                            //--- Correct satellite position (do to earth rotation) --------
                            Rot_X = Ls_Pvt::rotateSatellite(traveltime, X.col(i)); //armadillo

                            //--- Find DOA and range of satellites
                            Ls_Pvt::topocent(&d_visible_satellites_Az[i],
                                     &d_visible_satellites_El[i],
                                     &d_visible_satellites_Distance[i],
                                     pos.subvec(0,2),
                                     Rot_X - pos.subvec(0, 2));
                            if(traveltime < 0.1 && nmbOfSatellites > 3)
                                {
                                    //--- Find receiver's height
                                    Ls_Pvt::togeod(&dphi, &dlambda, &h, 6378137.0, 298.257223563, pos(0), pos(1), pos(2));
                                    // Add troposphere correction if the receiver is below the troposphere
                                    if (h > 15000)
                                        {
                                            //receiver is above the troposphere
                                            trop = 0.0;
                                        }
                                    else
                                        {
                                            //--- Find delay due to troposphere (in meters)
                                            Ls_Pvt::tropo(&trop, sin(d_visible_satellites_El[i] * GPS_PI / 180.0), h / 1000.0, 1013.0, 293.0, 50.0, 0.0, 0.0, 0.0);
                                            if(trop > 5.0 ) trop = 0.0; //check for erratic values
                                        }
                                }
                        }
                    //--- Apply the corrections ----------------------------------------
                    omc(i) = (obs(i) - norm(Rot_X - pos.subvec(0, 2), 2) - pos(3) - trop); // Armadillo

                    //--- Construct the A matrix ---------------------------------------
                    //Armadillo
                    A(i,0) = (-(Rot_X(0) - pos(0))) / obs(i);
                    A(i,1) = (-(Rot_X(1) - pos(1))) / obs(i);
                    A(i,2) = (-(Rot_X(2) - pos(2))) / obs(i);
                    A(i,3) = 1.0;
                }

            //--- Find position update ---------------------------------------------
            x = arma::solve(w*A, w*omc); // Armadillo

            //--- Apply position update --------------------------------------------
            pos = pos + x;
            if (arma::norm(x,2) < 1e-4)
            {
                break; // exit the loop because we assume that the LS algorithm has converged (err < 0.1 cm)
            }
        }

    try
    {
            //-- compute the Dilution Of Precision values
            d_Q = arma::inv(arma::htrans(A)*A);
    }
    catch(std::exception& e)
    {
            d_Q = arma::zeros(4,4);
    }
    return pos;
}
