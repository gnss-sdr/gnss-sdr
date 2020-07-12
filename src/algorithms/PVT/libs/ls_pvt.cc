/*!
 * \file ls_pvt.cc
 * \brief Implementation of a base class for Least Squares PVT solutions
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

#include "ls_pvt.h"
#include "MATH_CONSTANTS.h"
#include "geofunctions.h"
#include <glog/logging.h>
#include <stdexcept>

arma::vec Ls_Pvt::bancroftPos(const arma::mat& satpos, const arma::vec& obs)
{
    // BANCROFT Calculation of preliminary coordinates for a GPS receiver based on pseudoranges
    //          to 4 or more satellites. The ECEF coordinates are stored in satpos.
    //          The observed pseudoranges are stored in obs
    // Reference: Bancroft, S. (1985) An Algebraic Solution of the GPS Equations,
    //            IEEE Trans. Aerosp. and Elec. Systems, AES-21, Issue 1, pp. 56--59
    // Based on code by:
    // Kai Borre 04-30-95, improved by C.C. Goad 11-24-96
    //
    //  Test values to use in debugging
    //  B_pass =[ -11716227.778 -10118754.628  21741083.973 22163882.029;
    //            -12082643.974 -20428242.179  11741374.154 21492579.823;
    //             14373286.650 -10448439.349  19596404.858 21492492.771;
    //             10278432.244 -21116508.618 -12689101.970 25284588.982 ];
    //  Solution:    595025.053  -4856501.221  4078329.981
    //
    //  Test values to use in debugging
    //  B_pass = [14177509.188  -18814750.650   12243944.449  21119263.116;
    //            15097198.146   -4636098.555   21326705.426  22527063.486;
    //            23460341.997   -9433577.991    8174873.599  23674159.579;
    //            -8206498.071  -18217989.839   17605227.065  20951643.862;
    //             1399135.830  -17563786.820   19705534.862  20155386.649;
    //             6995655.459  -23537808.269   -9927906.485  24222112.972 ];
    //  Solution:     596902.683   -4847843.316    4088216.740

    arma::vec pos = arma::zeros(4, 1);
    arma::mat B_pass = arma::zeros(obs.size(), 4);
    B_pass.submat(0, 0, obs.size() - 1, 2) = satpos;
    B_pass.col(3) = obs;

    arma::mat B;
    arma::mat BBB;
    double traveltime = 0;
    for (int iter = 0; iter < 2; iter++)
        {
            B = B_pass;
            int m = arma::size(B, 0);
            for (int i = 0; i < m; i++)
                {
                    int x = B(i, 0);
                    int y = B(i, 1);
                    if (iter == 0)
                        {
                            traveltime = 0.072;
                        }
                    else
                        {
                            int z = B(i, 2);
                            double rho = (x - pos(0)) * (x - pos(0)) + (y - pos(1)) * (y - pos(1)) + (z - pos(2)) * (z - pos(2));
                            traveltime = sqrt(rho) / SPEED_OF_LIGHT_M_S;
                        }
                    double angle = traveltime * 7.292115147e-5;
                    double cosa = cos(angle);
                    double sina = sin(angle);
                    B(i, 0) = cosa * x + sina * y;
                    B(i, 1) = -sina * x + cosa * y;
                }  // % i-loop

            if (m > 3)
                {
                    BBB = arma::inv(B.t() * B) * B.t();
                }
            else
                {
                    BBB = arma::inv(B);
                }
            arma::vec e = arma::ones(m, 1);
            arma::vec alpha = arma::zeros(m, 1);
            for (int i = 0; i < m; i++)
                {
                    alpha(i) = lorentz(B.row(i).t(), B.row(i).t()) / 2.0;
                }
            arma::mat BBBe = BBB * e;
            arma::mat BBBalpha = BBB * alpha;
            double a = lorentz(BBBe, BBBe);
            double b = lorentz(BBBe, BBBalpha) - 1;
            double c = lorentz(BBBalpha, BBBalpha);
            double root = sqrt(b * b - a * c);
            arma::vec r = {(-b - root) / a, (-b + root) / a};
            arma::mat possible_pos = arma::zeros(4, 2);
            for (int i = 0; i < 2; i++)
                {
                    possible_pos.col(i) = r(i) * BBBe + BBBalpha;
                    possible_pos(3, i) = -possible_pos(3, i);
                }

            arma::vec abs_omc = arma::zeros(2, 1);
            for (int j = 0; j < m; j++)
                {
                    for (int i = 0; i < 2; i++)
                        {
                            double c_dt = possible_pos(3, i);
                            double calc = arma::norm(satpos.row(i).t() - possible_pos.col(i).rows(0, 2)) + c_dt;
                            double omc = obs(j) - calc;
                            abs_omc(i) = std::abs(omc);
                        }
                }  // % j-loop

            // discrimination between roots
            if (abs_omc(0) > abs_omc(1))
                {
                    pos = possible_pos.col(1);
                }
            else
                {
                    pos = possible_pos.col(0);
                }
        }  // % iter loop
    return pos;
}


double Ls_Pvt::lorentz(const arma::vec& x, const arma::vec& y)
{
    // LORENTZ  Calculates the Lorentz inner product of the two
    //          4 by 1 vectors x and y
    // Based on code by:
    // Kai Borre 04-22-95
    //
    //  M = diag([1 1 1 -1]);
    //  p = x'*M*y;

    return (x(0) * y(0) + x(1) * y(1) + x(2) * y(2) - x(3) * y(3));
}


arma::vec Ls_Pvt::leastSquarePos(const arma::mat& satpos, const arma::vec& obs, const arma::vec& w_vec)
{
    /* Computes the Least Squares Solution.
     *   Inputs:
     *       satpos      - Satellites positions in ECEF system: [X; Y; Z;]
     *       obs         - Observations - the pseudorange measurements to each satellite
     *       w           - weight vector
     *
     *   Returns:
     *       pos         - receiver position and receiver clock error
     *                   (in ECEF system: [X, Y, Z, dt])
     */

    //=== Initialization =======================================================
    constexpr double GPS_STARTOFFSET_MS = 68.802;  // [ms] Initial signal travel time
    int nmbOfIterations = 10;                      // TODO: include in config
    int nmbOfSatellites;
    nmbOfSatellites = satpos.n_cols;  // Armadillo
    arma::mat w = arma::zeros(nmbOfSatellites, nmbOfSatellites);
    w.diag() = w_vec;  // diagonal weight matrix

    std::array<double, 3> rx_pos = this->get_rx_pos();
    arma::vec pos = {rx_pos[0], rx_pos[1], rx_pos[2], 0};  // time error in METERS (time x speed)
    arma::mat A;
    arma::mat omc;
    A = arma::zeros(nmbOfSatellites, 4);
    omc = arma::zeros(nmbOfSatellites, 1);
    arma::mat X = satpos;
    arma::vec Rot_X;
    double rho2;
    double traveltime;
    double trop = 0.0;
    double dlambda;
    double dphi;
    double h;
    arma::vec x;

    //=== Iteratively find receiver position ===================================
    for (int iter = 0; iter < nmbOfIterations; iter++)
        {
            for (int i = 0; i < nmbOfSatellites; i++)
                {
                    if (iter == 0)
                        {
                            // --- Initialize variables at the first iteration -------------
                            Rot_X = X.col(i);  // Armadillo
                            trop = 0.0;
                        }
                    else
                        {
                            // --- Update equations ----------------------------------------
                            rho2 = (X(0, i) - pos(0)) *
                                       (X(0, i) - pos(0)) +
                                   (X(1, i) - pos(1)) *
                                       (X(1, i) - pos(1)) +
                                   (X(2, i) - pos(2)) *
                                       (X(2, i) - pos(2));
                            traveltime = sqrt(rho2) / SPEED_OF_LIGHT_M_S;

                            // --- Correct satellite position (do to earth rotation) -------
                            std::array<double, 3> rot_x = Ls_Pvt::rotateSatellite(traveltime, {X(0, i), X(1, i), X(2, i)});
                            Rot_X = {rot_x[0], rot_x[1], rot_x[2]};

                            // -- Find DOA and range of satellites
                            double* azim = nullptr;
                            double* elev = nullptr;
                            double* dist = nullptr;
                            topocent(azim, elev, dist, pos.subvec(0, 2), Rot_X - pos.subvec(0, 2));

                            if (traveltime < 0.1 && nmbOfSatellites > 3)
                                {
                                    // --- Find receiver's height
                                    togeod(&dphi, &dlambda, &h, 6378137.0, 298.257223563, pos(0), pos(1), pos(2));
                                    // Add troposphere correction if the receiver is below the troposphere
                                    if (h > 15000)
                                        {
                                            // receiver is above the troposphere
                                            trop = 0.0;
                                        }
                                    else
                                        {
                                            // --- Find delay due to troposphere (in meters)
                                            Ls_Pvt::tropo(&trop, sin(elev[0] * GNSS_PI / 180.0), h / 1000.0, 1013.0, 293.0, 50.0, 0.0, 0.0, 0.0);
                                            if (trop > 5.0)
                                                {
                                                    trop = 0.0;  // check for erratic values
                                                }
                                        }
                                }
                        }
                    // --- Apply the corrections ----------------------------------------
                    omc(i) = (obs(i) - norm(Rot_X - pos.subvec(0, 2), 2) - pos(3) - trop);  // Armadillo

                    // -- Construct the A matrix ---------------------------------------
                    // Armadillo
                    A(i, 0) = (-(Rot_X(0) - pos(0))) / obs(i);
                    A(i, 1) = (-(Rot_X(1) - pos(1))) / obs(i);
                    A(i, 2) = (-(Rot_X(2) - pos(2))) / obs(i);
                    A(i, 3) = 1.0;
                }

            // -- Find position update ---------------------------------------------
            x = arma::solve(w * A, w * omc);  // Armadillo

            // -- Apply position update --------------------------------------------
            pos = pos + x;
            if (arma::norm(x, 2) < 1e-4)
                {
                    break;  // exit the loop because we assume that the LS algorithm has converged (err < 0.1 cm)
                }
        }

    // check the consistency of the PVT solution
    if (((fabs(pos(3)) * 1000.0) / SPEED_OF_LIGHT_M_S) > GPS_STARTOFFSET_MS * 2)
        {
            LOG(WARNING) << "Receiver time offset out of range! Estimated RX Time error [s]:" << pos(3) / SPEED_OF_LIGHT_M_S;
            throw std::runtime_error("Receiver time offset out of range!");
        }
    return pos;
}


int Ls_Pvt::tropo(double* ddr_m, double sinel, double hsta_km, double p_mb, double t_kel, double hum, double hp_km, double htkel_km, double hhum_km)
{
    /*   Inputs:
           sinel     - sin of elevation angle of satellite
           hsta_km   - height of station in km
           p_mb      - atmospheric pressure in mb at height hp_km
           t_kel     - surface temperature in degrees Kelvin at height htkel_km
           hum       - humidity in % at height hhum_km
           hp_km     - height of pressure measurement in km
           htkel_km  - height of temperature measurement in km
           hhum_km   - height of humidity measurement in km

       Outputs:
           ddr_m     - range correction (meters)

     Reference
     Goad, C.C. & Goodman, L. (1974) A Modified Hopfield Tropospheric
     Refraction Correction Model. Paper presented at the
     American Geophysical Union Annual Fall Meeting, San
     Francisco, December 12-17

     Translated to C++ by Carles Fernandez from a Matlab implementation by Kai Borre
     */

    const double a_e = 6378.137;  // semi-major axis of earth ellipsoid
    const double b0 = 7.839257e-5;
    const double tlapse = -6.5;
    const double em = -978.77 / (2.8704e6 * tlapse * 1.0e-5);
    const double tkhum = t_kel + tlapse * (hhum_km - htkel_km);
    const double atkel = 7.5 * (tkhum - 273.15) / (237.3 + tkhum - 273.15);
    const double e0 = 0.0611 * hum * pow(10, atkel);
    const double tksea = t_kel - tlapse * htkel_km;
    const double tkelh = tksea + tlapse * hhum_km;
    const double e0sea = e0 * pow((tksea / tkelh), (4 * em));
    const double tkelp = tksea + tlapse * hp_km;
    const double psea = p_mb * pow((tksea / tkelp), em);

    if (sinel < 0)
        {
            sinel = 0.0;
        }

    double tropo_delay = 0.0;
    bool done = false;
    double refsea = 77.624e-6 / tksea;
    double htop = 1.1385e-5 / refsea;
    refsea = refsea * psea;
    double ref = refsea * pow(((htop - hsta_km) / htop), 4);

    double a;
    double b;
    double rtop;

    while (true)
        {
            rtop = pow((a_e + htop), 2) - pow((a_e + hsta_km), 2) * (1 - pow(sinel, 2));

            // check to see if geometry is crazy
            if (rtop < 0)
                {
                    rtop = 0;
                }

            rtop = sqrt(rtop) - (a_e + hsta_km) * sinel;

            a = -sinel / (htop - hsta_km);
            b = -b0 * (1 - pow(sinel, 2)) / (htop - hsta_km);

            arma::vec rn = arma::vec(8);
            rn.zeros();

            for (int i = 0; i < 8; i++)
                {
                    rn(i) = pow(rtop, (i + 1 + 1));
                }

            arma::rowvec alpha = {2 * a, 2 * pow(a, 2) + 4 * b / 3, a * (pow(a, 2) + 3 * b),
                pow(a, 4) / 5 + 2.4 * pow(a, 2) * b + 1.2 * pow(b, 2), 2 * a * b * (pow(a, 2) + 3 * b) / 3,
                pow(b, 2) * (6 * pow(a, 2) + 4 * b) * 1.428571e-1, 0, 0};

            if (pow(b, 2) > 1.0e-35)
                {
                    alpha(6) = a * pow(b, 3) / 2;
                    alpha(7) = pow(b, 4) / 9;
                }

            double dr = rtop;
            arma::mat aux_ = alpha * rn;
            dr = dr + aux_(0, 0);
            tropo_delay = tropo_delay + dr * ref * 1000;

            if (done == true)
                {
                    *ddr_m = tropo_delay;
                    break;
                }

            done = true;
            refsea = (371900.0e-6 / tksea - 12.92e-6) / tksea;
            htop = 1.1385e-5 * (1255 / tksea + 0.05) / refsea;
            ref = refsea * e0sea * pow(((htop - hsta_km) / htop), 4);
        }
    return 0;
}


std::array<double, 3> Ls_Pvt::rotateSatellite(double traveltime, const std::array<double, 3>& X_sat)
{
    /*
     *  Returns rotated satellite ECEF coordinates due to Earth
     * rotation during signal travel time
     *
     *   Inputs:
     *       travelTime  - signal travel time
     *       X_sat       - satellite's ECEF coordinates
     *
     *   Returns:
     *       X_sat_rot   - rotated satellite's coordinates (ECEF)
     */

    const double omegatau = GNSS_OMEGA_EARTH_DOT * traveltime;
    const double cosomg = cos(omegatau);
    const double sinomg = sin(omegatau);
    const double x = cosomg * X_sat[0] + sinomg * X_sat[1];
    const double y = -sinomg * X_sat[0] + cosomg * X_sat[1];

    std::array<double, 3> X_sat_rot = {x, y, X_sat[2]};
    return X_sat_rot;
}


double Ls_Pvt::get_gdop() const
{
    return 0.0;  // not implemented
}


double Ls_Pvt::get_pdop() const
{
    return 0.0;  // not implemented
}


double Ls_Pvt::get_hdop() const
{
    return 0.0;  // not implemented
}


double Ls_Pvt::get_vdop() const
{
    return 0.0;  // not implemented
}
