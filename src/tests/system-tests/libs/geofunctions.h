/*!
 * \file geofunctions.h
 * \brief A set of coordinate transformations functions and helpers,
 * some of them migrated from MATLAB, for geographic information systems.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GEOFUNCTIONS_H
#define GNSS_SDR_GEOFUNCTIONS_H

#include <armadillo>

arma::mat Skew_symmetric(const arma::vec &a);  //!< Calculates skew-symmetric matrix

double WGS84_g0(double Lat_rad);

double WGS84_geocentric_radius(double Lat_geodetic_rad);

/*!
 * \brief  Transformation of vector dx into topocentric coordinate
 *      system with origin at x
 *       Inputs:
 *         x           - vector origin coordinates (in ECEF system [X; Y; Z;])
 *         dx          - vector ([dX; dY; dZ;]).
 *
 *        Outputs:
 *          D           - vector length. Units like the input
 *          Az          - azimuth from north positive clockwise, degrees
 *          El          - elevation angle, degrees
 *
 *          Based on a Matlab function by Kai Borre
 */
int topocent(double *Az, double *El, double *D, const arma::vec &x, const arma::vec &dx);

/*!
 * \brief Subroutine to calculate geodetic coordinates latitude, longitude,
 *   height given Cartesian coordinates X,Y,Z, and reference ellipsoid
 *   values semi-major axis (a) and the inverse of flattening (finv).
 *
 *   The output units of angular quantities will be in decimal degrees
 *   (15.5 degrees not 15 deg 30 min). The output units of h will be the
 *   same as the units of X,Y,Z,a.
 *
 *         Inputs:
 *             a           - semi-major axis of the reference ellipsoid
 *             finv        - inverse of flattening of the reference ellipsoid
 *             X,Y,Z       - Cartesian coordinates
 *
 *         Outputs:
 *             dphi        - latitude
 *             dlambda     - longitude
 *             h           - height above reference ellipsoid
 *
 *             Based in a Matlab function by Kai Borre
 */
int togeod(double *dphi, double *dlambda, double *h, double a, double finv, double X, double Y, double Z);

arma::mat Gravity_ECEF(const arma::vec &r_eb_e);  //!< Calculates acceleration due to gravity resolved about ECEF-frame

/*!
 * \brief Conversion of Cartesian coordinates (X,Y,Z) to geographical
 *  coordinates (latitude, longitude, h) on a selected reference ellipsoid.
 *
 *    Choices of Reference Ellipsoid for Geographical Coordinates
 *             0. International Ellipsoid 1924
 *             1. International Ellipsoid 1967
 *             2. World Geodetic System 1972
 *             3. Geodetic Reference System 1980
 *             4. World Geodetic System 1984
 */
arma::vec cart2geo(const arma::vec &XYZ, int elipsoid_selection);

arma::vec LLH_to_deg(arma::vec &LLH);

double degtorad(double angleInDegrees);

double radtodeg(double angleInRadians);

double mstoknotsh(double MetersPerSeconds);

double mstokph(double Kph);

arma::vec CTM_to_Euler(arma::mat &C);

arma::mat Euler_to_CTM(const arma::vec &eul);

void ECEF_to_Geo(const arma::vec &r_eb_e, const arma::vec &v_eb_e, const arma::mat &C_b_e, arma::vec &LLH, arma::vec &v_eb_n, arma::mat &C_b_n);


/*!
 * \brief From Geographic to ECEF coordinates
 *
 *  Inputs:
 *    LLH           latitude (rad), longitude (rad), height (m)
 *    v_eb_n        velocity of body frame w.r.t. ECEF frame, resolved along
 *                  north, east, and down (m/s)
 *    C_b_n         body-to-NED coordinate transformation matrix
 *
 *  Outputs:
 *    r_eb_e        Cartesian position of body frame w.r.t. ECEF frame, resolved
 *                 along ECEF-frame axes (m)
 *    v_eb_e        velocity of body frame w.r.t. ECEF frame, resolved along
 *                  ECEF-frame axes (m/s)
 *    C_b_e         body-to-ECEF-frame coordinate transformation matrix
 *
 */
void Geo_to_ECEF(const arma::vec &LLH, const arma::vec &v_eb_n, const arma::mat &C_b_n, arma::vec &r_eb_e, arma::vec &v_eb_e, arma::mat &C_b_e);


/*!
 * \brief Converts curvilinear to Cartesian position and velocity
 * resolving axes from NED to ECEF
 * This function created 11/4/2012 by Paul Groves
 *
 * Inputs:
 *    L_b           latitude (rad)
 *    lambda_b      longitude (rad)
 *    h_b           height (m)
 *    v_eb_n        velocity of body frame w.r.t. ECEF frame, resolved along
 *                  north, east, and down (m/s)
 *
 * Outputs:
 *    r_eb_e        Cartesian position of body frame w.r.t. ECEF frame, resolved
 *                  along ECEF-frame axes (m)
 *    v_eb_e        velocity of body frame w.r.t. ECEF frame, resolved along
 *                  ECEF-frame axes (m/s)
 */
void pv_Geo_to_ECEF(double L_b, double lambda_b, double h_b, const arma::vec &v_eb_n, arma::vec &r_eb_e, arma::vec &v_eb_e);

/*!
 * \brief The Haversine formula determines the great-circle distance between two points on a sphere given their longitudes and latitudes.
 */
double great_circle_distance(double lat1, double lon1, double lat2, double lon2);


#endif
