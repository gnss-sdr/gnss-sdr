/*!
 * \file geofunctions.cc
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

#include "geofunctions.h"

const double STRP_PI = 3.1415926535898;  // Pi as defined in IS-GPS-200E

arma::mat Skew_symmetric(const arma::vec &a)
{
    arma::mat A = arma::zeros(3, 3);

    A << 0.0 << -a(2) << a(1) << arma::endr
      << a(2) << 0.0 << -a(0) << arma::endr
      << -a(1) << a(0) << 0 << arma::endr;

    //    {{0,    -a(2),  a(1)},
    //     {a(2),     0, -a(0)},
    //     {-a(1), a(0),     0}};
    return A;
}


double WGS84_g0(double Lat_rad)
{
    const double k = 0.001931853;        // normal gravity constant
    const double e2 = 0.00669438002290;  // the square of the first numerical eccentricity
    const double nge = 9.7803253359;     // normal gravity value on the equator (m/sec^2)
    double b = sin(Lat_rad);             // Lat in degrees
    b = b * b;
    double g0 = nge * (1 + k * b) / (sqrt(1 - e2 * b));
    return g0;
}


double WGS84_geocentric_radius(double Lat_geodetic_rad)
{
    // WGS84 earth model Geocentric radius (Eq. 2.88)
    const double WGS84_A = 6378137.0;         // Semi-major axis of the Earth, a [m]
    const double WGS84_IF = 298.257223563;    // Inverse flattening of the Earth
    const double WGS84_F = (1.0 / WGS84_IF);  // The flattening of the Earth
    // double WGS84_B=(WGS84_A*(1-WGS84_F));  // Semi-minor axis of the Earth [m]
    double WGS84_E = (sqrt(2 * WGS84_F - WGS84_F * WGS84_F));  // Eccentricity of the Earth

    // transverse radius of curvature
    double R_E = WGS84_A / sqrt(1 - WGS84_E * WGS84_E * sin(Lat_geodetic_rad) * sin(Lat_geodetic_rad));  // (Eq. 2.66)

    // geocentric radius at the Earth surface
    double r_eS = R_E * sqrt(cos(Lat_geodetic_rad) * cos(Lat_geodetic_rad) +
                             (1 - WGS84_E * WGS84_E) * (1 - WGS84_E * WGS84_E) * sin(Lat_geodetic_rad) * sin(Lat_geodetic_rad));  // (Eq. 2.88)
    return r_eS;
}


int topocent(double *Az, double *El, double *D, const arma::vec &x, const arma::vec &dx)
{
    double lambda;
    double phi;
    double h;
    const double dtr = STRP_PI / 180.0;
    const double a = 6378137.0;         // semi-major axis of the reference ellipsoid WGS-84
    const double finv = 298.257223563;  // inverse of flattening of the reference ellipsoid WGS-84

    // Transform x into geodetic coordinates
    togeod(&phi, &lambda, &h, a, finv, x(0), x(1), x(2));

    double cl = cos(lambda * dtr);
    double sl = sin(lambda * dtr);
    double cb = cos(phi * dtr);
    double sb = sin(phi * dtr);

    arma::mat F = {{-sl, -sb * cl, cb * cl},
        {cl, -sb * sl, cb * sl},
        {0.0, cb, sb}};

    arma::vec local_vector;

    local_vector = arma::htrans(F) * dx;

    double E = local_vector(0);
    double N = local_vector(1);
    double U = local_vector(2);

    double hor_dis;
    hor_dis = sqrt(E * E + N * N);

    if (hor_dis < 1.0E-20)
        {
            *Az = 0.0;
            *El = 90.0;
        }
    else
        {
            *Az = atan2(E, N) / dtr;
            *El = atan2(U, hor_dis) / dtr;
        }

    if (*Az < 0)
        {
            *Az = *Az + 360.0;
        }

    *D = sqrt(dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2));
    return 0;
}


int togeod(double *dphi, double *dlambda, double *h, double a, double finv, double X, double Y, double Z)
{
    *h = 0.0;
    const double tolsq = 1.e-10;  // tolerance to accept convergence
    const int maxit = 10;         // max number of iterations
    const double rtd = 180.0 / STRP_PI;

    // compute square of eccentricity
    double esq;
    if (finv < 1.0E-20)
        {
            esq = 0.0;
        }
    else
        {
            esq = (2.0 - 1.0 / finv) / finv;
        }

    // first guess
    double P = sqrt(X * X + Y * Y);  // P is distance from spin axis

    // direct calculation of longitude
    if (P > 1.0E-20)
        {
            *dlambda = atan2(Y, X) * rtd;
        }
    else
        {
            *dlambda = 0.0;
        }

    // correct longitude bound
    if (*dlambda < 0)
        {
            *dlambda = *dlambda + 360.0;
        }

    double r = sqrt(P * P + Z * Z);  // r is distance from origin (0,0,0)

    double sinphi;
    if (r > 1.0E-20)
        {
            sinphi = Z / r;
        }
    else
        {
            sinphi = 0.0;
        }
    *dphi = asin(sinphi);

    // initial value of height  =  distance from origin minus
    // approximate distance from origin to surface of ellipsoid
    if (r < 1.0E-20)
        {
            *h = 0.0;
            return 1;
        }

    *h = r - a * (1 - sinphi * sinphi / finv);

    // iterate
    double cosphi;
    double N_phi;
    double dP;
    double dZ;
    double oneesq = 1.0 - esq;

    for (int i = 0; i < maxit; i++)
        {
            sinphi = sin(*dphi);
            cosphi = cos(*dphi);

            // compute radius of curvature in prime vertical direction
            N_phi = a / sqrt(1.0 - esq * sinphi * sinphi);

            // compute residuals in P and Z
            dP = P - (N_phi + (*h)) * cosphi;
            dZ = Z - (N_phi * oneesq + (*h)) * sinphi;

            // update height and latitude
            *h = *h + (sinphi * dZ + cosphi * dP);
            *dphi = *dphi + (cosphi * dZ - sinphi * dP) / (N_phi + (*h));

            // test for convergence
            if ((dP * dP + dZ * dZ) < tolsq)
                {
                    break;
                }
            if (i == (maxit - 1))
                {
                    // LOG(WARNING) << "The computation of geodetic coordinates did not converge";
                }
        }
    *dphi = (*dphi) * rtd;
    return 0;
}


arma::mat Gravity_ECEF(const arma::vec &r_eb_e)
{
    // Parameters
    const double R_0 = 6378137.0;         // WGS84 Equatorial radius in meters
    const double mu = 3.986004418E14;     // WGS84 Earth gravitational constant (m^3 s^-2)
    const double J_2 = 1.082627E-3;       // WGS84 Earth's second gravitational constant
    const double omega_ie = 7.292115E-5;  // Earth rotation rate (rad/s)
    // Calculate distance from center of the Earth
    double mag_r = sqrt(arma::as_scalar(r_eb_e.t() * r_eb_e));
    // If the input position is 0,0,0, produce a dummy output
    arma::vec g = arma::zeros(3, 1);
    if (mag_r != 0)
        {
            // Calculate gravitational acceleration using (2.142)
            double z_scale = 5 * pow((r_eb_e(2) / mag_r), 2);
            arma::vec tmp_vec = {(1 - z_scale) * r_eb_e(0),
                (1 - z_scale) * r_eb_e(1),
                (3 - z_scale) * r_eb_e(2)};
            arma::vec gamma_ = (-mu / pow(mag_r, 3)) * (r_eb_e + 1.5 * J_2 * pow(R_0 / mag_r, 2) * tmp_vec);

            // Add centripetal acceleration using (2.133)
            g(0) = gamma_(0) + pow(omega_ie, 2) * r_eb_e(0);
            g(1) = gamma_(1) + pow(omega_ie, 2) * r_eb_e(1);
            g(2) = gamma_(2);
        }
    return g;
}


arma::vec LLH_to_deg(const arma::vec &LLH)
{
    const double rtd = 180.0 / STRP_PI;
    arma::vec deg = arma::zeros(3, 1);
    deg(0) = LLH(0) * rtd;
    deg(1) = LLH(1) * rtd;
    deg(2) = LLH(2);
    return deg;
}


double degtorad(double angleInDegrees)
{
    double angleInRadians = (STRP_PI / 180.0) * angleInDegrees;
    return angleInRadians;
}


double radtodeg(double angleInRadians)
{
    double angleInDegrees = (180.0 / STRP_PI) * angleInRadians;
    return angleInDegrees;
}


double mstoknotsh(double MetersPerSeconds)
{
    double knots = mstokph(MetersPerSeconds) * 0.539957;
    return knots;
}


double mstokph(double MetersPerSeconds)
{
    double kph = 3600.0 * MetersPerSeconds / 1e3;
    return kph;
}


arma::vec CTM_to_Euler(const arma::mat &C)
{
    // Calculate Euler angles using (2.23)
    arma::mat CTM = C;
    arma::vec eul = arma::zeros(3, 1);
    eul(0) = atan2(CTM(1, 2), CTM(2, 2));  // roll
    if (CTM(0, 2) < -1.0) CTM(0, 2) = -1.0;
    if (CTM(0, 2) > 1.0) CTM(0, 2) = 1.0;
    eul(1) = -asin(CTM(0, 2));             // pitch
    eul(2) = atan2(CTM(0, 1), CTM(0, 0));  // yaw
    return eul;
}


arma::mat Euler_to_CTM(const arma::vec &eul)
{
    // Eq.2.15
    // Euler angles to Attitude matrix is equivalent to rotate the body
    // in the three axes:
    //     arma::mat Ax= {{1,0,0}, {0,cos(Att_phi),sin(Att_phi)} ,{0,-sin(Att_phi),cos(Att_phi)}};
    //     arma::mat Ay= {{cos(Att_theta), 0, -sin(Att_theta)}, {0,1,0} , {sin(Att_theta), 0, cos(Att_theta)}};
    //     arma::mat Az= {{cos(Att_psi), sin(Att_psi), 0}, {-sin(Att_psi), cos(Att_psi), 0},{0,0,1}};
    //     arma::mat C_b_n=Ax*Ay*Az; // Attitude expressed in the LOCAL FRAME (NED)
    //    C_b_n=C_b_n.t();

    // Precalculate sines and cosines of the Euler angles
    double sin_phi = sin(eul(0));
    double cos_phi = cos(eul(0));
    double sin_theta = sin(eul(1));
    double cos_theta = cos(eul(1));
    double sin_psi = sin(eul(2));
    double cos_psi = cos(eul(2));

    // Calculate coordinate transformation matrix using (2.22)
    arma::mat C = {{cos_theta * cos_psi, cos_theta * sin_psi, -sin_theta},
        {-cos_phi * sin_psi + sin_phi * sin_theta * cos_psi, cos_phi * cos_psi + sin_phi * sin_theta * sin_psi, sin_phi * cos_theta},
        {sin_phi * sin_psi + cos_phi * sin_theta * cos_psi, -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi, cos_phi * cos_theta}};
    return C;
}


arma::vec cart2geo(const arma::vec &XYZ, int elipsoid_selection)
{
    const double a[5] = {6378388.0, 6378160.0, 6378135.0, 6378137.0, 6378137.0};
    const double f[5] = {1.0 / 297.0, 1.0 / 298.247, 1.0 / 298.26, 1.0 / 298.257222101, 1.0 / 298.257223563};

    double lambda = atan2(XYZ[1], XYZ[0]);
    double ex2 = (2.0 - f[elipsoid_selection]) * f[elipsoid_selection] / ((1.0 - f[elipsoid_selection]) * (1.0 - f[elipsoid_selection]));
    double c = a[elipsoid_selection] * sqrt(1.0 + ex2);
    double phi = atan(XYZ[2] / ((sqrt(XYZ[0] * XYZ[0] + XYZ[1] * XYZ[1]) * (1.0 - (2.0 - f[elipsoid_selection])) * f[elipsoid_selection])));

    double h = 0.1;
    double oldh = 0.0;
    double N;
    int iterations = 0;
    do
        {
            oldh = h;
            N = c / sqrt(1.0 + ex2 * (cos(phi) * cos(phi)));
            phi = atan(XYZ[2] / ((sqrt(XYZ[0] * XYZ[0] + XYZ[1] * XYZ[1]) * (1.0 - (2.0 - f[elipsoid_selection]) * f[elipsoid_selection] * N / (N + h)))));
            h = sqrt(XYZ[0] * XYZ[0] + XYZ[1] * XYZ[1]) / cos(phi) - N;
            iterations = iterations + 1;
            if (iterations > 100)
                {
                    // std::cout << "Failed to approximate h with desired precision. h-oldh= " << h - oldh;
                    break;
                }
        }
    while (std::fabs(h - oldh) > 1.0e-12);

    arma::vec LLH = {{phi, lambda, h}};  // radians
    return LLH;
}


void ECEF_to_Geo(const arma::vec &r_eb_e, const arma::vec &v_eb_e, const arma::mat &C_b_e, arma::vec &LLH, arma::vec &v_eb_n, arma::mat &C_b_n)
{
    // Compute the Latitude of the ECEF position
    LLH = cart2geo(r_eb_e, 4);  // ECEF -> WGS84 geographical

    // Calculate ECEF to Geographical coordinate transformation matrix using (2.150)
    double cos_lat = cos(LLH(0));
    double sin_lat = sin(LLH(0));
    double cos_long = cos(LLH(1));
    double sin_long = sin(LLH(1));
    // C++11 and arma >= 5.2
    //    arma::mat C_e_n = {{-sin_lat * cos_long, -sin_lat * sin_long,  cos_lat},
    //                      {-sin_long, cos_long, 0},
    //                      {-cos_lat * cos_long, -cos_lat * sin_long, -sin_lat}}; //ECEF to Geo
    arma::mat C_e_n = arma::zeros(3, 3);
    C_e_n << -sin_lat * cos_long << -sin_lat * sin_long << cos_lat << arma::endr
          << -sin_long << cos_long << 0 << arma::endr
          << -cos_lat * cos_long << -cos_lat * sin_long << -sin_lat << arma::endr;  // ECEF to Geo
    // Transform velocity using (2.73)
    v_eb_n = C_e_n * v_eb_e;

    C_b_n = C_e_n * C_b_e;  // Attitude conversion from ECEF to NED
}


void Geo_to_ECEF(const arma::vec &LLH, const arma::vec &v_eb_n, const arma::mat &C_b_n, arma::vec &r_eb_e, arma::vec &v_eb_e, arma::mat &C_b_e)
{
    // Parameters
    double R_0 = 6378137.0;      // WGS84 Equatorial radius in meters
    double e = 0.0818191908425;  // WGS84 eccentricity

    // Calculate transverse radius of curvature using (2.105)
    double R_E = R_0 / sqrt(1.0 - (e * sin(LLH(0))) * (e * sin(LLH(0))));

    // Convert position using (2.112)
    double cos_lat = cos(LLH(0));
    double sin_lat = sin(LLH(0));
    double cos_long = cos(LLH(1));
    double sin_long = sin(LLH(1));
    r_eb_e = {(R_E + LLH(2)) * cos_lat * cos_long,
        (R_E + LLH(2)) * cos_lat * sin_long,
        ((1 - e * e) * R_E + LLH(2)) * sin_lat};

    // Calculate ECEF to Geo coordinate transformation matrix using (2.150)
    // C++11 and arma>=5.2
    //    arma::mat C_e_n = {{-sin_lat * cos_long, -sin_lat * sin_long,  cos_lat},
    //                       {-sin_long,            cos_long,        0},
    //                       {-cos_lat * cos_long, -cos_lat * sin_long, -sin_lat}};
    arma::mat C_e_n = arma::zeros(3, 3);
    C_e_n << -sin_lat * cos_long << -sin_lat * sin_long << cos_lat << arma::endr
          << -sin_long << cos_long << 0 << arma::endr
          << -cos_lat * cos_long << -cos_lat * sin_long << -sin_lat << arma::endr;

    // Transform velocity using (2.73)
    v_eb_e = C_e_n.t() * v_eb_n;

    // Transform attitude using (2.15)
    C_b_e = C_e_n.t() * C_b_n;
}


void pv_Geo_to_ECEF(double L_b, double lambda_b, double h_b, const arma::vec &v_eb_n, arma::vec &r_eb_e, arma::vec &v_eb_e)
{
    // Parameters
    const double R_0 = 6378137.0;      // WGS84 Equatorial radius in meters
    const double e = 0.0818191908425;  // WGS84 eccentricity

    // Calculate transverse radius of curvature using (2.105)
    double R_E = R_0 / sqrt(1 - pow(e * sin(L_b), 2));

    // Convert position using (2.112)
    double cos_lat = cos(L_b);
    double sin_lat = sin(L_b);
    double cos_long = cos(lambda_b);
    double sin_long = sin(lambda_b);
    r_eb_e = {(R_E + h_b) * cos_lat * cos_long,
        (R_E + h_b) * cos_lat * sin_long,
        ((1 - pow(e, 2)) * R_E + h_b) * sin_lat};

    // Calculate ECEF to Geo coordinate transformation matrix using (2.150)
    arma::mat C_e_n = arma::zeros(3, 3);
    C_e_n << -sin_lat * cos_long << -sin_lat * sin_long << cos_lat << arma::endr
          << -sin_long << cos_long << 0 << arma::endr
          << -cos_lat * cos_long << -cos_lat * sin_long << -sin_lat << arma::endr;

    // Transform velocity using (2.73)
    v_eb_e = C_e_n.t() * v_eb_n;
}


double great_circle_distance(double lat1, double lon1, double lat2, double lon2)
{
    // The Haversine formula determines the great-circle distance between two points on a sphere given their longitudes and latitudes.
    // generally used geo measurement function
    double R = 6378.137;  // Radius of earth in KM
    double dLat = lat2 * STRP_PI / 180.0 - lat1 * STRP_PI / 180.0;
    double dLon = lon2 * STRP_PI / 180.0 - lon1 * STRP_PI / 180.0;
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1 * STRP_PI / 180.0) * cos(lat2 * STRP_PI / 180.0) *
                   sin(dLon / 2) * sin(dLon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double d = R * c;
    return d * 1000.0;  // meters
}


void cart2utm(const arma::vec &r_eb_e, int zone, arma::vec &r_enu)
{
    // Transformation of (X,Y,Z) to (E,N,U) in UTM, zone 'zone'
    //
    //   Inputs:
    //       r_eb_e       - Cartesian coordinates. Coordinates are referenced
    //                   with respect to the International Terrestrial Reference
    //                   Frame 1996 (ITRF96)
    //       zone        - UTM zone of the given position
    //
    //   Outputs:
    //       r_enu      - UTM coordinates (Easting, Northing, Uping)
    //
    // Originally written in Matlab by Kai Borre, Nov. 1994
    // Implemented in C++ by J.Arribas
    //
    // This implementation is based upon
    // O. Andersson & K. Poder (1981) Koordinattransformationer
    //  ved Geod\ae{}tisk Institut. Landinspekt\oe{}ren
    //  Vol. 30: 552--571 and Vol. 31: 76
    //
    // An excellent, general reference (KW) is
    // R. Koenig & K.H. Weise (1951) Mathematische Grundlagen der
    //  h\"oheren Geod\"asie und Kartographie.
    //  Erster Band, Springer Verlag
    //
    // Explanation of variables used:
    // f	   flattening of ellipsoid
    // a	   semi major axis in m
    // m0	   1 - scale at central meridian; for UTM 0.0004
    // Q_n	   normalized meridian quadrant
    // E0	   Easting of central meridian
    // L0	   Longitude of central meridian
    // bg	   constants for ellipsoidal geogr. to spherical geogr.
    // gb	   constants for spherical geogr. to ellipsoidal geogr.
    // gtu	   constants for ellipsoidal N, E to spherical N, E
    // utg	   constants for spherical N, E to ellipoidal N, E
    // tolutm	tolerance for utm, 1.2E-10*meridian quadrant
    // tolgeo	tolerance for geographical, 0.00040 second of arc
    //
    // B, L refer to latitude and longitude. Southern latitude is negative
    // International ellipsoid of 1924, valid for ED50

    double a = 6378388.0;
    double f = 1.0 / 297.0;
    double ex2 = (2.0 - f) * f / ((1.0 - f) * (1.0 - f));
    double c = a * sqrt(1.0 + ex2);
    arma::vec vec = r_eb_e;
    vec(2) = vec(2) - 4.5;
    double alpha = 0.756e-6;
    arma::mat R = {{1.0, -alpha, 0.0}, {alpha, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    arma::vec trans = {89.5, 93.8, 127.6};
    double scale = 0.9999988;
    arma::vec v = scale * R * vec + trans;  // coordinate vector in ED50
    double L = atan2(v(1), v(0));
    double N1 = 6395000.0;                                                                   // preliminary value
    double B = atan2(v(2) / ((1.0 - f) * (1.0 - f) * N1), arma::norm(v.subvec(0, 1)) / N1);  // preliminary value
    double U = 0.1;
    double oldU = 0.0;
    int iterations = 0;
    while (fabs(U - oldU) > 1.0E-4)
        {
            oldU = U;
            N1 = c / sqrt(1.0 + ex2 * (cos(B) * cos(B)));
            B = atan2(v(2) / ((1.0 - f) * (1.0 - f) * N1 + U), arma::norm(v.subvec(0, 1)) / (N1 + U));
            U = arma::norm(v.subvec(0, 1)) / cos(B) - N1;
            iterations = iterations + 1;
            if (iterations > 100)
                {
                    std::cout << "Failed to approximate U with desired precision. U-oldU:" << U - oldU << std::endl;
                    break;
                }
        }
    // Normalized meridian quadrant, KW p. 50 (96), p. 19 (38b), p. 5 (21)
    double m0 = 0.0004;
    double n = f / (2.0 - f);
    double m = n * n * (1.0 / 4.0 + n * n / 64.0);
    double w = (a * (-n - m0 + m * (1.0 - m0))) / (1.0 + n);
    double Q_n = a + w;

    // Easting and longitude of central meridian
    double E0 = 500000.0;
    double L0 = (zone - 30) * 6.0 - 3.0;

    // Check tolerance for reverse transformation
    // double tolutm = STRP_PI / 2.0 * 1.2e-10 * Q_n;
    // double tolgeo = 0.000040;
    //    Coefficients of trigonometric series
    //
    //         ellipsoidal to spherical geographical, KW p .186 --187, (51) - (52)
    //     bg[1] = n * (-2 + n * (2 / 3 + n * (4 / 3 + n * (-82 / 45))));
    //     bg[2] = n ^ 2 * (5 / 3 + n * (-16 / 15 + n * (-13 / 9)));
    //     bg[3] = n ^ 3 * (-26 / 15 + n * 34 / 21);
    //     bg[4] = n ^ 4 * 1237 / 630;
    //
    //     spherical to ellipsoidal geographical, KW p.190 --191, (61) - (62) % gb[1] = n * (2 + n * (-2 / 3 + n * (-2 + n * 116 / 45)));
    //     gb[2] = n ^ 2 * (7 / 3 + n * (-8 / 5 + n * (-227 / 45)));
    //     gb[3] = n ^ 3 * (56 / 15 + n * (-136 / 35));
    //     gb[4] = n ^ 4 * 4279 / 630;
    //
    //     spherical to ellipsoidal N, E, KW p.196, (69) % gtu[1] = n * (1 / 2 + n * (-2 / 3 + n * (5 / 16 + n * 41 / 180)));
    //     gtu[2] = n ^ 2 * (13 / 48 + n * (-3 / 5 + n * 557 / 1440));
    //     gtu[3] = n ^ 3 * (61 / 240 + n * (-103 / 140));
    //     gtu[4] = n ^ 4 * 49561 / 161280;
    //
    //     ellipsoidal to spherical N, E, KW p.194, (65) % utg[1] = n * (-1 / 2 + n * (2 / 3 + n * (-37 / 96 + n * 1 / 360)));
    //     utg[2] = n ^ 2 * (-1 / 48 + n * (-1 / 15 + n * 437 / 1440));
    //     utg[3] = n ^ 3 * (-17 / 480 + n * 37 / 840);
    //     utg[4] = n ^ 4 * (-4397 / 161280);
    //
    //     With f = 1 / 297 we get

    arma::colvec bg = {-3.37077907e-3,
        4.73444769e-6,
        -8.29914570e-9,
        1.58785330e-11};

    arma::colvec gb = {3.37077588e-3,
        6.62769080e-6,
        1.78718601e-8,
        5.49266312e-11};

    arma::colvec gtu = {8.41275991e-4,
        7.67306686e-7,
        1.21291230e-9,
        2.48508228e-12};

    arma::colvec utg = {-8.41276339e-4,
        -5.95619298e-8,
        -1.69485209e-10,
        -2.20473896e-13};

    // Ellipsoidal latitude, longitude to spherical latitude, longitude
    bool neg_geo = false;

    if (B < 0.0) neg_geo = true;

    double Bg_r = fabs(B);
    double res_clensin = clsin(bg, 4, 2.0 * Bg_r);
    Bg_r = Bg_r + res_clensin;
    L0 = L0 * STRP_PI / 180.0;
    double Lg_r = L - L0;

    // Spherical latitude, longitude to complementary spherical latitude % i.e.spherical N, E
    double cos_BN = cos(Bg_r);
    double Np = atan2(sin(Bg_r), cos(Lg_r) * cos_BN);
    double Ep = atanh(sin(Lg_r) * cos_BN);

    // Spherical normalized N, E to ellipsoidal N, E
    Np = 2.0 * Np;
    Ep = 2.0 * Ep;

    double dN;
    double dE;
    clksin(gtu, 4, Np, Ep, &dN, &dE);
    Np = Np / 2.0;
    Ep = Ep / 2.0;
    Np = Np + dN;
    Ep = Ep + dE;
    double N = Q_n * Np;
    double E = Q_n * Ep + E0;
    if (neg_geo)
        {
            N = -N + 20000000.0;
        }
    r_enu(0) = E;
    r_enu(1) = N;
    r_enu(2) = U;
}


double clsin(const arma::colvec &ar, int degree, double argument)
{
    // Clenshaw summation of sinus of argument.
    //
    // result = clsin(ar, degree, argument);
    //
    // Originally written in Matlab by Kai Borre
    // Implemented in C++ by J.Arribas

    double cos_arg = 2.0 * cos(argument);
    double hr1 = 0.0;
    double hr = 0.0;
    double hr2;
    for (int t = degree; t > 0; t--)
        {
            hr2 = hr1;
            hr1 = hr;
            hr = ar(t - 1) + cos_arg * hr1 - hr2;
        }

    return (hr * sin(argument));
}


void clksin(const arma::colvec &ar, int degree, double arg_real, double arg_imag, double *re, double *im)
{
    // Clenshaw summation of sinus with complex argument
    // [re, im] = clksin(ar, degree, arg_real, arg_imag);
    //
    // Originally written in Matlab by Kai Borre
    // Implemented in C++ by J.Arribas

    double sin_arg_r = sin(arg_real);
    double cos_arg_r = cos(arg_real);
    double sinh_arg_i = sinh(arg_imag);
    double cosh_arg_i = cosh(arg_imag);

    double r = 2.0 * cos_arg_r * cosh_arg_i;
    double i = -2.0 * sin_arg_r * sinh_arg_i;

    double hr1 = 0.0;
    double hr = 0.0;
    double hi1 = 0.0;
    double hi = 0.0;
    double hi2;
    double hr2;
    for (int t = degree; t > 0; t--)
        {
            hr2 = hr1;
            hr1 = hr;
            hi2 = hi1;
            hi1 = hi;
            double z = ar(t - 1) + r * hr1 - i * hi - hr2;
            hi = i * hr1 + r * hi1 - hi2;
            hr = z;
        }

    r = sin_arg_r * cosh_arg_i;
    i = cos_arg_r * sinh_arg_i;

    *re = r * hr - i * hi;
    *im = r * hi + i * hr;
}


int findUtmZone(double latitude_deg, double longitude_deg)
{
    // Function finds the UTM zone number for given longitude and latitude.
    // The longitude value must be between -180 (180 degree West) and 180 (180
    // degree East) degree. The latitude must be within -80 (80 degree South) and
    // 84 (84 degree North).
    //
    // utmZone = findUtmZone(latitude, longitude);
    //
    // Latitude and longitude must be in decimal degrees (e.g. 15.5 degrees not
    // 15 deg 30 min).
    //
    // Originally written in Matlab by Darius Plausinaitis
    // Implemented in C++ by J.Arribas

    // Check value bounds
    if ((longitude_deg > 180.0) || (longitude_deg < -180.0))
        std::cout << "Longitude value exceeds limits (-180:180).\n";

    if ((latitude_deg > 84.0) || (latitude_deg < -80.0))
        std::cout << "Latitude value exceeds limits (-80:84).\n";

    //
    // Find zone
    //

    // Start at 180 deg west = -180 deg
    int utmZone = floor((180 + longitude_deg) / 6) + 1;

    // Correct zone numbers for particular areas
    if (latitude_deg > 72.0)
        {
            // Corrections for zones 31 33 35 37
            if ((longitude_deg >= 0.0) && (longitude_deg < 9.0))
                {
                    utmZone = 31;
                }
            else if ((longitude_deg >= 9.0) && (longitude_deg < 21.0))
                {
                    utmZone = 33;
                }
            else if ((longitude_deg >= 21.0) && (longitude_deg < 33.0))
                {
                    utmZone = 35;
                }
            else if ((longitude_deg >= 33.0) && (longitude_deg < 42.0))
                {
                    utmZone = 37;
                }
        }
    else if ((latitude_deg >= 56.0) && (latitude_deg < 64.0))
        {
            // Correction for zone 32
            if ((longitude_deg >= 3.0) && (longitude_deg < 12.0))
                utmZone = 32;
        }
    return utmZone;
}
