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
#include "geofunctions.h"

#define STRP_G_SI 9.80665
#define STRP_PI 3.1415926535898  //!< Pi as defined in IS-GPS-200E

arma::mat Skew_symmetric(arma::vec a)
{
    arma::mat A = arma::zeros(3, 3);

    A << 0.0 << -a(2) << a(1) << arma::endr
      << a(2) << 0.0 << -a(0) << arma::endr
      << -a(1) << a(0) << 0 << arma::endr;

    //    {{0, -a(2),  a(1)},
    //         {a(2),     0, -a(0)},
    //         {-a(1),  a(0),     0}};
    return A;
}


double WGS84_g0(double Lat_rad)
{
    double k = 0.001931853;        //normal gravity constant
    double e2 = 0.00669438002290;  //the square of the first numerical eccentricity
    double nge = 9.7803253359;     //normal gravity value on the equator (m/sec^2)
    double b = sin(Lat_rad);       //Lat in degrees
    b = b * b;
    double g0 = nge * (1 + k * b) / (sqrt(1 - e2 * b));
    return g0;
}

double WGS84_geocentric_radius(double Lat_geodetic_rad)
{
    //WGS84 earth model Geocentric radius (Eq. 2.88)

    double WGS84_A = 6378137.0;       //Semi-major axis of the Earth, a [m]
    double WGS84_IF = 298.257223563;  //Inverse flattening of the Earth
    double WGS84_F = (1 / WGS84_IF);  //The flattening of the Earth
    //double WGS84_B=(WGS84_A*(1-WGS84_F)); // Semi-minor axis of the Earth [m]
    double WGS84_E = (sqrt(2 * WGS84_F - WGS84_F * WGS84_F));  //Eccentricity of the Earth

    //transverse radius of curvature
    double R_E = WGS84_A / sqrt(1 -
                                WGS84_E * WGS84_E *
                                    sin(Lat_geodetic_rad) *
                                    sin(Lat_geodetic_rad));  // (Eq. 2.66)

    //gocentric radius at the Earth surface
    double r_eS = R_E * sqrt(cos(Lat_geodetic_rad) * cos(Lat_geodetic_rad) +
                             (1 - WGS84_E * WGS84_E) * (1 - WGS84_E * WGS84_E) * sin(Lat_geodetic_rad) * sin(Lat_geodetic_rad));  // (Eq. 2.88)
    return r_eS;
}

int topocent(double *Az, double *El, double *D, const arma::vec &x, const arma::vec &dx)
{
    double lambda;
    double phi;
    double h;
    double dtr = STRP_PI / 180.0;
    double a = 6378137.0;         // semi-major axis of the reference ellipsoid WGS-84
    double finv = 298.257223563;  // inverse of flattening of the reference ellipsoid WGS-84

    // Transform x into geodetic coordinates
    togeod(&phi, &lambda, &h, a, finv, x(0), x(1), x(2));

    double cl = cos(lambda * dtr);
    double sl = sin(lambda * dtr);
    double cb = cos(phi * dtr);
    double sb = sin(phi * dtr);

    arma::mat F = arma::zeros(3, 3);

    F(0, 0) = -sl;
    F(0, 1) = -sb * cl;
    F(0, 2) = cb * cl;

    F(1, 0) = cl;
    F(1, 1) = -sb * sl;
    F(1, 2) = cb * sl;

    F(2, 0) = 0;
    F(2, 1) = cb;
    F(2, 2) = sb;

    arma::vec local_vector;

    local_vector = arma::htrans(F) * dx;

    double E = local_vector(0);
    double N = local_vector(1);
    double U = local_vector(2);

    double hor_dis;
    hor_dis = sqrt(E * E + N * N);

    if (hor_dis < 1.0E-20)
        {
            *Az = 0;
            *El = 90;
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
    *h = 0;
    double tolsq = 1.e-10;  // tolerance to accept convergence
    int maxit = 10;         // max number of iterations
    double rtd = 180.0 / STRP_PI;

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

    //direct calculation of longitude
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
            *h = 0;
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
            N_phi = a / sqrt(1 - esq * sinphi * sinphi);

            //    compute residuals in P and Z
            dP = P - (N_phi + (*h)) * cosphi;
            dZ = Z - (N_phi * oneesq + (*h)) * sinphi;

            //    update height and latitude
            *h = *h + (sinphi * dZ + cosphi * dP);
            *dphi = *dphi + (cosphi * dZ - sinphi * dP) / (N_phi + (*h));

            //     test for convergence
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

arma::mat Gravity_ECEF(arma::vec r_eb_e)
{
    //Parameters
    double R_0 = 6378137;           //WGS84 Equatorial radius in meters
    double mu = 3.986004418E14;     //WGS84 Earth gravitational constant (m^3 s^-2)
    double J_2 = 1.082627E-3;       //WGS84 Earth's second gravitational constant
    double omega_ie = 7.292115E-5;  // Earth rotation rate (rad/s)
    // Calculate distance from center of the Earth
    double mag_r = sqrt(arma::as_scalar(r_eb_e.t() * r_eb_e));
    // If the input position is 0,0,0, produce a dummy output
    arma::vec g = arma::zeros(3, 1);
    if (mag_r != 0)
        {
            //Calculate gravitational acceleration using (2.142)
            double z_scale = 5 * pow((r_eb_e(2) / mag_r), 2);
            arma::vec tmp_vec = {(1 - z_scale) * r_eb_e(0),
                (1 - z_scale) * r_eb_e(1),
                (3 - z_scale) * r_eb_e(2)};
            arma::vec gamma_ = (-mu / pow(mag_r, 3)) * (r_eb_e + 1.5 * J_2 * pow(R_0 / mag_r, 2) * tmp_vec);

            //Add centripetal acceleration using (2.133)
            g(0) = gamma_(0) + pow(omega_ie, 2) * r_eb_e(0);
            g(1) = gamma_(1) + pow(omega_ie, 2) * r_eb_e(1);
            g(2) = gamma_(2);
        }
    return g;
}

arma::vec LLH_to_deg(arma::vec LLH)
{
    double rtd = 180.0 / STRP_PI;
    LLH(0) = LLH(0) * rtd;
    LLH(1) = LLH(1) * rtd;
    return LLH;
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

arma::vec CTM_to_Euler(arma::mat C)
{
    //Calculate Euler angles using (2.23)
    arma::vec eul = arma::zeros(3, 1);
    eul(0) = atan2(C(1, 2), C(2, 2));  // roll
    if (C(0, 2) < -1.0) C(0, 2) = -1.0;
    if (C(0, 2) > 1.0) C(0, 2) = 1.0;
    eul(1) = -asin(C(0, 2));           // pitch
    eul(2) = atan2(C(0, 1), C(0, 0));  // yaw
    return eul;
}

arma::mat Euler_to_CTM(arma::vec eul)
{
    //Eq.2.15
    //Euler angles to Attitude matrix is equivalent to rotate the body
    //in the three axes:
    //     arma::mat Ax= {{1,0,0}, {0,cos(Att_phi),sin(Att_phi)} ,{0,-sin(Att_phi),cos(Att_phi)}};
    //     arma::mat Ay= {{cos(Att_theta), 0, -sin(Att_theta)}, {0,1,0} , {sin(Att_theta), 0, cos(Att_theta)}};
    //     arma::mat Az= {{cos(Att_psi), sin(Att_psi), 0}, {-sin(Att_psi), cos(Att_psi), 0},{0,0,1}};
    //     arma::mat C_b_n=Ax*Ay*Az; // Attitude expressed in the LOCAL FRAME (NED)
    //    C_b_n=C_b_n.t();

    //Precalculate sines and cosines of the Euler angles
    double sin_phi = sin(eul(0));
    double cos_phi = cos(eul(0));
    double sin_theta = sin(eul(1));
    double cos_theta = cos(eul(1));
    double sin_psi = sin(eul(2));
    double cos_psi = cos(eul(2));

    arma::mat C = arma::zeros(3, 3);
    //Calculate coordinate transformation matrix using (2.22)
    C(0, 0) = cos_theta * cos_psi;
    C(0, 1) = cos_theta * sin_psi;
    C(0, 2) = -sin_theta;
    C(1, 0) = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi;
    C(1, 1) = cos_phi * cos_psi + sin_phi * sin_theta * sin_psi;
    C(1, 2) = sin_phi * cos_theta;
    C(2, 0) = sin_phi * sin_psi + cos_phi * sin_theta * cos_psi;
    C(2, 1) = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi;
    C(2, 2) = cos_phi * cos_theta;
    return C;
}

arma::vec cart2geo(arma::vec XYZ, int elipsoid_selection)
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
            N = c / sqrt(1 + ex2 * (cos(phi) * cos(phi)));
            phi = atan(XYZ[2] / ((sqrt(XYZ[0] * XYZ[0] + XYZ[1] * XYZ[1]) * (1.0 - (2.0 - f[elipsoid_selection]) * f[elipsoid_selection] * N / (N + h)))));
            h = sqrt(XYZ[0] * XYZ[0] + XYZ[1] * XYZ[1]) / cos(phi) - N;
            iterations = iterations + 1;
            if (iterations > 100)
                {
                    //std::cout << "Failed to approximate h with desired precision. h-oldh= " << h - oldh;
                    break;
                }
        }
    while (std::abs(h - oldh) > 1.0e-12);

    arma::vec LLH = {{phi, lambda, h}};  //radians
    return LLH;
}

void ECEF_to_Geo(arma::vec r_eb_e, arma::vec v_eb_e, arma::mat C_b_e, arma::vec &LLH, arma::vec &v_eb_n, arma::mat &C_b_n)
{
    //Compute the Latitude of the ECEF position
    LLH = cart2geo(r_eb_e, 4);  //ECEF -> WGS84 geographical

    // Calculate ECEF to Geographical coordinate transformation matrix using (2.150)
    double cos_lat = cos(LLH(0));
    double sin_lat = sin(LLH(0));
    double cos_long = cos(LLH(1));
    double sin_long = sin(LLH(1));
    //C++11 and arma >= 5.2
    //    arma::mat C_e_n = {{-sin_lat * cos_long, -sin_lat * sin_long,  cos_lat},
    //                      {-sin_long, cos_long, 0},
    //                      {-cos_lat * cos_long, -cos_lat * sin_long, -sin_lat}}; //ECEF to Geo

    //C++98 arma <5.2
    arma::mat C_e_n = arma::zeros(3, 3);
    C_e_n << -sin_lat * cos_long << -sin_lat * sin_long << cos_lat << arma::endr
          << -sin_long << cos_long << 0 << arma::endr
          << -cos_lat * cos_long << -cos_lat * sin_long << -sin_lat << arma::endr;  //ECEF to Geo
    // Transform velocity using (2.73)
    v_eb_n = C_e_n * v_eb_e;

    C_b_n = C_e_n * C_b_e;  // Attitude conversion from ECEF to NED
}

void Geo_to_ECEF(arma::vec LLH, arma::vec v_eb_n, arma::mat C_b_n, arma::vec &r_eb_e, arma::vec &v_eb_e, arma::mat &C_b_e)
{
    // Parameters
    double R_0 = 6378137;        //WGS84 Equatorial radius in meters
    double e = 0.0818191908425;  //WGS84 eccentricity

    // Calculate transverse radius of curvature using (2.105)
    double R_E = R_0 / sqrt(1 - (e * sin(LLH(0))) * (e * sin(LLH(0))));

    // Convert position using (2.112)
    double cos_lat = cos(LLH(0));
    double sin_lat = sin(LLH(0));
    double cos_long = cos(LLH(1));
    double sin_long = sin(LLH(1));
    r_eb_e = {(R_E + LLH(2)) * cos_lat * cos_long,
        (R_E + LLH(2)) * cos_lat * sin_long,
        ((1 - e * e) * R_E + LLH(2)) * sin_lat};

    //Calculate ECEF to Geo coordinate transformation matrix using (2.150)
    //C++11 and arma>=5.2
    //    arma::mat C_e_n = {{-sin_lat * cos_long, -sin_lat * sin_long,  cos_lat},
    //                       {-sin_long,            cos_long,        0},
    //                       {-cos_lat * cos_long, -cos_lat * sin_long, -sin_lat}};
    //C++98 arma <5.2
    //Calculate ECEF to Geo coordinate transformation matrix using (2.150)
    arma::mat C_e_n = arma::zeros(3, 3);
    C_e_n << -sin_lat * cos_long << -sin_lat * sin_long << cos_lat << arma::endr
          << -sin_long << cos_long << 0 << arma::endr
          << -cos_lat * cos_long << -cos_lat * sin_long << -sin_lat << arma::endr;

    // Transform velocity using (2.73)
    v_eb_e = C_e_n.t() * v_eb_n;

    // Transform attitude using (2.15)
    C_b_e = C_e_n.t() * C_b_n;
}


void pv_Geo_to_ECEF(double L_b, double lambda_b, double h_b, arma::vec v_eb_n, arma::vec &r_eb_e, arma::vec &v_eb_e)
{
    //% Parameters
    double R_0 = 6378137;        //WGS84 Equatorial radius in meters
    double e = 0.0818191908425;  //WGS84 eccentricity

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
