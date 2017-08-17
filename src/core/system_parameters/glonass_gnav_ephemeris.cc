/*!
 * \file glonass_gnav_ephemeris.cc
 * \brief  Interface of a GLONASS GNAV EPHEMERIS storage and orbital model functions
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
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

#include "glonass_gnav_ephemeris.h"
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "GLONASS_L1_CA.h"
#include "gnss_satellite.h"

Glonass_Gnav_Ephemeris::Glonass_Gnav_Ephemeris()
{
    i_satellite_freq_channel = 0;
    d_m = 0.0;               //!< String number within frame [dimensionless]
    d_t_k = 0.0;             //!< Time referenced to the beginning of the frame within the current day [hours, minutes, seconds]
    d_t_b = 0.0;             //!< Index of a time interval within current day according to UTC(SU) + 03 hours 00 min. [minutes]
    d_M = 0.0;               //!< Type of satellite transmitting navigation signal [dimensionless]
    d_gamma_n = 0.0;         //!< Relative deviation of predicted carrier frequency value of n- satellite from nominal value at the instant tb [dimensionless]
    d_tau_n = 0.0;           //!< Correction to the nth satellite time (tn) relative to GLONASS time (te),
    // satellite positions
    d_satpos_X = 0.0;        //!< Earth-fixed coordinate x of the satellite in PZ-90.02 coordinate system [km].
    d_satpos_Y = 0.0;        //!< Earth-fixed coordinate y of the satellite in PZ-90.02 coordinate system [km]
    d_satpos_Z = 0.0;        //!< Earth-fixed coordinate z of the satellite in PZ-90.02 coordinate system [km]
    // Satellite velocity
    d_satvel_X = 0.0;        //!< Earth-fixed velocity coordinate x of the satellite in PZ-90.02 coordinate system [km/s]
    d_satvel_Y = 0.0;        //!< Earth-fixed velocity coordinate y of the satellite in PZ-90.02 coordinate system [km/s]
    d_satvel_Z = 0.0;        //!< Earth-fixed velocity coordinate z of the satellite in PZ-90.02 coordinate system [km/s]
    // Satellite acceleration
    d_satacc_X = 0.0;        //!< Earth-fixed acceleration coordinate x of the satellite in PZ-90.02 coordinate system [km/s^2]
    d_satacc_Y = 0.0;        //!< Earth-fixed acceleration coordinate y of the satellite in PZ-90.02 coordinate system [km/s^2]
    d_satacc_Z = 0.0;        //!< Earth-fixed acceleration coordinate z of the satellite in PZ-90.02 coordinate system [km/s^2]
    d_B_n = 0.0;             //!< Health flag [dimensionless]
    d_P = 0.0;               //!< Technological parameter of control segment, indication the satellite operation mode in respect of time parameters [dimensionless]
    d_N_T = 0.0;             //!< Current date, calendar number of day within four-year interval starting from the 1-st of January in a leap year [days]
    d_F_T = 0.0;             //!< Parameter that provides the predicted satellite user range accuracy at time tb [dimensionless]
    d_n = 0.0;               //!< Index of the satellite transmitting given navigation signal. It corresponds to a slot number within GLONASS constellation
    d_Delta_tau_n = 0.0;     //!< Time difference between navigation RF signal transmitted in L2 sub- band and aviation RF signal transmitted in L1 sub-band by nth satellite. [dimensionless]
    d_E_n = 0.0;             //!< Characterises "age" of a current information [days]
    d_P_1 = 0.0;             //!< Flag of the immediate data updating.
    d_P_2 = 0.0;             //!< Flag of oddness ("1") or evenness ("0") of the value of (tb) [dimensionless]
    d_P_3 = 0.0;             //!< Flag indicating a number of satellites for which almanac is transmitted within given frame: "1" corresponds to 5 satellites and "0" corresponds to 4 satellites [dimensionless]
    d_P_4 = 0.0;             //!< Flag to show that ephemeris parameters are present. "1" indicates that updated ephemeris or frequency/time parameters have been uploaded by the control segment [dimensionless]
    d_l3rd_n = 0.0;          //!< Health flag for nth satellite; ln = 0 indicates the n-th satellite is helthy, ln = 1 indicates malfunction of this nth satellite [dimensionless]
    d_l5th_n = 0.0;             //!< Health flag for nth satellite; ln = 0 indicates the n-th satellite is helthy, ln = 1 indicates malfunction of this nth satellite [dimensionless]

    // clock terms derived from ephemeris data
    d_satClkDrift = 0.0;    //!< GLONASS clock error
    d_dtr = 0.0;
}


boost::posix_time::ptime Glonass_Gnav_Ephemeris::compute_GLONASS_time(const double offset_time) const
{
    boost::posix_time::time_duration t(0, 0, offset_time);
    boost::gregorian::date d(d_yr, 1, d_N_T);
    boost::posix_time::ptime glonass_time(d, t);

    return glonass_time;
}


void Glonass_Gnav_Ephemeris::gravitational_perturbations()
{
    double T = 0.0;
    double sigma_days = 0.0;
    double tau_prime = 0.0;
    double Omega_moon = 0.0;
    double q_moon = 0.0;
    double q_sun = 0.0;

    double xi_star = 0.0;
    double eta_star = 0.0;
    double zeta_star = 0.0;
    double E_moon = 0.0;
    double E_sun = 0.0;
    double xi_11 = 0.0;
    double xi_12 = 0.0;
    double eta_11 = 0.0;
    double eta_12 = 0.0;
    double zeta_11 = 0.0;
    double zeta_12 = 0.0;

    double sin_theta_moon = 0.0;
    double cos_theta_moon = 0.0;
    double theta_moon = 0.0;
    double xi_moon_e = 0.0;
    double eta_moon_e = 0.0;
    double zeta_moon_e = 0.0;
    double r_moon_e = 0.0;

    double sin_theta_sun = 0.0;
    double cos_theta_sun = 0.0;
    double theta_sun = 0.0;
    double xi_sun_e = 0.0;
    double eta_sun_e = 0.0;
    double zeta_sun_e = 0.0;
    double r_sun_e = 0.0;

    double mu_bar_moon = 0.0;
    double x_bar_moon = 0.0;
    double y_bar_moon = 0.0;
    double z_bar_moon = 0.0;
    double Delta_o_moon = 0.0;
    double mu_bar_sun = 0.0;
    double x_bar_sun = 0.0;
    double y_bar_sun = 0.0;
    double z_bar_sun = 0.0;
    double Delta_o_sun = 0.0;


    double t_e = 0.0;

    //<! Directive Cosine computation
    /// \TODO Need to define sigma days
    T           = (27392.375 + sigma_days + (t_e/86400))/(36525);
    tau_prime   = GLONASS_TAU_0 + GLONASS_TAU_1 * T;
    Omega_moon  = GLONASS_MOON_OMEGA_0 + GLONASS_MOON_OMEGA_1*T;
    q_moon      = GLONASS_MOON_Q0 + GLONASS_MOON_Q1*T;
    q_sun       = GLONASS_SUN_Q0 + GLONASS_SUN_Q1*T;

    xi_star     = 1 - (cos(Omega_moon)*cos(Omega_moon))*(1 - cos(GLONASS_MOON_INCLINATION));
    eta_star    = sin(Omega_moon)*(sin(GLONASS_MOON_INCLINATION));
    zeta_star   = cos(Omega_moon)*(sin(GLONASS_MOON_INCLINATION));

    xi_11   = sin(Omega_moon)*cos(Omega_moon)*(1 - cos(GLONASS_MOON_INCLINATION));
    xi_12   = 1 - (sin(Omega_moon)*sin(Omega_moon))*(1 - cos(GLONASS_MOON_INCLINATION));
    eta_11  = xi_star*cos(GLONASS_EARTH_INCLINATION) - xi_star*sin(GLONASS_EARTH_INCLINATION);
    eta_12  = xi_11*cos(GLONASS_EARTH_INCLINATION) + eta_star*sin(GLONASS_EARTH_INCLINATION);
    zeta_11 = xi_star*sin(GLONASS_EARTH_INCLINATION) + zeta_star*cos(GLONASS_EARTH_INCLINATION);
    zeta_12 = xi_11*sin(GLONASS_EARTH_INCLINATION) + eta_star*cos(GLONASS_EARTH_INCLINATION);

    /// \TODO why is this calling sin(E_moon) in the same line where it is being computed
    E_moon          = q_moon + GLONASS_MOON_ECCENTRICITY*sin(E_moon);
    sin_theta_moon  = sqrt(1 - GLONASS_MOON_ECCENTRICITY*GLONASS_MOON_ECCENTRICITY)*sin(E_moon)/(1 - GLONASS_MOON_ECCENTRICITY*cos(E_moon));
    cos_theta_moon  = cos(E_moon - GLONASS_MOON_ECCENTRICITY)/(1 - GLONASS_MOON_ECCENTRICITY*cos(E_moon));
    theta_moon      = asin(sin_theta_moon);

    xi_moon_e   = sin(theta_moon + tau_prime)*xi_11 + cos(theta_moon + tau_prime)*xi_12;
    eta_moon_e  = sin(theta_moon + tau_prime)*eta_11 + cos(theta_moon + tau_prime)*eta_12;
    zeta_moon_e = sin(theta_moon + tau_prime)*zeta_11 + cos(theta_moon + tau_prime)*zeta_12;
    r_moon_e    = GLONASS_MOON_SEMI_MAJOR_AXIS*(1-GLONASS_MOON_ECCENTRICITY*cos(E_moon));

    /// \TODO why is this calling sin(E_moon) in the same line where it is being computed
    E_sun           = q_sun + GLONASS_SUN_ECCENTRICITY*sin(E_sun);
    sin_theta_sun   = sqrt(1 - GLONASS_SUN_ECCENTRICITY*GLONASS_SUN_ECCENTRICITY)*sin(E_sun)/(1 - GLONASS_SUN_ECCENTRICITY*cos(E_sun));
    cos_theta_sun   = cos(E_sun - GLONASS_SUN_ECCENTRICITY)/(1 - GLONASS_SUN_ECCENTRICITY*cos(E_sun));
    theta_sun       = asin(sin_theta_sun);

    xi_sun_e    = (cos_theta_sun*cos(GLONASS_SUN_OMEGA) - sin_theta_sun*sin(GLONASS_SUN_OMEGA));
    eta_sun_e   = (sin_theta_sun*cos(GLONASS_SUN_OMEGA) + cos_theta_sun*sin(GLONASS_SUN_OMEGA))*cos(GLONASS_EARTH_INCLINATION);
    zeta_sun_e  = (sin_theta_sun*cos(GLONASS_SUN_OMEGA) + cos_theta_sun*sin(GLONASS_SUN_OMEGA))*sin(GLONASS_EARTH_INCLINATION);
    r_sun_e     = GLONASS_SUN_SEMI_MAJOR_AXIS*(1-GLONASS_SUN_ECCENTRICITY*cos(E_sun));

    //<! Gravitational computation
    mu_bar_moon = GLONASS_MOON_GM/(r_moon_e*r_moon_e);
    x_bar_moon  = d_Xn/r_moon_e;
    y_bar_moon  = d_Yn/r_moon_e;
    z_bar_moon  = d_Zn/r_moon_e;
    Delta_o_moon = sqrt((xi_moon_e - x_bar_moon)*(xi_moon_e - x_bar_moon) + (eta_moon_e - y_bar_moon)*(eta_moon_e - y_bar_moon) + (zeta_moon_e - z_bar_moon)*(zeta_moon_e - z_bar_moon));

    mu_bar_sun = GLONASS_MOON_GM/(r_sun_e*r_sun_e);
    x_bar_sun  = d_Xn/r_sun_e;
    y_bar_sun  = d_Yn/r_sun_e;
    z_bar_sun  = d_Zn/r_sun_e;
    Delta_o_sun = sqrt((xi_sun_e - x_bar_sun)*(xi_sun_e - x_bar_sun) + (eta_sun_e - y_bar_sun)*(eta_sun_e - y_bar_sun) + (zeta_sun_e - z_bar_sun)*(zeta_sun_e - z_bar_sun));

    d_Jx_moon = mu_bar_moon*(xi_moon_e - x_bar_moon)/pow(Delta_o_moon,3.0) - xi_moon_e;
    d_Jy_moon = mu_bar_moon*(eta_moon_e - y_bar_moon)/pow(Delta_o_moon,3.0) - eta_moon_e;
    d_Jz_moon = mu_bar_moon*(zeta_moon_e - z_bar_moon)/pow(Delta_o_moon,3.0) - zeta_moon_e;

    d_Jx_sun = mu_bar_sun*(xi_sun_e - x_bar_sun)/pow(Delta_o_sun,3.0) - xi_sun_e;
    d_Jy_sun = mu_bar_sun*(eta_sun_e - y_bar_sun)/pow(Delta_o_sun,3.0) - eta_sun_e;
    d_Jz_sun = mu_bar_sun*(zeta_sun_e - z_bar_sun)/pow(Delta_o_sun,3.0) - zeta_sun_e;

}


double Glonass_Gnav_Ephemeris::check_t(double time)
{
    double corrTime;
    double half_day = 43200.0;     // seconds
    corrTime = time;
    if (time > half_day)
        {
            corrTime = time - 2.0 * half_day;
        }
    else if (time < -half_day)
        {
            corrTime = time + 2.0 * half_day;
        }
    return corrTime;
}

// FIXME Fix reference here
// 20.3.3.3.3.1 User Algorithm for SV Clock Correction.
double Glonass_Gnav_Ephemeris::sv_clock_drift(double transmitTime, double timeCorrUTC)
{
    double dt;
    dt = check_t(transmitTime - d_t_b);
    d_satClkDrift = -(d_tau_n + timeCorrUTC - d_gamma_n * dt);
    //Correct satellite group delay and missing relativistic term here
    //d_satClkDrift-=d_TGD;

    return d_satClkDrift;
}

// TODO is this the right formula
// compute the relativistic correction term
double Glonass_Gnav_Ephemeris::sv_clock_relativistic_term(double transmitTime)
{
    // Compute relativistic correction term
    d_dtr = 0.0;
    return d_dtr;
}


double Glonass_Gnav_Ephemeris::simplified_satellite_position(double transmitTime)
{
    double dt = 0.0;
    double tau = 0.0;
    double tk = 0.0;
    int numberOfIntegrations = 0;

    // RK 1
    double x_0 = 0.0;
    double y_0 = 0.0;
    double z_0 = 0.0;

    double Vx_0 = 0.0;
    double Vy_0 = 0.0;
    double Vz_0 = 0.0;

    double Ax_0 = 0.0;
    double Ay_0 = 0.0;
    double Az_0 = 0.0;

    double r0   = 0.0;
    double r0_2 = 0.0;
    double r0_3 = 0.0;
    double r0_5 = 0.0;

    double dVx_1  = 0.0;
    double dVy_1  = 0.0;
    double dVz_1  = 0.0;

    double dx_1  = 0.0;
    double dy_1  = 0.0;
    double dz_1  = 0.0;

    // Runge-Kutta Integration Stage K2
    double x_1   = 0.0;
    double y_1   = 0.0;
    double z_1   = 0.0;

    double Vx_1  = 0.0;
    double Vy_1  = 0.0;
    double Vz_1  = 0.0;

    double r1   = 0.0;
    double r1_2 = 0.0;
    double r1_3 = 0.0;
    double r1_5 = 0.0;

    double dVx_2  = 0.0;
    double dVy_2  = 0.0;
    double dVz_2  = 0.0;

    double dx_2 = 0.0;
    double dy_2 = 0.0;
    double dz_2 = 0.0;

    // Runge-Kutta Integration Stage K3
    double x_2   = 0.0;
    double y_2   = 0.0;
    double z_2   = 0.0;

    double Vx_2  = 0.0;
    double Vy_2  = 0.0;
    double Vz_2  = 0.0;

    double r2   = 0.0;
    double r2_2 = 0.0;
    double r2_3 = 0.0;
    double r2_5 = 0.0;

    double dVx_3  = 0.0;
    double dVy_3  = 0.0;
    double dVz_3  = 0.0;

    double dx_3 = 0.0;
    double dy_3 = 0.0;
    double dz_3 = 0.0;

    // Runge-Kutta Integration Stage K4
    double x_3   = 0.0;
    double y_3   = 0.0;
    double z_3   = 0.0;

    double Vx_3  = 0.0;
    double Vy_3  = 0.0;
    double Vz_3  = 0.0;

    double r3   = 0.0;
    double r3_2 = 0.0;
    double r3_3 = 0.0;
    double r3_5 = 0.0;

    double dVx_4  = 0.0;
    double dVy_4  = 0.0;
    double dVz_4  = 0.0;

    double dx_4 = 0.0;
    double dy_4 = 0.0;
    double dz_4 = 0.0;

    // Find time difference
    dt = check_t(transmitTime - d_t_b);

    // Calculate clock correction
    d_satClkDrift = -(d_tau_n + /*d_tau_c*/ - d_gamma_n * dt);

    // Find integration time
    tk    = dt - d_satClkDrift;

    // Check if to integrate forward or backward
    if(tk < 0)
        {
            tau      = -60;
        }
    else
        {
            tau      = 60;
        }

    // x,y,z coordinates to meters
    x_0  = d_Xn * 1e3;
    y_0  = d_Yn * 1e3;
    z_0  = d_Zn * 1e3;

    // x,y,z velocities to meters/s
    Vx_0 = d_VXn * 1e3;
    Vy_0 = d_VYn * 1e3;
    Vz_0 = d_VZn * 1e3;

    // x,y,z accelerations to meters/sec^2
    Ax_0 = d_AXn * 1e3;
    Ay_0 = d_AYn * 1e3;
    Az_0 = d_AZn * 1e3;

    for(numberOfIntegrations = tau; numberOfIntegrations < tk+tau; numberOfIntegrations += tau)
        {
            // Check if last integration step. If last integration step, make one more step that has the remaining time length...
            if(fabs(numberOfIntegrations) > fabs(tk))
                {
                    // if there is more time left to integrate...
                    if(fmod(tk,tau) != 0)
                        {
                            tau = fmod(tk,tau);
                        }
                    else // otherwise make a zero step.
                        {
                            tau = 0;
                        }
                }

            // Runge-Kutta Integration Stage K1
            r0      = sqrt(x_0*x_0 + y_0*y_0 + z_0*z_0);
            r0_2    = r0*r0;
            r0_3    = r0*r0*r0;
            r0_5    = r0*r0*r0*r0*r0;

            dVx_1  = - GLONASS_GM / r0_3 * x_0 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r0_5 * x_0 * (1 - 5 * (z_0*z_0) / r0_2) + pow(GLONASS_OMEGA_EARTH_DOT, 2) * x_0 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_0 + Ax_0;
            dVy_1  = - GLONASS_GM / r0_3 * y_0 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r0_5 * y_0 * (1 - 5 * (z_0*z_0) / r0_2) + pow(GLONASS_OMEGA_EARTH_DOT, 2) * y_0 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_0 + Ay_0;
            dVz_1  = - GLONASS_GM / r0_3 * z_0 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r0_5 * z_0 * (3 - 5 * (z_0*z_0) / r0_2) + Az_0;

            dx_1  = Vx_0;
            dy_1  = Vy_0;
            dz_1  = Vz_0;

            // Runge-Kutta Integration Stage K2
            Vx_1  = Vx_0 + 1/2 * tau * dVx_1;
            Vy_1  = Vy_0 + 1/2 * tau * dVy_1;
            Vz_1  = Vz_0 + 1/2 * tau * dVz_1;

            x_1   = x_0  + 1/2 * tau * dx_1;
            y_1   = y_0  + 1/2 * tau * dy_1;
            z_1   = z_0  + 1/2 * tau * dz_1;

            r1  = sqrt( x_1*x_1 + y_1*y_1 + z_1*z_1 );
            r1_2    = r1*r1;
            r1_3    = r1*r1*r1;
            r1_5    = r1*r1*r1*r1*r1;

            dVx_2  = - GLONASS_GM / r1_3 * x_1 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r1_5 * x_1 * (1 - 5 * (z_1*z_1) / r1_2) + pow(GLONASS_OMEGA_EARTH_DOT, 2) * x_1 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_1 + Ax_0;
            dVy_2  = - GLONASS_GM / r1_3 * y_1 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r1_5 * y_1 * (1 - 5 * (z_1*z_1) / r1_2) + pow(GLONASS_OMEGA_EARTH_DOT, 2) * y_1 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_1 + Ay_0;
            dVz_2  = - GLONASS_GM / r1_3 * z_1 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r1_5 * z_1 * (3 - 5 * (z_1*z_1) / r1_2) + Az_0;

            dx_2  = Vx_1;
            dy_2  = Vy_1;
            dz_2  = Vz_1;

            // Runge-Kutta Integration Stage K2
            Vx_2  = Vx_0 + 1/2 * tau * dVx_2;
            Vy_2  = Vy_0 + 1/2 * tau * dVy_2;
            Vz_2  = Vz_0 + 1/2 * tau * dVz_2;

            x_2   = x_0  + 1/2 * tau * dx_2;
            y_2   = y_0  + 1/2 * tau * dy_2;
            z_2   = z_0  + 1/2 * tau * dz_2;

            r2  = sqrt( x_2*x_2 + y_2*y_2 + z_2*z_2 );
            r2_2 = r2*r2;
            r2_3 = r2*r2*r2;
            r2_5 = r2*r2*r2*r2*r2;


            dVx_3  = - GLONASS_GM / r2_3 * x_2 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r2_5 * x_2 * (1 - 5 * (z_2*z_2) / r2_2) + pow(GLONASS_OMEGA_EARTH_DOT, 2) * x_2 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_2 + Ax_0;
            dVy_3  = - GLONASS_GM / r2_3 * y_2 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r2_5 * y_2 * (1 - 5 * (z_2*z_2) / r2_2) + pow(GLONASS_OMEGA_EARTH_DOT, 2) * y_2 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_2 + Ay_0;
            dVz_3  = - GLONASS_GM / r2_3 * z_2 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r2_5 * z_2 * (3 - 5 * (z_2*z_2) / r2_2) + Az_0;

            dx_3  = Vx_2;
            dy_3  = Vy_2;
            dz_3  = Vz_2;

            // Runge-Kutta Integration Stage K3
            Vx_3  = Vx_0 + tau * dVx_3;
            Vy_3  = Vy_0 + tau * dVy_3;
            Vz_3  = Vz_0 + tau * dVz_3;

            x_3   = x_0  + tau * dx_3;
            y_3   = y_0  + tau * dy_3;
            z_3   = z_0  + tau * dz_3;

            r3  = sqrt( x_3*x_3 + y_3*y_3 + z_3*z_3 );
            r3_2 = r3*r3;
            r3_3 = r3*r3*r3;
            r3_5 = r3*r3*r3*r3*r3;

            dVx_4  = - GLONASS_GM / r3_3 * x_3 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r3_5 * x_3 * (1 - 5 * (z_3*z_3) / r3_2) + pow(GLONASS_OMEGA_EARTH_DOT, 2) * x_3 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_3 + Ax_0;
            dVy_4  = - GLONASS_GM / r3_3 * y_3 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r3_5 * y_3 * (1 - 5 * (z_3*z_3) / r3_2) + pow(GLONASS_OMEGA_EARTH_DOT, 2) * y_3 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_3 + Ay_0;
            dVz_4  = - GLONASS_GM / r3_3 * z_3 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r3_5 * z_3 * (3 - 5 * (z_3*z_3) / r3_2) + Az_0;

            dx_4  = Vx_3;
            dy_4  = Vy_3;
            dz_4  = Vz_3;

            // Final results showcased here
            Vx_0  = Vx_0 + 1/6 * tau * ( dVx_1 + 2 * dVx_2 + 2 * dVx_3 + dVx_4 );
            Vy_0  = Vy_0 + 1/6 * tau * ( dVy_1 + 2 * dVy_2 + 2 * dVy_3 + dVy_4 );
            Vz_0  = Vz_0 + 1/6 * tau * ( dVz_1 + 2 * dVz_2 + 2 * dVz_3 + dVz_4 );

            x_0   = x_0 + 1/6 * tau * ( dx_1 + 2 * dx_2 + 2 * dx_3 + dx_4 );
            y_0   = y_0 + 1/6 * tau * ( dy_1 + 2 * dy_2 + 2 * dy_3 + dy_4 );
            z_0   = z_0 + 1/6 * tau * ( dz_1 + 2 * dz_2 + 2 * dz_3 + dz_4 );

        }

    // Reasign position, velocities and accelerations for next integration
    d_satpos_X = x_0;
    d_satpos_Y = y_0;
    d_satpos_Z = z_0;

    d_satvel_X = Vx_0;
    d_satvel_Y = Vy_0;
    d_satvel_Z = Vz_0;

    d_satacc_X = d_AXn;    // No change in accelerations reported over interval
    d_satacc_Y = d_AYn;    // No change in accelerations reported over interval
    d_satacc_Z = d_AZn;    // No change in accelerations reported over interval

    // Time from ephemeris reference clock
    //tk = check_t(transmitTime - d_Toc);

    //double dtr_s = d_A_f0 + d_A_f1 * tk + d_A_f2 * tk * tk;

    /* relativity correction */
    //dtr_s -= 2.0 * sqrt(GM * GLONASS_a) * d_e_eccentricity * sin(E) / (GPS_C_m_s * GPS_C_m_s);

    return 0;
}


double Glonass_Gnav_Ephemeris::satellite_position(double transmitTime)
{
    double dt = 0.0;
    double tk = 0.0;
    double tau = 0.0;
    int numberOfIntegrations = 0;

    // RK 1
    double x_0 = 0.0;
    double y_0 = 0.0;
    double z_0 = 0.0;

    double Vx_0 = 0.0;
    double Vy_0 = 0.0;
    double Vz_0 = 0.0;

    double Ax_0 = 0.0;
    double Ay_0 = 0.0;
    double Az_0 = 0.0;

    double r0   = 0.0;
    double r0_2 = 0.0;
    double r0_3 = 0.0;
    double r0_5 = 0.0;

    double dVx_1  = 0.0;
    double dVy_1  = 0.0;
    double dVz_1  = 0.0;

    double dx_1  = 0.0;
    double dy_1  = 0.0;
    double dz_1  = 0.0;

    // Runge-Kutta Integration Stage K2
    double x_1   = 0.0;
    double y_1   = 0.0;
    double z_1   = 0.0;

    double Vx_1  = 0.0;
    double Vy_1  = 0.0;
    double Vz_1  = 0.0;

    double r1   = 0.0;
    double r1_2 = 0.0;
    double r1_3 = 0.0;
    double r1_5 = 0.0;

    double dVx_2  = 0.0;
    double dVy_2  = 0.0;
    double dVz_2  = 0.0;

    double dx_2 = 0.0;
    double dy_2 = 0.0;
    double dz_2 = 0.0;

    // Runge-Kutta Integration Stage K3
    double x_2   = 0.0;
    double y_2   = 0.0;
    double z_2   = 0.0;

    double Vx_2  = 0.0;
    double Vy_2  = 0.0;
    double Vz_2  = 0.0;

    double r2   = 0.0;
    double r2_2 = 0.0;
    double r2_3 = 0.0;
    double r2_5 = 0.0;

    double dVx_3  = 0.0;
    double dVy_3  = 0.0;
    double dVz_3  = 0.0;

    double dx_3 = 0.0;
    double dy_3 = 0.0;
    double dz_3 = 0.0;

    // Runge-Kutta Integration Stage K4
    double x_3   = 0.0;
    double y_3   = 0.0;
    double z_3   = 0.0;

    double Vx_3  = 0.0;
    double Vy_3  = 0.0;
    double Vz_3  = 0.0;

    double r3   = 0.0;
    double r3_2 = 0.0;
    double r3_3 = 0.0;
    double r3_5 = 0.0;

    double dVx_4  = 0.0;
    double dVy_4  = 0.0;
    double dVz_4  = 0.0;

    double dx_4 = 0.0;
    double dy_4 = 0.0;
    double dz_4 = 0.0;

    // Find time difference
    dt = check_t(transmitTime - d_t_b);

    // Calculate clock correction
    d_satClkDrift = -(d_tau_n + /*d_tau_c*/ - d_gamma_n * dt);

    // Find integration time
    tk    = dt - d_satClkDrift;

    // Check if to integrate forward or backward
    if (tk < 0)
        {
            tau      = -60;
        }
    else
        {
            tau      = 60;
        }

    // Coordinates transformation to an inertial reference frame
    // x,y,z coordinates to meters
    x_0  = d_Xn * 1e3;
    y_0  = d_Yn * 1e3;
    z_0  = d_Zn * 1e3;

    // x,y,z velocities to meters/s
    Vx_0 = d_VXn * 1e3;
    Vy_0 = d_VYn * 1e3;
    Vz_0 = d_VZn * 1e3;

    // x,y,z accelerations to meters/sec^2
    Ax_0 = d_AXn * 1e3;
    Ay_0 = d_AYn * 1e3;
    Az_0 = d_AZn * 1e3;

    for(numberOfIntegrations = tau; numberOfIntegrations < tk+tau; numberOfIntegrations += tau)
        {
            // Check if last integration step. If last integration step, make one more step that has the remaining time length...
            if(fabs(numberOfIntegrations) > fabs(tk))
                {
                    // if there is more time left to integrate...
                    if (fmod(tk,tau) != 0)
                        {
                            tau = fmod(tk,tau);
                        }
                    else // otherwise make a zero step.
                        {
                            tau = 0;
                        }
                }

            // Runge-Kutta Integration Stage K1
            r0      = sqrt(x_0*x_0 + y_0*y_0 + z_0*z_0);
            r0_2    = r0*r0;
            r0_3    = r0*r0*r0;
            r0_5    = r0*r0*r0*r0*r0;

            dx_1 = Vx_0;
            dy_1 = Vy_0;
            dz_1 = Vz_0;

            dVx_1  = - GLONASS_GM / r0_3 * x_0 + 3/2 * GLONASS_C20 * GLONASS_GM * pow(GLONASS_EARTH_RADIUS, 2) / r0_5 * x_0 * (1 - 5 * 1 / (z_0*z_0)) + d_Jx_moon + d_Jx_sun;
            dVy_1  = - GLONASS_GM / r0_3 * y_0 + 3/2 * GLONASS_C20 * GLONASS_GM * pow(GLONASS_EARTH_RADIUS, 2) / r0_5 * y_0 * (1 - 5 * 1 / (z_0*z_0)) + d_Jy_moon + d_Jy_sun;
            dVz_1  = - GLONASS_GM / r0_3 * z_0 + 3/2 * GLONASS_C20 * GLONASS_GM * pow(GLONASS_EARTH_RADIUS, 2) / r0_5 * z_0 * (3 - 5 * 1 / (z_0*z_0)) + d_Jz_moon + d_Jz_sun;

            dx_1  = Vx_0;
            dy_1  = Vy_0;
            dz_1  = Vz_0;

            // Runge-Kutta Integration Stage K2
            Vx_1  = Vx_0 + 1/2 * tau * dVx_1;
            Vy_1  = Vy_0 + 1/2 * tau * dVy_1;
            Vz_1  = Vz_0 + 1/2 * tau * dVz_1;

            x_1   = x_0  + 1/2 * tau * dx_1;
            y_1   = y_0  + 1/2 * tau * dy_1;
            z_1   = z_0  + 1/2 * tau * dz_1;

            r1  = sqrt( x_1*x_1 + y_1*y_1 + z_1*z_1 );
            r1_2    = r1*r1;
            r1_3    = r1*r1*r1;
            r1_5    = r1*r1*r1*r1*r1;

            dVx_2  = - GLONASS_GM / r1_3 * x_1 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r1_5 * x_1 * (1 - 5 * (z_1*z_1) / r1_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * x_1 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_1 + Ax_0;
            dVy_2  = - GLONASS_GM / r1_3 * y_1 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r1_5 * y_1 * (1 - 5 * (z_1*z_1) / r1_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * y_1 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_1 + Ay_0;
            dVz_2  = - GLONASS_GM / r1_3 * z_1 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r1_5 * z_1 * (3 - 5 * (z_1*z_1) / r1_2) + Az_0;

            dx_2  = Vx_1;
            dy_2  = Vy_1;
            dz_2  = Vz_1;

            // Runge-Kutta Integration Stage K2
            Vx_2  = Vx_0 + 1/2 * tau * dVx_2;
            Vy_2  = Vy_0 + 1/2 * tau * dVy_2;
            Vz_2  = Vz_0 + 1/2 * tau * dVz_2;

            x_2   = x_0  + 1/2 * tau * dx_2;
            y_2   = y_0  + 1/2 * tau * dy_2;
            z_2   = z_0  + 1/2 * tau * dz_2;

            r2  = sqrt( x_2*x_2 + y_2*y_2 + z_2*z_2 );
            r2_2 = r2*r2;
            r2_3 = r2*r2*r2;
            r2_5 = r2*r2*r2*r2*r2;


            dVx_3  = - GLONASS_GM / r2_3 * x_2 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r2_5 * x_2 * (1 - 5 * (z_2*z_2) / r2_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * x_2 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_2 + Ax_0;
            dVy_3  = - GLONASS_GM / r2_3 * y_2 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r2_5 * y_2 * (1 - 5 * (z_2*z_2) / r2_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * y_2 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_2 + Ay_0;
            dVz_3  = - GLONASS_GM / r2_3 * z_2 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r2_5 * z_2 * (3 - 5 * (z_2*z_2) / r2_2) + Az_0;

            dx_3  = Vx_2;
            dy_3  = Vy_2;
            dz_3  = Vz_2;

            // Runge-Kutta Integration Stage K3
            Vx_3  = Vx_0 + tau * dVx_3;
            Vy_3  = Vy_0 + tau * dVy_3;
            Vz_3  = Vz_0 + tau * dVz_3;

            x_3   = x_0  + tau * dx_3;
            y_3   = y_0  + tau * dy_3;
            z_3   = z_0  + tau * dz_3;

            r3  = sqrt( x_3*x_3 + y_3*y_3 + z_3*z_3 );
            r3_2 = r3*r3;
            r3_3 = r3*r3*r3;
            r3_5 = r3*r3*r3*r3*r3;

            dVx_4  = - GLONASS_GM / r3_3 * x_3 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r3_5 * x_3 * (1 - 5 * (z_3*z_3) / r3_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * x_3 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_3 + Ax_0;
            dVy_4  = - GLONASS_GM / r3_3 * y_3 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r3_5 * y_3 * (1 - 5 * (z_3*z_3) / r3_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * y_3 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_3 + Ay_0;
            dVz_4  = - GLONASS_GM / r3_3 * z_3 - 3/2 * GLONASS_J2 * GLONASS_GM * pow(GLONASS_SEMI_MAJOR_AXIS, 2) / r3_5 * z_3 * (3 - 5 * (z_3*z_3) / r3_2) + Az_0;

            dx_4  = Vx_3;
            dy_4  = Vy_3;
            dz_4  = Vz_3;

            // Final results showcased here
            d_satvel_X  = Vx_0 + 1/6 * tau * ( dVx_1 + 2 * dVx_2 + 2 * dVx_3 + dVx_4 );
            d_satvel_Y  = Vy_0 + 1/6 * tau * ( dVy_1 + 2 * dVy_2 + 2 * dVy_3 + dVy_4 );
            d_satvel_Z  = Vz_0 + 1/6 * tau * ( dVz_1 + 2 * dVz_2 + 2 * dVz_3 + dVz_4 );

            d_satpos_X   = x_0 + 1/6 * tau * ( dx_1 + 2 * dx_2 + 2 * dx_3 + dx_4 );
            d_satpos_Y   = y_0 + 1/6 * tau * ( dy_1 + 2 * dy_2 + 2 * dy_3 + dy_4 );
            d_satpos_Z   = z_0 + 1/6 * tau * ( dz_1 + 2 * dz_2 + 2 * dz_3 + dz_4 );

        }

    // Time from ephemeris reference clock
    //tk = check_t(transmitTime - d_Toc);

    //double dtr_s = d_A_f0 + d_A_f1 * tk + d_A_f2 * tk * tk;

    /* relativity correction */
    //dtr_s -= 2.0 * sqrt(GM * GLONASS_a) * d_e_eccentricity * sin(E) / (GPS_C_m_s * GPS_C_m_s);

    return d_dtr;
}
