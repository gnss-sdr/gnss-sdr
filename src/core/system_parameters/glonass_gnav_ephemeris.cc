/*!
 * \file gps_ephemeris.cc
 * \brief  Interface of a GPS EPHEMERIS storage and orbital model functions
 *
 * See http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf Appendix II
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "gps_ephemeris.h"
#include <cmath>
#include "GLONASS_L1_CA.h"
#include "gnss_satellite.h"

Gps_Ephemeris::Gps_Ephemeris()
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
    d_l_n = 0.0;             //!< Health flag for nth satellite; ln = 0 indicates the n-th satellite is helthy, ln = 1 indicates malfunction of this nth satellite [dimensionless]

    // clock terms derived from ephemeris data
    d_satClkDrift = 0.0;    //!< GLONASS clock error
    d_dtr = 0.0;
}


void Glonass_GNAV_Ephemeris::gravitational_perturbations()
{
    double T;
    double sigma_days;
    double tau_prime;
    double Omega_moon;
    double q_moon;
    double q_sun;
    double Tau_11; double Tau_12;
    double Eta_11; double Eta_12;
    double xi_11; double xi_12;
    double xi_star;
    double eta_star;
    double zeta_star;
    double E_moon;
    double E_sun;
    double xi_11;
    double xi_12;
    double eta_11;
    double eta_12;
    double zeta_11;
    double zeta_12;

    double sin_theta_moon;
    double cos_theta_moon;
    double theta_moon;
    double xi_moon_e;
    double eta_moon_e;
    double zeta_moon_e;

    double sin_theta_sun;
    double cos_theta_sun;
    double theta_sun;
    double xi_sun_e;
    double eta_sun_e;
    double zeta_sun_e;

    double r_moon_e;
    double mu_bar_moon;
    double x_bar_moon;
    double y_bar_moon;
    double Delta_o_moon;
    double mu_bar_sun;
    double x_bar_sun;
    double y_bar_sun;
    double Delta_o_sun;


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
    mu_bar_moon = GLONASS_MOON_GM/(d_r_moon_e*d_r_moon_e);
    x_bar_moon  = d_satpos_X/d_r_moon_e;
    y_bar_moon  = d_satpos_Y/d_r_moon_e;
    z_bar_moon  = d_satpos_Z/d_r_moon_e;
    Delta_o_moon = sqrt((d_xi_moon_e - x_bar_moon)*(d_xi_moon_e - x_bar_moon) + (d_eta_moon_e - y_bar_moon)*(d_eta_moon_e - y_bar_moon) + (d_zeta_moon_e - z_bar_moon)*(d_zeta_moon_e - z_bar_moon));

    mu_bar_sun = GLONASS_MOON_GM/(d_r_sun_e*d_r_sun_e);
    x_bar_sun  = d_satpos_X/d_r_sun_e;
    y_bar_sun  = d_satpos_Y/d_r_sun_e;
    z_bar_sun  = d_satpos_Z/d_r_sun_e;
    Delta_o_sun = sqrt((d_xi_sun_e - x_bar_sun)*(d_xi_sun_e - x_bar_sun) + (d_eta_sun_e - y_bar_sun)*(d_eta_sun_e - y_bar_sun) + (d_zeta_sun_e - z_bar_sun)*(d_zeta_sun_e - z_bar_sun));

    d_Jx_moon = mu_bar_moon*(xi_moon_e - x_bar_moon)/pow(Delta_o_moon,3.0) - xi_moon_e;
    d_Jy_moon = mu_bar_moon*(eta_moon_e - y_bar_moon)/pow(Delta_o_moon,3.0) - eta_moon_e;
    d_Jz_moon = mu_bar_moon*(zeta_moon_e - z_bar_moon)/pow(Delta_o_moon,3.0) - zeta_moon_e;

    d_Jx_sun = mu_bar_sun*(xi_sun_e - x_bar_sun)/pow(Delta_o_sun,3.0) - xi_sun_e;
    d_Jy_sun = mu_bar_sun*(eta_sun_e - y_bar_sun)/pow(Delta_o_sun,3.0) - eta_sun_e;
    d_Jz_sun = mu_bar_sun*(zeta_sun_e - z_bar_sun)/pow(Delta_o_sun,3.0) - zeta_sun_e;

}


double Glonass_GNAV_Ephemeris::check_t(double time)
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


// 20.3.3.3.3.1 User Algorithm for SV Clock Correction.
double Glonass_GNAV_Ephemeris::sv_clock_drift(double transmitTime, double timeCorrUTC)
{
    double dt;
    dt = check_t(transmitTime - d_t_b);
    d_satClkDrift = -(d_tau_n + timeCorrUTC - d_gamma_n * dt);
    //Correct satellite group delay and missing relativistic term here
    //d_satClkDrift-=d_TGD;

    return d_satClkDrift;
}


// compute the relativistic correction term
double Glonass_GNAV_Ephemeris::sv_clock_relativistic_term(double transmitTime)
{
    double tk;
    double a;
    double n;
    double n0;
    double E;
    double E_old;
    double dE;
    double M;

    // Restore semi-major axis
    a = d_sqrt_A * d_sqrt_A;

    // Time from ephemeris reference epoch
    tk = check_t(transmitTime - d_Toe);

    // Computed mean motion
    n0 = sqrt(GM / (a * a * a));
    // Corrected mean motion
    n = n0 + d_Delta_n;
    // Mean anomaly
    M = d_M_0 + n * tk;

    // Reduce mean anomaly to between 0 and 2pi
    //M = fmod((M + 2.0 * GPS_PI), (2.0 * GPS_PI));

    // Initial guess of eccentric anomaly
    E = M;

    // --- Iteratively compute eccentric anomaly ----------------------------
    for (int ii = 1; ii < 20; ii++)
        {
            E_old   = E;
            E       = M + d_e_eccentricity * sin(E);
            dE      = fmod(E - E_old, 2.0 * GPS_PI);
            if (fabs(dE) < 1e-12)
                {
                    //Necessary precision is reached, exit from the loop
                    break;
                }
        }

    // Compute relativistic correction term
    d_dtr = F * d_e_eccentricity * d_sqrt_A * sin(E);
    return d_dtr;
}


double Glonass_GNAV_Ephemeris::simplifiedSatellitePosition(double transmitTime)
{
    double dt = 0.0;
    double tk = 0.0;
    int numberOfIntegrations = 0;

    // Find time difference
    dt = check_t(transmitTime - d_t_b);

    // Calculate clock correction
    satClkCorr = -(d_tau_n + d_tau_c - d_gamma_n * dt);

    // Find integration time
    tk    = dt - satClkCorr;

    // Check if to integrate forward or backward
    if tk < 0
        tau      = -60;
    else
        tau      = 60;
    end

    // x,y,z coordinates to meters
    x0  = d_satpos_X * 1e3;
    y0  = d_satpos_Y * 1e3;
    z0  = d_satpos_Z * 1e3;

    // x,y,z velocities to meters/s
    Vx0 = d_satvel_X * 1e3;
    Vy0 = d_satvel_Y * 1e3;
    Vz0 = d_satvel_Z * 1e3;

    // x,y,z accelerations to meters/sec^2
    Ax0 = d_satacc_X * 1e3;
    Ay0 = d_satacc_Y * 1e3;
    Az0 = d_satacc_Z * 1e3;

    for(numberOfIntegrations = tau; numberOfIntegrations < tk+tau; numberOfIntegrations += tau)
        {
            // Check if last integration step. If last integration step, make one more step that has the remaining time length...
            if(fabs(numberOfIntegrations) > fabs(tk))
                {
                    // if there is more time left to integrate...
                    if mod(tk,tau) ~= 0
                        {
                            tau = mod(time,tau);
                        }
                    else // otherwise make a zero step.
                        {
                            tau = 0;
                        }
                }

            // Runge-Kutta Integration Stage K1
            r0      = sqrt(x0*x0 + y0*y0 + z0*z0);
            r0_2    = r0*r0;
            r0_3    = r0*r0*r0;
            r0_5    = r0*r0*r0*r0*r0;

            dVx_1  = - GLONASS_GM / r0_3 * x0 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a * GLONASS_a) / r0_5 * x0 * (1 - 5 * (z0*z0) / r0_2) + (GLONASS_OMEGA_EARTH_DOT*GLONASS_OMEGA_EARTH_DOT) * x0 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy0 + Ax0;
            dVy_1  = - GLONASS_GM / r0_3 * y0 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a * GLONASS_a) / r0_5 * y0 * (1 - 5 * (z0*z0) / r0_2) + (GLONASS_OMEGA_EARTH_DOT*GLONASS_OMEGA_EARTH_DOT) * y0 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx0 + Ay0;
            dVz_1  = - GLONASS_GM / r0_3 * z0 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a * GLONASS_a) / r0_5 * z0 * (3 - 5 * (z0*z0) / r0_2) + Az0;

            dx_1  = Vx0;
            dy_1  = Vy0;
            dz_1  = Vz0;

            // Runge-Kutta Integration Stage K2
            Vx_1  = Vx0 + 1/2 * tau * dVx_1;
            Vy_1  = Vy0 + 1/2 * tau * dVy_1;
            Vz_1  = Vz0 + 1/2 * tau * dVz_1;

            x_1   = x0  + 1/2 * tau * dx_1;
            y_1   = y0  + 1/2 * tau * dy_1;
            z_1   = z0  + 1/2 * tau * dz_1;

            r1  = sqrt( x_1*x_1 + y_1*y_1 + z_1*z_1 );
            r1_2    = r1*r1;
            r1_3    = r1*r1*r1;
            r1_5    = r1*r1*r1*r1*r1;

            dVx_2  = - GLONASS_GM / r1_3 * x_1 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r1_5 * x_1 * (1 - 5 * (z_1*z_1) / r1_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * x_1 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_1 + Ax0;
            dVy_2  = - GLONASS_GM / r1_3 * y_1 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r1_5 * y_1 * (1 - 5 * (z_1*z_1) / r1_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * y_1 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_1 + Ay0;
            dVz_2  = - GLONASS_GM / r1_3 * z_1 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r1_5 * z_1 * (3 - 5 * (z_1*z_1) / r1_2) + Az0;

            dx_2  = Vx_1;
            dy_2  = Vy_1;
            dz_2  = Vz_1;

            // Runge-Kutta Integration Stage K2
            Vx_2  = Vx0 + 1/2 * tau * dVx_2;
            Vy_2  = Vy0 + 1/2 * tau * dVy_2;
            Vz_2  = Vz0 + 1/2 * tau * dVz_2;

            x_2   = x0  + 1/2 * tau * dx_2;
            y_2   = y0  + 1/2 * tau * dy_2;
            z_2   = z0  + 1/2 * tau * dz_2;

            r2  = sqrt( x_2*x_2 + y_2*y_2 + z_2*z_2 );
            r2_2 = r2*r2;
            r2_3 = r2*r2*r2;
            r2_5 = r2*r2*r2*r2*r2;


            dVx_3  = - GLONASS_GM / r2_3 * x_2 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r2_5 * x_2 * (1 - 5 * (z_2*z_2) / r2_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * x_2 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_2 + Ax0;
            dVy_3  = - GLONASS_GM / r2_3 * y_2 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r2_5 * y_2 * (1 - 5 * (z_2*z_2) / r2_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * y_2 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_2 + Ay0;
            dVz_3  = - GLONASS_GM / r2_3 * z_2 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r2_5 * z_2 * (3 - 5 * (z_2*z_2) / r2_2) + Az0;

            dx_3  = Vx_2;
            dy_3  = Vy_2;
            dz_3  = Vz_2;

            // Runge-Kutta Integration Stage K3
            Vx_3  = Vx0 + tau * dVx_3;
            Vy_3  = Vy0 + tau * dVy_3;
            Vz_3  = Vz0 + tau * dVz_3;

            x_3   = x0  + tau * dx_3;
            y_3   = y0  + tau * dy_3;
            z_3   = z0  + tau * dz_3;

            r3  = sqrt( x_3*x_3 + y_3*y_3 + z_3*z_3 );
            r3_2 = r3*r3;
            r3_3 = r3*r3*r3;
            r3_5 = r3*r3*r3*r3*r3;

            dVx_4  = - GLONASS_GM / r3_3 * x_3 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r3_5 * x_3 * (1 - 5 * (z_3*z_3) / r3_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * x_3 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_3 + Ax0;
            dVy_4  = - GLONASS_GM / r3_3 * y_3 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r3_5 * y_3 * (1 - 5 * (z_3*z_3) / r3_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * y_3 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_3 + Ay0;
            dVz_4  = - GLONASS_GM / r3_3 * z_3 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r3_5 * z_3 * (3 - 5 * (z_3*z_3) / r3_2) + Az0;

            dx_4  = Vx_3;
            dy_4  = Vy_3;
            dz_4  = Vz_3;

            // Final results showcased here
            Vx0  = Vx0 + 1/6 * tau * ( dVx_1 + 2 * dVx_2 + 2 * dVx_3 + dVx_4 );
            Vy0  = Vy0 + 1/6 * tau * ( dVy_1 + 2 * dVy_2 + 2 * dVy_3 + dVy_4 );
            Vz0  = Vz0 + 1/6 * tau * ( dVz_1 + 2 * dVz_2 + 2 * dVz_3 + dVz_4 );

            x0   = x0 + 1/6 * tau * ( dx_1 + 2 * dx_2 + 2 * dx_3 + dx_4 );
            y0   = y0 + 1/6 * tau * ( dy_1 + 2 * dy_2 + 2 * dy_3 + dy_4 );
            z0   = z0 + 1/6 * tau * ( dz_1 + 2 * dz_2 + 2 * dz_3 + dz_4 );

        }

    // Reasign position, velocities and accelerations for next integration
    d_satpos_X = x0;
    d_satpos_Y = y0;
    d_satpos_Z = z0;

    d_satvel_X = Vx0;
    d_satvel_Y = Vy0;
    d_satvel_Z = Vz0;

    d_satacc_X = d_satacc_X;    // No change in accelerations reported over interval
    d_satacc_Y = d_satacc_Y;    // No change in accelerations reported over interval
    d_satacc_Z = d_satacc_Z;    // No change in accelerations reported over interval

    // Time from ephemeris reference clock
    tk = check_t(transmitTime - d_Toc);

    double dtr_s = d_A_f0 + d_A_f1 * tk + d_A_f2 * tk * tk;

    /* relativity correction */
    dtr_s -= 2.0 * sqrt(GM * GLONASS_a) * d_e_eccentricity * sin(E) / (GPS_C_m_s * GPS_C_m_s);

    return dtr_s;
}


double Glonass_GNAV_Ephemeris::satellitePosition(double transmitTime)
{
    double dt = 0.0;
    double tk = 0.0;
    int numberOfIntegrations = 0;

    // Find time difference
    dt = check_t(transmitTime - d_t_b);

    // Calculate clock correction
    satClkCorr = -(d_tau_n + d_tau_c - d_gamma_n * dt);

    // Find integration time
    tk    = dt - satClkCorr;

    // Check if to integrate forward or backward
    if tk < 0
        tau      = -60;
    else
        tau      = 60;
    end

    // Coordinates transformation to an inertial reference frame
    // x,y,z coordinates to meters
    x0  = d_satpos_X * 1e3;
    y0  = d_satpos_Y * 1e3;
    z0  = d_satpos_Z * 1e3;

    // x,y,z velocities to meters/s
    Vx0 = d_satvel_X * 1e3;
    Vy0 = d_satvel_Y * 1e3;
    Vz0 = d_satvel_Z * 1e3;

    // x,y,z accelerations to meters/sec^2
    Ax0 = d_satacc_X * 1e3;
    Ay0 = d_satacc_Y * 1e3;
    Az0 = d_satacc_Z * 1e3;

    for(numberOfIntegrations = tau; numberOfIntegrations < tk+tau; numberOfIntegrations += tau)
        {
            // Check if last integration step. If last integration step, make one more step that has the remaining time length...
            if(fabs(numberOfIntegrations) > fabs(tk))
                {
                    // if there is more time left to integrate...
                    if mod(tk,tau) ~= 0
                        {
                            tau = mod(time,tau);
                        }
                    else // otherwise make a zero step.
                        {
                            tau = 0;
                        }
                }

            // Runge-Kutta Integration Stage K1
            r0      = sqrt(x0*x0 + y0*y0 + z0*z0);
            r0_2    = r0*r0;
            r0_3    = r0*r0*r0;
            r0_5    = r0*r0*r0*r0*r0;

            dx_1 = Vx0;
            dy_1 = Vy0;
            dz_1 = Vz0;

            dVx_1  = - GLONASS_GM / r0_3 * x0 + 3/2 * GLONASS_C20 * GLONASS_GM * (GLONASS_EARTH_RADIUS * GLONASS_EARTH_RADIUS) / r0_5 * x0 * (1 - 5 * 1 / (z0*z0)) + Jx_moon + Jx_sun;
            dVy_1  = - GLONASS_GM / r0_3 * y0 + 3/2 * GLONASS_C20 * GLONASS_GM * (GLONASS_EARTH_RADIUS * GLONASS_EARTH_RADIUS) / r0_5 * y0 * (1 - 5 * 1 / (z0*z0)) + Jy_moon + Jy_sun;
            dVz_1  = - GLONASS_GM / r0_3 * z0 + 3/2 * GLONASS_C20 * GLONASS_GM * (GLONASS_EARTH_RADIUS * GLONASS_EARTH_RADIUS) / r0_5 * z0 * (3 - 5 * 1 / (z0*z0)) + Jz_moon + Jz_sun;

            dx_1  = Vx0;
            dy_1  = Vy0;
            dz_1  = Vz0;

            // Runge-Kutta Integration Stage K2
            Vx_1  = Vx0 + 1/2 * tau * dVx_1;
            Vy_1  = Vy0 + 1/2 * tau * dVy_1;
            Vz_1  = Vz0 + 1/2 * tau * dVz_1;

            x_1   = x0  + 1/2 * tau * dx_1;
            y_1   = y0  + 1/2 * tau * dy_1;
            z_1   = z0  + 1/2 * tau * dz_1;

            r1  = sqrt( x_1*x_1 + y_1*y_1 + z_1*z_1 );
            r1_2    = r1*r1;
            r1_3    = r1*r1*r1;
            r1_5    = r1*r1*r1*r1*r1;

            dVx_2  = - GLONASS_GM / r1_3 * x_1 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r1_5 * x_1 * (1 - 5 * (z_1*z_1) / r1_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * x_1 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_1 + Ax0;
            dVy_2  = - GLONASS_GM / r1_3 * y_1 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r1_5 * y_1 * (1 - 5 * (z_1*z_1) / r1_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * y_1 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_1 + Ay0;
            dVz_2  = - GLONASS_GM / r1_3 * z_1 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r1_5 * z_1 * (3 - 5 * (z_1*z_1) / r1_2) + Az0;

            dx_2  = Vx_1;
            dy_2  = Vy_1;
            dz_2  = Vz_1;

            // Runge-Kutta Integration Stage K2
            Vx_2  = Vx0 + 1/2 * tau * dVx_2;
            Vy_2  = Vy0 + 1/2 * tau * dVy_2;
            Vz_2  = Vz0 + 1/2 * tau * dVz_2;

            x_2   = x0  + 1/2 * tau * dx_2;
            y_2   = y0  + 1/2 * tau * dy_2;
            z_2   = z0  + 1/2 * tau * dz_2;

            r2  = sqrt( x_2*x_2 + y_2*y_2 + z_2*z_2 );
            r2_2 = r2*r2;
            r2_3 = r2*r2*r2;
            r2_5 = r2*r2*r2*r2*r2;


            dVx_3  = - GLONASS_GM / r2_3 * x_2 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r2_5 * x_2 * (1 - 5 * (z_2*z_2) / r2_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * x_2 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_2 + Ax0;
            dVy_3  = - GLONASS_GM / r2_3 * y_2 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r2_5 * y_2 * (1 - 5 * (z_2*z_2) / r2_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * y_2 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_2 + Ay0;
            dVz_3  = - GLONASS_GM / r2_3 * z_2 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r2_5 * z_2 * (3 - 5 * (z_2*z_2) / r2_2) + Az0;

            dx_3  = Vx_2;
            dy_3  = Vy_2;
            dz_3  = Vz_2;

            // Runge-Kutta Integration Stage K3
            Vx_3  = Vx0 + tau * dVx_3;
            Vy_3  = Vy0 + tau * dVy_3;
            Vz_3  = Vz0 + tau * dVz_3;

            x_3   = x0  + tau * dx_3;
            y_3   = y0  + tau * dy_3;
            z_3   = z0  + tau * dz_3;

            r3  = sqrt( x_3*x_3 + y_3*y_3 + z_3*z_3 );
            r3_2 = r3*r3;
            r3_3 = r3*r3*r3;
            r3_5 = r3*r3*r3*r3*r3;

            dVx_4  = - GLONASS_GM / r3_3 * x_3 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r3_5 * x_3 * (1 - 5 * (z_3*z_3) / r3_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * x_3 + 2 * GLONASS_OMEGA_EARTH_DOT * Vy_3 + Ax0;
            dVy_4  = - GLONASS_GM / r3_3 * y_3 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r3_5 * y_3 * (1 - 5 * (z_3*z_3) / r3_2) + (GLONASS_OMEGA_EARTH_DOT * GLONASS_OMEGA_EARTH_DOT) * y_3 - 2 * GLONASS_OMEGA_EARTH_DOT * Vx_3 + Ay0;
            dVz_4  = - GLONASS_GM / r3_3 * z_3 - 3/2 * GLONASS_J2 * GLONASS_GM * (GLONASS_a*GLONASS_a) / r3_5 * z_3 * (3 - 5 * (z_3*z_3) / r3_2) + Az0;

            dx_4  = Vx_3;
            dy_4  = Vy_3;
            dz_4  = Vz_3;

            // Final results showcased here
            Vx0  = Vx0 + 1/6 * tau * ( dVx_1 + 2 * dVx_2 + 2 * dVx_3 + dVx_4 );
            Vy0  = Vy0 + 1/6 * tau * ( dVy_1 + 2 * dVy_2 + 2 * dVy_3 + dVy_4 );
            Vz0  = Vz0 + 1/6 * tau * ( dVz_1 + 2 * dVz_2 + 2 * dVz_3 + dVz_4 );

            x0   = x0 + 1/6 * tau * ( dx_1 + 2 * dx_2 + 2 * dx_3 + dx_4 );
            y0   = y0 + 1/6 * tau * ( dy_1 + 2 * dy_2 + 2 * dy_3 + dy_4 );
            z0   = z0 + 1/6 * tau * ( dz_1 + 2 * dz_2 + 2 * dz_3 + dz_4 );

        }

    // Time from ephemeris reference clock
    tk = check_t(transmitTime - d_Toc);

    double dtr_s = d_A_f0 + d_A_f1 * tk + d_A_f2 * tk * tk;

    /* relativity correction */
    dtr_s -= 2.0 * sqrt(GM * GLONASS_a) * d_e_eccentricity * sin(E) / (GPS_C_m_s * GPS_C_m_s);

    return dtr_s;
}
