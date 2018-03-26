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
#include "gnss_satellite.h"
#include "GLONASS_L1_L2_CA.h"
#include <cmath>


Glonass_Gnav_Ephemeris::Glonass_Gnav_Ephemeris()
{
    d_m = 0.0;            //!< String number within frame [dimensionless]
    d_t_k = 0.0;          //!< GLONASS Time (UTC(SU) + 3 h) referenced to the beginning of the frame within the current day [s]
    d_t_b = 0.0;          //!< Reference ephemeris relative time in GLONASS Time (UTC(SU) + 3 h). Index of a time interval within current day according to UTC(SU) + 03 hours 00 min. [s]
    d_M = 0.0;            //!< Type of satellite transmitting navigation signal [dimensionless]
    d_gamma_n = 0.0;      //!< Relative deviation of predicted carrier frequency value of n- satellite from nominal value at the instant tb [dimensionless]
    d_tau_n = 0.0;        //!< Correction to the nth satellite time (tn) relative to GLONASS time (te),
    d_Xn = 0.0;           //!< Earth-fixed coordinate x of the satellite in PZ-90.02 coordinate system [km].
    d_Yn = 0.0;           //!< Earth-fixed coordinate y of the satellite in PZ-90.02 coordinate system [km]
    d_Zn = 0.0;           //!< Earth-fixed coordinate z of the satellite in PZ-90.02 coordinate system [km]
    d_VXn = 0.0;          //!< Earth-fixed velocity coordinate x of the satellite in PZ-90.02 coordinate system [km/s]
    d_VYn = 0.0;          //!< Earth-fixed velocity coordinate y of the satellite in PZ-90.02 coordinate system [km/s]
    d_VZn = 0.0;          //!< Earth-fixed velocity coordinate z of the satellite in PZ-90.02 coordinate system [km/s]
    d_AXn = 0.0;          //!< Earth-fixed acceleration coordinate x of the satellite in PZ-90.02 coordinate system [km/s^2]
    d_AYn = 0.0;          //!< Earth-fixed acceleration coordinate y of the satellite in PZ-90.02 coordinate system [km/s^2]
    d_AZn = 0.0;          //!< Earth-fixed acceleration coordinate z of the satellite in PZ-90.02 coordinate system [km/s^2]
    d_B_n = 0.0;          //!< Health flag [dimensionless]
    d_P = 0.0;            //!< Technological parameter of control segment, indication the satellite operation mode in respect of time parameters [dimensionless]
    d_N_T = 0.0;          //!< Current date, calendar number of day within four-year interval starting from the 1-st of January in a leap year [days]
    d_F_T = 0.0;          //!< Parameter that provides the predicted satellite user range accuracy at time tb [dimensionless]
    d_n = 0.0;            //!< Index of the satellite transmitting given navigation signal. It corresponds to a slot number within GLONASS constellation
    d_Delta_tau_n = 0.0;  //!< Time difference between navigation RF signal transmitted in L2 sub- band and aviation RF signal transmitted in L1 sub-band by nth satellite. [dimensionless]
    d_E_n = 0.0;          //!< Characterises "age" of a current information [days]
    d_P_1 = 0.0;          //!< Flag of the immediate data updating [minutes]
    d_P_2 = false;        //!< Flag of oddness ("1") or evenness ("0") of the value of (tb) [dimensionless]
    d_P_3 = false;        //!< Flag indicating a number of satellites for which almanac is transmitted within given frame: "1" corresponds to 5 satellites and "0" corresponds to 4 satellites [dimensionless]
    d_P_4 = false;        //!< Flag to show that ephemeris parameters are present. "1" indicates that updated ephemeris or frequency/time parameters have been uploaded by the control segment [dimensionless]
    d_l3rd_n = false;     //!< Health flag for nth satellite; ln = 0 indicates the n-th satellite is helthy, ln = 1 indicates malfunction of this nth satellite [dimensionless]
    d_l5th_n = false;     //!< Health flag for nth satellite; ln = 0 indicates the n-th satellite is helthy, ln = 1 indicates malfunction of this nth satellite [dimensionless]

    // Satellite Identification Information
    i_satellite_freq_channel = 0;  //!< SV Frequency Channel Number
    i_satellite_PRN = 0;           //!< SV PRN Number, equivalent to slot number for compatibility with GPS
    i_satellite_slot_number = 0;   //!< SV Slot Number
    d_yr = 1972;                   //!< Current year, defaults to 1972 (UTC Epoch with leap seconds)
    d_satClkDrift = 0.0;           //!< GLONASS clock error
    d_dtr = 0.0;                   //!< relativistic clock correction term
    d_iode = 0.0;                  //!< Issue of data, ephemeris (Bit 0-6 of tb)
    d_tau_c = 0.0;
    d_TOW = 0.0;  // tow of the start of frame
    d_WN = 0.0;   //  week number of the start of frame
    d_tod = 0.0;
}


boost::posix_time::ptime Glonass_Gnav_Ephemeris::compute_GLONASS_time(const double offset_time) const
{
    boost::posix_time::time_duration t(0, 0, offset_time + d_tau_c + d_tau_n);
    boost::gregorian::date d1(d_yr, 1, 1);
    boost::gregorian::days d2(d_N_T - 1);
    boost::posix_time::ptime glonass_time(d1 + d2, t);

    return glonass_time;
}


boost::posix_time::ptime Glonass_Gnav_Ephemeris::glot_to_utc(const double offset_time, const double glot2utc_corr) const
{
    double tod = 0.0;
    double glot2utc = 3 * 3600;

    tod = offset_time - glot2utc + glot2utc_corr + d_tau_n;
    boost::posix_time::time_duration t(0, 0, tod);
    boost::gregorian::date d1(d_yr, 1, 1);
    boost::gregorian::days d2(d_N_T - 1);
    boost::posix_time::ptime utc_time(d1 + d2, t);

    return utc_time;
}


void Glonass_Gnav_Ephemeris::glot_to_gpst(double tod_offset, double glot2utc_corr, double glot2gpst_corr, double* wn, double* tow) const
{
    double tod = 0.0;
    double glot2utc = 3 * 3600;
    double days = 0.0;
    double total_sec = 0.0, sec_of_day = 0.0;
    int i = 0;

    boost::gregorian::date gps_epoch{1980, 1, 6};

    // tk is relative to UTC(SU) + 3.00 hrs, so we need to convert to utc and add corrections
    // tk plus 10 sec is the true tod since get_TOW is called when in str5
    tod = tod_offset - glot2utc;

    boost::posix_time::time_duration t(0, 0, tod);
    boost::gregorian::date d1(d_yr, 1, 1);
    boost::gregorian::days d2(d_N_T - 1);
    boost::posix_time::ptime utc_time(d1 + d2, t);
    boost::gregorian::date utc_date = utc_time.date();
    boost::posix_time::ptime gps_time;

    // Adjust for leap second correction
    for (i = 0; GLONASS_LEAP_SECONDS[i][0] > 0; i++)
        {
            boost::posix_time::time_duration t3(GLONASS_LEAP_SECONDS[i][3], GLONASS_LEAP_SECONDS[i][4], GLONASS_LEAP_SECONDS[i][5]);
            boost::gregorian::date d3(GLONASS_LEAP_SECONDS[i][0], GLONASS_LEAP_SECONDS[i][1], GLONASS_LEAP_SECONDS[i][2]);
            boost::posix_time::ptime ls_time(d3, t3);
            if (utc_time >= ls_time)
                {
                    // We add the leap second when going from utc to gpst
                    gps_time = utc_time + boost::posix_time::time_duration(0, 0, fabs(GLONASS_LEAP_SECONDS[i][6]));
                    break;
                }
        }

    // Total number of days
    std::string fdat = boost::posix_time::to_simple_string(gps_time);
    days = static_cast<double>((utc_date - gps_epoch).days());

    // Total number of seconds
    sec_of_day = static_cast<double>((gps_time.time_of_day()).total_seconds());
    total_sec = days * 86400 + sec_of_day;

    // Compute Week number
    *wn = floor(total_sec / 604800);

    // Compute the arithmetic modules to wrap around range
    *tow = total_sec - 604800 * floor(total_sec / 604800);
    // Perform corrections from fractional seconds
    *tow += glot2utc_corr + glot2gpst_corr;
}


double Glonass_Gnav_Ephemeris::check_t(double time)
{
    double corrTime;
    double half_day = 43200.0;  // seconds
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
