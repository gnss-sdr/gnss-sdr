/*!
 * \file glonass_gnav_ephemeris.cc
 * \brief  Interface of a GLONASS GNAV EPHEMERIS storage and orbital model functions
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "glonass_gnav_ephemeris.h"
#include "GLONASS_L1_L2_CA.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>
#include <cmath>
#include <string>


boost::posix_time::ptime Glonass_Gnav_Ephemeris::compute_GLONASS_time(double offset_time) const
{
    int J = 0;
    if (d_N_T >= 1 && d_N_T <= 366)
        {
            J = 1;
        }
    else if (d_N_T >= 367 && d_N_T <= 731)
        {
            J = 2;
        }
    else if (d_N_T >= 732 && d_N_T <= 1096)
        {
            J = 3;
        }
    else if (d_N_T >= 1097 && d_N_T <= 1461)
        {
            J = 4;
        }

    const boost::posix_time::time_duration t(0, 0, offset_time + d_tau_c + d_tau_n);
    const boost::gregorian::date d1(d_yr - J + 1.0, 1, 1);
    const boost::gregorian::days d2(d_N_T - 1);
    boost::posix_time::ptime glonass_time(d1 + d2, t);

    return glonass_time;
}


boost::posix_time::ptime Glonass_Gnav_Ephemeris::glot_to_utc(double offset_time, double glot2utc_corr) const
{
    const double glot2utc = 3 * 3600;
    int J = 0;
    if (d_N_T >= 1 && d_N_T <= 366)
        {
            J = 1;
        }
    else if (d_N_T >= 367 && d_N_T <= 731)
        {
            J = 2;
        }
    else if (d_N_T >= 732 && d_N_T <= 1096)
        {
            J = 3;
        }
    else if (d_N_T >= 1097 && d_N_T <= 1461)
        {
            J = 4;
        }

    const double tod = offset_time - glot2utc + glot2utc_corr + d_tau_n;
    const boost::posix_time::time_duration t(0, 0, tod);
    const boost::gregorian::date d1(d_yr - J + 1.0, 1, 1);
    const boost::gregorian::days d2(d_N_T - 1);
    boost::posix_time::ptime utc_time(d1 + d2, t);

    return utc_time;
}


void Glonass_Gnav_Ephemeris::glot_to_gpst(double tod_offset, double glot2utc_corr, double glot2gpst_corr, int32_t* wn, double* tow) const
{
    const double glot2utc = 3 * 3600;
    double days = 0.0;
    double total_sec = 0.0;
    double sec_of_day = 0.0;
    int i = 0;
    int J = 0;
    if (d_N_T >= 1 && d_N_T <= 366)
        {
            J = 1;
        }
    else if (d_N_T >= 367 && d_N_T <= 731)
        {
            J = 2;
        }
    else if (d_N_T >= 732 && d_N_T <= 1096)
        {
            J = 3;
        }
    else if (d_N_T >= 1097 && d_N_T <= 1461)
        {
            J = 4;
        }

    const boost::gregorian::date gps_epoch{1980, 1, 6};

    // tk is relative to UTC(SU) + 3.00 hrs, so we need to convert to utc and add corrections
    // tk plus 10 sec is the true tod since get_TOW is called when in str5
    const double tod = tod_offset - glot2utc;

    const boost::posix_time::time_duration t(0, 0, tod);
    const boost::gregorian::date d1(d_yr - J + 1.0, 1, 1);
    const boost::gregorian::days d2(d_N_T - 1);
    const boost::posix_time::ptime utc_time(d1 + d2, t);
    const boost::gregorian::date utc_date = utc_time.date();
    boost::posix_time::ptime gps_time;

    // Adjust for leap second correction
    for (i = 0; GLONASS_LEAP_SECONDS[i][0] > 0; i++)
        {
            const boost::posix_time::time_duration t3(GLONASS_LEAP_SECONDS[i][3], GLONASS_LEAP_SECONDS[i][4], GLONASS_LEAP_SECONDS[i][5]);
            const boost::gregorian::date d3(GLONASS_LEAP_SECONDS[i][0], GLONASS_LEAP_SECONDS[i][1], GLONASS_LEAP_SECONDS[i][2]);
            const boost::posix_time::ptime ls_time(d3, t3);
            if (utc_time >= ls_time)
                {
                    // We add the leap second when going from utc to gpst
                    gps_time = utc_time + boost::posix_time::time_duration(0, 0, fabs(GLONASS_LEAP_SECONDS[i][6]));
                    break;
                }
        }

    // Total number of days
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
    const double half_day = 43200.0;  // seconds
    double corrTime = time;
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
    const double dt = check_t(transmitTime - d_t_b);
    d_satClkDrift = -(d_tau_n + timeCorrUTC - d_gamma_n * dt);
    // Correct satellite group delay and missing relativistic term here
    // d_satClkDrift-=d_TGD;

    return d_satClkDrift;
}
