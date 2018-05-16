/*!
 * \file glonass_gnav_ephemeris_test.cc.cc
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 *
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


#include "gnss_signal_processing.h"
#include "glonass_gnav_ephemeris.h"


TEST(GlonassGnavEphemerisTest, ComputeGlonassTime)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2016;
    gnav_eph.d_N_T = 367;
    boost::posix_time::time_duration t(0, 0, 7560);
    boost::gregorian::date d(gnav_eph.d_yr, 1, 1);
    boost::gregorian::days d2(gnav_eph.d_N_T);
    d = d + d2;

    boost::gregorian::date expected_gdate;
    boost::posix_time::time_duration expected_gtime;

    boost::posix_time::ptime gtime = gnav_eph.compute_GLONASS_time(7560);
    expected_gdate = gtime.date();
    expected_gtime = gtime.time_of_day();

    // Perform assertions of decoded fields
    ASSERT_TRUE(expected_gdate.year() - d.year() < FLT_EPSILON);
    ASSERT_TRUE(expected_gdate.month() - d.month() < FLT_EPSILON);
    ASSERT_TRUE(expected_gdate.day() - d.day() < FLT_EPSILON);
    ASSERT_TRUE(expected_gtime.hours() - t.hours() < FLT_EPSILON);
    ASSERT_TRUE(expected_gtime.minutes() - t.minutes() < FLT_EPSILON);
    ASSERT_TRUE(expected_gtime.seconds() - t.seconds() < FLT_EPSILON);
}


/*!
 * \brief Testing conversion from GLONASST to GPST
 * \test Tests scenario for N_T when greater than 365 days. Possible values here from 1 to 365*4
 */
TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsT1)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2004;
    gnav_eph.d_N_T = 366 + 28;

    double glo2utc = 3600 * 3;
    double tod = 48600;
    double week = 0.0;
    double tow = 0.0;
    double true_leap_sec = 13;
    double true_week = 1307;
    double true_tow = 480600 + true_leap_sec;

    gnav_eph.glot_to_gpst(tod + glo2utc, 0.0, 0.0, &week, &tow);

    // Perform assertions of decoded fields
    ASSERT_TRUE(week - true_week < FLT_EPSILON);
    ASSERT_TRUE(tow - true_tow < FLT_EPSILON);
}


/*!
 * \brief Testing conversion from GLONASST to GPST
 * \test This version tests the conversion for offsets greater than 30 in a leap year
 */
TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsT2)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2016;
    gnav_eph.d_N_T = 268;

    double glo2utc = 3600 * 3;
    double tod = 7560;
    double week = 0.0;
    double tow = 0.0;
    double true_leap_sec = 17;
    double true_week = 1915;
    double true_tow = 518400 + true_leap_sec + tod;

    gnav_eph.glot_to_gpst(tod + glo2utc, 0.0, 0.0, &week, &tow);

    // Perform assertions of decoded fields
    ASSERT_TRUE(week - true_week < FLT_EPSILON);
    ASSERT_TRUE(tow - true_tow < FLT_EPSILON);
}


/*!
 * \brief Testing conversion from GLONASST to GPST
 * \test This version tests the conversion around the vicinity of February 29 days when in leap year
 */
TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsT3)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2016;
    gnav_eph.d_N_T = 62;

    double glo2utc = 3600 * 3;
    double tod = 7560;
    double week = 0.0;
    double tow = 0.0;
    double true_leap_sec = 17;
    double true_week = 1886;
    double true_tow = 259200 + true_leap_sec + tod;

    gnav_eph.glot_to_gpst(tod + glo2utc, 0.0, 0.0, &week, &tow);

    // Perform assertions of decoded fields
    ASSERT_TRUE(week - true_week < FLT_EPSILON);
    ASSERT_TRUE(tow - true_tow < FLT_EPSILON);
}
