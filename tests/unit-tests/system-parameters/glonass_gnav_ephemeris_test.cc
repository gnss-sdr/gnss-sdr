/*!
 * \file glonass_gnav_ephemeris_test.cc.cc
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 *
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
#include "gnss_signal_replica.h"


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


TEST(GlonassGnavEphemerisTest, ComputeGlonassTime2019)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2019;
    gnav_eph.d_N_T = 1366;
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


TEST(GlonassGnavEphemerisTest, ComputeGlonassTime2020)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2020;
    gnav_eph.d_N_T = 62;
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
    int week = 0.0;
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
    int week = 0.0;
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
    int week = 0.0;
    double tow = 0.0;
    double true_leap_sec = 17;
    double true_week = 1886;
    double true_tow = 259200 + true_leap_sec + tod;

    gnav_eph.glot_to_gpst(tod + glo2utc, 0.0, 0.0, &week, &tow);

    // Perform assertions of decoded fields
    ASSERT_TRUE(week - true_week < FLT_EPSILON);
    ASSERT_TRUE(tow - true_tow < FLT_EPSILON);
}


TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsT4)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2019;
    gnav_eph.d_N_T = 1366;

    double glo2utc = 3600 * 3;
    double tod = 7560;
    int week = 0.0;
    double tow = 0.0;
    double true_leap_sec = 18;
    double true_week = 2072;
    double true_tow = 447120 + true_leap_sec + tod;

    gnav_eph.glot_to_gpst(tod + glo2utc, 0.0, 0.0, &week, &tow);

    // Perform assertions of decoded fields
    ASSERT_TRUE(week - true_week < FLT_EPSILON);
    ASSERT_TRUE(tow - true_tow < FLT_EPSILON);
}


TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsT5)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2020;
    gnav_eph.d_N_T = 62;

    double glo2utc = 3600 * 3;
    double tod = 7560;
    int week = 0.0;
    double tow = 0.0;
    double true_leap_sec = 18;
    double true_week = 2095;
    double true_tow = 259200 + true_leap_sec + tod;

    gnav_eph.glot_to_gpst(tod + glo2utc, 0.0, 0.0, &week, &tow);

    // Perform assertions of decoded fields
    ASSERT_TRUE(week - true_week < FLT_EPSILON);
    ASSERT_TRUE(tow - true_tow < FLT_EPSILON);
}
