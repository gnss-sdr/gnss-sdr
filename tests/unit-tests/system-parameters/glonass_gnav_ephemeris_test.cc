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


#include "GLONASS_L1_L2_CA.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_utc_model.h"
#include "gnss_satellite.h"
#include "gnss_signal_replica.h"


TEST(GlonassGnavEphemerisTest, SatelliteRfLinkMatchesFrequencyChannelTable)
{
    EXPECT_EQ(2, Gnss_Satellite("Glonass", 20).get_rf_link());
    EXPECT_EQ(-5, Gnss_Satellite("Glonass", 27).get_rf_link());
    EXPECT_EQ(7, Gnss_Satellite("Glonass", 28).get_rf_link());

    for (const auto& prn_freq : GLONASS_PRN)
        {
            if (prn_freq.first == 0)
                {
                    continue;
                }

            const Gnss_Satellite satellite("Glonass", prn_freq.first);
            EXPECT_EQ(prn_freq.second, satellite.get_rf_link()) << "PRN " << prn_freq.first;
        }
}


TEST(GlonassGnavEphemerisTest, ComputeGlonassTime)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2016;
    gnav_eph.d_N_T = 367;  // 2016-01-02 in the GLONASS day scale

    const boost::posix_time::ptime gtime = gnav_eph.compute_GLONASS_time(7560);
    const boost::gregorian::date gdate = gtime.date();
    const boost::posix_time::time_duration gtod = gtime.time_of_day();

    // Perform assertions of decoded fields
    EXPECT_EQ(gdate.year(), 2016);
    EXPECT_EQ(gdate.month(), 1);
    EXPECT_EQ(gdate.day(), 2);
    EXPECT_EQ(gtod.hours(), 2);
    EXPECT_EQ(gtod.minutes(), 6);
    EXPECT_EQ(gtod.seconds(), 0);
}


TEST(GlonassGnavEphemerisTest, ComputeGlonassTime2019)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2019;
    gnav_eph.d_N_T = 1366;  // 2019-09-27 in the GLONASS day scale

    const boost::posix_time::ptime gtime = gnav_eph.compute_GLONASS_time(7560);
    const boost::gregorian::date gdate = gtime.date();
    const boost::posix_time::time_duration gtod = gtime.time_of_day();

    // Perform assertions of decoded fields
    EXPECT_EQ(gdate.year(), 2019);
    EXPECT_EQ(gdate.month(), 9);
    EXPECT_EQ(gdate.day(), 27);
    EXPECT_EQ(gtod.hours(), 2);
    EXPECT_EQ(gtod.minutes(), 6);
    EXPECT_EQ(gtod.seconds(), 0);
}


TEST(GlonassGnavEphemerisTest, ComputeGlonassTime2020)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2020;
    gnav_eph.d_N_T = 62;  // 2020-03-02, a Monday

    const boost::posix_time::ptime gtime = gnav_eph.compute_GLONASS_time(7560);
    const boost::gregorian::date gdate = gtime.date();
    const boost::posix_time::time_duration gtod = gtime.time_of_day();

    // Perform assertions of decoded fields
    EXPECT_EQ(gdate.year(), 2020);
    EXPECT_EQ(gdate.month(), 3);
    EXPECT_EQ(gdate.day(), 2);
    EXPECT_EQ(gtod.hours(), 2);
    EXPECT_EQ(gtod.minutes(), 6);
    EXPECT_EQ(gtod.seconds(), 0);
}


/*!
 * \brief Testing conversion from GLONASST to GPST
 * \test Tests scenario for N_T when greater than 365 days. Possible values here from 1 to 365*4
 */
TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsT1)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2004;
    gnav_eph.d_N_T = 366 + 28;  // 2004-01-29 in the GLONASS day scale, a Thursday

    double glo2utc = 3600 * 3;
    double tod = 48600;
    int week = 0;
    double tow = 0.0;
    double true_leap_sec = 13;
    int true_week = 1255;
    double true_tow = 345600 + true_leap_sec + tod;

    gnav_eph.glot_to_gpst(tod + glo2utc, 0.0, 0.0, &week, &tow);

    // Perform assertions of decoded fields
    EXPECT_EQ(week, true_week);
    EXPECT_DOUBLE_EQ(tow, true_tow);
}


/*!
 * \brief Testing conversion from GLONASST to GPST
 * \test This version tests the conversion for offsets greater than 30 in a leap year
 */
TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsT2)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2016;
    gnav_eph.d_N_T = 268;  // 2016-09-24, a Saturday

    double glo2utc = 3600 * 3;
    double tod = 7560;
    int week = 0;
    double tow = 0.0;
    double true_leap_sec = 17;
    int true_week = 1915;
    double true_tow = 518400 + true_leap_sec + tod;

    gnav_eph.glot_to_gpst(tod + glo2utc, 0.0, 0.0, &week, &tow);

    // Perform assertions of decoded fields
    EXPECT_EQ(week, true_week);
    EXPECT_DOUBLE_EQ(tow, true_tow);
}


/*!
 * \brief Testing conversion from GLONASST to GPST
 * \test This version tests the conversion around the vicinity of February 29 days when in leap year
 */
TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsT3)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2016;
    gnav_eph.d_N_T = 62;  // 2016-03-02, a Wednesday

    double glo2utc = 3600 * 3;
    double tod = 7560;
    int week = 0;
    double tow = 0.0;
    double true_leap_sec = 17;
    int true_week = 1886;
    double true_tow = 259200 + true_leap_sec + tod;

    gnav_eph.glot_to_gpst(tod + glo2utc, 0.0, 0.0, &week, &tow);

    // Perform assertions of decoded fields
    EXPECT_EQ(week, true_week);
    EXPECT_DOUBLE_EQ(tow, true_tow);
}


TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsT4)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2019;
    gnav_eph.d_N_T = 1366;  // 2019-09-27 in the GLONASS day scale, a Friday

    double glo2utc = 3600 * 3;
    double tod = 7560;
    int week = 0;
    double tow = 0.0;
    double true_leap_sec = 18;
    int true_week = 2072;
    double true_tow = 432000 + true_leap_sec + tod;

    gnav_eph.glot_to_gpst(tod + glo2utc, 0.0, 0.0, &week, &tow);

    // Perform assertions of decoded fields
    EXPECT_EQ(week, true_week);
    EXPECT_DOUBLE_EQ(tow, true_tow);
}


TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsT5)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2020;
    gnav_eph.d_N_T = 62;  // 2020-03-02, a Monday

    double glo2utc = 3600 * 3;
    double tod = 7560;
    int week = 0;
    double tow = 0.0;
    double true_leap_sec = 18;
    int true_week = 2095;
    double true_tow = 86400 + true_leap_sec + tod;

    gnav_eph.glot_to_gpst(tod + glo2utc, 0.0, 0.0, &week, &tow);

    // Perform assertions of decoded fields
    EXPECT_EQ(week, true_week);
    EXPECT_DOUBLE_EQ(tow, true_tow);
}


/*!
 * \brief Testing conversion from GLONASST to GPST across the UTC day boundary
 * \test The GLONASS time of day falls less than one leap-second interval
 * before 03:00:00, so the UTC epoch lies just before midnight and the leap
 * second correction rolls the GPS time into the next day
 */
TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsT6)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2019;
    gnav_eph.d_N_T = 1366;  // 2019-09-27 in the GLONASS day scale

    int week = 0;
    double tow = 0.0;

    // GLONASS tod 02:59:50 -> UTC 2019-09-26 23:59:50 -> GPST 2019-09-27 00:00:08
    gnav_eph.glot_to_gpst(10790, 0.0, 0.0, &week, &tow);

    EXPECT_EQ(week, 2072);
    EXPECT_DOUBLE_EQ(tow, 432008.0);
}


/*!
 * \brief Testing conversion from GLONASST to GPST when a leap second announced
 * by the KP word is in effect but not yet in the receiver's leap second table
 */
TEST(GlonassGnavEphemerisTest, ConvertGlonassT2GpsTKpLeapSecond)
{
    Glonass_Gnav_Ephemeris gnav_eph;
    gnav_eph.d_yr = 2019;
    gnav_eph.d_N_T = 1366;  // 2019-09-27, a Friday
    gnav_eph.d_kp_leap_correction_s = 1.0;

    int week = 0;
    double tow = 0.0;

    // UTC tod 02:06:00 -> GPST tow = 5 days + 7560 s + 18 s + 1 s announced leap
    gnav_eph.glot_to_gpst(7560 + 10800, 0.0, 0.0, &week, &tow);

    EXPECT_EQ(week, 2072);
    EXPECT_DOUBLE_EQ(tow, 432000.0 + 7560.0 + 18.0 + 1.0);
}


/*!
 * \brief Testing the GLONASST to UTC(SU) conversion of the UTC model:
 * tUTC = tGLO + tau_c - 3 hrs (GLONASS ICD Edition 5.1, Section 4.5)
 */
TEST(GlonassGnavEphemerisTest, UtcModelUtcTime)
{
    Glonass_Gnav_Utc_Model gnav_utc_model;
    gnav_utc_model.d_tau_c = -1.0 / 1024.0;

    // GLONASS tod 12:00:00 -> UTC 09:00:00 (minus tau_c correction)
    EXPECT_DOUBLE_EQ(gnav_utc_model.utc_time(43200.0), 32400.0 - 1.0 / 1024.0);
}
