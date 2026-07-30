/*!
 * \file gnss_ephemeris_posvel_test.cc.cc
 * \author Vladislav P, 2026. vladisslav2011(at)gmail.com
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

#include "galileo_ephemeris.h"
#include "gnss_ephemeris.h"
#include "gps_cnav_ephemeris.h"
#include "gps_ephemeris.h"


TEST(GnssEphemerisPosVelTest, NAV)
{
    Gps_Ephemeris e{};
    e.PRN = 15;
    e.M_0 = 1.35464234665685956e+00;
    e.delta_n = 5.10199823299105835e-09;
    e.ecc = 1.70680778101086582e-02;
    e.sqrtA = 5.15376512145996094e+03;
    e.OMEGA_0 = 2.35100387398234556e+00;
    e.i_0 = 9.44768674001770559e-01;
    e.omega = 1.54160131395114042e+00;
    e.OMEGAdot = -8.57285709390860145e-09;
    e.idot = -2.30366738556474118e-10;
    e.Cuc = -6.73159956932067871e-06;
    e.Cus = 3.66382300853729248e-06;
    e.Crc = 3.05843750000000000e+02;
    e.Crs = -1.23937500000000000e+02;
    e.Cic = -1.17346644401550293e-07;
    e.Cis = -3.91155481338500977e-07;
    e.toe = 331200;
    e.toc = 331200;
    e.af0 = 4.32340428233146667e-04;
    e.af1 = 3.06954461848363200e-12;
    e.af2 = 0.00000000000000000e+00;
    e.WN = 381;
    e.tow = 326166;
    e.satClkDrift = 0.00000000000000000e+00;
    e.dtr = 0.00000000000000000e+00;
    e.IODE_SF2 = 30;
    e.IODE_SF3 = 30;
    e.code_on_L2 = 1;
    e.L2_P_data_flag = 0;
    e.SV_accuracy = 0;
    e.SV_health = 0;
    e.TGD = -1.07102096080780029e-08;
    e.IODC = 30;
    e.AODO = 16200;
    e.fit_interval_flag = 0;
    e.spare1 = 0.00000000000000000e+00;
    e.spare2 = 0.00000000000000000e+00;
    e.integrity_status_flag = 0;
    e.alert_flag = 0;
    e.antispoofing_flag = 1;
    auto dopplerL1 = e.predicted_doppler(590328., 0., 0., 0., 0., 0., 0., 1);
    e.satellitePosition(590328.);
    EXPECT_NEAR(98.676019, dopplerL1, 1e-4);
    EXPECT_NEAR(26325247.164428, e.satpos_X, 1e-4);
    EXPECT_NEAR(1943900.228289, e.satpos_Y, 1e-4);
    EXPECT_NEAR(2494201.363945, e.satpos_Z, 1e-4);
    EXPECT_NEAR(333.268032, e.satvel_X, 1e-4);
    EXPECT_NEAR(384.414587, e.satvel_Y, 1e-4);
    EXPECT_NEAR(-3116.921783, e.satvel_Z, 1e-4);
    double satvel_X = e.satvel_X;
    double satvel_Y = e.satvel_Y;
    double satvel_Z = e.satvel_Z;
    e.satellitePosition(590327.995);
    double satpos_X = e.satpos_X;
    double satpos_Y = e.satpos_Y;
    double satpos_Z = e.satpos_Z;
    e.satellitePosition(590328.005);
    double d_X = (e.satpos_X - satpos_X) * 100.;
    double d_Y = (e.satpos_Y - satpos_Y) * 100.;
    double d_Z = (e.satpos_Z - satpos_Z) * 100.;
    EXPECT_NEAR(d_X, satvel_X, 1e-4);
    EXPECT_NEAR(d_Y, satvel_Y, 1e-4);
    EXPECT_NEAR(d_Z, satvel_Z, 1e-4);
}


TEST(GnssEphemerisPosVelTest, CNAV)
{
    Gps_CNAV_Ephemeris e{};
    e.PRN = 1;
    e.M_0 = 8.45797918146817151e-01;
    e.delta_n = 4.55893989814592299e-09;
    e.ecc = 1.83155917329713681e-03;
    e.sqrtA = 5.15360200490880015e+03;
    e.OMEGA_0 = 6.26039239835774475e-01;
    e.i_0 = 9.57107201573828559e-01;
    e.omega = 1.91825002944197659e-01;
    e.OMEGAdot = -8.21850013985512873e-09;
    e.idot = 1.84293390845179697e-10;
    e.Cuc = 3.64799052476882935e-06;
    e.Cus = 3.35834920406341553e-06;
    e.Crc = 3.11757812500000000e+02;
    e.Crs = 6.96562500000000000e+01;
    e.Cic = 4.00468707084655762e-08;
    e.Cis = 4.00468707084655762e-08;
    e.toe = 0;
    e.toc = 588600;
    e.af0 = 2.13293504202738377e-04;
    e.af1 = -9.59232693276135251e-12;
    e.af2 = 0.00000000000000000e+00;
    e.WN = 2428;
    e.tow = 590328;
    e.satClkDrift = 0.00000000000000000e+00;
    e.dtr = 0.00000000000000000e+00;
    e.toe1 = 588600;
    e.toe2 = 588600;
    e.WNop = 124;
    e.top = 519300;
    e.URAED = -1;
    e.URANED0 = -2;
    e.URANED1 = 3;
    e.URANED2 = 7;
    e.URA = -1;
    e.URA0 = -2.00000000000000000e+00;
    e.URA1 = 3.00000000000000000e+00;
    e.URA2 = 7.00000000000000000e+00;
    e.delta_ndot = -8.06351598383372435e-14;
    e.TGD = -8.84756445884704424e-09;
    e.ISCL1 = -2.91038304567336984e-10;
    e.ISCL2 = 5.64614310860633767e-09;
    e.ISCL5I = -5.52972778677940265e-10;
    e.ISCL5Q = -6.69388100504875080e-10;
    e.delta_A = -9.63750000000000000e+01;
    e.Adot = 7.12776184082031250e-03;
    e.delta_OMEGAdot = -5.03592405216479423e-11;
    e.integrity_status_flag = 0;
    e.l2c_phasing_flag = 0;
    e.alert_flag = 0;
    e.antispoofing_flag = 0;
    auto dopplerL2 = e.predicted_doppler(590328., 0., 0., 0., 0., 0., 0., 2);
    auto dopplerL5 = e.predicted_doppler(590328., 0., 0., 0., 0., 0., 0., 5);
    e.satellitePosition(590328.);
    EXPECT_NEAR(-411.339708, dopplerL2, 1e-4);
    EXPECT_NEAR(-394.200554, dopplerL5, 1e-4);
    EXPECT_NEAR(11987089.716145, e.satpos_X, 1e-4);
    EXPECT_NEAR(13994848.326179, e.satpos_Y, 1e-4);
    EXPECT_NEAR(-19106970.005881, e.satpos_Z, 1e-4);
    EXPECT_NEAR(-411.226253, e.satvel_X, 1e-4);
    EXPECT_NEAR( 2397.343969, e.satvel_Y, 1e-4);
    EXPECT_NEAR(1507.249499, e.satvel_Z, 1e-4);
    double satvel_X = e.satvel_X;
    double satvel_Y = e.satvel_Y;
    double satvel_Z = e.satvel_Z;
    e.satellitePosition(590327.995);
    double satpos_X = e.satpos_X;
    double satpos_Y = e.satpos_Y;
    double satpos_Z = e.satpos_Z;
    e.satellitePosition(590328.005);
    double d_X = (e.satpos_X - satpos_X) * 100.;
    double d_Y = (e.satpos_Y - satpos_Y) * 100.;
    double d_Z = (e.satpos_Z - satpos_Z) * 100.;
    EXPECT_NEAR(d_X, satvel_X, 1e-4);
    EXPECT_NEAR(d_Y, satvel_Y, 1e-4);
    EXPECT_NEAR(d_Z, satvel_Z, 1e-4);
}


TEST(GnssEphemerisPosVelTest, Galileo)
{
    Galileo_Ephemeris e{};
    e.PRN = 13;
    e.M_0 = 2.41163546760016079e+00;
    e.delta_n = 2.66832543217894277e-09;
    e.ecc = 4.77104913443326882e-05;
    e.sqrtA = 5.44060108375549316e+03;
    e.OMEGA_0 = 6.47008588356165260e-01;
    e.i_0 = 9.97650131401018214e-01;
    e.omega = -7.14945752017439456e-02;
    e.OMEGAdot = -5.50165773755647669e-09;
    e.idot = 3.22870591713259829e-10;
    e.Cuc = 6.63474202156066895e-06;
    e.Cus = 2.07684934139251709e-06;
    e.Crc = 3.11656250000000000e+02;
    e.Crs = 1.40062500000000000e+02;
    e.Cic = -2.79396772384643555e-08;
    e.Cis = 4.47034835815429688e-08;
    e.toe = 57000;
    e.toc = 57000;
    e.af0 = -6.03639055043458871e-05;
    e.af1 = -9.37916411203332043e-13;
    e.af2 = 0.00000000000000000e+00;
    e.WN = 2429;
    e.tow = 59670;
    e.satClkDrift = 0.00000000000000000e+00;
    e.dtr = 0.00000000000000000e+00;
    e.IOD_ephemeris = 95;
    e.IOD_nav = 0;
    e.SISA = 107;
    e.E5a_HS = 0;
    e.E5b_HS = 0;
    e.E1B_HS = 0;
    e.E5a_DVS = 0;
    e.E5b_DVS = 0;
    e.E1B_DVS = 0;
    e.BGD_E1E5a = 4.42378222942352212e-09;
    e.BGD_E1E5b = 0.00000000000000000e+00;
    e.flag_all_ephemeris = 1;
    e.nav_message_type = static_cast<Galileo_Nav_Message_Type>(2);
    auto dopplerL1 = e.predicted_doppler(59328., 0., 0., 0., 0., 0., 0., 1);
    auto dopplerL5 = e.predicted_doppler(59328., 0., 0., 0., 0., 0., 0., 5);
    auto dopplerL6 = e.predicted_doppler(59328., 0., 0., 0., 0., 0., 0., 6);
    auto dopplerL7 = e.predicted_doppler(59328., 0., 0., 0., 0., 0., 0., 7);
    auto dopplerL8 = e.predicted_doppler(59328., 0., 0., 0., 0., 0., 0., 8);
    e.satellitePosition(59328.);
    EXPECT_NEAR(1250.566176, dopplerL1, 1e-4);
    EXPECT_NEAR(933.864352, dopplerL5, 1e-4);
    EXPECT_NEAR(1015.069948, dopplerL6, 1e-4);
    EXPECT_NEAR(958.226030, dopplerL7, 1e-4);
    EXPECT_NEAR(946.045191, dopplerL8, 1e-4);
    EXPECT_NEAR(18121174.847871, e.satpos_X, 1e-4);
    EXPECT_NEAR(-19974436.330324, e.satpos_Y, 1e-4);
    EXPECT_NEAR(12202101.756204, e.satpos_Z, 1e-4);
    EXPECT_NEAR(977.807753, e.satvel_X, 1e-4);
    EXPECT_NEAR(-754.332969, e.satvel_Y, 1e-4);
    EXPECT_NEAR(-2686.561660, e.satvel_Z, 1e-4);
    double satvel_X = e.satvel_X;
    double satvel_Y = e.satvel_Y;
    double satvel_Z = e.satvel_Z;
    e.satellitePosition(59327.995);
    double satpos_X = e.satpos_X;
    double satpos_Y = e.satpos_Y;
    double satpos_Z = e.satpos_Z;
    e.satellitePosition(59328.005);
    double d_X = (e.satpos_X - satpos_X) * 100.;
    double d_Y = (e.satpos_Y - satpos_Y) * 100.;
    double d_Z = (e.satpos_Z - satpos_Z) * 100.;
    EXPECT_NEAR(d_X, satvel_X, 1e-4);
    EXPECT_NEAR(d_Y, satvel_Y, 1e-4);
    EXPECT_NEAR(d_Z, satvel_Z, 1e-4);
}
