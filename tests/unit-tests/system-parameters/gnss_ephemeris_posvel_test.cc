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


#include "gnss_ephemeris.h"
#include "gps_cnav_ephemeris.h"


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

