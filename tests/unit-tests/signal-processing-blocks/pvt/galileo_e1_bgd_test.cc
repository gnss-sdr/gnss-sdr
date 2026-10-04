/*!
 * \file galileo_e1_bgd_test.cc
 * \brief Tests for Galileo Broadcast Group Delay selection in PVT
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "rtklib_pntpos.h"
#include "rtklib_ppp.h"
#include <gtest/gtest.h>
#include <cmath>

namespace
{
constexpr double BGD_E1_E5A_S = 2.0e-8;
constexpr double BGD_E1_E5B_S = 4.0e-8;
constexpr double E1_WAVELENGTH_M = SPEED_OF_LIGHT_M_S / FREQ1;
constexpr double E5A_WAVELENGTH_M = SPEED_OF_LIGHT_M_S / FREQ5;
constexpr double E5B_WAVELENGTH_M = SPEED_OF_LIGHT_M_S / FREQ7;

void set_galileo_test_ephemeris(nav_t& nav, eph_t& eph, int sat)
{
    eph.sat = sat;
    eph.code = 1;  // I/NAV E1B data source
    eph.tgd[0] = BGD_E1_E5A_S;
    eph.tgd[1] = BGD_E1_E5B_S;
    nav.n = 1;
    nav.eph = &eph;
}
}  // namespace


TEST(GalileoE1BgdTest, SingleFrequencyPositioningUsesE1E5bBgd)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GAL, 1);
    ASSERT_GT(sat, 0);
    set_galileo_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = E1_WAVELENGTH_M;
    nav.lam[sat - 1][2] = E5A_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.code[0] = CODE_L1C;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    EXPECT_NEAR(obs.P[0] - SPEED_OF_LIGHT_M_S * BGD_E1_E5B_S, corrected_pseudorange, 1.0e-6);
}


TEST(GalileoE1BgdTest, SingleFrequencyE1FromFnavUsesE1E5aBgd)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GAL, 1);
    ASSERT_GT(sat, 0);
    set_galileo_test_ephemeris(nav, eph, sat);
    eph.code = 2;  // F/NAV E5a data source

    nav.lam[sat - 1][0] = E1_WAVELENGTH_M;
    nav.lam[sat - 1][2] = E5A_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.code[0] = CODE_L1C;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    EXPECT_NEAR(obs.P[0] - SPEED_OF_LIGHT_M_S * BGD_E1_E5A_S, corrected_pseudorange, 1.0e-6);
}


TEST(GalileoE1BgdTest, SingleFrequencyE5aKeepsE1E5aBgd)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GAL, 1);
    ASSERT_GT(sat, 0);
    set_galileo_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = E1_WAVELENGTH_M;
    nav.lam[sat - 1][2] = E5A_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[2] = 24000000.0;
    obs.code[2] = CODE_L5X;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);
    const double gamma = std::pow(E5A_WAVELENGTH_M / E1_WAVELENGTH_M, 2.0);
    const double expected = obs.P[2] - gamma * SPEED_OF_LIGHT_M_S * BGD_E1_E5A_S;

    EXPECT_NEAR(expected, corrected_pseudorange, 1.0e-6);
    // E5a-only measurement: modeled E1 iono must be scaled by (f_E1/f_E5a)^2
    EXPECT_DOUBLE_EQ(gamma, iono_scale);
}


TEST(GalileoE1BgdTest, SingleFrequencyE5bUsesE1E5bBgd)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GAL, 1);
    ASSERT_GT(sat, 0);
    set_galileo_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = E1_WAVELENGTH_M;
    nav.lam[sat - 1][2] = E5B_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[2] = 24000000.0;
    obs.code[2] = CODE_L7X;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);
    const double gamma = std::pow(E5B_WAVELENGTH_M / E1_WAVELENGTH_M, 2.0);
    const double expected = obs.P[2] - gamma * SPEED_OF_LIGHT_M_S * BGD_E1_E5B_S;

    EXPECT_NEAR(expected, corrected_pseudorange, 1.0e-6);
    // E5b-only measurement: modeled E1 iono must be scaled by (f_E1/f_E5b)^2
    EXPECT_DOUBLE_EQ(gamma, iono_scale);
}


TEST(GalileoE1BgdTest, DualFrequencyE1E5aBroadcastModeUsesE1WithItsBgd)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GAL, 1);
    ASSERT_GT(sat, 0);
    set_galileo_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = E1_WAVELENGTH_M;
    nav.lam[sat - 1][2] = E5A_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.P[2] = 24000008.0;
    obs.code[0] = CODE_L1C;
    obs.code[2] = CODE_L5X;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    // With a broadcast iono model the E5a range present in the record is not
    // combined with E1: the E1 range is used alone with the BGD matching the
    // ephemeris clock model (I/NAV here), and the modeled iono delay applies
    // unscaled. Combining per satellite would mix clock references within the
    // constellation whenever a satellite lacks E5a.
    const double expected = obs.P[0] - SPEED_OF_LIGHT_M_S * BGD_E1_E5B_S;
    EXPECT_NEAR(expected, corrected_pseudorange, 1.0e-6);
    EXPECT_DOUBLE_EQ(1.0, iono_scale);
}


TEST(GalileoE1BgdTest, PppUsesE1E5bBgd)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GAL, 1);
    ASSERT_GT(sat, 0);
    set_galileo_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = E1_WAVELENGTH_M;
    nav.lam[sat - 1][1] = E5B_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.L[0] = 100.0;
    obs.code[0] = CODE_L1C;
    options.ionoopt = IONOOPT_OFF;

    const double position[3] = {0.0, 0.0, 0.0};
    const double azel[2] = {0.0, 1.0};
    double measurements[2] = {};
    double variances[2] = {};
    int cycle_slip = 0;

    ASSERT_EQ(1, corrmeas(&obs, &nav, position, azel, &options, nullptr, nullptr, 0.0,
                     measurements, variances, &cycle_slip));
    EXPECT_NEAR(obs.P[0] - SPEED_OF_LIGHT_M_S * BGD_E1_E5B_S, measurements[1], 1.0e-6);
}


TEST(GalileoE1BgdTest, PppUsesE1E5aBgdFromFnav)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GAL, 1);
    ASSERT_GT(sat, 0);
    set_galileo_test_ephemeris(nav, eph, sat);
    eph.code = 2;  // F/NAV E5a data source

    nav.lam[sat - 1][0] = E1_WAVELENGTH_M;
    nav.lam[sat - 1][1] = E5A_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.L[0] = 100.0;
    obs.code[0] = CODE_L1C;
    options.ionoopt = IONOOPT_OFF;

    const double position[3] = {0.0, 0.0, 0.0};
    const double azel[2] = {0.0, 1.0};
    double measurements[2] = {};
    double variances[2] = {};
    int cycle_slip = 0;

    ASSERT_EQ(1, corrmeas(&obs, &nav, position, azel, &options, nullptr, nullptr, 0.0,
                     measurements, variances, &cycle_slip));
    EXPECT_NEAR(obs.P[0] - SPEED_OF_LIGHT_M_S * BGD_E1_E5A_S, measurements[1], 1.0e-6);
}
