/*!
 * \file bds_tgd_iono_test.cc
 * \brief Tests for BeiDou B1I/B3I group-delay and ionospheric handling in SPP
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

#include "Beidou_CNAV1.h"
#include "beidou_bdgim.h"
#include "rtklib_pntpos.h"
#include "rtklib_rtkcmn.h"
#include <gtest/gtest.h>
#include <cmath>

namespace
{
constexpr double TGD1_S = 5.0e-9;
constexpr double B1I_WAVELENGTH_M = SPEED_OF_LIGHT_M_S / FREQ1_BDS;
constexpr double B1C_WAVELENGTH_M = SPEED_OF_LIGHT_M_S / FREQ1;
constexpr double B3I_WAVELENGTH_M = SPEED_OF_LIGHT_M_S / FREQ3_BDS;

void set_bds_test_ephemeris(nav_t& nav, eph_t& eph, int sat)
{
    eph.sat = sat;
    eph.tgd[0] = TGD1_S;  // TGD1 (B1I), the B3I-referenced clock needs none
    nav.n = 1;
    nav.eph = &eph;
}
}  // namespace


TEST(BdsTgdIonoTest, SingleFrequencyB1IAppliesMinusTgd1)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_BDS, 6);
    ASSERT_GT(sat, 0);
    set_bds_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = B1I_WAVELENGTH_M;
    nav.lam[sat - 1][2] = B3I_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.code[0] = CODE_L2I;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    EXPECT_NEAR(obs.P[0] - SPEED_OF_LIGHT_M_S * TGD1_S, corrected_pseudorange, 1.0e-6);
    EXPECT_DOUBLE_EQ(1.0, iono_scale);
}


TEST(BdsTgdIonoTest, SingleFrequencyB1cCombinedUsesCnav1PilotTgd)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_BDS, 6);
    ASSERT_GT(sat, 0);
    eph.sat = sat;
    eph.code = BDS_EPH_SOURCE_CNAV1;
    eph.tgd[0] = 1.0e-9;
    eph.tgd[2] = 2.0e-9;
    nav.n = 1;
    nav.eph = &eph;

    nav.lam[sat - 1][0] = B1C_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.code[0] = CODE_L1X;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange =
        prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    EXPECT_NEAR(obs.P[0] - SPEED_OF_LIGHT_M_S * eph.tgd[0],
        corrected_pseudorange, 1.0e-6);
    EXPECT_DOUBLE_EQ(1.0, iono_scale);
}


TEST(BdsTgdIonoTest, SingleFrequencyB3IHasNoTgdAndDefersIonoScalingToRescode)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_BDS, 6);
    ASSERT_GT(sat, 0);
    set_bds_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = B1I_WAVELENGTH_M;
    nav.lam[sat - 1][2] = B3I_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[2] = 24000000.0;
    obs.code[2] = CODE_L6I;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    // B3I is the clock reference signal: no group-delay correction
    EXPECT_NEAR(obs.P[2], corrected_pseudorange, 1.0e-6);
    // rescode() scales the L1-referenced ionosphere model using the actual
    // observation slot, so prange() must not apply the B1I/B3I factor again.
    EXPECT_DOUBLE_EQ(1.0, iono_scale);
}


TEST(BdsTgdIonoTest, DualFrequencyB1IB3IRemovesTgd1BeforeCombining)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_BDS, 6);
    ASSERT_GT(sat, 0);
    set_bds_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = B1I_WAVELENGTH_M;
    nav.lam[sat - 1][2] = B3I_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.P[2] = 24000006.0;
    obs.code[0] = CODE_L2I;
    obs.code[2] = CODE_L6I;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    // The DNAV clock is referenced to B3I, so the iono-free combination must
    // use the TGD1-corrected B1I pseudorange:
    // PC = (gamma13*(P1 - c*TGD1) - P2) / (gamma13 - 1)
    const double gamma13 = std::pow(B3I_WAVELENGTH_M / B1I_WAVELENGTH_M, 2.0);
    const double p1_corr = obs.P[0] - SPEED_OF_LIGHT_M_S * TGD1_S;
    const double expected = (gamma13 * p1_corr - obs.P[2]) / (gamma13 - 1.0);
    EXPECT_NEAR(expected, corrected_pseudorange, 1.0e-6);
    EXPECT_DOUBLE_EQ(0.0, iono_scale);
}


TEST(BdsTgdIonoTest, DualFrequencyB1IB3IIonoFreeModeAlsoRemovesTgd1)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_BDS, 6);
    ASSERT_GT(sat, 0);
    set_bds_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = B1I_WAVELENGTH_M;
    nav.lam[sat - 1][2] = B3I_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.P[2] = 24000006.0;
    obs.code[0] = CODE_L2I;
    obs.code[2] = CODE_L6I;
    options.ionoopt = IONOOPT_IFLC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    const double gamma13 = std::pow(B3I_WAVELENGTH_M / B1I_WAVELENGTH_M, 2.0);
    const double p1_corr = obs.P[0] - SPEED_OF_LIGHT_M_S * TGD1_S;
    const double expected = (gamma13 * p1_corr - obs.P[2]) / (gamma13 - 1.0);
    EXPECT_NEAR(expected, corrected_pseudorange, 1.0e-6);
    EXPECT_DOUBLE_EQ(0.0, iono_scale);
}


TEST(BdsTgdIonoTest, IonocorrPrefersBdsKlobucharForBdsSatellites)
{
    nav_t nav{};
    const double ep[6] = {2026, 7, 31, 12, 0, 0};
    const gtime_t t = epoch2time(ep);
    const double pos[3] = {0.7, 2.0, 100.0};
    const double azel[2] = {0.0, 1.0};

    const double ion_gps[8] = {0.1118e-7, -0.7451e-8, -0.5961e-7, 0.1192e-6,
        0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7};
    const double ion_bds[8] = {0.2235e-7, -0.1490e-7, -0.1192e-6, 0.2384e-6,
        0.1290e6, -0.1966e6, -0.2621e6, 0.8389e6};
    for (int k = 0; k < 8; k++)
        {
            nav.ion_gps[k] = ion_gps[k];
            nav.ion_cmp[k] = ion_bds[k];
        }

    double ion = 0.0;
    double var = 0.0;

    // GPS satellite: GPS coefficients, unscaled
    ASSERT_EQ(1, ionocorr(t, &nav, satno(SYS_GPS, 1), pos, azel, IONOOPT_BRDC, &ion, &var));
    EXPECT_NEAR(ionmodel(t, nav.ion_gps, pos, azel), ion, 1.0e-9);

    // BDS satellite: BDS coefficients (B1I-referenced), expressed at GPS L1
    ASSERT_EQ(1, ionocorr(t, &nav, satno(SYS_BDS, 6), pos, azel, IONOOPT_BRDC, &ion, &var));
    const double expected_bds = ionmodel(t, nav.ion_cmp, pos, azel) * std::pow(FREQ1_BDS / FREQ1, 2.0);
    EXPECT_NEAR(expected_bds, ion, 1.0e-9);
    EXPECT_GT(ion, 0.0);

    // BDS satellite without BDS iono: falls back to the GPS coefficients
    for (int k = 0; k < 8; k++)
        {
            nav.ion_cmp[k] = 0.0;
        }
    ASSERT_EQ(1, ionocorr(t, &nav, satno(SYS_BDS, 6), pos, azel, IONOOPT_BRDC, &ion, &var));
    EXPECT_NEAR(ionmodel(t, nav.ion_gps, pos, azel), ion, 1.0e-9);
}


TEST(BdsTgdIonoTest, IonocorrUsesBdgimForB1cCombined)
{
    nav_t nav{};
    const double ep[6] = {2026, 7, 31, 12, 0, 0};
    const gtime_t time = epoch2time(ep);
    const double position[3] = {40.0 * D2R, 116.0 * D2R, 100.0};
    const double azel[2] = {45.0 * D2R, 40.0 * D2R};
    const double coefficients[9] = {10.0, 1.0, 0.5, -0.5, 0.2, 0.1, -0.1, 0.05, -0.05};
    for (int i = 0; i < 9; ++i)
        {
            nav.ion_bdgim[i] = coefficients[i];
        }
    nav.ion_bdgim_valid = 1;
    nav.ion_cmp[0] = 0.25e-7;

    const double reference_epoch[6] = {2000, 1, 1, 12, 0, 0};
    const double mjd = 51544.5 +
                       timediff(gpst2utc(time), epoch2time(reference_epoch)) / 86400.0;
    const double expected = beidou_bdgim_delay_m(mjd, position[0], position[1],
        azel[0], azel[1], nav.ion_bdgim, FREQ1);
    double ionosphere_delay = 0.0;
    double variance = 0.0;

    ASSERT_EQ(1, ionocorr(time, &nav, satno(SYS_BDS, 6), position, azel,
                     IONOOPT_BRDC, &ionosphere_delay, &variance, CODE_L1X));
    EXPECT_NEAR(expected, ionosphere_delay, 1.0e-9);
}


TEST(BdsTgdIonoTest, SingleFrequencyB3ISppScalesBroadcastIonoExactlyOnce)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_BDS, 6);
    ASSERT_GT(sat, 0);
    set_bds_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = B1I_WAVELENGTH_M;
    nav.lam[sat - 1][2] = B3I_WAVELENGTH_M;
    const double ion_bds[8] = {0.2235e-7, -0.1490e-7, -0.1192e-6, 0.2384e-6,
        0.1290e6, -0.1966e6, -0.2621e6, 0.8389e6};
    for (int k = 0; k < 8; ++k)
        {
            nav.ion_cmp[k] = ion_bds[k];
        }

    const double ep[6] = {2026, 7, 31, 12, 0, 0};
    obs.time = epoch2time(ep);
    obs.sat = sat;
    obs.code[2] = CODE_L6I;
    obs.SNR[2] = 160;

    constexpr int output_rows = 5;  // one observation plus four clock constraints
    double satellite_state[6] = {RE_WGS84 + 20200000.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double receiver_state[NX] = {RE_WGS84, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double line_of_sight[3] = {};
    const double geometric_range = geodist(satellite_state, receiver_state, line_of_sight);
    ASSERT_GT(geometric_range, 0.0);

    double receiver_position[3] = {};
    double expected_azel[2] = {};
    ecef2pos(receiver_state, receiver_position);
    ASSERT_GT(satazel(receiver_position, line_of_sight, expected_azel), 0.0);
    const double b1i_iono = ionmodel(obs.time, nav.ion_cmp,
        receiver_position, expected_azel);
    const double gamma13 = std::pow(FREQ1_BDS / FREQ3_BDS, 2.0);
    const double expected_b3i_iono = b1i_iono * gamma13;
    ASSERT_GT(expected_b3i_iono, 0.0);
    obs.P[2] = geometric_range + expected_b3i_iono;

    options.navsys = SYS_BDS;
    options.ionoopt = IONOOPT_BRDC;
    options.tropopt = TROPOPT_OFF;

    const double satellite_clock[2] = {};
    const double ephemeris_variance[1] = {};
    const int satellite_health[1] = {};
    double residuals[output_rows] = {};
    double design_matrix[NX * output_rows] = {};
    double variances[output_rows] = {};
    double azel[2] = {};
    int valid_satellite[1] = {};
    double pseudorange_residual[1] = {};
    int valid_satellite_count = 0;

    const int residual_count = rescode(1, &obs, 1, satellite_state,
        satellite_clock, ephemeris_variance, satellite_health, &nav,
        receiver_state, &options, residuals, design_matrix, variances, azel,
        valid_satellite, pseudorange_residual, &valid_satellite_count);

    EXPECT_EQ(5, residual_count);
    EXPECT_EQ(1, valid_satellite_count);
    EXPECT_EQ(1, valid_satellite[0]);
    EXPECT_NEAR(0.0, residuals[0], 1.0e-6);
    EXPECT_NEAR(0.0, pseudorange_residual[0], 1.0e-6);
}
