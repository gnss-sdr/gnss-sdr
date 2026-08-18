/*!
 * \file gps_tgd_isc_test.cc
 * \brief Tests for GPS TGD / inter-signal correction application in prange()
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
#include <gtest/gtest.h>
#include <cmath>

namespace
{
constexpr double TGD_S = 1.0e-8;
constexpr double ISC_L1CA_S = -2.0e-9;
constexpr double ISC_L2C_S = -3.0e-9;
constexpr double ISC_L5I5_S = -4.0e-9;
constexpr double L1_WAVELENGTH_M = SPEED_OF_LIGHT_M_S / FREQ1;
constexpr double L2_WAVELENGTH_M = SPEED_OF_LIGHT_M_S / FREQ2;
constexpr double L5_WAVELENGTH_M = SPEED_OF_LIGHT_M_S / FREQ5;

void set_gps_test_ephemeris(nav_t& nav, eph_t& eph, int sat)
{
    eph.sat = sat;
    eph.tgd[0] = TGD_S;
    eph.isc[0] = ISC_L1CA_S;
    eph.isc[1] = ISC_L2C_S;
    eph.isc[2] = ISC_L5I5_S;
    nav.n = 1;
    nav.eph = &eph;
}
}  // namespace


TEST(GpsTgdIscTest, SingleFrequencyL1AppliesMinusTgd)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GPS, 1);
    ASSERT_GT(sat, 0);
    set_gps_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = L1_WAVELENGTH_M;
    nav.lam[sat - 1][1] = L2_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.code[0] = CODE_L1C;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    // IS-GPS-200 20.3.3.3.3.2: (dtSV)_L1 = dtSV - TGD  =>  PC = P1 - c*TGD
    EXPECT_NEAR(obs.P[0] - SPEED_OF_LIGHT_M_S * TGD_S, corrected_pseudorange, 1.0e-6);
    EXPECT_DOUBLE_EQ(1.0, iono_scale);
}


TEST(GpsTgdIscTest, SingleFrequencyL1ConvertsP1P2DcbToTgdEquivalent)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GPS, 1);
    ASSERT_GT(sat, 0);
    set_gps_test_ephemeris(nav, eph, sat);

    constexpr double dcb_p1_p2_m = 3.0;
    nav.cbias[sat - 1][0] = dcb_p1_p2_m;
    nav.lam[sat - 1][0] = L1_WAVELENGTH_M;
    nav.lam[sat - 1][1] = L2_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.code[0] = CODE_L1C;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    // A raw P1-P2 DCB (here 3 m) takes precedence over the broadcast TGD and
    // must be converted to its TGD equivalent: TGD = DCB / (1 - gamma12),
    // so PC = P1 - DCB / (1 - gamma12), matching upstream RTKLIB
    const double gamma12 = std::pow(L2_WAVELENGTH_M / L1_WAVELENGTH_M, 2.0);
    const double expected = obs.P[0] - dcb_p1_p2_m / (1.0 - gamma12);
    EXPECT_NEAR(expected, corrected_pseudorange, 1.0e-6);
}


TEST(GpsTgdIscTest, DualFrequencyL1L2AppliesMinusTgdToL1Pseudorange)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GPS, 1);
    ASSERT_GT(sat, 0);
    set_gps_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = L1_WAVELENGTH_M;
    nav.lam[sat - 1][1] = L2_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[0] = 24000000.0;
    obs.P[1] = 24000005.0;
    obs.code[0] = CODE_L1C;
    obs.code[1] = CODE_L2S;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    // With a single-frequency iono model, the L1 C/A pseudorange referenced to
    // the LNAV clock takes the same correction as the L1-only case: P1 - c*TGD.
    // A '+' sign here doubles the group-delay error (up to ~8 m).
    EXPECT_NEAR(obs.P[0] - SPEED_OF_LIGHT_M_S * TGD_S, corrected_pseudorange, 1.0e-6);
    // The measurement is L1-referenced, so the modeled iono delay applies in full
    EXPECT_DOUBLE_EQ(1.0, iono_scale);
}


TEST(GpsTgdIscTest, SingleFrequencyL5ScalesModeledIonoByGamma15)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GPS, 1);
    ASSERT_GT(sat, 0);
    set_gps_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = L1_WAVELENGTH_M;
    nav.lam[sat - 1][2] = L5_WAVELENGTH_M;
    obs.sat = sat;
    obs.P[2] = 24000000.0;
    obs.code[2] = CODE_L5X;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    // The modeled L1 iono delay must be scaled by gamma15 = (f_L1/f_L5)^2 for
    // an L5-only pseudorange (the PC group-delay value itself is not asserted
    // here; its sign convention is exercised by the CNAV correction tests)
    const double gamma15 = std::pow(L5_WAVELENGTH_M / L1_WAVELENGTH_M, 2.0);
    EXPECT_NE(0.0, corrected_pseudorange);
    EXPECT_DOUBLE_EQ(gamma15, iono_scale);
}


TEST(GpsTgdIscTest, DualFrequencyL1L5UsesIscL1caOnGammaWeightedTerm)
{
    nav_t nav{};
    eph_t eph{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GPS, 1);
    ASSERT_GT(sat, 0);
    set_gps_test_ephemeris(nav, eph, sat);

    nav.lam[sat - 1][0] = L1_WAVELENGTH_M;
    nav.lam[sat - 1][2] = L5_WAVELENGTH_M;
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

    // IS-GPS-705 20.3.3.3.1.2.2 dual-frequency (L1 C/A, L5 I5) correction:
    // PC = (P5 + c*ISC_L5I5 - gamma15*(P1 + c*ISC_L1CA)) / (1 - gamma15) - c*TGD
    const double gamma15 = std::pow(L5_WAVELENGTH_M / L1_WAVELENGTH_M, 2.0);
    const double p1_corr = obs.P[0] + SPEED_OF_LIGHT_M_S * ISC_L1CA_S;
    const double p5_corr = obs.P[2] + SPEED_OF_LIGHT_M_S * ISC_L5I5_S;
    const double expected = (p5_corr - gamma15 * p1_corr) / (1.0 - gamma15) - SPEED_OF_LIGHT_M_S * TGD_S;

    EXPECT_NEAR(expected, corrected_pseudorange, 1.0e-6);
    // Iono-free combination: the modeled iono delay must not be applied on top
    EXPECT_DOUBLE_EQ(0.0, iono_scale);
}


TEST(GlonassDcbTest, SingleFrequencyL2AppliesP1P2DcbOnce)
{
    nav_t nav{};
    obsd_t obs{};
    prcopt_t options{};
    const int sat = satno(SYS_GLO, 1);
    ASSERT_GT(sat, 0);

    constexpr double dcb_p1_p2_m = 3.0;
    const double l1_wavelength_m = SPEED_OF_LIGHT_M_S / FREQ1_GLO;
    const double l2_wavelength_m = SPEED_OF_LIGHT_M_S / FREQ2_GLO;
    nav.cbias[sat - 1][0] = dcb_p1_p2_m;
    nav.lam[sat - 1][0] = l1_wavelength_m;
    nav.lam[sat - 1][1] = l2_wavelength_m;
    obs.sat = sat;
    obs.P[1] = 24000000.0;
    obs.code[1] = CODE_L2C;
    options.ionoopt = IONOOPT_BRDC;

    const double azel[2] = {0.0, 1.0};
    double variance = 0.0;
    double iono_scale = -1.0;
    const double corrected_pseudorange = prange(&obs, &nav, azel, 0, &options, &variance, &iono_scale);

    const double gamma12 = std::pow(l2_wavelength_m / l1_wavelength_m, 2.0);
    const double expected = obs.P[1] - gamma12 * dcb_p1_p2_m / (1.0 - gamma12);
    EXPECT_NEAR(expected, corrected_pseudorange, 1.0e-6);
    EXPECT_DOUBLE_EQ(gamma12, iono_scale);
}


TEST(SppVarerrTest, IonoFreeCombinationAmplifiesVarianceByUpstreamFactorNine)
{
    // the ionosphere-free noise amplification is (f1^2/(f1^2-f2^2))^2 +
    // (f2^2/(f1^2-f2^2))^2, approximately 8.9 for GPS L1/L2, which upstream
    // RTKLIB rounds to 3^2 = 9; rescode's dual-frequency fallback applies the
    // same factor, so this pins the canonical value
    prcopt_t opt{};
    opt.err[1] = 0.003;
    opt.err[2] = 0.003;
    opt.eratio[0] = 100.0;

    obsd_t obs{};
    const double elevation_rad = 45.0 * D2R;

    opt.ionoopt = IONOOPT_OFF;
    const double variance_single = varerr(&opt, &obs, elevation_rad, SYS_GPS);
    ASSERT_GT(variance_single, 0.0);

    opt.ionoopt = IONOOPT_IFLC;
    const double variance_iono_free = varerr(&opt, &obs, elevation_rad, SYS_GPS);

    EXPECT_DOUBLE_EQ(9.0, variance_iono_free / variance_single);
}


TEST(SppVarerrTest, SnrWeightingFollowsTheMeasurementSlot)
{
    // GNSS-SDR places single-band L2C/L5/E5a observations in slots 1/2 with
    // slot 0 empty: the SNR term must read the slot that carries the
    // pseudorange, or those satellites are weighted as if their SNR were zero
    prcopt_t opt{};
    opt.err[1] = 0.003;
    opt.err[2] = 0.003;
    opt.err[5] = 52.0;   // error_snr_max (dB-Hz)
    opt.err[6] = 0.005;  // error_factor_snr (m)
    opt.eratio[0] = 100.0;
    opt.ionoopt = IONOOPT_OFF;
    const double elevation_rad = 45.0 * D2R;
    const unsigned char snr_45dbhz = 180;  // in 0.25 dB-Hz units

    obsd_t slot0_observation{};
    slot0_observation.P[0] = 22.0e6;
    slot0_observation.SNR[0] = snr_45dbhz;

    obsd_t slot2_observation{};  // an L5/E5a-only observation lives in slot 2
    slot2_observation.P[2] = 22.0e6;
    slot2_observation.SNR[2] = snr_45dbhz;

    const double variance_slot0 = varerr(&opt, &slot0_observation, elevation_rad, SYS_GPS);
    const double variance_slot2 = varerr(&opt, &slot2_observation, elevation_rad, SYS_GPS);
    EXPECT_DOUBLE_EQ(variance_slot0, variance_slot2);

    obsd_t zero_snr_observation{};
    zero_snr_observation.P[0] = 22.0e6;
    const double variance_zero_snr = varerr(&opt, &zero_snr_observation, elevation_rad, SYS_GPS);
    EXPECT_GT(variance_zero_snr, 100.0 * variance_slot2);
}
