/*!
 * \file beidou_b1c_pvt_helpers_test.cc
 * \brief Unit tests for B1C PVT helpers (TGD/ISC, seleph, band/wavelength map)
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
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
#include "MATH_CONSTANTS.h"
#include "gnss_frequencies.h"
#include "gnss_obs_codes.h"
#include "gnss_synchro.h"
#include "rtklib.h"
#include "rtklib_conversions.h"
#include "rtklib_ephemeris.h"
#include "rtklib_pntpos.h"
#include "rtklib_rtkcmn.h"
#include <gtest/gtest.h>
#include <cstring>
#include <string>
#include <vector>

namespace
{
eph_t make_bds_eph(int sat, int code, double tgd0, double tgd2, gtime_t toe)
{
    eph_t e{};
    std::memset(&e, 0, sizeof(e));
    e.sat = sat;
    e.code = code;
    e.A = 27906100.0;
    e.e = 0.01;
    e.toe = toe;
    e.toc = toe;
    e.toes = 0.0;
    e.tgd[0] = tgd0;
    e.tgd[2] = tgd2;
    return e;
}

Gnss_Synchro make_bds_synchro(const char *signal, uint32_t prn, double pr_m)
{
    Gnss_Synchro gs{};
    gs.System = 'C';
    gs.Signal[0] = signal[0];
    gs.Signal[1] = signal[1];
    gs.Signal[2] = '\0';
    gs.PRN = prn;
    gs.Pseudorange_m = pr_m;
    gs.Carrier_Doppler_hz = 100.0;
    gs.Carrier_phase_rads = 0.0;
    gs.CN0_dB_hz = 40.0;
    gs.Flag_valid_pseudorange = true;
    return gs;
}
}  // namespace

TEST(BeidouB1cPvtHelpersTest, GettgdAppliesIscForDataComponent)
{
    const int sat = NSATGPS + NSATGLO + NSATGAL + NSATQZS + 19;
    const double ep[] = {2020, 1, 1, 0, 0, 0};
    const gtime_t t0 = epoch2time(ep);
    std::vector<eph_t> ephs;
    ephs.push_back(make_bds_eph(sat, BDS_EPH_SOURCE_CNAV1, 1.0e-9, 2.0e-9, t0));
    nav_t nav;
    std::memset(&nav, 0, sizeof(nav));
    nav.eph = ephs.data();
    nav.n = 1;

    const double tgd_pilot = gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L1P));
    const double tgd_data = gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L1D));
    EXPECT_NEAR(tgd_pilot, SPEED_OF_LIGHT_M_S * 1.0e-9, 1.0e-6);
    EXPECT_NEAR(tgd_data, SPEED_OF_LIGHT_M_S * 3.0e-9, 1.0e-6);
}

TEST(BeidouB1cPvtHelpersTest, SelephPrefersCnav1WhenRequested)
{
    const int sat = NSATGPS + NSATGLO + NSATGAL + NSATQZS + 20;
    const double ep[] = {2020, 6, 1, 12, 0, 0};
    const gtime_t t0 = epoch2time(ep);
    std::vector<eph_t> ephs;
    ephs.push_back(make_bds_eph(sat, 1, 1.0e-9, 0.0, t0));                       /* DNAV */
    ephs.push_back(make_bds_eph(sat, BDS_EPH_SOURCE_CNAV1, 2.0e-9, 1.0e-9, t0)); /* CNAV1 */
    nav_t nav;
    std::memset(&nav, 0, sizeof(nav));
    nav.eph = ephs.data();
    nav.n = 2;

    eph_t *e_cnav = seleph(t0, sat, -1, &nav, BDS_EPH_SOURCE_CNAV1);
    eph_t *e_dnav = seleph(t0, sat, -1, &nav, 0);
    ASSERT_NE(e_cnav, static_cast<eph_t *>(nullptr));
    ASSERT_NE(e_dnav, static_cast<eph_t *>(nullptr));
    EXPECT_EQ(e_cnav->code, BDS_EPH_SOURCE_CNAV1);
    EXPECT_NE(e_dnav->code, BDS_EPH_SOURCE_CNAV1);
}

TEST(BeidouB1cPvtHelpersTest, SelephCnav1PreferenceFallsBackWithoutWeakeningStrictSelection)
{
    const int sat = NSATGPS + NSATGLO + NSATGAL + NSATQZS + 20;
    const double ep[] = {2020, 6, 1, 12, 0, 0};
    const gtime_t t0 = epoch2time(ep);
    std::vector<eph_t> ephs;
    ephs.push_back(make_bds_eph(sat, 1, 1.0e-9, 0.0, t0)); /* DNAV */
    ephs.push_back(make_bds_eph(
        sat, BDS_EPH_SOURCE_CNAV1, 2.0e-9, 1.0e-9, timeadd(t0, -60.0))); /* CNAV1 */
    nav_t nav;
    std::memset(&nav, 0, sizeof(nav));
    nav.eph = ephs.data();
    nav.n = 2;

    /* Prefer the matching B-CNAV1 source even when DNAV has the closer toe. */
    eph_t *preferred = seleph(t0, sat, -1, &nav, BDS_EPH_SELECTION_CNAV1_PREFERRED);
    ASSERT_NE(preferred, static_cast<eph_t *>(nullptr));
    EXPECT_EQ(BDS_EPH_SOURCE_CNAV1, preferred->code);
    eph_t *preferred_iode = seleph(t0, sat, 0, &nav, BDS_EPH_SELECTION_CNAV1_PREFERRED);
    ASSERT_NE(preferred_iode, static_cast<eph_t *>(nullptr));
    EXPECT_EQ(BDS_EPH_SOURCE_CNAV1, preferred_iode->code);

    /* MT1042/DNAV remains usable only through the preference-mode fallback;
       callers asking for strict B-CNAV1 retain the old filtering contract. */
    nav.n = 1;
    eph_t *fallback = seleph(t0, sat, -1, &nav, BDS_EPH_SELECTION_CNAV1_PREFERRED);
    eph_t *strict = seleph(t0, sat, -1, &nav, BDS_EPH_SOURCE_CNAV1);
    ASSERT_NE(fallback, static_cast<eph_t *>(nullptr));
    EXPECT_NE(BDS_EPH_SOURCE_CNAV1, fallback->code);
    EXPECT_EQ(strict, static_cast<eph_t *>(nullptr));
}

TEST(BeidouB1cPvtHelpersTest, SatpossLimitsDnavFallbackToExplicitRelativePolicy)
{
    const int sat = satno(SYS_BDS, 23);
    const double ep[] = {2020, 6, 1, 12, 0, 0};
    const gtime_t t0 = epoch2time(ep);
    eph_t dnav = make_bds_eph(sat, 1, 5.0e-9, 0.0, t0);
    nav_t nav{};
    nav.eph = &dnav;
    nav.n = 1;
    obsd_t observation{};
    observation.time = t0;
    observation.sat = static_cast<unsigned char>(sat);
    observation.P[0] = 24000000.0;
    observation.code[0] = CODE_L1X;

    double strict_position[6] = {};
    double strict_clock[2] = {};
    double strict_variance = 0.0;
    int strict_health = 0;
    satposs(t0, &observation, 1, &nav, EPHOPT_BRDC, strict_position,
        strict_clock, &strict_variance, &strict_health);
    EXPECT_DOUBLE_EQ(0.0, norm_rtk(strict_position, 3));

    double relative_position[6] = {};
    double relative_clock[2] = {};
    double relative_variance = 0.0;
    int relative_health = 0;
    satposs_relative(t0, &observation, 1, &nav, EPHOPT_BRDC, relative_position,
        relative_clock, &relative_variance, &relative_health);
    EXPECT_GT(norm_rtk(relative_position, 3), 1.0e6);
}

/* satwavelen BDS: frq0=B1I, frq1=B2, frq2=B3I */
TEST(BeidouB1cPvtHelpersTest, SatwavelenBdsKeepsOfficialB1B2B3Map)
{
    const int sat = satno(SYS_BDS, 19);
    nav_t nav{};
    const double lam0 = satwavelen(sat, 0, &nav);
    const double lam1 = satwavelen(sat, 1, &nav);
    const double lam2 = satwavelen(sat, 2, &nav);

    EXPECT_NEAR(lam0, SPEED_OF_LIGHT_M_S / FREQ1_BDS, 1.0e-9);
    EXPECT_NEAR(lam1, SPEED_OF_LIGHT_M_S / FREQ2_BDS, 1.0e-9);
    EXPECT_NEAR(lam2, SPEED_OF_LIGHT_M_S / FREQ3_BDS, 1.0e-9);

    EXPECT_NE(lam1, SPEED_OF_LIGHT_M_S / FREQ1);
    EXPECT_NE(lam0, SPEED_OF_LIGHT_M_S / FREQ1);
}

TEST(BeidouB1cPvtHelpersTest, SignalFreqMapB1cUsesFreq1)
{
    ASSERT_NE(SIGNAL_FREQ_MAP.find("1D"), SIGNAL_FREQ_MAP.end());
    ASSERT_NE(SIGNAL_FREQ_MAP.find("B1"), SIGNAL_FREQ_MAP.end());
    EXPECT_DOUBLE_EQ(SIGNAL_FREQ_MAP.at("1D"), FREQ1);
    EXPECT_DOUBLE_EQ(SIGNAL_FREQ_MAP.at("B1"), FREQ1_BDS);
    EXPECT_NE(SIGNAL_FREQ_MAP.at("1D"), SIGNAL_FREQ_MAP.at("B1"));
}

/* B1I and B1C both use slot 0; distinguished by RINEX code. */
TEST(BeidouB1cPvtHelpersTest, InsertObsPlacesB1iAndB1cOnSlot0WithDistinctCodes)
{
    constexpr int k_band = 0;
    constexpr uint32_t prn = 19;
    constexpr int week = 2100;

    Gnss_Synchro b1i = make_bds_synchro("B1", prn, 2.2e7);
    Gnss_Synchro b1c = make_bds_synchro("1D", prn, 2.1e7);

    obsd_t obs_b1i{};
    obs_b1i = insert_obs_to_rtklib(obs_b1i, b1i, week, k_band);
    EXPECT_EQ(obs_b1i.code[k_band], static_cast<unsigned char>(CODE_L2I));
    EXPECT_NEAR(obs_b1i.P[k_band], 2.2e7, 1.0e-3);

    obsd_t obs_b1c{};
    obs_b1c = insert_obs_to_rtklib(obs_b1c, b1c, week, k_band);
    EXPECT_EQ(obs_b1c.code[k_band], static_cast<unsigned char>(CODE_L1P));
    EXPECT_NEAR(obs_b1c.P[k_band], 2.1e7, 1.0e-3);
}

TEST(BeidouB1cPvtHelpersTest, GettgdPrefersMatchingEphTypeWhenBothPresent)
{
    const int sat = NSATGPS + NSATGLO + NSATGAL + NSATQZS + 21;
    const double ep[] = {2021, 3, 1, 0, 0, 0};
    const gtime_t t0 = epoch2time(ep);
    std::vector<eph_t> ephs;
    ephs.push_back(make_bds_eph(sat, 1, 5.0e-9, 0.0, t0));                       /* DNAV TGD1 */
    ephs.push_back(make_bds_eph(sat, BDS_EPH_SOURCE_CNAV1, 1.0e-9, 2.0e-9, t0)); /* CNAV1 */
    nav_t nav;
    std::memset(&nav, 0, sizeof(nav));
    nav.eph = ephs.data();
    nav.n = 2;

    const double tgd_b1c = gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L1D));
    const double tgd_b1c_combined =
        gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L1X));
    const double tgd_b1i = gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L2I));
    EXPECT_NEAR(tgd_b1c, SPEED_OF_LIGHT_M_S * 3.0e-9, 1.0e-6);
    EXPECT_NEAR(tgd_b1c_combined, SPEED_OF_LIGHT_M_S * 1.0e-9, 1.0e-6);
    EXPECT_NEAR(tgd_b1i, SPEED_OF_LIGHT_M_S * 5.0e-9, 1.0e-6);
}

TEST(BeidouB1cPvtHelpersTest, GettgdReturnsZeroWhenOnlyWrongEphFamilyPresent)
{
    const int sat = NSATGPS + NSATGLO + NSATGAL + NSATQZS + 22;
    const double ep[] = {2021, 3, 1, 0, 0, 0};
    const gtime_t t0 = epoch2time(ep);
    std::vector<eph_t> ephs;
    ephs.push_back(make_bds_eph(sat, 1, 5.0e-9, 0.0, t0)); /* DNAV only */
    nav_t nav;
    std::memset(&nav, 0, sizeof(nav));
    nav.eph = ephs.data();
    nav.n = 1;

    EXPECT_DOUBLE_EQ(gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L1D)), 0.0);
    EXPECT_DOUBLE_EQ(gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L1P)), 0.0);
    EXPECT_DOUBLE_EQ(gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L1X)), 0.0);
}

/* CODE_L1P also means GPS L1P; CNAV1 filtering is SYS_BDS-only. */
TEST(BeidouB1cPvtHelpersTest, GettgdGpsIgnoresB1cCodeSelection)
{
    const int sat = satno(SYS_GPS, 5);
    const double ep[] = {2020, 1, 1, 0, 0, 0};
    const gtime_t t0 = epoch2time(ep);
    eph_t e{};
    std::memset(&e, 0, sizeof(e));
    e.sat = sat;
    e.code = 0; /* LNAV, not CNAV1 */
    e.tgd[0] = 4.0e-9;
    e.toe = t0;
    e.toc = t0;
    nav_t nav;
    std::memset(&nav, 0, sizeof(nav));
    nav.eph = &e;
    nav.n = 1;

    EXPECT_NEAR(gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L1C)), SPEED_OF_LIGHT_M_S * 4.0e-9, 1.0e-6);
    EXPECT_NEAR(gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L1P)), SPEED_OF_LIGHT_M_S * 4.0e-9, 1.0e-6);
}
