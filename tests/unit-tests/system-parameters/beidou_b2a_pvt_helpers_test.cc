/*!
 * \file beidou_b2a_pvt_helpers_test.cc
 * \brief Unit tests for B2a PVT helpers (CNAV2 TGD/ISC, obs code, wavelength)
 * \author huangchuhan, 2026. huangchh37(at)mail2.sysu.edu.cn
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
#include "Beidou_CNAV2.h"
#include "MATH_CONSTANTS.h"
#include "beidou_cnav1_ephemeris.h"
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
#include <map>
#include <string>
#include <vector>

namespace
{
eph_t make_bds_b2a_eph(int sat, int code, double tgd0, double tgd1, double tgd2, gtime_t toe)
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
    e.tgd[1] = tgd1;
    e.tgd[2] = tgd2;
    return e;
}

Gnss_Synchro make_bds_b2a_synchro(const char *signal, uint32_t prn, double pr_m)
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

TEST(BeidouB2aPvtHelpersTest, GettgdAppliesTgdB2apAndIscB2adForDataComponent)
{
    const int sat = NSATGPS + NSATGLO + NSATGAL + NSATQZS + 27;
    const double ep[] = {2021, 12, 15, 3, 0, 0};
    const gtime_t t0 = epoch2time(ep);
    std::vector<eph_t> ephs;
    ephs.push_back(make_bds_b2a_eph(sat, BDS_EPH_SOURCE_CNAV2, 0.0, 2.0e-9, 3.0e-9, t0));
    nav_t nav{};
    nav.eph = ephs.data();
    nav.n = 1;

    const double tgd_b2a = gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L5D));
    EXPECT_NEAR(tgd_b2a, SPEED_OF_LIGHT_M_S * 5.0e-9, 1.0e-6);
}

TEST(BeidouB2aPvtHelpersTest, GettgdDoesNotUseCnav1OrDnavForB2aObs)
{
    const int sat = NSATGPS + NSATGLO + NSATGAL + NSATQZS + 28;
    const double ep[] = {2021, 12, 15, 3, 0, 0};
    const gtime_t t0 = epoch2time(ep);
    std::vector<eph_t> ephs;
    ephs.push_back(make_bds_b2a_eph(sat, 1, 5.0e-9, 0.0, 0.0, t0));                          /* DNAV */
    ephs.push_back(make_bds_b2a_eph(sat, BDS_EPH_SOURCE_CNAV1, 1.0e-9, 4.0e-9, 2.0e-9, t0)); /* CNAV1 */
    nav_t nav{};
    nav.eph = ephs.data();
    nav.n = 2;

    EXPECT_DOUBLE_EQ(gettgd_bds_by_obs_code(sat, &nav, static_cast<unsigned char>(CODE_L5D)), 0.0);
}

TEST(BeidouB2aPvtHelpersTest, SelephPrefersCnav2WhenRequested)
{
    const int sat = NSATGPS + NSATGLO + NSATGAL + NSATQZS + 37;
    const double ep[] = {2021, 12, 15, 3, 0, 0};
    const gtime_t t0 = epoch2time(ep);
    std::vector<eph_t> ephs;
    ephs.push_back(make_bds_b2a_eph(sat, BDS_EPH_SOURCE_CNAV1, 1.0e-9, 0.0, 0.0, t0));
    ephs.push_back(make_bds_b2a_eph(sat, BDS_EPH_SOURCE_CNAV2, 0.0, 2.0e-9, 3.0e-9, t0));
    nav_t nav{};
    nav.eph = ephs.data();
    nav.n = 2;

    eph_t *e_cnav2 = seleph(t0, sat, -1, &nav, BDS_EPH_SOURCE_CNAV2);
    eph_t *e_cnav1 = seleph(t0, sat, -1, &nav, BDS_EPH_SOURCE_CNAV1);
    ASSERT_NE(e_cnav2, static_cast<eph_t *>(nullptr));
    ASSERT_NE(e_cnav1, static_cast<eph_t *>(nullptr));
    EXPECT_EQ(e_cnav2->code, BDS_EPH_SOURCE_CNAV2);
    EXPECT_EQ(e_cnav1->code, BDS_EPH_SOURCE_CNAV1);
}

TEST(BeidouB2aPvtHelpersTest, SatwavelenBdsKeepsOfficialB1B2B3Map)
{
    const int sat = satno(SYS_BDS, 27);
    nav_t nav{};
    EXPECT_NEAR(satwavelen(sat, 0, &nav), SPEED_OF_LIGHT_M_S / FREQ1_BDS, 1.0e-9);
    EXPECT_NEAR(satwavelen(sat, 1, &nav), SPEED_OF_LIGHT_M_S / FREQ2_BDS, 1.0e-9);
    EXPECT_NEAR(satwavelen(sat, 2, &nav), SPEED_OF_LIGHT_M_S / FREQ3_BDS, 1.0e-9);
    EXPECT_NE(satwavelen(sat, 2, &nav), SPEED_OF_LIGHT_M_S / FREQ5);
}

TEST(BeidouB2aPvtHelpersTest, SignalFreqMapB2aUsesFreq5)
{
    ASSERT_NE(SIGNAL_FREQ_MAP.find("5D"), SIGNAL_FREQ_MAP.end());
    EXPECT_DOUBLE_EQ(SIGNAL_FREQ_MAP.at("5D"), FREQ5);
    EXPECT_DOUBLE_EQ(SIGNAL_FREQ_MAP.at("B2"), FREQ2_BDS);
    EXPECT_NE(SIGNAL_FREQ_MAP.at("5D"), SIGNAL_FREQ_MAP.at("B2"));
}

TEST(BeidouB2aPvtHelpersTest, InsertObsPlacesB2aOnSlot0WithCodeL5d)
{
    constexpr int k_band = 0;
    Gnss_Synchro b2a = make_bds_b2a_synchro("5D", 27, 2.3e7);
    obsd_t obs{};
    obs = insert_obs_to_rtklib(obs, b2a, 2188, k_band);
    EXPECT_EQ(obs.code[k_band], static_cast<unsigned char>(CODE_L5D));
    EXPECT_NEAR(obs.P[k_band], 2.3e7, 1.0e-3);
}


TEST(BeidouB2aPvtHelpersTest, InsertObsPlacesB2aOnSlot2WithCodeL5d)
{
    constexpr int k_band = 2;
    Gnss_Synchro b2a = make_bds_b2a_synchro("5D", 27, 2.3e7);
    obsd_t obs{};
    obs = insert_obs_to_rtklib(obs, b2a, 2188, k_band);
    EXPECT_EQ(obs.code[k_band], static_cast<unsigned char>(CODE_L5D));
    EXPECT_EQ(obs.code[0], static_cast<unsigned char>(CODE_NONE));
}


TEST(BeidouB2aPvtHelpersTest, Cnav1AndCnav2MapsAreIndependent)
{
    std::map<int, Beidou_Cnav1_Ephemeris> cnav1;
    std::map<int, Beidou_Cnav1_Ephemeris> cnav2;
    Beidou_Cnav1_Ephemeris e1{};
    e1.PRN = 27;
    e1.sig_type = BDS_EPH_SOURCE_CNAV1;
    e1.toe = 3000;
    Beidou_Cnav1_Ephemeris e2{};
    e2.PRN = 27;
    e2.sig_type = BDS_EPH_SOURCE_CNAV2;
    e2.toe = 6000;
    cnav1[27] = e1;
    cnav2[27] = e2;
    EXPECT_EQ(cnav1[27].sig_type, BDS_EPH_SOURCE_CNAV1);
    EXPECT_EQ(cnav2[27].sig_type, BDS_EPH_SOURCE_CNAV2);
    EXPECT_EQ(cnav1[27].toe, 3000);
    EXPECT_EQ(cnav2[27].toe, 6000);
}
