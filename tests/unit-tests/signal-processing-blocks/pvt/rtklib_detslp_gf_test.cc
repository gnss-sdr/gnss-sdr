/*!
 * \file rtklib_detslp_gf_test.cc
 * \brief Unit tests for uncombined PPP geometry-free cycle-slip detection
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

#include "rtklib.h"
#include "rtklib_ppp.h"
#include "rtklib_rtkcmn.h"
#include <gtest/gtest.h>
#include <array>
#include <memory>

class RtklibDetslpGfTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rtk.reset(new rtk_t{});
        nav.reset(new nav_t{});
        rtk->opt.ionoopt = IONOOPT_EST;
        rtk->opt.thresslip = 0.05;
        for (size_t i = 0; i < obs.size(); i++)
            {
                obs[i].sat = satno(i < 3 ? SYS_GPS : SYS_GAL, static_cast<int>(i) + 1);
                for (int f = 0; f < NFREQ; f++)
                    {
                        nav->lam[obs[i].sat - 1][f] = LAM_CARR[f];
                        obs[i].L[f] = 100000.0 - 10000.0 * f;
                    }
            }
    }

    void detect()
    {
        detslp_gf_est(rtk.get(), obs.data(), static_cast<int>(obs.size()), nav.get());
    }

    void expect_slips(size_t satellite, bool first, bool second, bool third)
    {
        const auto& ssat = rtk->ssat[obs[satellite].sat - 1];
        EXPECT_EQ(first, (ssat.slip[0] & 1) != 0);
        EXPECT_EQ(second, (ssat.slip[1] & 1) != 0);
        EXPECT_EQ(third, (ssat.slip[2] & 1) != 0);
    }

    void check_returning_band(int returning_slot)
    {
        detect();
        const double phase = obs[0].L[returning_slot];
        obs[0].L[returning_slot] = 0.0;
        detect();
        obs[0].L[returning_slot] = phase;
        obs[0].L[0] += 10.0;
        detect();

        // The returning pair has no previous measurement to identify which
        // band slipped: reset both bands of the pair that can be compared.
        expect_slips(0, true, returning_slot == 2, returning_slot == 1);
        for (size_t i = 1; i < obs.size(); i++)
            {
                expect_slips(i, false, false, false);
            }
    }

    std::unique_ptr<rtk_t> rtk;
    std::unique_ptr<nav_t> nav;
    std::array<obsd_t, 4> obs{};
};


TEST_F(RtklibDetslpGfTest, DetectsSlipOnTheOnlySatelliteOfAConstellation)
{
    detect();
    obs[3].L[2] += 10.0;
    detect();
    expect_slips(3, false, false, true);
    for (size_t i = 0; i < 3; i++)
        {
            expect_slips(i, false, false, false);
        }
}


TEST_F(RtklibDetslpGfTest, DetectsSlipOnTheOnlySatelliteWithABandPair)
{
    obs[1].L[2] = obs[2].L[2] = 0.0;
    detect();
    obs[0].L[2] += 10.0;
    detect();
    expect_slips(0, false, false, true);
    for (size_t i = 1; i < obs.size(); i++)
        {
            expect_slips(i, false, false, false);
        }
}


TEST_F(RtklibDetslpGfTest, ReturningThirdBandDoesNotHideFirstBandSlip)
{
    check_returning_band(2);
}


TEST_F(RtklibDetslpGfTest, ReturningSecondBandDoesNotHideFirstBandSlip)
{
    check_returning_band(1);
}


TEST_F(RtklibDetslpGfTest, ContinuousOtherPairIdentifiesTheSlippedBand)
{
    detect();
    obs[0].L[1] += 10.0;
    detect();
    expect_slips(0, false, true, false);
    for (size_t i = 1; i < obs.size(); i++)
        {
            expect_slips(i, false, false, false);
        }
}


TEST_F(RtklibDetslpGfTest, RejectsCommonBandDriftAndDetectsSuperimposedSlip)
{
    detect();
    for (size_t i = 0; i < 3; i++)
        {
            obs[i].L[2] += 20.0;
        }
    detect();
    for (size_t i = 0; i < obs.size(); i++)
        {
            expect_slips(i, false, false, false);
        }

    for (size_t i = 0; i < 3; i++)
        {
            obs[i].L[2] += 20.0;
        }
    obs[0].L[2] += 10.0;
    detect();
    expect_slips(0, false, false, true);
    for (size_t i = 1; i < obs.size(); i++)
        {
            expect_slips(i, false, false, false);
        }
}


TEST_F(RtklibDetslpGfTest, FirstMeasurementsDoNotFlagSlips)
{
    detect();
    for (size_t i = 0; i < obs.size(); i++)
        {
            expect_slips(i, false, false, false);
        }
}
