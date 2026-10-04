/*!
 * \file rtklib_detslp_dop_test.cc
 * \brief Unit tests for the doppler / phase-difference cycle-slip detector
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
#include "rtklib_rtkcmn.h"
#include "rtklib_rtkpos.h"
#include <gtest/gtest.h>
#include <memory>

namespace
{
constexpr double LAMBDA_L1 = SPEED_OF_LIGHT_M_S / 1575.42e6;
constexpr double PHASE_RATE_CYCLES_S = 1000.0;
constexpr int NUM_SATS = 4;

struct DetslpDopFixture
{
    DetslpDopFixture()
        : rtk(std::make_unique<rtk_t>()),
          nav(std::make_unique<nav_t>())
    {
        *rtk = {};
        *nav = {};
        rtk->opt.nf = 1;

        const double ep[] = {2026, 8, 10, 0, 0, 0.0};
        t_prev = epoch2time(ep);
        t_now = timeadd(t_prev, 1.0);

        for (int i = 0; i < NUM_SATS; i++)
            {
                ix[i] = i;
                obs[i] = {};
                obs[i].sat = i + 1;
                obs[i].time = t_now;
                /* doppler consistent with the phase advance on every satellite */
                obs[i].L[0] = 100.0 + PHASE_RATE_CYCLES_S;
                obs[i].D[0] = -PHASE_RATE_CYCLES_S;
                rtk->ssat[i].ph[0][0] = 100.0;
                rtk->ssat[i].pt[0][0] = t_prev;
                nav->lam[i][0] = LAMBDA_L1;
            }
    }
    std::unique_ptr<rtk_t> rtk;
    std::unique_ptr<nav_t> nav;
    obsd_t obs[NUM_SATS]{};
    int ix[NUM_SATS]{};
    gtime_t t_prev{};
    gtime_t t_now{};
};
}  // namespace


TEST(RtklibDetslpDopTest, FlagsOnlyTheSatelliteWithInconsistentPhase)
{
    DetslpDopFixture f;
    /* 25-cycle slip on the last satellite: phase jumps, doppler does not */
    f.obs[NUM_SATS - 1].L[0] += 25.0;

    /* threshold 0 disables the test */
    f.rtk->opt.thresdop = 0.0;
    detslp_dop(f.rtk.get(), f.obs, f.ix, NUM_SATS, 1, f.nav.get());
    for (int i = 0; i < NUM_SATS; i++)
        {
            EXPECT_EQ(f.rtk->ssat[i].slip[0] & 1, 0) << "sat " << i + 1;
        }

    f.rtk->opt.thresdop = 1.0;
    detslp_dop(f.rtk.get(), f.obs, f.ix, NUM_SATS, 1, f.nav.get());
    for (int i = 0; i < NUM_SATS - 1; i++)
        {
            EXPECT_EQ(f.rtk->ssat[i].slip[0] & 1, 0) << "sat " << i + 1;
        }
    EXPECT_EQ(f.rtk->ssat[NUM_SATS - 1].slip[0] & 1, 1);
}


TEST(RtklibDetslpDopTest, ImmuneToCommonReceiverClockJump)
{
    DetslpDopFixture f;
    /* a receiver clock jump advances every carrier phase by the same amount;
       the median common range-rate error must absorb it without false slips */
    for (int i = 0; i < NUM_SATS; i++)
        {
            f.obs[i].L[0] += 2000.0;
        }

    f.rtk->opt.thresdop = 1.0;
    detslp_dop(f.rtk.get(), f.obs, f.ix, NUM_SATS, 1, f.nav.get());
    for (int i = 0; i < NUM_SATS; i++)
        {
            EXPECT_EQ(f.rtk->ssat[i].slip[0] & 1, 0) << "sat " << i + 1;
        }
}


TEST(RtklibDetslpDopTest, DetectsSlipOnTopOfClockJump)
{
    DetslpDopFixture f;
    for (int i = 0; i < NUM_SATS; i++)
        {
            f.obs[i].L[0] += 2000.0;
        }
    f.obs[0].L[0] += 25.0; /* slip on the first satellite, on top of the jump */

    f.rtk->opt.thresdop = 1.0;
    detslp_dop(f.rtk.get(), f.obs, f.ix, NUM_SATS, 1, f.nav.get());
    EXPECT_EQ(f.rtk->ssat[0].slip[0] & 1, 1);
    for (int i = 1; i < NUM_SATS; i++)
        {
            EXPECT_EQ(f.rtk->ssat[i].slip[0] & 1, 0) << "sat " << i + 1;
        }
}
