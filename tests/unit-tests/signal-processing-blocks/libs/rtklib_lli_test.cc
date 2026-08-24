/*!
 * \file rtklib_lli_test.cc
 * \brief This file implements tests for the loss-of-lock indicator carried from
 * the Observables block to the RTKLIB observation structure.
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

#include "gnss_synchro.h"
#include "rtklib_conversions.h"
#include <gtest/gtest.h>
#include <cstring>

namespace
{
Gnss_Synchro make_gps_l1_observation()
{
    Gnss_Synchro observation = Gnss_Synchro();
    observation.System = 'G';
    std::memcpy(static_cast<void*>(observation.Signal), "1C", 3);
    observation.PRN = 11;
    observation.Pseudorange_m = 22000000.0;
    observation.Carrier_phase_rads = 1234.5;
    observation.Carrier_Doppler_hz = -1500.0;
    observation.CN0_dB_hz = 45.0;
    return observation;
}

unsigned int lli_of(const Gnss_Synchro& observation)
{
    obsd_t rtklib_observation{};
    return insert_obs_to_rtklib(rtklib_observation, observation, 2200, 0).LLI[0];
}
}  // namespace


TEST(RtklibLliTest, ACleanObservationCarriesNoIndicator)
{
    EXPECT_EQ(0U, lli_of(make_gps_l1_observation()));
}


TEST(RtklibLliTest, ACycleSlipSetsBitZero)
{
    Gnss_Synchro observation = make_gps_l1_observation();
    observation.Flag_cycle_slip = true;
    EXPECT_EQ(1U, lli_of(observation));
}


TEST(RtklibLliTest, AHalfCycleSlipSetsBitZero)
{
    // A half-cycle re-resolution is a one-epoch slip event. LLI bit 1 is a
    // persistent "half-cycle unresolved" state whose every transition counts
    // as a slip in detslp_ll, so mapping the pulse there would reset the
    // phase bias a second time when the flag drops one epoch later
    Gnss_Synchro observation = make_gps_l1_observation();
    observation.Flag_half_cycle_slip = true;
    EXPECT_EQ(1U, lli_of(observation));
}


TEST(RtklibLliTest, BothConditionsShareBitZero)
{
    Gnss_Synchro observation = make_gps_l1_observation();
    observation.Flag_cycle_slip = true;
    observation.Flag_half_cycle_slip = true;
    EXPECT_EQ(1U, lli_of(observation));
}
