/*!
 * \file observables_phase_continuity_test.cc
 * \brief This file implements tests for the carrier phase continuity
 * bookkeeping of the observables block.
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

#include "hybrid_observables_gs.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>

namespace
{
constexpr double EPOCH_INTERVAL_S = 0.02;

bool is_discontinuous(bool has_previous_observation,
    uint64_t last_valid_epoch,
    uint64_t current_epoch,
    uint32_t last_valid_prn = 7,
    uint32_t current_prn = 7)
{
    return hybrid_observables_gs::phase_stream_is_discontinuous(has_previous_observation,
        last_valid_epoch, current_epoch, last_valid_prn, current_prn, EPOCH_INTERVAL_S);
}
}  // namespace


TEST(ObservablesPhaseContinuityTest, ConsecutiveObservationsOfTheSameSatelliteAreContinuous)
{
    EXPECT_FALSE(is_discontinuous(true, 41, 42));
}


TEST(ObservablesPhaseContinuityTest, TheFirstObservationOfAChannelIsDiscontinuous)
{
    EXPECT_TRUE(is_discontinuous(false, 0, 1, 0, 7));
}


TEST(ObservablesPhaseContinuityTest, ASatelliteChangeOnTheChannelIsDiscontinuous)
{
    EXPECT_TRUE(is_discontinuous(true, 41, 42, 7, 11));
}


TEST(ObservablesPhaseContinuityTest, ShortInterruptionsDoNotBreakContinuity)
{
    // dropping a few epochs is a transient failure to interpolate the tracking
    // history: the channel never lost lock, so the ambiguity is unchanged
    EXPECT_FALSE(is_discontinuous(true, 41, 43));
    EXPECT_FALSE(is_discontinuous(true, 41, 50));
}


TEST(ObservablesPhaseContinuityTest, InterruptionsLongerThanAReacquisitionAreDiscontinuous)
{
    // Round to the nearest epoch: truncating would turn the x87 quotient
    // 49.999... into an off-by-one epoch count on 32-bit x86
    const auto epochs_in_gap = static_cast<uint64_t>(std::llround(
        hybrid_observables_gs::MIN_REACQUISITION_GAP_S / EPOCH_INTERVAL_S));
    EXPECT_FALSE(is_discontinuous(true, 100, 100 + epochs_in_gap));
    EXPECT_TRUE(is_discontinuous(true, 100, 100 + epochs_in_gap + 1));
    EXPECT_TRUE(is_discontinuous(true, 100, 100 + 10 * epochs_in_gap));
}


TEST(ObservablesPhaseContinuityTest, RepeatedOrOutOfOrderEpochsAreDiscontinuous)
{
    EXPECT_TRUE(is_discontinuous(true, 41, 41));
    EXPECT_TRUE(is_discontinuous(true, 41, 40));
}


TEST(ObservablesHalfCycleTest, AStableCostasPolarityIsNotAHalfCycleSlip)
{
    EXPECT_FALSE(hybrid_observables_gs::half_cycle_ambiguity_changed(true, false, 7, 7, false, false));
    // a satellite permanently locked at 180 degrees is not ambiguous either: the
    // Telemetry Decoder compensates it on every symbol
    EXPECT_FALSE(hybrid_observables_gs::half_cycle_ambiguity_changed(true, false, 7, 7, true, true));
}


TEST(ObservablesHalfCycleTest, APolarityChangeIsAHalfCycleSlip)
{
    EXPECT_TRUE(hybrid_observables_gs::half_cycle_ambiguity_changed(true, false, 7, 7, false, true));
    EXPECT_TRUE(hybrid_observables_gs::half_cycle_ambiguity_changed(true, false, 7, 7, true, false));
}


TEST(ObservablesHalfCycleTest, ANewAmbiguitySupersedesTheHalfCycleReport)
{
    // no previous observation to compare against
    EXPECT_FALSE(hybrid_observables_gs::half_cycle_ambiguity_changed(false, false, 7, 7, false, true));
    // the carrier phase ambiguity is already reported as new
    EXPECT_FALSE(hybrid_observables_gs::half_cycle_ambiguity_changed(true, true, 7, 7, false, true));
    // another satellite on the channel
    EXPECT_FALSE(hybrid_observables_gs::half_cycle_ambiguity_changed(true, false, 7, 11, false, true));
}


TEST(ObservablesPhaseContinuityTest, ContinuityIsPreservedAcrossTheEpochCounterRange)
{
    const uint64_t large_epoch = 4300000000ULL;
    EXPECT_FALSE(is_discontinuous(true, large_epoch, large_epoch + 1));
    EXPECT_TRUE(is_discontinuous(true, large_epoch, large_epoch + 1000));
}
