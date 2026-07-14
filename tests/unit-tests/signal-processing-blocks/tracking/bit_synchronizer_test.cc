/*!
 * \file bit_synchronizer_test.cc
 * \brief Deterministic tests for the histogram-based bit synchronizer.
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

#include "bit_synchronizer.h"
#include <gtest/gtest.h>
#include <cmath>
#include <complex>
#include <cstdint>

namespace
{
HistogramBitSynchronizer::Config default_test_config()
{
    HistogramBitSynchronizer::Config cfg;
    cfg.bit_period_ms = 20;
    cfg.epoch_ms = 1;
    cfg.min_events_for_lock = 4;
    cfg.dominance_ratio = 1.0;
    cfg.runner_up_margin = 0.0;
    cfg.stable_best_required = 1;
    cfg.min_prompt_mag = 0.0F;
    cfg.transition_window_epochs = 3;
    cfg.transition_confidence = 0.6;
    cfg.tentative_events_required = 0;
    cfg.use_phase_dot_detector = true;
    return cfg;
}


bool feed_alternating_bits(HistogramBitSynchronizer& synchronizer,
    int edge_phase,
    int transition_count,
    float carrier_phase_step_rad = 0.0F,
    int detection_window_epochs = 3)
{
    const int period = synchronizer.bins();
    const int first_edge = (edge_phase == 0) ? period : edge_phase;
    const int epochs = first_edge + (transition_count - 1) * period + detection_window_epochs;
    float polarity = 1.0F;
    bool lock_event = false;

    for (int k = 0; k < epochs; ++k)
        {
            if (k > 0 && (k % period) == edge_phase)
                {
                    polarity = -polarity;
                }
            const float phase = carrier_phase_step_rad * static_cast<float>(k);
            const std::complex<float> prompt = polarity * std::complex<float>(std::cos(phase), std::sin(phase));
            lock_event = synchronizer.update(prompt, true) || lock_event;
        }
    return lock_event;
}


void feed_until_epoch(HistogramBitSynchronizer& synchronizer,
    std::int64_t target_epoch,
    float& polarity)
{
    while (synchronizer.get_epoch_count() <= target_epoch)
        {
            synchronizer.update(std::complex<float>(polarity, 0.0F), true);
        }
}


void add_transition_at(HistogramBitSynchronizer& synchronizer,
    std::int64_t epoch,
    float& polarity)
{
    feed_until_epoch(synchronizer, epoch - 1, polarity);
    polarity = -polarity;
    synchronizer.update(std::complex<float>(polarity, 0.0F), true);
}
}  // namespace


TEST(BitSynchronizerTest, LocksAtKnownPhase)
{
    HistogramBitSynchronizer synchronizer(default_test_config());

    EXPECT_TRUE(feed_alternating_bits(synchronizer, 7, 4));
    EXPECT_TRUE(synchronizer.locked());
    EXPECT_EQ(synchronizer.edge_phase(), 7);
}


TEST(BitSynchronizerTest, HandlesPhaseZeroWraparound)
{
    HistogramBitSynchronizer::Config cfg = default_test_config();
    cfg.min_events_for_lock = 3;
    HistogramBitSynchronizer synchronizer(cfg);

    EXPECT_TRUE(feed_alternating_bits(synchronizer, 0, 3));
    EXPECT_EQ(synchronizer.edge_phase(), 0);
    EXPECT_TRUE(synchronizer.is_edge_epoch(0));
    EXPECT_TRUE(synchronizer.is_edge_epoch(20));
    EXPECT_FALSE(synchronizer.is_edge_epoch(19));
}


TEST(BitSynchronizerTest, IgnoresPromptsBelowMagnitudeThreshold)
{
    HistogramBitSynchronizer::Config cfg = default_test_config();
    cfg.bit_period_ms = 5;
    cfg.min_events_for_lock = 1;
    cfg.min_prompt_mag = 0.5F;
    cfg.transition_window_epochs = 1;
    HistogramBitSynchronizer synchronizer(cfg);

    EXPECT_FALSE(synchronizer.update(std::complex<float>(1.0F, 0.0F), true));
    EXPECT_FALSE(synchronizer.update(std::complex<float>(0.1F, 0.0F), true));
    EXPECT_FALSE(synchronizer.update(std::complex<float>(-1.0F, 0.0F), true));
    EXPECT_FALSE(synchronizer.locked());

    EXPECT_FALSE(synchronizer.update(std::complex<float>(-1.0F, 0.0F), true));
    EXPECT_TRUE(synchronizer.update(std::complex<float>(1.0F, 0.0F), true));
    EXPECT_EQ(synchronizer.edge_phase(), 4);
}


TEST(BitSynchronizerTest, AdjacentBinCompetitionRequiresDominantWinner)
{
    HistogramBitSynchronizer::Config cfg = default_test_config();
    cfg.bit_period_ms = 10;
    cfg.min_events_for_lock = 6;
    cfg.dominance_ratio = 0.5;
    cfg.runner_up_margin = 0.4;
    cfg.stable_best_required = 2;
    cfg.transition_window_epochs = 1;
    HistogramBitSynchronizer synchronizer(cfg);
    float polarity = 1.0F;

    synchronizer.update(std::complex<float>(polarity, 0.0F), true);
    add_transition_at(synchronizer, 4, polarity);
    add_transition_at(synchronizer, 15, polarity);
    add_transition_at(synchronizer, 24, polarity);
    add_transition_at(synchronizer, 35, polarity);
    add_transition_at(synchronizer, 44, polarity);
    add_transition_at(synchronizer, 54, polarity);

    EXPECT_FALSE(synchronizer.locked());

    add_transition_at(synchronizer, 64, polarity);
    EXPECT_FALSE(synchronizer.locked());
    add_transition_at(synchronizer, 74, polarity);
    EXPECT_TRUE(synchronizer.locked());
    EXPECT_EQ(synchronizer.edge_phase(), 4);
}


TEST(BitSynchronizerTest, RequiresIndependentEvidenceForStability)
{
    HistogramBitSynchronizer::Config cfg = default_test_config();
    cfg.bit_period_ms = 5;
    cfg.min_events_for_lock = 1;
    cfg.stable_best_required = 2;
    cfg.transition_window_epochs = 1;
    HistogramBitSynchronizer synchronizer(cfg);

    synchronizer.update(std::complex<float>(1.0F, 0.0F), true);
    EXPECT_FALSE(synchronizer.update(std::complex<float>(-1.0F, 0.0F), true));
    for (int k = 0; k < 4; ++k)
        {
            EXPECT_FALSE(synchronizer.update(std::complex<float>(-1.0F, 0.0F), true));
        }
    EXPECT_FALSE(synchronizer.locked());

    EXPECT_TRUE(synchronizer.update(std::complex<float>(1.0F, 0.0F), true));
    EXPECT_TRUE(synchronizer.locked());
    EXPECT_EQ(synchronizer.edge_phase(), 1);
}


TEST(BitSynchronizerTest, ToleratesResidualCarrierRotation)
{
    HistogramBitSynchronizer synchronizer(default_test_config());

    EXPECT_TRUE(feed_alternating_bits(synchronizer, 3, 4, 0.2F));
    EXPECT_TRUE(synchronizer.locked());
    EXPECT_EQ(synchronizer.edge_phase(), 3);
}


TEST(BitSynchronizerTest, RequiresConfiguredTentativeConfirmations)
{
    HistogramBitSynchronizer::Config cfg = default_test_config();
    cfg.bit_period_ms = 10;
    cfg.min_events_for_lock = 2;
    cfg.stable_best_required = 1;
    cfg.transition_window_epochs = 1;
    cfg.tentative_events_required = 2;
    HistogramBitSynchronizer synchronizer(cfg);
    float polarity = 1.0F;

    synchronizer.update(std::complex<float>(polarity, 0.0F), true);
    add_transition_at(synchronizer, 4, polarity);
    add_transition_at(synchronizer, 14, polarity);
    EXPECT_FALSE(synchronizer.locked());

    add_transition_at(synchronizer, 24, polarity);
    EXPECT_FALSE(synchronizer.locked());
    add_transition_at(synchronizer, 34, polarity);
    EXPECT_TRUE(synchronizer.locked());
    EXPECT_EQ(synchronizer.edge_phase(), 4);
}


TEST(BitSynchronizerTest, RejectsMismatchedTentativeConfirmation)
{
    HistogramBitSynchronizer::Config cfg = default_test_config();
    cfg.bit_period_ms = 10;
    cfg.min_events_for_lock = 2;
    cfg.dominance_ratio = 0.5;
    cfg.runner_up_margin = 0.1;
    cfg.transition_window_epochs = 1;
    cfg.tentative_events_required = 2;
    HistogramBitSynchronizer synchronizer(cfg);
    float polarity = 1.0F;

    synchronizer.update(std::complex<float>(polarity, 0.0F), true);
    add_transition_at(synchronizer, 4, polarity);
    add_transition_at(synchronizer, 14, polarity);
    EXPECT_FALSE(synchronizer.locked());

    add_transition_at(synchronizer, 25, polarity);
    EXPECT_FALSE(synchronizer.locked());
    add_transition_at(synchronizer, 34, polarity);
    EXPECT_FALSE(synchronizer.locked());
    add_transition_at(synchronizer, 44, polarity);
    EXPECT_TRUE(synchronizer.locked());
    EXPECT_EQ(synchronizer.edge_phase(), 4);
}


TEST(BitSynchronizerTest, ResetClearsStateAndAllowsReacquisition)
{
    HistogramBitSynchronizer::Config cfg = default_test_config();
    cfg.min_events_for_lock = 2;
    HistogramBitSynchronizer synchronizer(cfg);

    ASSERT_TRUE(feed_alternating_bits(synchronizer, 7, 2));
    ASSERT_EQ(synchronizer.edge_phase(), 7);

    synchronizer.reset();
    EXPECT_FALSE(synchronizer.locked());
    EXPECT_EQ(synchronizer.edge_phase(), -1);
    EXPECT_EQ(synchronizer.epochs_until_next_edge(), -1);

    EXPECT_TRUE(feed_alternating_bits(synchronizer, 11, 2));
    EXPECT_EQ(synchronizer.edge_phase(), 11);
}
