/*!
 * \file frequency_error_reduction_test.cc
 * \brief  This file implements tests for the tracking frequency-error
 * reduction mechanism and bit/secondary-code synchronization.
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

#include "dll_pll_conf.h"
#include "dll_pll_veml_tracking.h"
#include "gnss_synchro.h"
#include "in_memory_configuration.h"
#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/null_source.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

TEST(FrequencyErrorReductionConfTest, DefaultIsDisabled)
{
    auto config = std::make_shared<InMemoryConfiguration>();
    Dll_Pll_Conf trk_params;
    trk_params.SetFromConfiguration(config.get(), "Tracking_1C");

    EXPECT_EQ(0U, trk_params.f_error_step_num);
}


TEST(FrequencyErrorReductionConfTest, OddValuePassesThroughUnchanged)
{
    auto config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Tracking_1C.f_error_step_num", "5");
    Dll_Pll_Conf trk_params;
    trk_params.SetFromConfiguration(config.get(), "Tracking_1C");

    EXPECT_EQ(5U, trk_params.f_error_step_num);
}


TEST(FrequencyErrorReductionConfTest, EvenValueGetsRoundedUpToOdd)
{
    auto config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Tracking_1C.f_error_step_num", "4");
    Dll_Pll_Conf trk_params;
    trk_params.SetFromConfiguration(config.get(), "Tracking_1C");

    EXPECT_EQ(5U, trk_params.f_error_step_num);
}


TEST(FrequencyErrorReductionConfTest, ZeroStaysZero)
{
    auto config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Tracking_1C.f_error_step_num", "0");
    Dll_Pll_Conf trk_params;
    trk_params.SetFromConfiguration(config.get(), "Tracking_1C");

    EXPECT_EQ(0U, trk_params.f_error_step_num);
}


TEST(FrequencyErrorReductionConfTest, AccumulationAndDopplerStepPassThrough)
{
    auto config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Tracking_1C.f_error_accumulation", "10");
    config->set_property("Tracking_1C.f_error_doppler_step", "125.0");
    Dll_Pll_Conf trk_params;
    trk_params.SetFromConfiguration(config.get(), "Tracking_1C");

    EXPECT_EQ(10U, trk_params.f_error_accumulation);
    EXPECT_DOUBLE_EQ(125.0, trk_params.f_error_doppler_step);
}


TEST(FrequencyErrorReductionBinMultiplierTest, CenterAndAlternatingOutward)
{
    EXPECT_DOUBLE_EQ(0.0, dll_pll_veml_tracking::f_error_bin_multiplier(0));
    EXPECT_DOUBLE_EQ(1.0, dll_pll_veml_tracking::f_error_bin_multiplier(1));
    EXPECT_DOUBLE_EQ(-1.0, dll_pll_veml_tracking::f_error_bin_multiplier(2));
    EXPECT_DOUBLE_EQ(2.0, dll_pll_veml_tracking::f_error_bin_multiplier(3));
    EXPECT_DOUBLE_EQ(-2.0, dll_pll_veml_tracking::f_error_bin_multiplier(4));
    EXPECT_DOUBLE_EQ(3.0, dll_pll_veml_tracking::f_error_bin_multiplier(5));
    EXPECT_DOUBLE_EQ(-3.0, dll_pll_veml_tracking::f_error_bin_multiplier(6));
}


TEST(FrequencyErrorReductionBinMultiplierTest, MatchesFiveBinLegacyPattern)
{
    // f_error_step_num = 5 must reproduce the original fixed 5-bin scan order:
    // +0, +step, -step, +2*step, -2*step
    const std::vector<double> expected = {0.0, 1.0, -1.0, 2.0, -2.0};
    for (uint32_t i = 0; i < expected.size(); i++)
        {
            EXPECT_DOUBLE_EQ(expected[i], dll_pll_veml_tracking::f_error_bin_multiplier(i));
        }
}


TEST(FrequencyErrorReductionConfTest, ZeroAccumulationIsClamped)
{
    InMemoryConfiguration config;
    config.set_property("Tracking_1C.f_error_step_num", "5");
    config.set_property("Tracking_1C.f_error_accumulation", "0");
    Dll_Pll_Conf params;
    params.SetFromConfiguration(&config, "Tracking_1C");

    EXPECT_EQ(1U, params.f_error_accumulation);
}


class DllPllTrackingFrequencyErrorTest : public ::testing::Test
{
protected:
    void make_tracking(bool galileo = false, uint32_t accumulation = 20U)
    {
        Dll_Pll_Conf params;
        params.fs_in = 2000000.0;
        params.vector_length = galileo ? 8000U : 2000U;
        params.high_dyn = true;
        params.f_error_step_num = 5;
        params.f_error_accumulation = accumulation;
        params.f_error_dump_filename.clear();
        if (galileo)
            {
                params.system = 'E';
                params.signal[1] = 'B';
            }
        synchro = Gnss_Synchro();
        synchro.PRN = 1;
        synchro.System = params.system;
        synchro.Signal[0] = params.signal[0];
        synchro.Signal[1] = params.signal[1];
        synchro.Acq_samplestamp_samples = 1000;
        tracking = dll_pll_veml_make_tracking(params);
        tracking->set_gnss_synchro(&synchro);
        tracking->start_tracking();
    }

    void check_polarity(bool galileo, bool rotating)
    {
        make_tracking(galileo);
        for (float polarity : {1.0F, -1.0F})
            {
                tracking->d_Prompt_circular_buffer.clear();
                for (uint32_t i = 0; i < tracking->d_secondary_code_length; ++i)
                    {
                        const float symbol = tracking->d_secondary_code_string[i] == '0' ? -polarity : polarity;
                        // End beyond quadrature to distinguish the current polarity from the
                        // arbitrary phase at the start of a differential correlation window.
                        const float phase = rotating ? 2.0F * static_cast<float>(i) / static_cast<float>(tracking->d_secondary_code_length - 1) : 0.0F;
                        tracking->d_Prompt_circular_buffer.push_back(symbol * gr_complex(std::cos(phase), std::sin(phase)));
                    }
                ASSERT_TRUE(tracking->acquire_secondary());
                EXPECT_EQ((polarity < 0.0F) != rotating, tracking->d_Flag_PLL_180_deg_phase_locked);
            }
    }

    void check_scan(uint32_t accumulation)
    {
        make_tracking(false, accumulation);
        const uint32_t count = std::max(1U, accumulation);
        ASSERT_EQ(count, tracking->d_trk_parameters.f_error_accumulation);
        tracking->d_state = 5;
        tracking->d_f_error_num_bins = 5;
        tracking->d_f_error_bin_index = 0;
        tracking->d_f_error_accum_counter = 0;
        tracking->d_f_error_center_doppler_hz = 0.0;
        tracking->d_f_error_power.assign(5, 0.0);
        tracking->d_f_error_prompt_samples.assign(5, std::vector<gr_complex>(count));
        tracking->d_code_freq_chips = tracking->d_code_chip_rate;
        tracking->d_current_prn_length_samples = 2000;
        for (uint32_t epoch = 0; epoch < 5 * count; ++epoch)
            {
                const uint32_t bin = tracking->d_f_error_bin_index;
                const float amplitude = (bin == 3 ? 10.0F : 1.0F) + 0.01F * static_cast<float>(epoch);
                *tracking->d_Prompt = gr_complex(amplitude, 0.1F);
                tracking->run_f_error_scan_step();
                tracking->update_tracking_vars(false);
                EXPECT_DOUBLE_EQ(0.0, tracking->d_carrier_phase_rate_step_rad);
                EXPECT_DOUBLE_EQ(0.0, tracking->d_code_phase_rate_step_chips);
                EXPECT_TRUE(tracking->d_carr_ph_history.empty());
                EXPECT_TRUE(tracking->d_code_ph_history.empty());
                EXPECT_NEAR(2.0 * std::acos(-1.0) * tracking->d_carrier_doppler_hz / tracking->d_trk_parameters.fs_in,
                    tracking->d_carrier_phase_step_rad, 1e-12);
            }
        EXPECT_EQ(5U, tracking->d_f_error_bin_index);
        EXPECT_DOUBLE_EQ(500.0, tracking->d_carrier_doppler_hz);
        tracking->begin_wide_tracking(200000);
        tracking->d_E_accu = gr_complex(1.0F, 0.0F);
        tracking->d_P_accu = gr_complex(1.0F, 0.0F);
        tracking->d_L_accu = gr_complex(1.0F, 0.0F);
        tracking->run_dll_pll();
        tracking->update_tracking_vars();
        EXPECT_NEAR(500.0, tracking->d_carrier_doppler_hz, 1e-5);
        EXPECT_DOUBLE_EQ(0.0, tracking->d_carrier_phase_rate_step_rad);
        EXPECT_EQ(1U, tracking->d_carr_ph_history.size());
    }

    void check_timers(bool scan)
    {
        make_tracking();
        const auto fs = static_cast<uint64_t>(tracking->d_trk_parameters.fs_in);
        const uint64_t acquisition_sample = synchro.Acq_samplestamp_samples;
        const uint64_t scan_end = acquisition_sample + 30 * fs;
        if (scan)
            {
                tracking->d_state = 5;
                EXPECT_EQ(0U, tracking->tracking_elapsed_seconds(scan_end));
                // Even if pull-in expired before scan entry, completion restores it.
                tracking->d_pull_in_transitory = false;
                tracking->d_carrier_lock_fail_counter = 50;
                tracking->d_code_lock_fail_counter = 50;
            }
        tracking->begin_wide_tracking(scan_end);
        EXPECT_EQ(2, tracking->d_state);
        EXPECT_EQ(acquisition_sample, tracking->d_acq_sample_stamp);
        if (scan)
            {
                EXPECT_TRUE(tracking->d_pull_in_transitory);
                EXPECT_EQ(0, tracking->d_carrier_lock_fail_counter);
                EXPECT_EQ(0, tracking->d_code_lock_fail_counter);
                EXPECT_EQ(0U, tracking->tracking_elapsed_seconds(scan_end));
                EXPECT_EQ(5U, tracking->tracking_elapsed_seconds(scan_end + 5 * fs));
                EXPECT_EQ(20U, tracking->tracking_elapsed_seconds(scan_end + 20 * fs));
                tracking->start_tracking();
                EXPECT_EQ(30U, tracking->tracking_elapsed_seconds(scan_end));
            }
        else
            {
                EXPECT_EQ(30U, tracking->tracking_elapsed_seconds(scan_end));
            }
    }

    void run_long_scan()
    {
        Dll_Pll_Conf params;
        params.signal[0] = '2';
        params.signal[1] = 'S';
        params.vector_length = 40000;
        params.f_error_step_num = 21;
        params.f_error_accumulation = 20;
        params.f_error_dump_filename.clear();
        params.high_dyn = true;
        params.enable_fll_pull_in = true;
        synchro = Gnss_Synchro();
        synchro.PRN = 1;
        synchro.System = 'G';
        synchro.Signal[0] = '2';
        synchro.Signal[1] = 'S';
        tracking = dll_pll_veml_make_tracking(params);
        tracking->set_gnss_synchro(&synchro);
        tracking->start_tracking();

        auto top_block = gr::make_top_block("Frequency error scan timers");
        auto source = gr::blocks::null_source::make(sizeof(gr_complex));
        // 8.4 seconds of passive scanning, initial alignment, and a few wide epochs.
        auto head = gr::blocks::head::make(sizeof(gr_complex), 424ULL * params.vector_length);
        auto sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(source, 0, head, 0);
        top_block->connect(head, 0, tracking, 0);
        top_block->connect(tracking, 0, sink, 0);
        top_block->run();

        EXPECT_EQ(21U, tracking->d_f_error_bin_index);
        EXPECT_EQ(2, tracking->d_state);
        EXPECT_TRUE(tracking->d_pull_in_transitory);
        EXPECT_EQ(0U, tracking->tracking_elapsed_seconds(tracking->nitems_read(0)));
        EXPECT_GT(tracking->nitems_read(0), 8ULL * static_cast<uint64_t>(params.fs_in));
        EXPECT_DOUBLE_EQ(0.0, tracking->d_carrier_phase_rate_step_rad);
    }

    Gnss_Synchro synchro;
    dll_pll_veml_tracking_sptr tracking;
};


TEST_F(DllPllTrackingFrequencyErrorTest, GpsPreamblePreservesBothPolarities)
{
    check_polarity(false, false);
}


TEST_F(DllPllTrackingFrequencyErrorTest, GalileoSecondaryPreservesBothPolarities)
{
    check_polarity(true, false);
}


TEST_F(DllPllTrackingFrequencyErrorTest, PolarityFollowsLatestPromptPhase)
{
    check_polarity(false, true);
}


TEST_F(DllPllTrackingFrequencyErrorTest, ScanDoesNotInjectDopplerRate)
{
    check_scan(20);
}


TEST_F(DllPllTrackingFrequencyErrorTest, DirectConstructionWithZeroAccumulationIsSafe)
{
    check_scan(0);
}


TEST_F(DllPllTrackingFrequencyErrorTest, ScanRestartsTrackingTimers)
{
    check_timers(true);
}


TEST_F(DllPllTrackingFrequencyErrorTest, DisabledScanPreservesAcquisitionTimeOrigin)
{
    check_timers(false);
}


TEST_F(DllPllTrackingFrequencyErrorTest, LongL2cScanRetainsFllPullIn)
{
    run_long_scan();
}
