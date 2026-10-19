/*!
 * \file gnss_flowgraph_test.cc
 * \brief  This file implements tests for a flowgraph
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "acquisition_interface.h"
#include "channel.h"
#include "channel_interface.h"
#include "concurrent_queue.h"
#include "file_configuration.h"
#include "file_signal_source.h"
#include "gnss_block_interface.h"
#include "gnss_flowgraph.h"
#include "in_memory_configuration.h"
#include "pass_through.h"
#include "tracking_interface.h"
#include <gtest/gtest.h>
#include <utility>


TEST(GNSSFlowgraph /*unused*/, InstantiateConnectStartStopOldNotation /*unused*/)
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.SUPL_gps_enabled", "false");
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.repeat", "true");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat";
    config->set_property("SignalSource.filename", std::move(filename));
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("Channels_1C.count", "1");
    config->set_property("Channels.in_acquisition", "1");
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.threshold", "1");
    config->set_property("Acquisition_1C.doppler_max", "5000");
    config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("Observables.implementation", "Hybrid_Observables");
    config->set_property("PVT.implementation", "RTKLIB_PVT");

    std::shared_ptr<GNSSFlowgraph> flowgraph = std::make_shared<GNSSFlowgraph>(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());

    EXPECT_NO_THROW(flowgraph->connect());
    EXPECT_TRUE(flowgraph->connected());

    EXPECT_NO_THROW(flowgraph->start());
    EXPECT_TRUE(flowgraph->running());
    flowgraph->stop();
    EXPECT_FALSE(flowgraph->running());
}


TEST(GNSSFlowgraph /*unused*/, InstantiateConnectStartStop /*unused*/)
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.repeat", "true");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat";
    config->set_property("SignalSource.filename", std::move(filename));
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("Channels_1C.count", "8");
    config->set_property("Channels.in_acquisition", "1");
    config->set_property("Channel.signal", "1C");
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.threshold", "1");
    config->set_property("Acquisition_1C.doppler_max", "5000");
    config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("Observables.implementation", "Hybrid_Observables");
    config->set_property("PVT.implementation", "RTKLIB_PVT");

    std::shared_ptr<GNSSFlowgraph> flowgraph = std::make_shared<GNSSFlowgraph>(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());

    EXPECT_NO_THROW(flowgraph->connect());
    EXPECT_TRUE(flowgraph->connected());

    EXPECT_NO_THROW(flowgraph->start());
    EXPECT_TRUE(flowgraph->running());
    flowgraph->stop();
    EXPECT_FALSE(flowgraph->running());
}

TEST(GNSSFlowgraph /*unused*/, InstantiateConnectStartStopGalileoE1B /*unused*/)
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.repeat", "true");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat";
    config->set_property("SignalSource.filename", std::move(filename));
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("Channels_1B.count", "8");
    config->set_property("Channels.in_acquisition", "1");
    config->set_property("Channel.signal", "1B");
    config->set_property("Acquisition_1B.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B.threshold", "1");
    config->set_property("Acquisition_1B.doppler_max", "5000");
    config->set_property("Tracking_1B.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    config->set_property("TelemetryDecoder_1B.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("Observables.implementation", "Hybrid_Observables");
    config->set_property("PVT.implementation", "RTKLIB_PVT");

    std::shared_ptr<GNSSFlowgraph> flowgraph = std::make_shared<GNSSFlowgraph>(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());

    EXPECT_NO_THROW(flowgraph->connect());
    EXPECT_TRUE(flowgraph->connected());

    EXPECT_NO_THROW(flowgraph->start());
    EXPECT_TRUE(flowgraph->running());
    flowgraph->stop();
    EXPECT_FALSE(flowgraph->running());
}


TEST(GNSSFlowgraph /*unused*/, InstantiateConnectStartStopHybrid /*unused*/)
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.repeat", "true");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat";
    config->set_property("SignalSource.filename", std::move(filename));
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("Channels_1C.count", "8");
    config->set_property("Channels_1B.count", "8");
    config->set_property("Channels.in_acquisition", "1");

    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.threshold", "1");
    config->set_property("Acquisition_1C.doppler_max", "5000");


    config->set_property("Acquisition_1B.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B.threshold", "1");
    config->set_property("Acquisition_1B.doppler_max", "5000");

    config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1B.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");

    config->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B.implementation", "Galileo_E1B_Telemetry_Decoder");

    config->set_property("Observables.implementation", "Hybrid_Observables");
    config->set_property("PVT.implementation", "RTKLIB_PVT");

    std::shared_ptr<GNSSFlowgraph> flowgraph = std::make_shared<GNSSFlowgraph>(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());

    EXPECT_NO_THROW(flowgraph->connect());
    EXPECT_TRUE(flowgraph->connected());

    EXPECT_NO_THROW(flowgraph->start());
    EXPECT_TRUE(flowgraph->running());
    flowgraph->stop();
    EXPECT_FALSE(flowgraph->running());
}

class ConditionedFlowgraphTest : public ::testing::Test
{
protected:
    std::shared_ptr<InMemoryConfiguration> configuration()
    {
        auto config = std::make_shared<InMemoryConfiguration>();
        config->supersede_property("GNSS-SDR.internal_fs_sps", "4000000");
        config->supersede_property("GNSS-SDR.SUPL_gps_enabled", "false");
        config->supersede_property("SignalSource.implementation", "File_Signal_Source");
        config->supersede_property("SignalSource.sampling_frequency", "4000000");
        config->supersede_property("SignalSource.item_type", "gr_complex");
        config->supersede_property("SignalSource.repeat", "true");
        config->supersede_property("SignalSource.filename", std::string(TEST_PATH) + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat");
        config->supersede_property("SignalConditioner.implementation", "Bypass");
        config->supersede_property("Channels_1C.count", "1");
        config->supersede_property("Channels.in_acquisition", "1");
        config->supersede_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
        config->supersede_property("Acquisition_1C.threshold", "1");
        config->supersede_property("Acquisition_1C.doppler_max", "5000");
        config->supersede_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
        config->supersede_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
        config->supersede_property("Observables.implementation", "Hybrid_Observables");
        config->supersede_property("PVT.implementation", "RTKLIB_PVT");
        config->supersede_property("PVT.output_enabled", "false");
        return config;
    }
    void exercise(const std::shared_ptr<InMemoryConfiguration>& config)
    {
        GNSSFlowgraph graph(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());
        graph.connect();
        ASSERT_TRUE(graph.connected());
        graph.start();
        ASSERT_TRUE(graph.running());
        graph.stop();
        EXPECT_FALSE(graph.running());
        graph.disconnect();
        EXPECT_FALSE(graph.connected());
    }
};

TEST_F(ConditionedFlowgraphTest, BypassAndBatchedConditioning)
{
    for (const auto& implementation : {"Bypass", "Pass_Through", "Signal_Conditioner"})
        {
            for (const auto& duration : {"0", "10"})
                {
                    auto config = configuration();
                    config->supersede_property("SignalConditioner.implementation", implementation);
                    config->supersede_property("SignalConditioner.batch_size_ms", duration);
                    exercise(config);
                }
        }
}

TEST_F(ConditionedFlowgraphTest, AcquisitionResamplerUsesResolvedEndpoint)
{
    auto config = configuration();
    config->supersede_property("GNSS-SDR.use_acquisition_resampler", "true");
    config->supersede_property("SignalConditioner.batch_size_ms", "10");
    exercise(config);
}

TEST_F(ConditionedFlowgraphTest, UnusedBypassedSource)
{
    auto config = configuration();
    config->supersede_property("GNSS-SDR.num_sources", "2");
    for (int i = 0; i < 2; ++i)
        {
            const std::string role = "SignalSource" + std::to_string(i);
            config->supersede_property(role + ".implementation", "File_Signal_Source");
            config->supersede_property(role + ".filename", std::string(TEST_PATH) + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat");
            config->supersede_property(role + ".item_type", "gr_complex");
            config->supersede_property(role + ".sampling_frequency", "4000000");
            config->supersede_property(role + ".repeat", "true");
            config->supersede_property("SignalConditioner" + std::to_string(i) + ".implementation", "Bypass");
        }
    config->supersede_property("SignalConditioner0.batch_size_ms", "10");
    config->supersede_property("Channel0.RF_channel_ID", "0");
    exercise(config);
}

TEST_F(ConditionedFlowgraphTest, RejectsInvalidBatchDuration)
{
    for (const auto& duration : {"-1", "nan", "inf", "1e100"})
        {
            auto config = configuration();
            config->supersede_property("SignalConditioner.batch_size_ms", duration);
            GNSSFlowgraph graph(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());
            graph.connect();
            EXPECT_FALSE(graph.connected()) << duration;
        }
}

TEST_F(ConditionedFlowgraphTest, MultiplePortsAndNonzeroRfSelection)
{
    auto config = configuration();
    config->supersede_property("SignalSource.implementation", "Multichannel_File_Signal_Source");
    config->supersede_property("SignalSource.total_channels", "2");
    config->supersede_property("SignalSource.RF_channels", "2");
    for (int i = 0; i < 2; ++i)
        {
            config->supersede_property("SignalSource.filename" + std::to_string(i), std::string(TEST_PATH) + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat");
            config->supersede_property("SignalConditioner" + std::to_string(i) + ".implementation", "Bypass");
            config->supersede_property("SignalConditioner" + std::to_string(i) + ".batch_size_ms", "10");
        }
    config->supersede_property("Channel0.RF_channel_ID", "1");
    exercise(config);
}

TEST_F(ConditionedFlowgraphTest, RejectsMismatchedFormatAfterIdentityRemoval)
{
    auto config = configuration();
    config->supersede_property("SignalConditioner.implementation", "Pass_Through");
    config->supersede_property("SignalConditioner.item_type", "short");
    config->supersede_property("SignalConditioner.batch_size_ms", "10");
    GNSSFlowgraph graph(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());
    graph.connect();
    EXPECT_FALSE(graph.connected());
}

TEST_F(ConditionedFlowgraphTest, L2cWithSmallBatches)
{
    auto config = configuration();
    config->supersede_property("Channels_1C.count", "0");
    config->supersede_property("Channels_2S.count", "1");
    config->supersede_property("SignalConditioner.batch_size_ms", "1");
    config->supersede_property("Acquisition_2S.implementation", "GPS_L2_M_PCPS_Acquisition");
    config->supersede_property("Acquisition_2S.threshold", "1");
    config->supersede_property("Tracking_2S.implementation", "GPS_L2_M_DLL_PLL_Tracking");
    config->supersede_property("TelemetryDecoder_2S.implementation", "GPS_L2C_Telemetry_Decoder");
    exercise(config);
}

TEST_F(ConditionedFlowgraphTest, SharedSourceOutputFeedsMultipleConditioners)
{
    auto config = configuration();
    config->supersede_property("SignalSource.RF_channels", "2");
    config->supersede_property("SignalConditioner0.implementation", "Bypass");
    config->supersede_property("SignalConditioner1.implementation", "Bypass");
    config->supersede_property("SignalConditioner0.batch_size_ms", "10");
    config->supersede_property("SignalConditioner1.batch_size_ms", "10");
    config->supersede_property("Channel0.RF_channel_ID", "1");
    exercise(config);
}


TEST_F(ConditionedFlowgraphTest, DefaultPlanningNeedsNoConfigurationEdits)
{
    auto config = configuration();
    config->supersede_property("GNSS-SDR.internal_fs_sps", "2000000");
    config->supersede_property("SignalSource.sampling_frequency", "2000000");
    config->supersede_property("SignalConditioner.implementation", "Pass_Through");
    GNSSFlowgraph graph(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());
    graph.connect();
    ASSERT_TRUE(graph.connected());
    ASSERT_EQ(1U, graph.conditioner_scheduling_plans().size());
    const auto& plan = graph.conditioner_scheduling_plans()[0];
    EXPECT_TRUE(plan.automatic);
    EXPECT_EQ(3U, plan.readers);  // Acquisition, tracking, and sample counter.
    EXPECT_EQ(20000U, plan.batch_items);
    EXPECT_TRUE(plan.remove_identity);
    graph.disconnect();
}

TEST_F(ConditionedFlowgraphTest, PlanningCountsSharedAcquisitionResamplerOnce)
{
    auto config = configuration();
    config->supersede_property("SignalConditioner.implementation", "Pass_Through");
    config->supersede_property("GNSS-SDR.internal_fs_sps", "8000000");
    config->supersede_property("GNSS-SDR.use_acquisition_resampler", "true");
    config->supersede_property("Channels_1C.count", "2");
    GNSSFlowgraph graph(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());
    graph.connect();
    ASSERT_TRUE(graph.connected());
    ASSERT_EQ(1U, graph.conditioner_scheduling_plans().size());
    EXPECT_EQ(4U, graph.conditioner_scheduling_plans()[0].readers);
    graph.disconnect();
}


TEST_F(ConditionedFlowgraphTest, PacedSourcesUseTheSamePortableDefault)
{
    auto config = configuration();
    config->supersede_property("SignalConditioner.implementation", "Pass_Through");
    config->supersede_property("SignalSource.enable_throttle_control", "true");
    GNSSFlowgraph graph(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());
    graph.connect();
    ASSERT_TRUE(graph.connected());
    EXPECT_EQ(40000U, graph.conditioner_scheduling_plans().at(0).batch_items);
    graph.start();
    ASSERT_TRUE(graph.running());
    graph.stop();
    graph.disconnect();
}


TEST_F(ConditionedFlowgraphTest, ReconnectRebuildsConditionerAndSharedResamplerEdges)
{
    auto config = configuration();
    config->supersede_property("SignalConditioner.implementation", "Signal_Conditioner");
    config->supersede_property("GNSS-SDR.internal_fs_sps", "8000000");
    config->supersede_property("GNSS-SDR.use_acquisition_resampler", "true");
    config->supersede_property("SignalConditioner.batch_size_ms", "2");
    GNSSFlowgraph graph(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());
    for (int run = 0; run < 2; ++run)
        {
            graph.connect();
            ASSERT_TRUE(graph.connected());
            graph.start();
            ASSERT_TRUE(graph.running());
            if (run == 0)
                {
                    graph.stop();
                }
            graph.disconnect();
            ASSERT_FALSE(graph.running());
            ASSERT_FALSE(graph.connected());
        }
}


TEST_F(ConditionedFlowgraphTest, FailedPlanCanBeCorrectedAndRetried)
{
    auto config = configuration();
    config->supersede_property("SignalConditioner.max_batch_latency_ms", "nan");
    GNSSFlowgraph graph(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());
    graph.connect();
    ASSERT_FALSE(graph.connected());
    config->supersede_property("SignalConditioner.max_batch_latency_ms", "20");
    graph.connect();
    ASSERT_TRUE(graph.connected());
    graph.start();
    ASSERT_TRUE(graph.running());
    graph.disconnect();
    EXPECT_FALSE(graph.running());
}


TEST_F(ConditionedFlowgraphTest, AutomaticPlanningSupportsMultipleRfStreams)
{
    auto config = configuration();
    config->supersede_property("SignalSource.implementation", "Multichannel_File_Signal_Source");
    config->supersede_property("SignalSource.total_channels", "2");
    config->supersede_property("SignalSource.RF_channels", "2");
    config->supersede_property("Channels_1C.count", "2");
    for (int i = 0; i < 2; ++i)
        {
            const auto index = std::to_string(i);
            config->supersede_property("SignalSource.filename" + index, std::string(TEST_PATH) + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat");
            config->supersede_property("SignalConditioner" + index + ".implementation", "Pass_Through");
            config->supersede_property("Channel" + index + ".RF_channel_ID", index);
        }
    GNSSFlowgraph graph(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());
    graph.connect();
    ASSERT_TRUE(graph.connected());
    ASSERT_EQ(2U, graph.conditioner_scheduling_plans().size());
    for (const auto& plan : graph.conditioner_scheduling_plans())
        {
            EXPECT_TRUE(plan.automatic);
            EXPECT_TRUE(plan.remove_identity);
            EXPECT_EQ(40000U, plan.batch_items);
        }
    EXPECT_EQ(3U, graph.conditioner_scheduling_plans()[0].readers);
    EXPECT_EQ(2U, graph.conditioner_scheduling_plans()[1].readers);
    graph.start();
    ASSERT_TRUE(graph.running());
    graph.disconnect();
    EXPECT_FALSE(graph.running());
}


TEST_F(ConditionedFlowgraphTest, IdentityRemovalIsPlannedAfterConstructionAndOnReconnect)
{
    for (const auto* implementation : {"Pass_Through", "Signal_Conditioner"})
        {
            auto config = configuration();
            config->supersede_property("SignalConditioner.implementation", implementation);
            config->supersede_property("SignalConditioner.batch_size_ms", "10");
            config->supersede_property("SignalConditioner.remove_pass_through", "false");
            GNSSFlowgraph graph(config, std::make_shared<Concurrent_Queue<pmt::pmt_t>>());
            for (bool remove : {true, false, true})
                {
                    config->supersede_property("SignalConditioner.remove_pass_through", remove ? "true" : "false");
                    graph.connect();
                    ASSERT_TRUE(graph.connected());
                    ASSERT_EQ(1U, graph.conditioner_scheduling_plans().size());
                    const auto& plan = graph.conditioner_scheduling_plans()[0];
                    EXPECT_EQ(remove, plan.remove_identity);
                    EXPECT_EQ(40000U, plan.batch_items);
                    graph.start();
                    ASSERT_TRUE(graph.running());
                    graph.disconnect();
                    EXPECT_FALSE(graph.connected());
                    EXPECT_FALSE(graph.running());
                }
        }
}
