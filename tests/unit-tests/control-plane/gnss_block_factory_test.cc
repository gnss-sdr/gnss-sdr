/*!
 * \file gnss_block_factory_test.cc
 * \brief This class implements a Unit Test for the GNSSBlockFactory class.
 * \authors <ul>
 *          <li> Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
 *
 * This class test the instantiation of all blocks in gnss_block_factory
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
#include "concurrent_queue.h"
#include "conditioner_scheduling_policy.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_make_unique.h"
#include "in_memory_configuration.h"
#include "signal_source_interface.h"
#include "tracking_interface.h"
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <limits>
#include <utility>
#include <vector>

TEST(GNSSBlockFactoryTest, InstantiateFileSignalSource)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalSource.implementation", "File_Signal_Source");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
    configuration->set_property("SignalSource.filename", std::move(filename));
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    // Example of a block as a shared_ptr
    auto signal_source = block_factory::GetSignalSource(configuration.get(), queue.get());
    EXPECT_STREQ("SignalSource", signal_source->role().c_str());
    EXPECT_STREQ("File_Signal_Source", signal_source->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongSignalSource)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalSource.implementation", "Parapsychological_Source");
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    // Example of a block as a unique_ptr
    auto signal_source = block_factory::GetSignalSource(configuration.get(), queue.get());
    EXPECT_EQ(nullptr, signal_source);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongSignalSource2)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalSource.implementation", "Pass_Through");
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    // Example of a block as a unique_ptr
    auto signal_source = block_factory::GetSignalSource(configuration.get(), queue.get());
    EXPECT_EQ(nullptr, signal_source);
}


TEST(GNSSBlockFactoryTest, InstantiateSignalConditioner)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalConditioner.implementation", "Signal_Conditioner");
    auto signal_conditioner = block_factory::GetSignalConditioner(configuration.get());
    EXPECT_STREQ("SignalConditioner", signal_conditioner->role().c_str());
    EXPECT_STREQ("Signal_Conditioner", signal_conditioner->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongSignalConditioner)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalConditioner.implementation", "Signal_Ruinder");
    auto signal_conditioner = block_factory::GetSignalConditioner(configuration.get());
    EXPECT_EQ(nullptr, signal_conditioner);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongSignalConditioner2)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalConditioner.implementation", "Fir_Filter");
    auto signal_conditioner = block_factory::GetSignalConditioner(configuration.get());
    EXPECT_EQ(nullptr, signal_conditioner);
}


TEST(GNSSBlockFactoryTest, InstantiateFIRFilter)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();

    configuration->set_property("InputFilter.implementation", "Fir_Filter");

    configuration->set_property("InputFilter.number_of_taps", "4");
    configuration->set_property("InputFilter.number_of_bands", "2");

    configuration->set_property("InputFilter.band1_begin", "0.0");
    configuration->set_property("InputFilter.band1_end", "0.45");
    configuration->set_property("InputFilter.band2_begin", "0.55");
    configuration->set_property("InputFilter.band2_end", "1.0");

    configuration->set_property("InputFilter.ampl1_begin", "1.0");
    configuration->set_property("InputFilter.ampl1_end", "1.0");
    configuration->set_property("InputFilter.ampl2_begin", "0.0");
    configuration->set_property("InputFilter.ampl2_end", "0.0");

    configuration->set_property("InputFilter.band1_error", "1.0");
    configuration->set_property("InputFilter.band2_error", "1.0");

    configuration->set_property("InputFilter.filter_type", "bandpass");
    configuration->set_property("InputFilter.grid_density", "16");

    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);

    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Fir_Filter", input_filter->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateFreqXlatingFIRFilter)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();

    configuration->set_property("InputFilter.implementation", "Freq_Xlating_Fir_Filter");

    configuration->set_property("InputFilter.number_of_taps", "4");
    configuration->set_property("InputFilter.number_of_bands", "2");

    configuration->set_property("InputFilter.band1_begin", "0.0");
    configuration->set_property("InputFilter.band1_end", "0.45");
    configuration->set_property("InputFilter.band2_begin", "0.55");
    configuration->set_property("InputFilter.band2_end", "1.0");

    configuration->set_property("InputFilter.ampl1_begin", "1.0");
    configuration->set_property("InputFilter.ampl1_end", "1.0");
    configuration->set_property("InputFilter.ampl2_begin", "0.0");
    configuration->set_property("InputFilter.ampl2_end", "0.0");

    configuration->set_property("InputFilter.band1_error", "1.0");
    configuration->set_property("InputFilter.band2_error", "1.0");

    configuration->set_property("InputFilter.filter_type", "bandpass");
    configuration->set_property("InputFilter.grid_density", "16");

    configuration->set_property("InputFilter.sampling_frequency", "4000000");
    configuration->set_property("InputFilter.IF", "34000");

    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);

    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Freq_Xlating_Fir_Filter", input_filter->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiatePulseBlankingFilter)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    configuration->set_property("InputFilter.implementation", "Pulse_Blanking_Filter");
    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);
    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Pulse_Blanking_Filter", input_filter->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateNotchFilter)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    configuration->set_property("InputFilter.implementation", "Notch_Filter");
    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);
    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Notch_Filter", input_filter->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateNotchFilterLite)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    configuration->set_property("InputFilter.implementation", "Notch_Filter_Lite");
    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);
    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Notch_Filter_Lite", input_filter->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongFilter)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    configuration->set_property("InputFilter.implementation", "Pollen_Filter");
    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);
    EXPECT_EQ(nullptr, input_filter);
}


TEST(GNSSBlockFactoryTest, InstantiateDirectResampler)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Resampler.implementation", "Direct_Resampler");
    auto resampler = block_factory::GetBlock(configuration.get(), "Resampler", 1, 1);
    EXPECT_STREQ("Resampler", resampler->role().c_str());
    EXPECT_STREQ("Direct_Resampler", resampler->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongResampler)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Resampler.implementation", "RaNdOm_Resampler");
    auto resampler = block_factory::GetBlock(configuration.get(), "Resampler", 1, 1);
    EXPECT_EQ(nullptr, resampler);
}


TEST(GNSSBlockFactoryTest, InstantiateGpsL1CaPcpsAcquisition)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Acquisition");
    auto acquisition = block_factory::GetBlock(configuration.get(), "Acquisition", 1, 0);
    EXPECT_STREQ("Acquisition", acquisition->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_PCPS_Acquisition", acquisition->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateGpsL1CaPcpsQuickSyncAcquisition)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_QuickSync_Acquisition");
    auto acquisition = block_factory::GetBlock(configuration.get(), "Acquisition", 1, 0);
    EXPECT_STREQ("Acquisition", acquisition->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_PCPS_QuickSync_Acquisition", acquisition->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateGalileoE1PcpsQuickSyncAmbiguousAcquisition)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Acquisition.implementation", "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition");
    auto acquisition = block_factory::GetBlock(configuration.get(), "Acquisition", 1, 0);
    EXPECT_STREQ("Acquisition", acquisition->role().c_str());
    EXPECT_STREQ("Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition", acquisition->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateGalileoE1PcpsAmbiguousAcquisition)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Acquisition.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    auto acquisition = block_factory::GetBlock(configuration.get(), "Acquisition", 1, 0);
    EXPECT_STREQ("Acquisition", acquisition->role().c_str());
    EXPECT_STREQ("Galileo_E1_PCPS_Ambiguous_Acquisition", acquisition->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongAcquisition)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Alchemy");
    auto acq_ = block_factory::GetBlock(configuration.get(), "Acquisition", 1, 0);
    EXPECT_EQ(nullptr, acq_);
}


TEST(GNSSBlockFactoryTest, InstantiateDllPllTrackingAdapterGpsL1Ca)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Tracking.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    auto tracking = block_factory::GetBlock(configuration.get(), "Tracking", 1, 1);
    EXPECT_STREQ("Tracking", tracking->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_DLL_PLL_Tracking", tracking->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateGpsL1CaTcpConnectorTracking)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Tracking.implementation", "GPS_L1_CA_TCP_CONNECTOR_Tracking");
    auto tracking = block_factory::GetBlock(configuration.get(), "Tracking", 1, 1);
    EXPECT_STREQ("Tracking", tracking->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_TCP_CONNECTOR_Tracking", tracking->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateDllPllTrackingAdapterGalileoE1)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Tracking.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    auto tracking = block_factory::GetBlock(configuration.get(), "Tracking", 1, 1);
    EXPECT_STREQ("Tracking", tracking->role().c_str());
    EXPECT_STREQ("Galileo_E1_DLL_PLL_VEML_Tracking", tracking->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongTracking)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Tracking.implementation", "The perfect tracking");
    auto trk_ = block_factory::GetBlock(configuration.get(), "Tracking", 1, 1);
    EXPECT_EQ(nullptr, trk_);
}


TEST(GNSSBlockFactoryTest, InstantiateTelemetryDecoderAdapterGpsL1Ca)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("TelemetryDecoder.implementation", "GPS_L1_CA_Telemetry_Decoder");
    auto telemetry_decoder = block_factory::GetBlock(configuration.get(), "TelemetryDecoder", 1, 1);
    EXPECT_STREQ("TelemetryDecoder", telemetry_decoder->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_Telemetry_Decoder", telemetry_decoder->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongTelemetryDecoder)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("TelemetryDecoder.implementation", "GPS_Xenomorphic_Telemetry_Decoder");
    auto telemetry_decoder = block_factory::GetBlock(configuration.get(), "TelemetryDecoder", 1, 1);
    EXPECT_EQ(nullptr, telemetry_decoder);
}


TEST(GNSSBlockFactoryTest, InstantiateEmptyTelemetryDecoder)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("TelemetryDecoder.implementation", std::string(""));
    auto telemetry_decoder = block_factory::GetBlock(configuration.get(), "TelemetryDecoder", 1, 1);
    EXPECT_EQ(nullptr, telemetry_decoder);
}


TEST(GNSSBlockFactoryTest, InstantiateChannels)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Channels_1C.count", "2");
    configuration->set_property("Channels_1E.count", "0");
    configuration->set_property("Channels.in_acquisition", "2");
    configuration->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    configuration->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    configuration->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    auto channels = block_factory::GetChannels(configuration.get(), queue.get());
    EXPECT_EQ(static_cast<unsigned int>(2), channels.size());
    channels.erase(channels.begin(), channels.end());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongObservables)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Observables.implementation", "Supercalifragilistic_Observables");
    auto observables = block_factory::GetObservables(configuration.get());
    EXPECT_EQ(nullptr, observables);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongObservables2)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Observables.implementation", "Pass_Through");
    auto observables = block_factory::GetObservables(configuration.get());
    EXPECT_EQ(nullptr, observables);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongObservables3)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Observables.implementation", "RTKLIB_PVT");
    auto observables = block_factory::GetObservables(configuration.get());
    EXPECT_EQ(nullptr, observables);
}


TEST(GNSSBlockFactoryTest, InstantiateObservables)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Observables.implementation", "Hybrid_Observables");
    auto observables = block_factory::GetObservables(configuration.get());
    EXPECT_STREQ("Observables", observables->role().c_str());
    EXPECT_STREQ("Hybrid_Observables", observables->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateRTKLIBPvt)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("PVT.implementation", "RTKLIB_PVT");
    auto pvt = block_factory::GetPVT(configuration.get());
    EXPECT_STREQ("PVT", pvt->role().c_str());
    EXPECT_STREQ("RTKLIB_PVT", pvt->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongPvt)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("PVT.implementation", "Pepito");
    auto pvt = block_factory::GetPVT(configuration.get());
    EXPECT_EQ(nullptr, pvt);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongPvt2)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("PVT.implementation", "Pass_Through");
    auto pvt = block_factory::GetPVT(configuration.get());
    EXPECT_EQ(nullptr, pvt);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongPvt3)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("PVT.implementation", "Quantum_Particle_PVT");
    auto pvt = block_factory::GetPVT(configuration.get());
    EXPECT_EQ(nullptr, pvt);
}


TEST(GNSSBlockFactoryTest, BypassHasNoProcessingBlocks)
{
    InMemoryConfiguration config;
    config.supersede_property("SignalConditioner.implementation", "Bypass");
    auto conditioner = block_factory::GetSignalConditioner(&config);
    ASSERT_TRUE(conditioner != nullptr);
    EXPECT_TRUE(conditioner->is_identity());
    EXPECT_FALSE(conditioner->get_left_block());
    EXPECT_FALSE(conditioner->get_right_block());
    EXPECT_EQ(0U, conditioner->item_size());
}


TEST(GNSSBlockFactoryTest, BypassRejectsConflictingProcessing)
{
    InMemoryConfiguration config;
    config.supersede_property("SignalConditioner.implementation", "Bypass");
    config.supersede_property("InputFilter.implementation", "Freq_Xlating_Fir_Filter");
    EXPECT_THROW(block_factory::GetSignalConditioner(&config), std::invalid_argument);
    config.supersede_property("InputFilter.implementation", "Pass_Through");
    config.supersede_property("Resampler.inverted_spectrum", "true");
    EXPECT_THROW(block_factory::GetSignalConditioner(&config), std::invalid_argument);
}


TEST(GNSSBlockFactoryTest, FactoryRetainsPassThroughUntilPlanning)
{
    InMemoryConfiguration config;
    config.supersede_property("SignalConditioner.implementation", "Pass_Through");
    config.supersede_property("SignalConditioner.batch_size_ms", "10");
    auto conditioner = block_factory::GetSignalConditioner(&config);
    EXPECT_TRUE(conditioner->get_right_block().get() != nullptr);
    EXPECT_TRUE(conditioner->is_identity());
    EXPECT_EQ(sizeof(gr_complex), conditioner->item_size());
    config.supersede_property("SignalConditioner.inverted_spectrum", "true");
    conditioner = block_factory::GetSignalConditioner(&config);
    EXPECT_TRUE(conditioner->get_right_block().get() != nullptr);
    EXPECT_FALSE(conditioner->is_identity());
}


TEST(GNSSBlockFactoryTest, FactoryRetainsStagesRegardlessOfSchedulingOverrides)
{
    for (const auto* duration : {"auto", "0", "10"})
        {
            for (const auto* remove : {"false", "true"})
                {
                    InMemoryConfiguration config;
                    config.supersede_property("SignalConditioner.implementation", "Signal_Conditioner");
                    config.supersede_property("SignalConditioner.batch_size_ms", duration);
                    config.supersede_property("SignalConditioner.remove_pass_through", remove);
                    auto conditioner = block_factory::GetSignalConditioner(&config);
                    ASSERT_TRUE(conditioner != nullptr);
                    EXPECT_TRUE(conditioner->get_left_block().get() != nullptr);
                    EXPECT_TRUE(conditioner->get_right_block().get() != nullptr);
                    EXPECT_NE(conditioner->get_left_block(), conditioner->get_right_block());
                    config.supersede_property("SignalConditioner.implementation", "Pass_Through");
                    conditioner = block_factory::GetSignalConditioner(&config);
                    ASSERT_TRUE(conditioner != nullptr);
                    EXPECT_TRUE(conditioner->get_right_block().get() != nullptr);
                    EXPECT_TRUE(conditioner->is_identity());
                }
        }
}


TEST(ConditionerSchedulingPolicyTest, AutomaticCopyReplacementAndSafeFallbacks)
{
    InMemoryConfiguration config;
    ConditionerSchedulingPolicy::Request request;
    request.sample_rate = 2500000;
    request.item_size = 8;
    request.readers = 25;
    request.identity_stages = 1;
    auto plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_TRUE(plan.automatic);
    EXPECT_TRUE(plan.remove_identity);
    EXPECT_EQ(25000U, plan.batch_items);
    EXPECT_EQ(10.0, plan.max_latency_ms);
    request.identity_stages = 0;
    EXPECT_EQ(0U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).batch_items);
    request.identity_stages = 1;
    request.eligible = false;
    EXPECT_EQ(0U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).batch_items);
    request.eligible = true;
    for (size_t readers : {0U, 1U})
        {
            request.readers = readers;
            EXPECT_EQ(0U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).batch_items);
        }
    for (size_t readers : {2U, 25U, 41U, 1000U})
        {
            request.readers = readers;
            EXPECT_EQ(25000U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).batch_items);
        }
}


TEST(ConditionerSchedulingPolicyTest, LimitsAndExplicitOverrides)
{
    InMemoryConfiguration config;
    ConditionerSchedulingPolicy::Request request;
    request.sample_rate = 2500000;
    request.item_size = 8;
    request.readers = 25;
    request.identity_stages = 3;
    config.supersede_property("GNSS-SDR.observable_interval_ms", "2");
    auto plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(5000U, plan.batch_items);
    EXPECT_EQ(2.0, plan.max_latency_ms);
    config.supersede_property("GNSS-SDR.max_source_buffer_samples", "4096");
    EXPECT_EQ(2048U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).batch_items);
    config.supersede_property("SignalConditioner.batch_size_ms", "1");
    plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_FALSE(plan.automatic);
    EXPECT_EQ(2048U, plan.batch_items);
    EXPECT_TRUE(plan.remove_identity);
    config.supersede_property("SignalConditioner.batch_size_ms", "0");
    plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(0U, plan.batch_items);
    EXPECT_FALSE(plan.remove_identity);
    config.supersede_property("SignalConditioner.batch_size_ms", "auto");
    config.supersede_property("SignalConditioner.remove_pass_through", "false");
    plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(0U, plan.batch_items);
    EXPECT_FALSE(plan.remove_identity);
}


TEST(ConditionerSchedulingPolicyTest, RejectsInvalidAndConflictingOptions)
{
    InMemoryConfiguration config;
    ConditionerSchedulingPolicy::Request request;
    request.sample_rate = 2500000;
    request.item_size = 8;
    for (const auto* value : {"nan", "inf", "-1", "10junk", "", "1e100"})
        {
            config.supersede_property("SignalConditioner.batch_size_ms", value);
            EXPECT_THROW(ConditionerSchedulingPolicy::select(config, "SignalConditioner", request), std::invalid_argument);
        }
    config.supersede_property("SignalConditioner.batch_size_ms", "20");
    EXPECT_EQ(20.0, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).max_latency_ms);
    config.supersede_property("SignalConditioner.max_batch_latency_ms", "10");
    EXPECT_THROW(ConditionerSchedulingPolicy::select(config, "SignalConditioner", request), std::invalid_argument);
}


TEST(ConditionerSchedulingPolicyTest, ConsumerWindowsConstrainAutomaticSelection)
{
    InMemoryConfiguration config;
    // Without extra depth, the buffer holds a batch on top of the largest
    // consumer window, in whole batches.
    config.supersede_property("SignalConditioner.output_buffer_ms", "0");
    ConditionerSchedulingPolicy::Request request;
    request.sample_rate = 2500000;
    request.item_size = 8;
    request.readers = 25;
    request.identity_stages = 1;
    request.minimum_input_items = 50000;
    auto plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(25000U, plan.batch_items);
    EXPECT_EQ(75000U, plan.min_buffer_items);
    EXPECT_EQ(50000U, plan.reader_window_items);
    request.minimum_input_items = 140000;
    plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(25000U, plan.batch_items);
    EXPECT_EQ(175000U, plan.min_buffer_items);
    config.supersede_property("GNSS-SDR.max_source_buffer_samples", "131072");
    EXPECT_EQ(0U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).batch_items);
    config.supersede_property("GNSS-SDR.max_source_buffer_samples", "0");
    config.supersede_property("SignalConditioner.batch_size_ms", "1");
    plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(2500U, plan.batch_items);
    EXPECT_EQ(142500U, plan.min_buffer_items);
    request.windows_known = false;
    EXPECT_THROW(ConditionerSchedulingPolicy::select(config, "SignalConditioner", request), std::invalid_argument);
    request.windows_known = true;
    config.supersede_property("GNSS-SDR.max_source_buffer_samples", "8192");
    EXPECT_THROW(ConditionerSchedulingPolicy::select(config, "SignalConditioner", request), std::invalid_argument);
}


TEST(ConditionerSchedulingPolicyTest, BatchPlusConsumerWindowMustFitTheBufferLimit)
{
    InMemoryConfiguration config;
    config.supersede_property("SignalConditioner.batch_size_ms", "1");
    config.supersede_property("GNSS-SDR.max_source_buffer_samples", "5120");
    ConditionerSchedulingPolicy::Request request;
    request.sample_rate = 1024000;
    request.item_size = 8;
    request.minimum_input_items = 3500;
    const auto plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(1024U, plan.batch_items);
    EXPECT_EQ(5120U, plan.min_buffer_items);
    for (const auto* limit : {"4096", "bad", "nan", "1.5", "-1", "1"})
        {
            config.supersede_property("GNSS-SDR.max_source_buffer_samples", limit);
            EXPECT_THROW(ConditionerSchedulingPolicy::select(config, "SignalConditioner", request), std::invalid_argument);
        }
}


TEST(ConditionerSchedulingPolicyTest, OutputBufferFollowsObservablesDelay)
{
    InMemoryConfiguration config;
    ConditionerSchedulingPolicy::Request request;
    request.sample_rate = 4000000;
    request.item_size = 8;
    request.readers = 25;
    request.identity_stages = 1;
    request.channel_input_items = 80000;  // two 10 ms code periods
    request.minimum_input_items = 80000;
    // 280 ms of observables delay, less the channel window, a symbol and the guard.
    auto plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(40000U, plan.batch_items);
    EXPECT_EQ(180.0, plan.alignment_budget_ms);
    EXPECT_EQ(720000U, plan.min_buffer_items);
    // A stream that does not feed the receiver clock can also trail it by what
    // is queued in front of it.
    request.feeds_clock = false;
    plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(153.0, plan.alignment_budget_ms);
    EXPECT_EQ(600000U, plan.min_buffer_items);
    request.feeds_clock = true;
    // An unthrottled replay keeps the receiver waiting for its slowest reader.
    request.real_time = false;
    plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(120000U, plan.min_buffer_items);
    config.supersede_property("SignalConditioner.output_buffer_ms", "100");
    EXPECT_EQ(400000U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).min_buffer_items);
    request.real_time = true;
    // The depth also applies when no batching block is inserted.
    request.identity_stages = 0;
    plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(0U, plan.batch_items);
    EXPECT_EQ(400000U, plan.min_buffer_items);
    config.supersede_property("SignalConditioner.output_buffer_ms", "auto");
    EXPECT_EQ(720000U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).min_buffer_items);
    config.supersede_property("GNSS-SDR.max_source_buffer_samples", "100000");
    EXPECT_EQ(100000U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).min_buffer_items);
    config.supersede_property("GNSS-SDR.max_source_buffer_samples", "0");
    request.identity_stages = 1;
    // Shorter observable intervals leave less room, down to none.
    config.supersede_property("GNSS-SDR.observable_interval_ms", "10");
    plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(90.0, plan.alignment_budget_ms);
    EXPECT_EQ(360000U, plan.min_buffer_items);
    config.supersede_property("GNSS-SDR.observable_interval_ms", "2");
    plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
    EXPECT_EQ(8000U, plan.batch_items);
    EXPECT_EQ(0.0, plan.alignment_budget_ms);
    EXPECT_EQ(88000U, plan.min_buffer_items);
    config.supersede_property("GNSS-SDR.observable_interval_ms", "20");
    for (const auto* depth : {"280.5", "1000", "-1", "nan", "deep", ""})
        {
            config.supersede_property("SignalConditioner.output_buffer_ms", depth);
            EXPECT_THROW(ConditionerSchedulingPolicy::select(config, "SignalConditioner", request), std::invalid_argument);
        }
}


TEST(ConditionerSchedulingPolicyTest, MemoryBudgetUsesBytesAcrossSampleFormats)
{
    InMemoryConfiguration config;
    // Reach the byte budgets for every item size, including single bytes.
    config.supersede_property("SignalConditioner.max_batch_latency_ms", "20");
    ConditionerSchedulingPolicy::Request request;
    request.sample_rate = 100000000;
    request.readers = 64;
    request.identity_stages = 1;
    request.minimum_input_items = 2000000;
    for (size_t item_size : {1U, 2U, 4U, 8U, 16U})
        {
            request.item_size = item_size;
            const auto plan = ConditionerSchedulingPolicy::select(config, "SignalConditioner", request);
            EXPECT_TRUE(plan.remove_identity);
            EXPECT_EQ(512U * 1024U, plan.batch_items * item_size);
            // Whole batches, with room for one on top of the consumer window.
            EXPECT_EQ(0U, plan.min_buffer_items % plan.batch_items);
            EXPECT_GE(plan.min_buffer_items, plan.batch_items + request.minimum_input_items);
            EXPECT_LT(plan.min_buffer_items, 2 * plan.batch_items + request.minimum_input_items + 8U * 1024U * 1024U / item_size);
        }
    // The automatic depth stays within its own byte budget.
    request.minimum_input_items = 0;
    request.item_size = 8;
    EXPECT_EQ(1048576U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).min_buffer_items);
}


TEST(ConditionerSchedulingPolicyTest, TinyAndExtremeRequestsCannotOverflowAutomaticSelection)
{
    InMemoryConfiguration config;
    ConditionerSchedulingPolicy::Request request;
    request.sample_rate = 1.0;
    request.item_size = 8;
    request.readers = 2;
    request.identity_stages = 1;
    EXPECT_EQ(0U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).batch_items);
    request.sample_rate = std::numeric_limits<double>::max();
    EXPECT_EQ(65536U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).batch_items);
    request.item_size = std::numeric_limits<size_t>::max();
    EXPECT_EQ(0U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).batch_items);
    request.item_size = 8;
    request.minimum_input_items = std::numeric_limits<uint64_t>::max();
    EXPECT_EQ(0U, ConditionerSchedulingPolicy::select(config, "SignalConditioner", request).batch_items);
}


TEST(ConditionerSchedulingPolicyTest, RejectsLongDeadlinesAndFixedBatches)
{
    InMemoryConfiguration config;
    ConditionerSchedulingPolicy::Request request;
    request.sample_rate = 2500000;
    request.item_size = 8;
    request.readers = 25;
    request.identity_stages = 1;
    // An enlarged observable interval must not enlarge the batch ceiling.
    config.supersede_property("GNSS-SDR.observable_interval_ms", "1000");
    EXPECT_EQ(10.0, ConditionerSchedulingPolicy::select(config, "SignalConditioner0", request).max_latency_ms);
    config.supersede_property("SignalConditioner0.max_batch_latency_ms", "20");
    EXPECT_EQ(50000U, ConditionerSchedulingPolicy::select(config, "SignalConditioner0", request).batch_items);
    for (const auto* delay : {"20.001", "40", "511", "60000"})
        {
            config.supersede_property("SignalConditioner0.max_batch_latency_ms", delay);
            EXPECT_THROW(ConditionerSchedulingPolicy::select(config, "SignalConditioner0", request), std::invalid_argument);
        }
    config.supersede_property("SignalConditioner0.max_batch_latency_ms", "20");
    for (const auto* duration : {"20.001", "40", "511", "60000"})
        {
            config.supersede_property("SignalConditioner.batch_size_ms", duration);
            // No explicit deadline: the fixed duration must not raise its default.
            EXPECT_THROW(ConditionerSchedulingPolicy::select(config, "SignalConditioner", request), std::invalid_argument);
            config.supersede_property("SignalConditioner0.batch_size_ms", duration);
            EXPECT_THROW(ConditionerSchedulingPolicy::select(config, "SignalConditioner0", request), std::invalid_argument);
        }
}
