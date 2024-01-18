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

    config->set_property("Acquisition_1C0.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C0.threshold", "1");
    config->set_property("Acquisition_1C0.doppler_max", "5000");
    config->set_property("Acquisition_1C1.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C1.threshold", "1");
    config->set_property("Acquisition_1C1.doppler_max", "5000");
    config->set_property("Acquisition_1C2.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C2.threshold", "1");
    config->set_property("Acquisition_1C2.doppler_max", "5000");
    config->set_property("Acquisition_1C3.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C3.threshold", "1");
    config->set_property("Acquisition_1C3.doppler_max", "5000");
    config->set_property("Acquisition_1C4.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C4.threshold", "1");
    config->set_property("Acquisition_1C4.doppler_max", "5000");
    config->set_property("Acquisition_1C5.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C5.threshold", "1");
    config->set_property("Acquisition_1C5.doppler_max", "5000");
    config->set_property("Acquisition_1C6.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C6.threshold", "1");
    config->set_property("Acquisition_1C6.doppler_max", "5000");
    config->set_property("Acquisition_1C7.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C7.threshold", "1");
    config->set_property("Acquisition_1C7.doppler_max", "5000");

    config->set_property("Acquisition_1B8.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B8.threshold", "1");
    config->set_property("Acquisition_1B8.doppler_max", "5000");
    config->set_property("Acquisition_1B9.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B9.threshold", "1");
    config->set_property("Acquisition_1B9.doppler_max", "5000");
    config->set_property("Acquisition_1B10.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B10.threshold", "1");
    config->set_property("Acquisition_1B10.doppler_max", "5000");
    config->set_property("Acquisition_1B11.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B11.threshold", "1");
    config->set_property("Acquisition_1B11.doppler_max", "5000");
    config->set_property("Acquisition_1B12.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B12.threshold", "1");
    config->set_property("Acquisition_1B12.doppler_max", "5000");
    config->set_property("Acquisition_1B13.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B13.threshold", "1");
    config->set_property("Acquisition_1B13.doppler_max", "5000");
    config->set_property("Acquisition_1B14.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B14.threshold", "1");
    config->set_property("Acquisition_1B14.doppler_max", "5000");
    config->set_property("Acquisition_1B15.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    config->set_property("Acquisition_1B15.threshold", "1");
    config->set_property("Acquisition_1B15.doppler_max", "5000");

    config->set_property("Tracking_1C0.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C1.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C2.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C3.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C4.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C5.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C6.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C7.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1B8.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    config->set_property("Tracking_1B9.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    config->set_property("Tracking_1B10.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    config->set_property("Tracking_1B11.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    config->set_property("Tracking_1B12.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    config->set_property("Tracking_1B13.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    config->set_property("Tracking_1B14.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    config->set_property("Tracking_1B15.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");

    config->set_property("TelemetryDecoder_1C0.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C1.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C2.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C3.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C4.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C5.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C6.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C7.implementation", "GPS_L1_CA_Telemetry_Decoder");

    config->set_property("TelemetryDecoder_1B8.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B9.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B10.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B11.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B12.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B13.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B14.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B15.implementation", "Galileo_E1B_Telemetry_Decoder");

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
