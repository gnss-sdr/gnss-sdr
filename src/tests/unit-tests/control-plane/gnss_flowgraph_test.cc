/*!
 * \file gnss_flowgraph_test.cc
 * \brief  This file implements tests for a flowgraph
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include <gnuradio/msg_queue.h>
#include <gtest/gtest.h>
#include "gnss_flowgraph.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "file_configuration.h"
#include "channel.h"
#include "acquisition_interface.h"
#include "tracking_interface.h"
#include "channel_interface.h"
#include "pass_through.h"
#include "file_signal_source.h"


TEST(GNSSFlowgraph, InstantiateConnectStartStopOldNotation)
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
    config->set_property("SignalSource.filename", filename);
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

    std::shared_ptr<GNSSFlowgraph> flowgraph = std::make_shared<GNSSFlowgraph>(config, gr::msg_queue::make(0));

    EXPECT_NO_THROW(flowgraph->connect());
    EXPECT_TRUE(flowgraph->connected());

    EXPECT_NO_THROW(flowgraph->start());
    EXPECT_TRUE(flowgraph->running());
    flowgraph->stop();
    EXPECT_FALSE(flowgraph->running());
}


TEST(GNSSFlowgraph, InstantiateConnectStartStop)
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.repeat", "true");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat";
    config->set_property("SignalSource.filename", filename);
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

    std::shared_ptr<GNSSFlowgraph> flowgraph = std::make_shared<GNSSFlowgraph>(config, gr::msg_queue::make(0));

    EXPECT_NO_THROW(flowgraph->connect());
    EXPECT_TRUE(flowgraph->connected());

    EXPECT_NO_THROW(flowgraph->start());
    EXPECT_TRUE(flowgraph->running());
    flowgraph->stop();
    EXPECT_FALSE(flowgraph->running());
}

TEST(GNSSFlowgraph, InstantiateConnectStartStopGalileoE1B)
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.repeat", "true");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat";
    config->set_property("SignalSource.filename", filename);
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

    std::shared_ptr<GNSSFlowgraph> flowgraph = std::make_shared<GNSSFlowgraph>(config, gr::msg_queue::make(0));

    EXPECT_NO_THROW(flowgraph->connect());
    EXPECT_TRUE(flowgraph->connected());

    EXPECT_NO_THROW(flowgraph->start());
    EXPECT_TRUE(flowgraph->running());
    flowgraph->stop();
    EXPECT_FALSE(flowgraph->running());
}


TEST(GNSSFlowgraph, InstantiateConnectStartStopHybrid)
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.repeat", "true");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat";
    config->set_property("SignalSource.filename", filename);
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
    config->set_property("TelemetryDecoder_1C0.decimation_factor", "4");
    config->set_property("TelemetryDecoder_1C1.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C1.decimation_factor", "4");
    config->set_property("TelemetryDecoder_1C2.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C2.decimation_factor", "4");
    config->set_property("TelemetryDecoder_1C3.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C3.decimation_factor", "4");
    config->set_property("TelemetryDecoder_1C4.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C4.decimation_factor", "4");
    config->set_property("TelemetryDecoder_1C5.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C5.decimation_factor", "4");
    config->set_property("TelemetryDecoder_1C6.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C6.decimation_factor", "4");
    config->set_property("TelemetryDecoder_1C7.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C7.decimation_factor", "4");

    config->set_property("TelemetryDecoder_1B8.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B8.decimation_factor", "1");
    config->set_property("TelemetryDecoder_1B9.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B9.decimation_factor", "1");
    config->set_property("TelemetryDecoder_1B10.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B10.decimation_factor", "1");
    config->set_property("TelemetryDecoder_1B11.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B11.decimation_factor", "1");
    config->set_property("TelemetryDecoder_1B12.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B12.decimation_factor", "1");
    config->set_property("TelemetryDecoder_1B13.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B13.decimation_factor", "1");
    config->set_property("TelemetryDecoder_1B14.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B14.decimation_factor", "1");
    config->set_property("TelemetryDecoder_1B15.implementation", "Galileo_E1B_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1B15.decimation_factor", "1");

    config->set_property("Observables.implementation", "Hybrid_Observables");
    config->set_property("PVT.implementation", "RTKLIB_PVT");

    std::shared_ptr<GNSSFlowgraph> flowgraph = std::make_shared<GNSSFlowgraph>(config, gr::msg_queue::make(0));

    EXPECT_NO_THROW(flowgraph->connect());
    EXPECT_TRUE(flowgraph->connected());

    EXPECT_NO_THROW(flowgraph->start());
    EXPECT_TRUE(flowgraph->running());
    flowgraph->stop();
    EXPECT_FALSE(flowgraph->running());
}
