/*!
 * \file gnss_flowgraph_test.cc
 * \brief  This file implements tests for a flowgraph
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
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
#include "null_sink_output_filter.h"
#include "pass_through.h"
#include "file_signal_source.h"


TEST(GNSSFlowgraph, InstantiateConnectStartStop)
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.SUPL_gps_enabled", "false");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.repeat", "true");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat";
    config->set_property("SignalSource.filename", filename);
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("Channels_GPS.count", "1");
    config->set_property("Channels.in_acquisition", "1");
    config->set_property("Channel.system", "GPS");
    config->set_property("Channel.signal", "1C");
    config->set_property("Acquisition_GPS.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_GPS.threshold", "1");
    config->set_property("Acquisition_GPS.doppler_max", "5000");
    config->set_property("Acquisition_GPS.doppler_min", "-5000");
    config->set_property("Tracking_GPS.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("TelemetryDecoder_GPS.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("Observables.implementation", "GPS_L1_CA_Observables");
    config->set_property("PVT.implementation", "GPS_L1_CA_PVT");
    config->set_property("OutputFilter.implementation", "Null_Sink_Output_Filter");
    config->set_property("OutputFilter.item_type", "gr_complex");

    std::shared_ptr<GNSSFlowgraph> flowgraph = std::make_shared<GNSSFlowgraph>(config, gr::msg_queue::make(0));

    EXPECT_NO_THROW(flowgraph->connect());
    EXPECT_TRUE(flowgraph->connected());

    EXPECT_NO_THROW(flowgraph->start());
    EXPECT_TRUE(flowgraph->running());
    flowgraph->stop();
    EXPECT_FALSE(flowgraph->running());
}



