/*!
 * \file gnss_flowgraph_test.cc
 * \brief  This file implements tests for a flowgraph
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#include <gtest/gtest.h>
#include <gnuradio/msg_queue.h>
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
    InMemoryConfiguration* config = new InMemoryConfiguration();

    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.filename", "../src/tests/signal_samples/Galileo_E1_ID_1_Fs_4Msps_8ms.dat");
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("Channels.count", "2");
    config->set_property("Channels.acquisition.implementation", "Pass_Through");
    config->set_property("Channels.tracking.implementation", "Pass_Through");
    config->set_property("Channels.observables.implementation", "Pass_Through");
    config->set_property("Observables.implementation", "GPS_L1_CA_Observables");
    config->set_property("PVT.implementation", "GPS_L1_CA_PVT");
    config->set_property("OutputFilter.implementation", "Null_Sink_Output_Filter");

    GNSSFlowgraph* flowgraph = new GNSSFlowgraph(config, gr::msg_queue::make(0));

    EXPECT_STREQ("File_Signal_Source", flowgraph->signal_source()->implementation().c_str());
    EXPECT_STREQ("Pass_Through", flowgraph->signal_conditioner()->implementation().c_str());
    EXPECT_STREQ("Channel", flowgraph->channel(0)->implementation().c_str());
    EXPECT_STREQ("Pass_Through", ((Channel*)flowgraph->channel(0))->acquisition()->implementation().c_str());
    EXPECT_STREQ("Pass_Through", ((Channel*)flowgraph->channel(0))->tracking()->implementation().c_str());
    EXPECT_STREQ("Channel", flowgraph->channel(1)->implementation().c_str());
    EXPECT_STREQ("Pass_Through", ((Channel*)flowgraph->channel(1))->acquisition()->implementation().c_str());
    EXPECT_STREQ("Pass_Through", ((Channel*)flowgraph->channel(1))->tracking()->implementation().c_str());
    EXPECT_STREQ("GPS_L1_CA_Observables", flowgraph->observables()->implementation().c_str());
    EXPECT_STREQ("GPS_L1_CA_PVT", flowgraph->pvt()->implementation().c_str());
    EXPECT_STREQ("Null_Sink_Output_Filter", flowgraph->output_filter()->implementation().c_str());

    EXPECT_NO_THROW(flowgraph->connect());
    EXPECT_TRUE(flowgraph->connected());
    EXPECT_NO_THROW(flowgraph->start());
    EXPECT_TRUE(flowgraph->running());
    flowgraph->stop();
    EXPECT_FALSE(flowgraph->running());

    delete flowgraph;
}
