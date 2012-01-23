
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class implements a Unit Tests for the class GNSSFlowgraph.
 *
 */

#include <gtest/gtest.h>
#include <gnuradio/gr_msg_queue.h>
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

    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("Channels.count", "2");
    config->set_property("Channels.acquisition.implementation", "Pass_Through");
    config->set_property("Channels.tracking.implementation", "Pass_Through");
    config->set_property("Channels.observables.implementation", "Pass_Through");
    config->set_property("Observables.implementation", "GpsL1CaObservables");
    config->set_property("PVT.implementation", "GPS_L1_CA_PVT");
    config->set_property("OutputFilter.implementation", "NullSinkOutputFilter");

    GNSSFlowgraph* flowgraph = new GNSSFlowgraph(config, gr_make_msg_queue(0));

    EXPECT_STREQ("FileSignalSource", flowgraph->signal_source()->implementation().c_str());
    EXPECT_STREQ("Pass_Through", flowgraph->signal_conditioner()->implementation().c_str());
    EXPECT_STREQ("Channel", flowgraph->channel(0)->implementation().c_str());
    EXPECT_STREQ("Pass_Through", ((Channel*)flowgraph->channel(0))->acquisition()->implementation().c_str());
    EXPECT_STREQ("Pass_Through", ((Channel*)flowgraph->channel(0))->tracking()->implementation().c_str());
    EXPECT_STREQ("Channel", flowgraph->channel(1)->implementation().c_str());
    EXPECT_STREQ("Pass_Through", ((Channel*)flowgraph->channel(1))->acquisition()->implementation().c_str());
    EXPECT_STREQ("Pass_Through", ((Channel*)flowgraph->channel(1))->tracking()->implementation().c_str());
    EXPECT_STREQ("GpsL1CaObservables", flowgraph->observables()->implementation().c_str());
    EXPECT_STREQ("GPS_L1_CA_PVT", flowgraph->pvt()->implementation().c_str());
    EXPECT_STREQ("NullSinkOutputFilter", flowgraph->output_filter()->implementation().c_str());

    EXPECT_NO_THROW(flowgraph->connect());
    EXPECT_TRUE(flowgraph->connected());
    EXPECT_NO_THROW(flowgraph->start());
    EXPECT_TRUE(flowgraph->running());
    flowgraph->stop();
    EXPECT_FALSE(flowgraph->running());

    delete flowgraph;
}
