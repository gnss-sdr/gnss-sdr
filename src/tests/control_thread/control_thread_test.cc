/*!
 * \file control_thread_test.cc
 * \brief  This file implements tests for the ControlThread.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2013. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2013  (see AUTHORS file for a list of contributors)
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
#include "control_thread.h"
#include "in_memory_configuration.h"
#include "control_thread.h"
#include <boost/lexical_cast.hpp>
#include "gps_ephemeris.h"
#include "gps_iono.h"
#include "gps_utc_model.h"
#include "gps_almanac.h"

#include "galileo_ephemeris.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "galileo_almanac.h"

#include "concurrent_queue.h"
#include "concurrent_map.h"
#include <unistd.h>
#include <gnuradio/message.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "gnss_flowgraph.h"
#include "file_configuration.h"
#include "control_message_factory.h"
#include <boost/thread/thread.hpp>

extern concurrent_map<Gps_Ephemeris> global_gps_ephemeris_map;
extern concurrent_map<Gps_Iono> global_gps_iono_map;
extern concurrent_map<Gps_Utc_Model> global_gps_utc_model_map;
extern concurrent_map<Gps_Almanac> global_gps_almanac_map;
extern concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

extern concurrent_queue<Gps_Ephemeris> global_gps_ephemeris_queue;
extern concurrent_queue<Gps_Iono> global_gps_iono_queue;
extern concurrent_queue<Gps_Utc_Model> global_gps_utc_model_queue;
extern concurrent_queue<Gps_Almanac> global_gps_almanac_queue;
extern concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;

extern concurrent_map<Galileo_Ephemeris> global_galileo_ephemeris_map;
extern concurrent_map<Galileo_Iono> global_galileo_iono_map;
extern concurrent_map<Galileo_Utc_Model> global_galileo_utc_model_map;
extern concurrent_map<Galileo_Almanac> global_galileo_almanac_map;
//extern concurrent_map<Galileo_Acq_Assist> global_gps_acq_assist_map;

extern concurrent_queue<Galileo_Ephemeris> global_galileo_ephemeris_queue;
extern concurrent_queue<Galileo_Iono> global_galileo_iono_queue;
extern concurrent_queue<Galileo_Utc_Model> global_galileo_utc_model_queue;
extern concurrent_queue<Galileo_Almanac> global_galileo_almanac_queue;



TEST(Control_Thread_Test, InstantiateRunControlMessages)
{
    InMemoryConfiguration *config = new InMemoryConfiguration();

    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.filename", "../src/tests/signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat");
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.repeat", "true");
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("SignalConditioner.item_type", "gr_complex");
    config->set_property("Channels.count", "2");
    config->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition.item_type", "gr_complex");
    config->set_property("Tracking.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking.item_type", "gr_complex");
    config->set_property("TelemetryDecoder.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder.item_type", "gr_complex");
    config->set_property("Observables.implementation", "GPS_L1_CA_Observables");
    config->set_property("Observables.item_type", "gr_complex");
    config->set_property("PVT.implementation", "GPS_L1_CA_PVT");
    config->set_property("PVT.item_type", "gr_complex");
    config->set_property("OutputFilter.implementation", "Null_Sink_Output_Filter");
    config->set_property("OutputFilter.item_type", "gr_complex");

    std::unique_ptr<ControlThread> control_thread(new ControlThread(config));

    gr::msg_queue::sptr control_queue = gr::msg_queue::make(0);
    ControlMessageFactory *control_msg_factory = new ControlMessageFactory();

    control_queue->handle(control_msg_factory->GetQueueMessage(0,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(1,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(2,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(3,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(4,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(5,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(6,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(7,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(8,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(9,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(10,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(11,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(12,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(13,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(14,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(15,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(16,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(200,0));

    control_thread->set_control_queue(control_queue);

    control_thread->run();

    unsigned int expected18 = 18;
    unsigned int expected1 = 1;
    EXPECT_EQ(expected18, control_thread->processed_control_messages());
    EXPECT_EQ(expected1, control_thread->applied_actions());

    delete config;
    //delete control_thread;
    delete control_msg_factory;
}





TEST(Control_Thread_Test, InstantiateRunControlMessages2)
{
    InMemoryConfiguration *config = new InMemoryConfiguration();
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    config->set_property("SignalSource.filename", "../src/tests/signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat");
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("SignalConditioner.item_type", "gr_complex");
    config->set_property("Channels.count", "1");
    config->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition.item_type", "gr_complex");
    config->set_property("Tracking.implementation", "GPS_L1_CA_DLL_FLL_PLL_Tracking");
    config->set_property("Tracking.item_type", "gr_complex");
    config->set_property("TelemetryDecoder.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder.item_type", "gr_complex");
    config->set_property("Observables.implementation", "GPS_L1_CA_Observables");
    config->set_property("Observables.item_type", "gr_complex");
    config->set_property("PVT.implementation", "GPS_L1_CA_PVT");
    config->set_property("PVT.item_type", "gr_complex");
    config->set_property("OutputFilter.implementation", "Null_Sink_Output_Filter");
    config->set_property("OutputFilter.item_type", "gr_complex");

    ControlThread *control_thread = new ControlThread(config);

    gr::msg_queue::sptr control_queue = gr::msg_queue::make(0);
    ControlMessageFactory *control_msg_factory = new ControlMessageFactory();

    control_queue->handle(control_msg_factory->GetQueueMessage(0,0));
    control_queue->handle(control_msg_factory->GetQueueMessage(0,2));
    control_queue->handle(control_msg_factory->GetQueueMessage(0,1));
    control_queue->handle(control_msg_factory->GetQueueMessage(0,3));
    control_queue->handle(control_msg_factory->GetQueueMessage(200,0));

    control_thread->set_control_queue(control_queue);

    control_thread->run();
    unsigned int expected5 = 5;
    unsigned int expected1 = 1;
    EXPECT_EQ(expected5, control_thread->processed_control_messages());
    EXPECT_EQ(expected1, control_thread->applied_actions());

    delete config;
    delete control_thread;
    delete control_msg_factory;
}
