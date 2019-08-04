/*!
 * \file control_thread_test.cc
 * \brief  This file implements tests for the ControlThread.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2013. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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


#include "channel_event.h"
#include "command_event.h"
#include "concurrent_queue.h"
#include "control_thread.h"
#include "in_memory_configuration.h"
#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <chrono>
#include <exception>
#include <memory>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>


class ControlThreadTest : public ::testing::Test
{
public:
    static int stop_receiver();
    typedef struct
    {
        long mtype;  // required by SysV message
        double message;
    } message_buffer;
};


int ControlThreadTest::stop_receiver()
{
    message_buffer msg_stop;
    msg_stop.mtype = 1;
    msg_stop.message = -200.0;
    int msqid_stop = -1;
    int msgsend_size = sizeof(msg_stop.message);
    key_t key_stop = 1102;

    // wait for the receiver control queue to be created
    while (((msqid_stop = msgget(key_stop, 0644))) == -1)
        {
        }

    // wait for a couple of seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Stop the receiver
    msgsnd(msqid_stop, &msg_stop, msgsend_size, IPC_NOWAIT);

    return 0;
}


TEST_F(ControlThreadTest /*unused*/, InstantiateRunControlMessages /*unused*/)
{
    std::shared_ptr<InMemoryConfiguration> config = std::make_shared<InMemoryConfiguration>();

    config->set_property("SignalSource.implementation", "File_Signal_Source");
    std::string path = std::string(TEST_PATH);
    std::string file = path + "signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat";
    const char* file_name = file.c_str();
    config->set_property("SignalSource.filename", file_name);
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.repeat", "true");
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("SignalConditioner.item_type", "gr_complex");
    config->set_property("Channels_1C.count", "2");
    config->set_property("Channels_1E.count", "0");
    config->set_property("Channels.in_acquisition", "1");
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.threshold", "1");
    config->set_property("Acquisition_1C.doppler_max", "5000");
    config->set_property("Acquisition_1C.doppler_min", "-5000");
    config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C.item_type", "gr_complex");
    config->set_property("Observables.implementation", "Hybrid_Observables");
    config->set_property("Observables.item_type", "gr_complex");
    config->set_property("PVT.implementation", "RTKLIB_PVT");
    config->set_property("PVT.item_type", "gr_complex");
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");

    std::shared_ptr<ControlThread> control_thread = std::make_shared<ControlThread>(config);

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    control_queue->push(pmt::make_any(channel_event_make(0, 0)));
    control_queue->push(pmt::make_any(channel_event_make(1, 0)));
    control_queue->push(pmt::make_any(command_event_make(200, 0)));

    control_thread->set_control_queue(control_queue);
    try
        {
            control_thread->run();
        }
    catch (const boost::exception& e)
        {
            std::cout << "Boost exception: " << boost::diagnostic_information(e);
        }
    catch (const std::exception& ex)
        {
            std::cout << "STD exception: " << ex.what();
        }

    unsigned int expected3 = 3;
    unsigned int expected1 = 1;
    EXPECT_EQ(expected3, control_thread->processed_control_messages());
    EXPECT_EQ(expected1, control_thread->applied_actions());
}


TEST_F(ControlThreadTest /*unused*/, InstantiateRunControlMessages2 /*unused*/)
{
    std::shared_ptr<InMemoryConfiguration> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    std::string path = std::string(TEST_PATH);
    std::string file = path + "signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat";
    const char* file_name = file.c_str();
    config->set_property("SignalSource.filename", file_name);
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.repeat", "true");
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("SignalConditioner.item_type", "gr_complex");
    config->set_property("Channels_1C.count", "4");
    config->set_property("Channels_1E.count", "0");
    config->set_property("Channels.in_acquisition", "1");
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.threshold", "1");
    config->set_property("Acquisition_1C.doppler_max", "5000");
    config->set_property("Acquisition_1C.doppler_min", "-5000");
    config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C.item_type", "gr_complex");
    config->set_property("Observables.implementation", "Hybrid_Observables");
    config->set_property("Observables.item_type", "gr_complex");
    config->set_property("PVT.implementation", "RTKLIB_PVT");
    config->set_property("PVT.item_type", "gr_complex");
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");

    std::unique_ptr<ControlThread> control_thread2(new ControlThread(config));
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue2 = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();

    control_queue2->push(pmt::make_any(channel_event_make(0, 0)));
    control_queue2->push(pmt::make_any(channel_event_make(2, 0)));
    control_queue2->push(pmt::make_any(channel_event_make(1, 0)));
    control_queue2->push(pmt::make_any(channel_event_make(3, 0)));
    control_queue2->push(pmt::make_any(command_event_make(200, 0)));


    control_thread2->set_control_queue(control_queue2);

    try
        {
            control_thread2->run();
        }
    catch (const boost::exception& e)
        {
            std::cout << "Boost exception: " << boost::diagnostic_information(e);
        }
    catch (const std::exception& ex)
        {
            std::cout << "STD exception: " << ex.what();
        }

    unsigned int expected5 = 5;
    unsigned int expected1 = 1;
    EXPECT_EQ(expected5, control_thread2->processed_control_messages());
    EXPECT_EQ(expected1, control_thread2->applied_actions());
}


TEST_F(ControlThreadTest /*unused*/, StopReceiverProgrammatically /*unused*/)
{
    std::shared_ptr<InMemoryConfiguration> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("SignalSource.implementation", "File_Signal_Source");
    std::string path = std::string(TEST_PATH);
    std::string file = path + "signal_samples/GSoC_CTTC_capture_2012_07_26_4Msps_4ms.dat";
    const char* file_name = file.c_str();
    config->set_property("SignalSource.filename", file_name);
    config->set_property("SignalSource.item_type", "gr_complex");
    config->set_property("SignalSource.sampling_frequency", "4000000");
    config->set_property("SignalSource.repeat", "true");
    config->set_property("SignalConditioner.implementation", "Pass_Through");
    config->set_property("SignalConditioner.item_type", "gr_complex");
    config->set_property("Channels_1C.count", "4");
    config->set_property("Channels_1E.count", "0");
    config->set_property("Channels.in_acquisition", "1");
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.threshold", "1");
    config->set_property("Acquisition_1C.doppler_max", "5000");
    config->set_property("Acquisition_1C.doppler_min", "-5000");
    config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C.item_type", "gr_complex");
    config->set_property("Observables.implementation", "Hybrid_Observables");
    config->set_property("Observables.item_type", "gr_complex");
    config->set_property("PVT.implementation", "RTKLIB_PVT");
    config->set_property("PVT.item_type", "gr_complex");
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");

    std::shared_ptr<ControlThread> control_thread = std::make_shared<ControlThread>(config);
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    control_thread->set_control_queue(control_queue);

    std::thread stop_receiver_thread(stop_receiver);

    try
        {
            control_thread->run();
        }
    catch (const boost::exception& e)
        {
            std::cout << "Boost exception: " << boost::diagnostic_information(e);
        }
    catch (const std::exception& ex)
        {
            std::cout << "STD exception: " << ex.what();
        }

    stop_receiver_thread.join();
}
