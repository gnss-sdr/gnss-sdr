/*!
 * \file control_thread_test.cc
 * \brief  This file implements tests for the ControlThread.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2013. cfernandez(at)cttc.es
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


#include "channel_event.h"
#include "command_event.h"
#include "concurrent_queue.h"
#include "control_thread.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_make_unique.h"
#include "in_memory_configuration.h"
#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception_ptr.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/lexical_cast.hpp>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <chrono>
#include <exception>
#include <memory>
#include <thread>
#include <unistd.h>

#if USE_GLOG_AND_GFLAGS
#include <gflags/gflags.h>
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

class ControlThreadTest : public ::testing::Test
{
public:
    static int stop_receiver();
};


int ControlThreadTest::stop_receiver()
{
    const std::string queue_name = "receiver_control_queue";
    std::unique_ptr<boost::interprocess::message_queue> d_mq;
    try
        {
            bool queue_found = false;

            while (!queue_found)
                {
                    try
                        {
                            // Attempt to open the message queue
                            d_mq = std::make_unique<boost::interprocess::message_queue>(boost::interprocess::open_only, queue_name.c_str());
                            queue_found = true;  // Queue found
                        }
                    catch (const boost::interprocess::interprocess_exception&)
                        {
                            // Queue not found, wait and retry
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                }


            double stop_message = -200.0;

            // Wait for a couple of seconds before sending
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Send the double value
            d_mq->send(&stop_message, sizeof(stop_message), 0);  // Priority 0
            return 0;
        }
    catch (const boost::interprocess::interprocess_exception& e)
        {
            std::cerr << "Failed to send stop message: " << e.what() << std::endl;
            return -1;
        }
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

    ASSERT_TRUE(fs::exists(file));

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
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
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

    ASSERT_TRUE(fs::exists(file));

    auto control_thread2 = std::make_unique<ControlThread>(config);
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
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
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

    ASSERT_TRUE(fs::exists(file));

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
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
