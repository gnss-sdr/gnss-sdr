/*!
 * \file ttff_gps_l1.cc
 * \brief  This class implements a test for measuring
 * the Time-To-First-Fix
 * \author Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2016  (see AUTHORS file for a list of contributors)
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

#include <ctime>
#include <chrono>
#include <numeric>
#include <string>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <thread>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include "in_memory_configuration.h"
#include "file_configuration.h"
#include "concurrent_queue.h"
#include "concurrent_map.h"
#include "control_thread.h"
#include "gnss_flowgraph.h"
#include "gps_acq_assist.h"


DEFINE_int32(fs_in, 4000000, "Sampling rate, in Ms/s");
DEFINE_int32(max_measurement_duration, 90, "Maximum time waiting for a position fix, in seconds");
DEFINE_int32(num_measurements, 2, "Number of measurements");
DEFINE_string(device_address, "192.168.40.2", "USRP device IP address");
DEFINE_string(subdevice, "B:0", "USRP subdevice");

// For GPS NAVIGATION (L1)
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

std::vector<double> TTFF_v;

typedef struct  {
    long mtype; //required by sys v message
    double ttff;
} ttff_msgbuf;


void receive_msg()
{
    ttff_msgbuf msg;
    double ttff_msg = 0.0;
    int msgrcv_size = sizeof(msg.ttff);
    int msqid;
    key_t key = 1101;
    // wait for the queue to be created
    while((msqid = msgget(key, 0644)) == -1){}

    if (msgrcv(msqid, &msg, msgrcv_size, 1, 0) != -1)
        {
            ttff_msg = msg.ttff;
            if( (ttff_msg != 0) && (ttff_msg != -1))
                {
                    TTFF_v.push_back(ttff_msg / (100 * 10)); // Fix this !  averaging_depth * output_rate_ms
                    LOG(INFO) << "Valid Time-To-First-Fix: " << ttff_msg / (100 * 10) << "[s]";
                }

            if(ttff_msg != -1)
                {
                    receive_msg();
                }
        }
    return;
}

void print_TTFF_report(const std::vector<double> & ttff_v)
{
    std::vector<double> ttff = ttff_v;
    double sum = std::accumulate(ttff.begin(), ttff.end(), 0.0);
    double mean = sum / ttff.size();
    double sq_sum = std::inner_product(ttff.begin(), ttff.end(), ttff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / ttff.size() - mean * mean);
    std::cout << "---------------------------" << std::endl;
    std::cout << " Time-To-First FIX Report" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Valid measurements (" << ttff.size() << "/" << FLAGS_num_measurements << "): ";
    for(double ttff_ : ttff) std::cout << ttff_ << " ";
    std::cout << std::endl;
    std::cout << "TTFF mean: " << mean << " [s]" << std::endl;
    std::cout << "TTFF stdev: " << stdev << " [s]" << std::endl;
    std::cout << "---------------------------" << std::endl;
}


TEST(TTFF_GPS_L1_CA_Test, ColdStart)
{
    std::shared_ptr<InMemoryConfiguration> config;
    std::shared_ptr<FileConfiguration> config2;
    unsigned int num_measurements = 0;
    unsigned int num_valid_measurements = 0;
    config = std::make_shared<InMemoryConfiguration>();
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "../../conf/gnss-sdr_GPS_L1_USRP_X300_realtime.conf";
    config2 = std::make_shared<FileConfiguration>(filename);
    config2->set_property("SignalSource.samples", std::to_string(FLAGS_fs_in * FLAGS_max_measurement_duration));

    double central_freq = 1575420000.0;
    double gain_dB = 40.0;

    // Set the Signal Source
    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(FLAGS_fs_in));
    config->set_property("SignalSource.item_type", "cshort");
    config->set_property("SignalSource.implementation", "UHD_Signal_Source");
    config->set_property("SignalSource.freq", std::to_string(central_freq));
    config->set_property("SignalSource.sampling_frequency", std::to_string(FLAGS_fs_in));
    config->set_property("SignalSource.gain", std::to_string(gain_dB));
    config->set_property("SignalSource.subdevice", FLAGS_subdevice);
    config->set_property("SignalSource.samples", std::to_string(FLAGS_fs_in * FLAGS_max_measurement_duration));
    config->set_property("SignalSource.device_address", FLAGS_device_address);

    // Set the Signal Conditioner
    config->set_property("SignalConditioner.implementation", "Signal_Conditioner");
    config->set_property("DataTypeAdapter.implementation", "Pass_Through");
    config->set_property("DataTypeAdapter.item_type", "cshort");
    config->set_property("InputFilter.implementation", "Fir_Filter");
    config->set_property("InputFilter.dump", "false");
    config->set_property("InputFilter.input_item_type", "cshort");
    config->set_property("InputFilter.output_item_type", "gr_complex");
    config->set_property("InputFilter.taps_item_type", "float");
    config->set_property("InputFilter.number_of_taps", std::to_string(11));
    config->set_property("InputFilter.number_of_bands", std::to_string(2));
    config->set_property("InputFilter.band1_begin", std::to_string(0.0));
    config->set_property("InputFilter.band1_end", std::to_string(0.48));
    config->set_property("InputFilter.band2_begin", std::to_string(0.52));
    config->set_property("InputFilter.band2_end", std::to_string(1.0));
    config->set_property("InputFilter.ampl1_begin", std::to_string(1.0));
    config->set_property("InputFilter.ampl1_end", std::to_string(1.0));
    config->set_property("InputFilter.ampl2_begin", std::to_string(0.0));
    config->set_property("InputFilter.ampl2_end", std::to_string(0.0));
    config->set_property("InputFilter.band1_error", std::to_string(1.0));
    config->set_property("InputFilter.band2_error", std::to_string(1.0));
    config->set_property("InputFilter.filter_type", "bandpass");
    config->set_property("InputFilter.grid_density", std::to_string(16));
    config->set_property("InputFilter.sampling_frequency", std::to_string(FLAGS_fs_in));
    config->set_property("InputFilter.IF", std::to_string(0));
    config->set_property("Resampler.implementation", "Pass_Through");
    config->set_property("Resampler.dump", "false");
    config->set_property("Resampler.item_type", "gr_complex");
    config->set_property("Resampler.sample_freq_in", std::to_string(FLAGS_fs_in));
    config->set_property("Resampler.sample_freq_out", std::to_string(FLAGS_fs_in));

    // Set the number of Channels
    config->set_property("Channels_1C.count", std::to_string(8));
    config->set_property("Channels.in_acquisition", std::to_string(1));
    config->set_property("Channel.signal", "1C");

    // Set Acquisition
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Tong_Acquisition");
    config->set_property("Acquisition_1C.item_type", "gr_complex");
    config->set_property("Acquisition_1C.if", std::to_string(0));
    config->set_property("Acquisition_1C.coherent_integration_time_ms", std::to_string(1));
    config->set_property("Acquisition_1C.threshold", std::to_string(0.01));
    config->set_property("Acquisition_1C.doppler_max", std::to_string(8000));
    config->set_property("Acquisition_1C.doppler_step", std::to_string(500));
    config->set_property("Acquisition_1C.bit_transition_flag", "false");
    config->set_property("Acquisition_1C.max_dwells", std::to_string(1));
    config->set_property("Acquisition_1C.tong_init_val", std::to_string(2));
    config->set_property("Acquisition_1C.tong_max_val", std::to_string(10));
    config->set_property("Acquisition_1C.tong_max_dwells", std::to_string(30));

    // Set Tracking
    config->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("Tracking_1C.if", std::to_string(0));
    config->set_property("Tracking_1C.dump", "false");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");
    config->set_property("Tracking_1C.pll_bw_hz", std::to_string(30.0));
    config->set_property("Tracking_1C.dll_bw_hz", std::to_string(4.0));
    config->set_property("Tracking_1C.order", std::to_string(3));
    config->set_property("Tracking_1C.early_late_space_chips", std::to_string(0.5));

    // Set Telemetry
    config->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
    config->set_property("TelemetryDecoder_1C.dump", "false");
    config->set_property("TelemetryDecoder_1C.decimation_factor", std::to_string(1));

    // Set Observables
    config->set_property("Observables.implementation", "GPS_L1_CA_Observables");
    config->set_property("Observables.dump", "false");
    config->set_property("Observables.dump_filename", "./observables.dat");

    // Set PVT
    config->set_property("PVT.implementation", "GPS_L1_CA_PVT");
    config->set_property("PVT.averaging_depth", std::to_string(10));
    config->set_property("PVT.flag_averaging", "true");
    config->set_property("PVT.output_rate_ms", std::to_string(100));
    config->set_property("PVT.display_rate_ms", std::to_string(500));
    config->set_property("PVT.dump_filename", "./PVT");
    config->set_property("PVT.nmea_dump_filename", "./gnss_sdr_pvt.nmea");
    config->set_property("PVT.flag_nmea_tty_port", "false");
    config->set_property("PVT.nmea_dump_devname", "/dev/pts/4");
    config->set_property("PVT.flag_rtcm_server", "false");
    config->set_property("PVT.flag_rtcm_tty_port", "false");
    config->set_property("PVT.rtcm_dump_devname", "/dev/pts/1");
    config->set_property("PVT.dump", "false");


    int n;
    for(n = 0; n < FLAGS_num_measurements; n++)
        {
            // reset start( hot /warm / cold )
            // COLD START
            config->set_property("GNSS-SDR.SUPL_gps_enabled", "false");
            config->set_property("GNSS-SDR.SUPL_read_gps_assistance_xml", "false");
            config2->set_property("GNSS-SDR.SUPL_read_gps_assistance_xml", "false");
            config2->set_property("PVT.flag_rtcm_server", "false");

            std::shared_ptr<ControlThread> control_thread = std::make_shared<ControlThread>(config2);

            // record startup time
            struct timeval tv;
            gettimeofday(&tv, NULL);
            long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

            // start receiver
            try
            {
                    control_thread->run();
            }
            catch( boost::exception & e )
            {
                    std::cout << "Boost exception: " << boost::diagnostic_information(e);
            }
            catch(std::exception const&  ex)
            {
                    std::cout  << "STD exception: " << ex.what();
            }

            // stop clock
            gettimeofday(&tv, NULL);
            long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
            double ttff = static_cast<double>(end - begin) / 1000000.0;

            std::shared_ptr<GNSSFlowgraph> flowgraph = control_thread->flowgraph();
            EXPECT_FALSE(flowgraph->running());

            num_measurements = num_measurements + 1;
            std::cout << "Measurement " << num_measurements << ", which took " << ttff << " seconds." << std::endl;
            std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(5));
        }

    // Print TTFF report
    print_TTFF_report(TTFF_v);

}



int main(int argc, char **argv)
{
    std::cout << "Running Time-To-First-Fix test..." << std::endl;
    int res = 0;
    TTFF_v.clear();
    testing::InitGoogleTest(&argc, argv);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    // Create Sys V message queue to read TFFF measurements
    key_t sysv_msg_key;
    int sysv_msqid;
    sysv_msg_key = 1101;
    int msgflg = IPC_CREAT | 0666;
    if ((sysv_msqid = msgget(sysv_msg_key, msgflg )) == -1)
    {
        std::cout<<"GNSS-SDR can not create message queues!" << std::endl;
        throw new std::exception();
    }

    // Start queue thread
    std::thread receive_msg_thread(receive_msg);

    // Run the Tests
    try
    {
            res = RUN_ALL_TESTS();
    }
    catch(...)
    {
            LOG(WARNING) << "Unexpected catch";
    }

    // Terminate the queue thread
    ttff_msgbuf msg;
    msg.mtype = 1;
    msg.ttff = -1;
    int msgsend_size;
    msgsend_size = sizeof(msg.ttff);
    msgsnd(sysv_msqid, &msg, msgsend_size, IPC_NOWAIT);
    receive_msg_thread.join();

    google::ShutDownCommandLineFlags();
    return res;
}
