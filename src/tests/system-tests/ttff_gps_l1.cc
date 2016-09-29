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
#include <string>
#include <thread>
#include "in_memory_configuration.h"
#include "concurrent_queue.h"
#include "concurrent_map.h"
#include "control_thread.h"
#include "gnss_flowgraph.h"
#include "gps_acq_assist.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>


DEFINE_int32(fs_in, 4000000, "Sampling rate, in Ms/s");
DEFINE_int32(max_measurement_duration, 90, "Maximum time waiting for a position fix, in seconds");
DEFINE_int32(num_measurements, 2, "Number of measurements");
DEFINE_string(device_address, "192.168.40.2", "USRP device IP address");
DEFINE_string(subdevice, "B:0", "USRP subdevice");

// For GPS NAVIGATION (L1)
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

class TTFF_GPS_L1_CA_Test: public ::testing::Test
{
public:
    std::shared_ptr<InMemoryConfiguration> config;
};

TEST_F(TTFF_GPS_L1_CA_Test, ColdStart)
{
    unsigned int num_measurements = 0;
    unsigned int num_valid_measurements = 0;
    config = std::make_shared<InMemoryConfiguration>();
    google::InitGoogleLogging("ttff");

    // Set the Signal Source
    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(FLAGS_fs_in));
    config->set_property("SignalSource.item_type", "cshort");
    config->set_property("SignalSource.implementation", "UHD_Signal_Source");
    config->set_property("SignalSource.freq", std::to_string(1575420000));
    config->set_property("SignalSource.gain", std::to_string(40));
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
    config->set_property("InputFilter.sampling_frequency", std::to_string(4000000));
    config->set_property("InputFilter.IF", std::to_string(0));
    config->set_property("Resampler.implementation", "Pass_Through");
    config->set_property("Resampler.dump", "false");
    config->set_property("Resampler.item_type", "gr_complex");
    config->set_property("Resampler.sample_freq_in", std::to_string(4000000));
    config->set_property("Resampler.sample_freq_out", std::to_string(4000000));

    // Set the number of Channels
    config->set_property("Channels_1C.count", std::to_string(8));
    config->set_property("Channels.in_acquisition", std::to_string(1));
    config->set_property("Channel.signal", "1C");

    // Set Acquisition
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.item_type", "gr_complex");
    config->set_property("Acquisition_1C.if", std::to_string(0));
    config->set_property("Acquisition_1C.coherent_integration_time_ms", std::to_string(1));
    config->set_property("Acquisition_1C.threshold", std::to_string(0.01));
    config->set_property("Acquisition_1C.doppler_max", std::to_string(8000));
    config->set_property("Acquisition_1C.doppler_step", std::to_string(500));
    config->set_property("Acquisition_1C.bit_transition_flag", "false");
    config->set_property("Acquisition_1C.max_dwells", std::to_string(1));

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


    bool valid_pvt_received = false;

    int n;
    for(n = 0; n < FLAGS_num_measurements; n++) //
        {
            // reset start( hot /warm / cold )
            // COLD START
            config->set_property("GNSS-SDR.SUPL_gps_enabled", "false");
            config->set_property("GNSS-SDR.SUPL_read_gps_assistance_xml", "false");

            std::shared_ptr<ControlThread> control_thread = std::make_shared<ControlThread>(config);

            //  - start clock
            // record startup time
            struct timeval tv;
            gettimeofday(&tv, NULL);
            long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

            //      - start receiver
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

            //    - if (pvt_fix | max_waiting_time)
            //        - stop_clock
            gettimeofday(&tv, NULL);
            long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
            double ttff = static_cast<double>(end - begin) / 1000000.0;

            std::shared_ptr<GNSSFlowgraph> flowgraph = control_thread->flowgraph();
            EXPECT_FALSE(flowgraph->running());
            num_measurements = num_measurements + 1;
            std::cout << "Measurement " << num_measurements << ", which took " << ttff << " seconds." << std::endl;
            std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(5));
            // if (pvt_fix) num_valid_measurements = num_valid_measurements + 1;
        }
    std::cout << "BYE " << num_measurements << std::endl;
    // Compute min, max, mean, stdev,

    // Print TTFF report

}
