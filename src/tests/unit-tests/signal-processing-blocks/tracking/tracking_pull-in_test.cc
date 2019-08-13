/*!
 * \file tracking_test.cc
 * \brief  This class implements a tracking Pull-In test for GPS_L1_CA_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
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

#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include "GPS_L5.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "acquisition_msg_rx.h"
#include "concurrent_queue.h"
#include "galileo_e1_pcps_ambiguous_acquisition.h"
#include "galileo_e5a_noncoherent_iq_acquisition_caf.h"
#include "galileo_e5a_pcps_acquisition.h"
#include "gnss_block_factory.h"
#include "gnss_sdr_valve.h"
#include "gnuplot_i.h"
#include "gps_l1_ca_pcps_acquisition.h"
#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "gps_l2_m_pcps_acquisition.h"
#include "gps_l5i_pcps_acquisition.h"
#include "in_memory_configuration.h"
#include "signal_generator_flags.h"
#include "test_flags.h"
#include "tracking_dump_reader.h"
#include "tracking_interface.h"
#include "tracking_tests_flags.h"
#include "tracking_true_obs_reader.h"
#include <armadillo>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <chrono>
#include <cstdint>
#include <utility>
#include <vector>

#ifdef GR_GREATER_38
#include <gnuradio/filter/fir_filter_blk.h>
#else
#include <gnuradio/filter/fir_filter_ccf.h>
#endif

#if HAS_STD_FILESYSTEM
#if HAS_STD_FILESYSTEM_EXPERIMENTAL
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif


// ######## GNURADIO TRACKING BLOCK MESSAGE RECEVER #########
class TrackingPullInTest_msg_rx;

using TrackingPullInTest_msg_rx_sptr = boost::shared_ptr<TrackingPullInTest_msg_rx>;

TrackingPullInTest_msg_rx_sptr TrackingPullInTest_msg_rx_make();

class TrackingPullInTest_msg_rx : public gr::block
{
private:
    friend TrackingPullInTest_msg_rx_sptr TrackingPullInTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    TrackingPullInTest_msg_rx();

public:
    int rx_message;
    ~TrackingPullInTest_msg_rx();  //!< Default destructor
};


TrackingPullInTest_msg_rx_sptr TrackingPullInTest_msg_rx_make()
{
    return TrackingPullInTest_msg_rx_sptr(new TrackingPullInTest_msg_rx());
}


void TrackingPullInTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;  //3 -> loss of lock
            //std::cout << "Received trk message: " << rx_message << std::endl;
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_tracking Bad cast!";
            rx_message = 0;
        }
}


TrackingPullInTest_msg_rx::TrackingPullInTest_msg_rx() : gr::block("TrackingPullInTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&TrackingPullInTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


TrackingPullInTest_msg_rx::~TrackingPullInTest_msg_rx() = default;


// ###########################################################

class TrackingPullInTest : public ::testing::Test
{
public:
    enum StringValue
    {
        evGPS_1C,
        evGPS_2S,
        evGPS_L5,
        evSBAS_1C,
        evGAL_1B,
        evGAL_5X,
        evGLO_1G,
        evGLO_2G
    };
    std::map<std::string, StringValue> mapStringValues_;

    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;
    std::string p6;
    std::string implementation = FLAGS_trk_test_implementation;

    const int baseband_sampling_freq = FLAGS_fs_gen_sps;

    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_signal_file;

    std::map<int, double> doppler_measurements_map;
    std::map<int, double> code_delay_measurements_map;
    std::map<int, uint64_t> acq_samplestamp_map;

    int configure_generator(double CN0_dBHz, int file_idx);
    int generate_signal();
    std::vector<double> check_results_doppler(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value,
        double& mean_error,
        double& std_dev_error);
    std::vector<double> check_results_acc_carrier_phase(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value,
        double& mean_error,
        double& std_dev_error);
    std::vector<double> check_results_codephase(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value,
        double& mean_error,
        double& std_dev_error);

    TrackingPullInTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
        mapStringValues_["1C"] = evGPS_1C;
        mapStringValues_["2S"] = evGPS_2S;
        mapStringValues_["L5"] = evGPS_L5;
        mapStringValues_["1B"] = evGAL_1B;
        mapStringValues_["5X"] = evGAL_5X;
        mapStringValues_["1G"] = evGLO_1G;
        mapStringValues_["2G"] = evGLO_2G;
    }

    ~TrackingPullInTest() = default;

    void configure_receiver(double PLL_wide_bw_hz,
        double DLL_wide_bw_hz,
        double PLL_narrow_bw_hz,
        double DLL_narrow_bw_hz,
        int extend_correlation_symbols);

    bool acquire_signal(int SV_ID);
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue;
};


int TrackingPullInTest::configure_generator(double CN0_dBHz, int file_idx)
{
    // Configure signal generator
    generator_binary = FLAGS_generator_binary;

    p1 = std::string("-rinex_nav_file=") + FLAGS_rinex_nav_file;
    if (FLAGS_dynamic_position.empty())
        {
            p2 = std::string("-static_position=") + FLAGS_static_position + std::string(",") + std::to_string(FLAGS_duration * 10);
        }
    else
        {
            p2 = std::string("-obs_pos_file=") + std::string(FLAGS_dynamic_position);
        }
    p3 = std::string("-rinex_obs_file=") + FLAGS_filename_rinex_obs;                    // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + FLAGS_signal_file + std::to_string(file_idx);  // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);       //Baseband sampling frequency [MSps]
    p6 = std::string("-CN0_dBHz=") + std::to_string(CN0_dBHz);                          // Signal generator CN0
    return 0;
}


int TrackingPullInTest::generate_signal()
{
    int child_status;

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], &p6[0], nullptr};

    int pid;
    if ((pid = fork()) == -1)
        {
            perror("fork err");
        }
    else if (pid == 0)
        {
            execv(&generator_binary[0], parmList);
            std::cout << "Return not expected. Must be an execv err." << std::endl;
            std::terminate();
        }

    waitpid(pid, &child_status, 0);

    std::cout << "Signal and Observables RINEX and RAW files created." << std::endl;
    return 0;
}


void TrackingPullInTest::configure_receiver(
    double PLL_wide_bw_hz,
    double DLL_wide_bw_hz,
    double PLL_narrow_bw_hz,
    double DLL_narrow_bw_hz,
    int extend_correlation_symbols)
{
    config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Tracking.dump", "true");
    config->set_property("Tracking.dump_filename", "./tracking_ch_");
    config->set_property("Tracking.implementation", implementation);
    config->set_property("Tracking.item_type", "gr_complex");
    config->set_property("Tracking.pll_bw_hz", std::to_string(PLL_wide_bw_hz));
    config->set_property("Tracking.dll_bw_hz", std::to_string(DLL_wide_bw_hz));
    config->set_property("Tracking.extend_correlation_symbols", std::to_string(extend_correlation_symbols));
    config->set_property("Tracking.pll_bw_narrow_hz", std::to_string(PLL_narrow_bw_hz));
    config->set_property("Tracking.dll_bw_narrow_hz", std::to_string(DLL_narrow_bw_hz));
    gnss_synchro.PRN = FLAGS_test_satellite_PRN;
    gnss_synchro.Channel_ID = 0;
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    std::string System_and_Signal;
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking")
        {
            gnss_synchro.System = 'G';
            std::string signal = "1C";
            System_and_Signal = "GPS L1 CA";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.early_late_space_narrow_chips", "0.5");
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking")
        {
            gnss_synchro.System = 'E';
            std::string signal = "1B";
            System_and_Signal = "Galileo E1B";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.15");
            config->set_property("Tracking.very_early_late_space_chips", "0.6");
            config->set_property("Tracking.early_late_space_narrow_chips", "0.15");
            config->set_property("Tracking.very_early_late_space_narrow_chips", "0.6");
            config->set_property("Tracking.track_pilot", "true");
        }
    else if (implementation == "GPS_L2_M_DLL_PLL_Tracking")
        {
            gnss_synchro.System = 'G';
            std::string signal = "2S";
            System_and_Signal = "GPS L2CM";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "true");
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking" or implementation == "Galileo_E5a_DLL_PLL_Tracking_b")
        {
            gnss_synchro.System = 'E';
            std::string signal = "5X";
            System_and_Signal = "Galileo E5a";
            signal.copy(gnss_synchro.Signal, 2, 0);
            if (implementation == "Galileo_E5a_DLL_PLL_Tracking_b")
                {
                    config->supersede_property("Tracking.implementation", std::string("Galileo_E5a_DLL_PLL_Tracking"));
                }
            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "true");
            config->set_property("Tracking.order", "2");
        }
    else if (implementation == "GPS_L5_DLL_PLL_Tracking")
        {
            gnss_synchro.System = 'G';
            std::string signal = "L5";
            System_and_Signal = "GPS L5I";
            signal.copy(gnss_synchro.Signal, 2, 0);
            config->set_property("Tracking.early_late_space_chips", "0.5");
            config->set_property("Tracking.track_pilot", "true");
            config->set_property("Tracking.order", "2");
        }
    else
        {
            std::cout << "The test can not run with the selected tracking implementation\n ";
            throw(std::exception());
        }

    std::cout << "*****************************************\n";
    std::cout << "*** Tracking configuration parameters ***\n";
    std::cout << "*****************************************\n";
    std::cout << "Signal: " << System_and_Signal << "\n";
    std::cout << "implementation: " << config->property("Tracking.implementation", std::string("undefined")) << " \n";
    std::cout << "pll_bw_hz: " << config->property("Tracking.pll_bw_hz", 0.0) << " Hz\n";
    std::cout << "dll_bw_hz: " << config->property("Tracking.dll_bw_hz", 0.0) << " Hz\n";
    std::cout << "pll_bw_narrow_hz: " << config->property("Tracking.pll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "dll_bw_narrow_hz: " << config->property("Tracking.dll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "extend_correlation_symbols: " << config->property("Tracking.extend_correlation_symbols", 0) << " Symbols\n";
    std::cout << "*****************************************\n";
    std::cout << "*****************************************\n";
}


bool TrackingPullInTest::acquire_signal(int SV_ID)
{
    // 1. Setup GNU Radio flowgraph (file_source -> Acquisition_10m)
    gr::top_block_sptr top_block;
    top_block = gr::make_top_block("Acquisition test");

    // Satellite signal definition
    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.Channel_ID = 0;


    config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));
    // Enable automatic resampler for the acquisition, if required
    if (FLAGS_use_acquisition_resampler == true)
        {
            config->set_property("GNSS-SDR.use_acquisition_resampler", "true");
        }
    config->set_property("Acquisition.blocking_on_standby", "true");
    config->set_property("Acquisition.blocking", "true");
    config->set_property("Acquisition.dump", "false");
    config->set_property("Acquisition.dump_filename", "./data/acquisition.dat");
    config->set_property("Acquisition.use_CFAR_algorithm", "false");

    std::shared_ptr<AcquisitionInterface> acquisition;

    std::string System_and_Signal;
    std::string signal;
    //create the correspondign acquisition block according to the desired tracking signal
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking")
        {
            tmp_gnss_synchro.System = 'G';
            signal = "1C";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "GPS L1 CA";
            config->set_property("Acquisition.max_dwells", std::to_string(FLAGS_external_signal_acquisition_dwells));
            //acquisition = std::make_shared<GpsL1CaPcpsAcquisitionFineDoppler>(config.get(), "Acquisition", 1, 0);
            acquisition = std::make_shared<GpsL1CaPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking")
        {
            tmp_gnss_synchro.System = 'E';
            signal = "1B";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "Galileo E1B";
            config->set_property("Acquisition.max_dwells", std::to_string(FLAGS_external_signal_acquisition_dwells));
            acquisition = std::make_shared<GalileoE1PcpsAmbiguousAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else if (implementation == "GPS_L2_M_DLL_PLL_Tracking")
        {
            tmp_gnss_synchro.System = 'G';
            signal = "2S";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "GPS L2CM";
            config->set_property("Acquisition.max_dwells", std::to_string(FLAGS_external_signal_acquisition_dwells));
            acquisition = std::make_shared<GpsL2MPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_b")
        {
            tmp_gnss_synchro.System = 'E';
            signal = "5X";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "Galileo E5a";
            config->set_property("Acquisition_5X.coherent_integration_time_ms", "1");
            config->set_property("Acquisition.max_dwells", std::to_string(FLAGS_external_signal_acquisition_dwells));
            config->set_property("Acquisition.CAF_window_hz", "0");  //  **Only for E5a** Resolves doppler ambiguity averaging the specified BW in the winner code delay. If set to 0 CAF filter is deactivated. Recommended value 3000 Hz
            config->set_property("Acquisition.Zero_padding", "0");   //**Only for E5a** Avoids power loss and doppler ambiguity in bit transitions by correlating one code with twice the input data length, ensuring that at least one full code is present without transitions. If set to 1 it is ON, if set to 0 it is OFF.
            config->set_property("Acquisition.bit_transition_flag", "false");
            acquisition = std::make_shared<GalileoE5aNoncoherentIQAcquisitionCaf>(config.get(), "Acquisition", 1, 0);
        }

    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking")
        {
            tmp_gnss_synchro.System = 'E';
            signal = "5X";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "Galileo E5a";
            config->set_property("Acquisition.max_dwells", std::to_string(FLAGS_external_signal_acquisition_dwells));
            acquisition = std::make_shared<GalileoE5aPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else if (implementation == "GPS_L5_DLL_PLL_Tracking")
        {
            tmp_gnss_synchro.System = 'G';
            signal = "L5";
            const char* str = signal.c_str();                                  // get a C style null terminated string
            std::memcpy(static_cast<void*>(tmp_gnss_synchro.Signal), str, 3);  // copy string into synchro char array: 2 char + null
            tmp_gnss_synchro.PRN = SV_ID;
            System_and_Signal = "GPS L5I";
            config->set_property("Acquisition.max_dwells", std::to_string(FLAGS_external_signal_acquisition_dwells));
            acquisition = std::make_shared<GpsL5iPcpsAcquisition>(config.get(), "Acquisition", 1, 0);
        }
    else
        {
            std::cout << "The test can not run with the selected tracking implementation\n ";
            throw(std::exception());
        }

    acquisition->set_gnss_synchro(&tmp_gnss_synchro);
    acquisition->set_channel(0);
    acquisition->set_doppler_max(config->property("Acquisition.doppler_max", FLAGS_external_signal_acquisition_doppler_max_hz));
    acquisition->set_doppler_step(config->property("Acquisition.doppler_step", FLAGS_external_signal_acquisition_doppler_step_hz));
    acquisition->set_threshold(config->property("Acquisition.threshold", FLAGS_external_signal_acquisition_threshold));
    acquisition->init();
    acquisition->set_local_code();
    acquisition->set_state(1);  // Ensure that acquisition starts at the first sample
    acquisition->connect(top_block);

    gr::blocks::file_source::sptr file_source;
    std::string file = FLAGS_signal_file;
    const char* file_name = file.c_str();
    file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);
    file_source->seek(2 * FLAGS_skip_samples, SEEK_SET);  //skip head. ibyte, two bytes per complex sample
    gr::blocks::interleaved_char_to_complex::sptr gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();
    //gr::blocks::head::sptr head_samples = gr::blocks::head::make(sizeof(gr_complex), baseband_sampling_freq * FLAGS_duration);

    top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);

    // Enable automatic resampler for the acquisition, if required
    if (FLAGS_use_acquisition_resampler == true)
        {
            //create acquisition resamplers if required
            double resampler_ratio = 1.0;

            double opt_fs = baseband_sampling_freq;
            //find the signal associated to this channel
            switch (mapStringValues_[signal])
                {
                case evGPS_1C:
                    opt_fs = GPS_L1_CA_OPT_ACQ_FS_HZ;
                    break;
                case evGPS_2S:
                    opt_fs = GPS_L2C_OPT_ACQ_FS_HZ;
                    break;
                case evGPS_L5:
                    opt_fs = GPS_L5_OPT_ACQ_FS_HZ;
                    break;
                case evSBAS_1C:
                    opt_fs = GPS_L1_CA_OPT_ACQ_FS_HZ;
                    break;
                case evGAL_1B:
                    opt_fs = GALILEO_E1_OPT_ACQ_FS_HZ;
                    break;
                case evGAL_5X:
                    opt_fs = GALILEO_E5A_OPT_ACQ_FS_HZ;
                    break;
                case evGLO_1G:
                    opt_fs = baseband_sampling_freq;
                    break;
                case evGLO_2G:
                    opt_fs = baseband_sampling_freq;
                    break;
                }
            if (opt_fs < baseband_sampling_freq)
                {
                    resampler_ratio = baseband_sampling_freq / opt_fs;
                    int decimation = floor(resampler_ratio);
                    while (baseband_sampling_freq % decimation > 0)
                        {
                            decimation--;
                        };
                    double acq_fs = baseband_sampling_freq / decimation;

                    if (decimation > 1)
                        {
                            //create a FIR low pass filter
                            std::vector<float> taps;
                            taps = gr::filter::firdes::low_pass(1.0,
                                baseband_sampling_freq,
                                acq_fs / 2.1,
                                acq_fs / 10,
                                gr::filter::firdes::win_type::WIN_HAMMING);
                            std::cout << "Enabled decimation low pass filter with " << taps.size() << " taps and decimation factor of " << decimation << std::endl;
                            acquisition->set_resampler_latency((taps.size() - 1) / 2);
                            gr::basic_block_sptr fir_filter_ccf_ = gr::filter::fir_filter_ccf::make(decimation, taps);
                            top_block->connect(gr_interleaved_char_to_complex, 0, fir_filter_ccf_, 0);
                            top_block->connect(fir_filter_ccf_, 0, acquisition->get_left_block(), 0);
                        }
                    else
                        {
                            std::cout << "Disabled acquisition resampler because the input sampling frequency is too low\n";
                            top_block->connect(gr_interleaved_char_to_complex, 0, acquisition->get_left_block(), 0);
                        }
                }
            else
                {
                    std::cout << "Disabled acquisition resampler because the input sampling frequency is too low\n";
                    top_block->connect(gr_interleaved_char_to_complex, 0, acquisition->get_left_block(), 0);
                }
        }
    else
        {
            top_block->connect(gr_interleaved_char_to_complex, 0, acquisition->get_left_block(), 0);
            //top_block->connect(head_samples, 0, acquisition->get_left_block(), 0);
        }


    boost::shared_ptr<Acquisition_msg_rx> msg_rx;
    try
        {
            msg_rx = Acquisition_msg_rx_make();
        }
    catch (const std::exception& e)
        {
            std::cout << "Failure connecting the message port system: " << e.what() << std::endl;
            exit(0);
        }

    msg_rx->top_block = top_block;
    top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    // 5. Run the flowgraph
    // Get visible GPS satellites (positive acquisitions with Doppler measurements)
    // record startup time
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds;
    start = std::chrono::system_clock::now();

    bool start_msg = true;

    doppler_measurements_map.clear();
    code_delay_measurements_map.clear();
    acq_samplestamp_map.clear();

    unsigned int MAX_PRN_IDX = 0;

    switch (tmp_gnss_synchro.System)
        {
        case 'G':
            MAX_PRN_IDX = 33;
            break;
        case 'E':
            MAX_PRN_IDX = 37;
            break;
        default:
            MAX_PRN_IDX = 33;
        }

    for (unsigned int PRN = 1; PRN < MAX_PRN_IDX; PRN++)
        {
            tmp_gnss_synchro.PRN = PRN;
            acquisition->set_gnss_synchro(&tmp_gnss_synchro);
            acquisition->init();
            acquisition->set_local_code();
            acquisition->reset();
            acquisition->set_state(1);
            msg_rx->rx_message = 0;
            top_block->run();
            if (start_msg == true)
                {
                    std::cout << "Reading external signal file: " << FLAGS_signal_file << std::endl;
                    std::cout << "Searching for " << System_and_Signal << " Satellites..." << std::endl;
                    std::cout << "[";
                    start_msg = false;
                }
            while (msg_rx->rx_message == 0)
                {
                    usleep(100000);
                }
            if (msg_rx->rx_message == 1)
                {
                    std::cout << " " << PRN << " ";
                    doppler_measurements_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_doppler_hz));
                    code_delay_measurements_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_delay_samples));
                    acq_samplestamp_map.insert(std::pair<int, double>(PRN, tmp_gnss_synchro.Acq_samplestamp_samples));
                }
            else
                {
                    std::cout << " . ";
                }
            top_block->stop();
            file_source->seek(2 * FLAGS_skip_samples, SEEK_SET);  //skip head. ibyte, two bytes per complex sample
            std::cout.flush();
        }
    std::cout << "]" << std::endl;
    std::cout << "-------------------------------------------\n";

    for (auto& x : doppler_measurements_map)
        {
            std::cout << "DETECTED SATELLITE " << System_and_Signal << " PRN: " << x.first << " with Doppler: " << x.second << " [Hz], code phase: " << code_delay_measurements_map.at(x.first) << " [samples] at signal SampleStamp " << acq_samplestamp_map.at(x.first) << "\n";
        }

    // report the elapsed time
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "Total signal acquisition run time "
              << elapsed_seconds.count()
              << " [seconds]" << std::endl;
    return true;
}

TEST_F(TrackingPullInTest, ValidationOfResults)
{
    //*************************************************
    //***** STEP 1: Prepare the parameters sweep ******
    //*************************************************
    std::vector<double>
        acq_doppler_error_hz_values;
    std::vector<std::vector<double>> acq_delay_error_chips_values;  //vector of vector

    for (double doppler_hz = FLAGS_acq_Doppler_error_hz_start; doppler_hz >= FLAGS_acq_Doppler_error_hz_stop; doppler_hz = doppler_hz + FLAGS_acq_Doppler_error_hz_step)
        {
            acq_doppler_error_hz_values.push_back(doppler_hz);
            std::vector<double> tmp_vector;
            //Code Delay Sweep
            for (double code_delay_chips = FLAGS_acq_Delay_error_chips_start; code_delay_chips >= FLAGS_acq_Delay_error_chips_stop; code_delay_chips = code_delay_chips + FLAGS_acq_Delay_error_chips_step)
                {
                    tmp_vector.push_back(code_delay_chips);
                }
            acq_delay_error_chips_values.push_back(tmp_vector);
        }


    //***********************************************************
    //***** STEP 2: Generate the input signal (if required) *****
    //***********************************************************
    std::vector<double> generator_CN0_values;
    if (FLAGS_enable_external_signal_file)
        {
            generator_CN0_values.push_back(999);  // an external input signal capture is selected, no CN0 information available
        }
    else
        {
            if (FLAGS_CN0_dBHz_start == FLAGS_CN0_dBHz_stop)
                {
                    generator_CN0_values.push_back(FLAGS_CN0_dBHz_start);
                }
            else
                {
                    for (double cn0 = FLAGS_CN0_dBHz_start; cn0 > FLAGS_CN0_dBHz_stop; cn0 = cn0 - FLAGS_CN0_dB_step)
                        {
                            generator_CN0_values.push_back(cn0);
                        }
                }
        }

    // use generator or use an external capture file
    if (FLAGS_enable_external_signal_file)
        {
            //create and configure an acquisition block and perform an acquisition to obtain the synchronization parameters
            ASSERT_EQ(acquire_signal(FLAGS_test_satellite_PRN), true);
            bool found_satellite = doppler_measurements_map.find(FLAGS_test_satellite_PRN) != doppler_measurements_map.end();
            EXPECT_TRUE(found_satellite) << "Error: satellite SV: " << FLAGS_test_satellite_PRN << " is not acquired";
            if (!found_satellite)
                {
                    return;
                }
        }
    else
        {
            for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
                {
                    // Configure the signal generator
                    configure_generator(generator_CN0_values.at(current_cn0_idx), current_cn0_idx);
                    // Generate signal raw signal samples and observations RINEX file
                    if (FLAGS_disable_generator == false)
                        {
                            generate_signal();
                        }
                }
        }


    configure_receiver(FLAGS_PLL_bw_hz_start,
        FLAGS_DLL_bw_hz_start,
        FLAGS_PLL_narrow_bw_hz,
        FLAGS_DLL_narrow_bw_hz,
        FLAGS_extend_correlation_symbols);

    //******************************************************************************************
    //***** Obtain the initial signal sinchronization parameters (emulating an acquisition) ****
    //******************************************************************************************
    int test_satellite_PRN = 0;
    double true_acq_doppler_hz = 0.0;
    double true_acq_delay_samples = 0.0;
    uint64_t acq_samplestamp_samples = 0;

    Tracking_True_Obs_Reader true_obs_data;
    if (!FLAGS_enable_external_signal_file)
        {
            test_satellite_PRN = FLAGS_test_satellite_PRN;
            std::string true_obs_file = std::string("./gps_l1_ca_obs_prn");
            true_obs_file.append(std::to_string(test_satellite_PRN));
            true_obs_file.append(".dat");
            true_obs_data.close_obs_file();
            ASSERT_EQ(true_obs_data.open_obs_file(true_obs_file), true) << "Failure opening true observables file";
            // load acquisition data based on the first epoch of the true observations
            ASSERT_EQ(true_obs_data.read_binary_obs(), true)
                << "Failure reading true tracking dump file." << std::endl
                << "Maybe sat PRN #" + std::to_string(FLAGS_test_satellite_PRN) +
                       " is not available?";
            std::cout << "Testing satellite PRN=" << test_satellite_PRN << std::endl;
            std::cout << "True Initial Doppler " << true_obs_data.doppler_l1_hz << " [Hz], true Initial code delay [Chips]=" << true_obs_data.prn_delay_chips << "[Chips]" << std::endl;
            true_acq_doppler_hz = true_obs_data.doppler_l1_hz;
            true_acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_obs_data.prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * static_cast<double>(baseband_sampling_freq) * GPS_L1_CA_CODE_PERIOD;
            acq_samplestamp_samples = 0;
        }
    else
        {
            true_acq_doppler_hz = doppler_measurements_map.find(FLAGS_test_satellite_PRN)->second;
            true_acq_delay_samples = code_delay_measurements_map.find(FLAGS_test_satellite_PRN)->second;
            acq_samplestamp_samples = 0;
            std::cout << "Estimated Initial Doppler " << true_acq_doppler_hz
                      << " [Hz], estimated Initial code delay " << true_acq_delay_samples << " [Samples]"
                      << " Acquisition SampleStamp is " << acq_samplestamp_map.find(FLAGS_test_satellite_PRN)->second << std::endl;
        }

    // create the msg queue for valve

    queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    long long int acq_to_trk_delay_samples = ceil(static_cast<double>(FLAGS_fs_gen_sps) * FLAGS_acq_to_trk_delay_s);
    auto resetable_valve_ = gnss_sdr_make_valve(sizeof(gr_complex), acq_to_trk_delay_samples, queue, false);

    //CN0 LOOP
    std::vector<std::vector<double>> pull_in_results_v_v;

    for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
        {
            std::vector<double> pull_in_results_v;
            for (unsigned int current_acq_doppler_error_idx = 0; current_acq_doppler_error_idx < acq_doppler_error_hz_values.size(); current_acq_doppler_error_idx++)
                {
                    for (unsigned int current_acq_code_error_idx = 0; current_acq_code_error_idx < acq_delay_error_chips_values.at(current_acq_doppler_error_idx).size(); current_acq_code_error_idx++)
                        {
                            gnss_synchro.Acq_samplestamp_samples = acq_samplestamp_samples;
                            //simulate a Doppler error in acquisition
                            gnss_synchro.Acq_doppler_hz = true_acq_doppler_hz + acq_doppler_error_hz_values.at(current_acq_doppler_error_idx);
                            //simulate Code Delay error in acquisition
                            gnss_synchro.Acq_delay_samples = true_acq_delay_samples + (acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx) / GPS_L1_CA_CODE_RATE_HZ) * static_cast<double>(baseband_sampling_freq);

                            //create flowgraph
                            top_block = gr::make_top_block("Tracking test");
                            std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking", config->property("Tracking.implementation", std::string("undefined")), 1, 1);
                            std::shared_ptr<TrackingInterface> tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);
                            boost::shared_ptr<TrackingPullInTest_msg_rx> msg_rx = TrackingPullInTest_msg_rx_make();


                            ASSERT_NO_THROW({
                                tracking->set_channel(gnss_synchro.Channel_ID);
                            }) << "Failure setting channel.";

                            ASSERT_NO_THROW({
                                tracking->set_gnss_synchro(&gnss_synchro);
                            }) << "Failure setting gnss_synchro.";

                            ASSERT_NO_THROW({
                                tracking->connect(top_block);
                            }) << "Failure connecting tracking to the top_block.";

                            std::string file;
                            ASSERT_NO_THROW({
                                if (!FLAGS_enable_external_signal_file)
                                    {
                                        file = "./" + filename_raw_data + std::to_string(current_cn0_idx);
                                    }
                                else
                                    {
                                        file = FLAGS_signal_file;
                                    }
                                const char* file_name = file.c_str();
                                gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);
                                gr::blocks::interleaved_char_to_complex::sptr gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();
                                gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
                                gr::blocks::head::sptr head_samples = gr::blocks::head::make(sizeof(gr_complex), baseband_sampling_freq * FLAGS_duration);
                                top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
                                top_block->connect(gr_interleaved_char_to_complex, 0, head_samples, 0);
                                if (acq_to_trk_delay_samples > 0)
                                    {
                                        top_block->connect(head_samples, 0, resetable_valve_, 0);
                                        top_block->connect(resetable_valve_, 0, tracking->get_left_block(), 0);
                                    }
                                else
                                    {
                                        top_block->connect(head_samples, 0, tracking->get_left_block(), 0);
                                    }
                                top_block->connect(tracking->get_right_block(), 0, sink, 0);
                                top_block->msg_connect(tracking->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
                                file_source->seek(2 * FLAGS_skip_samples, 0);  //skip head. ibyte, two bytes per complex sample
                            }) << "Failure connecting the blocks of tracking test.";


                            //********************************************************************
                            //***** STEP 5: Perform the signal tracking and read the results *****
                            //********************************************************************
                            std::cout << "--- START TRACKING WITH PULL-IN ERROR: " << acq_doppler_error_hz_values.at(current_acq_doppler_error_idx) << " [Hz] and " << acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx) << " [Chips] ---" << std::endl;
                            std::chrono::time_point<std::chrono::system_clock> start, end;
                            if (acq_to_trk_delay_samples > 0)
                                {
                                    EXPECT_NO_THROW({
                                        start = std::chrono::system_clock::now();
                                        std::cout << "--- SIMULATING A PULL-IN DELAY OF " << FLAGS_acq_to_trk_delay_s << " SECONDS ---\n";
                                        top_block->start();
                                        std::cout << " Waiting for valve...\n";
                                        //wait the valve message indicating the circulation of the amount of samples of the delay
                                        pmt::pmt_t msg;
                                        queue->wait_and_pop(msg);
                                        std::cout << " Starting tracking...\n";
                                        tracking->start_tracking();
                                        resetable_valve_->open_valve();
                                        std::cout << " Waiting flowgraph..\n";
                                        top_block->wait();
                                        end = std::chrono::system_clock::now();
                                    }) << "Failure running the top_block.";
                                }
                            else
                                {
                                    tracking->start_tracking();
                                    std::chrono::time_point<std::chrono::system_clock> start, end;
                                    EXPECT_NO_THROW({
                                        start = std::chrono::system_clock::now();
                                        top_block->run();  // Start threads and wait
                                        end = std::chrono::system_clock::now();
                                    }) << "Failure running the top_block.";
                                }

                            std::chrono::duration<double> elapsed_seconds = end - start;
                            std::cout << "Signal tracking completed in " << elapsed_seconds.count() << " seconds" << std::endl;

                            pull_in_results_v.push_back(msg_rx->rx_message != 3);  //save last asynchronous tracking message in order to detect a loss of lock

                            //********************************
                            //***** STEP 7: Plot results *****
                            //********************************
                            if (FLAGS_plot_detail_level >= 2 and FLAGS_show_plots)
                                {
                                    //load the measured values
                                    Tracking_Dump_Reader trk_dump;
                                    ASSERT_EQ(trk_dump.open_obs_file(std::string("./tracking_ch_0.dat")), true)
                                        << "Failure opening tracking dump file";

                                    int64_t n_measured_epochs = trk_dump.num_epochs();
                                    //todo: use vectors instead
                                    arma::vec trk_timestamp_s = arma::zeros(n_measured_epochs, 1);
                                    arma::vec trk_acc_carrier_phase_cycles = arma::zeros(n_measured_epochs, 1);
                                    arma::vec trk_Doppler_Hz = arma::zeros(n_measured_epochs, 1);
                                    arma::vec trk_prn_delay_chips = arma::zeros(n_measured_epochs, 1);
                                    std::vector<double> timestamp_s;
                                    std::vector<double> prompt;
                                    std::vector<double> early;
                                    std::vector<double> late;
                                    std::vector<double> v_early;
                                    std::vector<double> v_late;
                                    std::vector<double> promptI;
                                    std::vector<double> promptQ;
                                    std::vector<double> CN0_dBHz;
                                    std::vector<double> Doppler;
                                    int64_t epoch_counter = 0;
                                    while (trk_dump.read_binary_obs())
                                        {
                                            trk_timestamp_s(epoch_counter) = static_cast<double>(trk_dump.PRN_start_sample_count) / static_cast<double>(baseband_sampling_freq);
                                            trk_acc_carrier_phase_cycles(epoch_counter) = trk_dump.acc_carrier_phase_rad / GPS_TWO_PI;
                                            trk_Doppler_Hz(epoch_counter) = trk_dump.carrier_doppler_hz;
                                            double delay_chips = GPS_L1_CA_CODE_LENGTH_CHIPS - GPS_L1_CA_CODE_LENGTH_CHIPS * (fmod((static_cast<double>(trk_dump.PRN_start_sample_count) + trk_dump.aux1) / static_cast<double>(baseband_sampling_freq), 1.0e-3) / 1.0e-3);

                                            trk_prn_delay_chips(epoch_counter) = delay_chips;

                                            timestamp_s.push_back(trk_timestamp_s(epoch_counter));
                                            prompt.push_back(trk_dump.abs_P);
                                            early.push_back(trk_dump.abs_E);
                                            late.push_back(trk_dump.abs_L);
                                            v_early.push_back(trk_dump.abs_VE);
                                            v_late.push_back(trk_dump.abs_VL);
                                            promptI.push_back(trk_dump.prompt_I);
                                            promptQ.push_back(trk_dump.prompt_Q);
                                            CN0_dBHz.push_back(trk_dump.CN0_SNV_dB_Hz);
                                            Doppler.push_back(trk_dump.carrier_doppler_hz);
                                            epoch_counter++;
                                        }


                                    const std::string gnuplot_executable(FLAGS_gnuplot_executable);
                                    if (gnuplot_executable.empty())
                                        {
                                            std::cout << "WARNING: Although the flag show_plots has been set to TRUE," << std::endl;
                                            std::cout << "gnuplot has not been found in your system." << std::endl;
                                            std::cout << "Test results will not be plotted." << std::endl;
                                        }
                                    else
                                        {
                                            try
                                                {
                                                    fs::path p(gnuplot_executable);
                                                    fs::path dir = p.parent_path();
                                                    const std::string& gnuplot_path = dir.native();
                                                    Gnuplot::set_GNUPlotPath(gnuplot_path);
                                                    auto decimate = static_cast<unsigned int>(FLAGS_plot_decimate);

                                                    if (FLAGS_plot_detail_level >= 2 and FLAGS_show_plots)
                                                        {
                                                            Gnuplot g1("linespoints");
                                                            g1.showonscreen();  // window output
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g1.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, " + "PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g1.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips], PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }

                                                            g1.set_grid();
                                                            g1.set_xlabel("Time [s]");
                                                            g1.set_ylabel("Correlators' output");
                                                            //g1.cmd("set key box opaque");
                                                            g1.plot_xy(trk_timestamp_s, prompt, "Prompt", decimate);
                                                            g1.plot_xy(trk_timestamp_s, early, "Early", decimate);
                                                            g1.plot_xy(trk_timestamp_s, late, "Late", decimate);
                                                            if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking")
                                                                {
                                                                    g1.plot_xy(trk_timestamp_s, v_early, "Very Early", decimate);
                                                                    g1.plot_xy(trk_timestamp_s, v_late, "Very Late", decimate);
                                                                }
                                                            g1.set_legend();
                                                            g1.savetops("Correlators_outputs");

                                                            Gnuplot g2("points");
                                                            g2.showonscreen();  // window output
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g2.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz Constellation " + "PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g2.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips], PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }

                                                            g2.set_grid();
                                                            g2.set_xlabel("Inphase");
                                                            g2.set_ylabel("Quadrature");
                                                            //g2.cmd("set size ratio -1");
                                                            g2.plot_xy(promptI, promptQ);
                                                            g2.savetops("Constellation");

                                                            Gnuplot g3("linespoints");
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g3.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g3.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips] PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            g3.set_grid();
                                                            g3.set_xlabel("Time [s]");
                                                            g3.set_ylabel("Reported CN0 [dB-Hz]");
                                                            g3.cmd("set key box opaque");

                                                            g3.plot_xy(trk_timestamp_s, CN0_dBHz,
                                                                std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + "[dB-Hz]", decimate);

                                                            g3.set_legend();
                                                            g3.savetops("CN0_output");

                                                            g3.showonscreen();  // window output

                                                            Gnuplot g4("linespoints");
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g4.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g4.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips] PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            g4.set_grid();
                                                            g4.set_xlabel("Time [s]");
                                                            g4.set_ylabel("Estimated Doppler [Hz]");
                                                            g4.cmd("set key box opaque");

                                                            g4.plot_xy(trk_timestamp_s, Doppler,
                                                                std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + "[dB-Hz]", decimate);

                                                            g4.set_legend();
                                                            g4.savetops("Doppler");

                                                            g4.showonscreen();  // window output
                                                        }
                                                }
                                            catch (const GnuplotException& ge)
                                                {
                                                    std::cout << ge.what() << std::endl;
                                                }
                                        }
                                }  //end plot

                        }  //end acquisition Delay errors loop
                }          //end acquisition Doppler errors loop
            pull_in_results_v_v.push_back(pull_in_results_v);


        }  //end CN0 LOOP
           //build the mesh grid
    std::vector<double> doppler_error_mesh;
    std::vector<double> code_delay_error_mesh;
    for (unsigned int current_acq_doppler_error_idx = 0; current_acq_doppler_error_idx < acq_doppler_error_hz_values.size(); current_acq_doppler_error_idx++)
        {
            for (unsigned int current_acq_code_error_idx = 0; current_acq_code_error_idx < acq_delay_error_chips_values.at(current_acq_doppler_error_idx).size(); current_acq_code_error_idx++)
                {
                    doppler_error_mesh.push_back(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx));
                    code_delay_error_mesh.push_back(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx));
                }
        }

    for (unsigned int current_cn0_idx = 0; current_cn0_idx < generator_CN0_values.size(); current_cn0_idx++)
        {
            std::vector<double> pull_in_result_mesh;
            pull_in_result_mesh = pull_in_results_v_v.at(current_cn0_idx);
            //plot grid
            Gnuplot g4("points palette pointsize 2 pointtype 7");
            if (FLAGS_show_plots)
                {
                    g4.showonscreen();  // window output
                }
            else
                {
                    g4.disablescreen();
                }
            g4.cmd(R"(set palette defined ( 0 "black", 1 "green" ))");
            g4.cmd("set key off");
            g4.cmd("set view map");
            std::string title;
            if (!FLAGS_enable_external_signal_file)
                {
                    title = std::string("Tracking Pull-in result grid at CN0:" + std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + " [dB-Hz], PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz].");
                }
            else
                {
                    title = std::string("Tracking Pull-in result grid, PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                }

            g4.set_title(title);
            g4.set_grid();
            g4.set_xlabel("Acquisition Doppler error [Hz]");
            g4.set_ylabel("Acquisition Code Delay error [Chips]");
            g4.cmd("set cbrange[0:1]");
            g4.plot_xyz(doppler_error_mesh,
                code_delay_error_mesh,
                pull_in_result_mesh);
            g4.set_legend();
            if (!FLAGS_enable_external_signal_file)
                {
                    g4.savetops("trk_pull_in_grid_" + std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))));
                    g4.savetopdf("trk_pull_in_grid_" + std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))), 12);
                }
            else
                {
                    g4.savetops("trk_pull_in_grid_external_file");
                    g4.savetopdf("trk_pull_in_grid_external_file", 12);
                }
        }
}
