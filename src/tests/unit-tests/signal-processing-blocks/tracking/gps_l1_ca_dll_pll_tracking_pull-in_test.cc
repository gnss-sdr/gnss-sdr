/*!
 * \file gps_l1_ca_dll_pll_tracking_test.cc
 * \brief  This class implements a tracking Pull-In test for GPS_L1_CA_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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

#include <chrono>
#include <unistd.h>
#include <vector>
#include <armadillo>
#include <boost/filesystem.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gtest/gtest.h>
#include "GPS_L1_CA.h"
#include "gnss_block_factory.h"
#include "tracking_interface.h"
#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "in_memory_configuration.h"
#include "tracking_true_obs_reader.h"
#include "tracking_dump_reader.h"
#include "signal_generator_flags.h"
#include "gnuplot_i.h"
#include "test_flags.h"
#include "tracking_tests_flags.h"


// ######## GNURADIO ACQUISITION BLOCK MESSAGE RECEVER #########
class Acquisition_msg_rx;

typedef boost::shared_ptr<Acquisition_msg_rx> Acquisition_msg_rx_sptr;

Acquisition_msg_rx_sptr Acquisition_msg_rx_make();


class Acquisition_msg_rx : public gr::block
{
private:
    friend Acquisition_msg_rx_sptr Acquisition_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    Acquisition_msg_rx();

public:
    int rx_message;
    ~Acquisition_msg_rx();  //!< Default destructor
};


Acquisition_msg_rx_sptr Acquisition_msg_rx_make()
{
    return Acquisition_msg_rx_sptr(new Acquisition_msg_rx());
}


void Acquisition_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            long int message = pmt::to_long(msg);
            rx_message = message;
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_acquisition Bad cast!\n";
            rx_message = 0;
        }
}


Acquisition_msg_rx::Acquisition_msg_rx() : gr::block("Acquisition_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&Acquisition_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


Acquisition_msg_rx::~Acquisition_msg_rx() {}
// ######## GNURADIO TRACKING BLOCK MESSAGE RECEVER #########
class GpsL1CADllPllTrackingPullInTest_msg_rx;

typedef boost::shared_ptr<GpsL1CADllPllTrackingPullInTest_msg_rx> GpsL1CADllPllTrackingPullInTest_msg_rx_sptr;

GpsL1CADllPllTrackingPullInTest_msg_rx_sptr GpsL1CADllPllTrackingPullInTest_msg_rx_make();

class GpsL1CADllPllTrackingPullInTest_msg_rx : public gr::block
{
private:
    friend GpsL1CADllPllTrackingPullInTest_msg_rx_sptr GpsL1CADllPllTrackingPullInTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL1CADllPllTrackingPullInTest_msg_rx();

public:
    int rx_message;
    ~GpsL1CADllPllTrackingPullInTest_msg_rx();  //!< Default destructor
};


GpsL1CADllPllTrackingPullInTest_msg_rx_sptr GpsL1CADllPllTrackingPullInTest_msg_rx_make()
{
    return GpsL1CADllPllTrackingPullInTest_msg_rx_sptr(new GpsL1CADllPllTrackingPullInTest_msg_rx());
}


void GpsL1CADllPllTrackingPullInTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            long int message = pmt::to_long(msg);
            rx_message = message;  //3 -> loss of lock
            //std::cout << "Received trk message: " << rx_message << std::endl;
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_tracking Bad cast!";
            rx_message = 0;
        }
}


GpsL1CADllPllTrackingPullInTest_msg_rx::GpsL1CADllPllTrackingPullInTest_msg_rx() : gr::block("GpsL1CADllPllTrackingPullInTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL1CADllPllTrackingPullInTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


GpsL1CADllPllTrackingPullInTest_msg_rx::~GpsL1CADllPllTrackingPullInTest_msg_rx()
{
}


// ###########################################################

class GpsL1CADllPllTrackingPullInTest : public ::testing::Test
{
public:
    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;
    std::string p6;
    std::string implementation = "GPS_L1_CA_DLL_PLL_Tracking";  //"GPS_L1_CA_DLL_PLL_C_Aid_Tracking";

    const int baseband_sampling_freq = FLAGS_fs_gen_sps;

    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_signal_file;

    std::map<int, double> doppler_measurements_map;
    std::map<int, double> code_delay_measurements_map;
    std::map<int, unsigned long int> acq_samplestamp_map;

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

    GpsL1CADllPllTrackingPullInTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL1CADllPllTrackingPullInTest()
    {
    }

    void configure_receiver(double PLL_wide_bw_hz,
        double DLL_wide_bw_hz,
        double PLL_narrow_bw_hz,
        double DLL_narrow_bw_hz,
        int extend_correlation_symbols);

    bool acquire_GPS_L1CA_signal(int SV_ID);
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


int GpsL1CADllPllTrackingPullInTest::configure_generator(double CN0_dBHz, int file_idx)
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


int GpsL1CADllPllTrackingPullInTest::generate_signal()
{
    int child_status;

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], &p6[0], NULL};

    int pid;
    if ((pid = fork()) == -1)
        perror("fork err");
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


void GpsL1CADllPllTrackingPullInTest::configure_receiver(
    double PLL_wide_bw_hz,
    double DLL_wide_bw_hz,
    double PLL_narrow_bw_hz,
    double DLL_narrow_bw_hz,
    int extend_correlation_symbols)
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = FLAGS_test_satellite_PRN;

    config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));
    // Set Tracking
    config->set_property("Tracking_1C.implementation", implementation);
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("Tracking_1C.pll_bw_hz", std::to_string(PLL_wide_bw_hz));
    config->set_property("Tracking_1C.dll_bw_hz", std::to_string(DLL_wide_bw_hz));
    config->set_property("Tracking_1C.early_late_space_chips", "0.5");
    config->set_property("Tracking_1C.extend_correlation_symbols", std::to_string(extend_correlation_symbols));
    config->set_property("Tracking_1C.pll_bw_narrow_hz", std::to_string(PLL_narrow_bw_hz));
    config->set_property("Tracking_1C.dll_bw_narrow_hz", std::to_string(DLL_narrow_bw_hz));
    config->set_property("Tracking_1C.early_late_space_narrow_chips", "0.5");
    config->set_property("Tracking_1C.dump", "true");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");

    std::cout << "*****************************************\n";
    std::cout << "*** Tracking configuration parameters ***\n";
    std::cout << "*****************************************\n";
    std::cout << "pll_bw_hz: " << config->property("Tracking_1C.pll_bw_hz", 0.0) << " Hz\n";
    std::cout << "dll_bw_hz: " << config->property("Tracking_1C.dll_bw_hz", 0.0) << " Hz\n";
    std::cout << "pll_bw_narrow_hz: " << config->property("Tracking_1C.pll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "dll_bw_narrow_hz: " << config->property("Tracking_1C.dll_bw_narrow_hz", 0.0) << " Hz\n";
    std::cout << "extend_correlation_symbols: " << config->property("Tracking_1C.extend_correlation_symbols", 0) << " Symbols\n";
    std::cout << "*****************************************\n";
    std::cout << "*****************************************\n";
}


bool GpsL1CADllPllTrackingPullInTest::acquire_GPS_L1CA_signal(int SV_ID)
{
    // 1. Setup GNU Radio flowgraph (file_source -> Acquisition_10m)
    gr::top_block_sptr top_block;
    top_block = gr::make_top_block("Acquisition test");

    // Satellite signal definition
    Gnss_Synchro tmp_gnss_synchro;
    tmp_gnss_synchro.Channel_ID = 0;
    tmp_gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(tmp_gnss_synchro.Signal, 2, 0);
    tmp_gnss_synchro.PRN = SV_ID;

    config = std::make_shared<InMemoryConfiguration>();
    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    GNSSBlockFactory block_factory;
    GpsL1CaPcpsAcquisitionFineDoppler* acquisition;
    acquisition = new GpsL1CaPcpsAcquisitionFineDoppler(config.get(), "Acquisition", 1, 1);

    acquisition->set_channel(1);
    acquisition->set_gnss_synchro(&tmp_gnss_synchro);
    acquisition->set_threshold(config->property("Acquisition.threshold", 0.005));
    acquisition->set_doppler_max(config->property("Acquisition.doppler_max", 10000));
    acquisition->set_doppler_step(config->property("Acquisition.doppler_step", 250));

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

    gr::blocks::file_source::sptr file_source;
    std::string file = FLAGS_signal_file;
    const char* file_name = file.c_str();
    file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);
    gr::blocks::interleaved_char_to_complex::sptr gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();
    gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
    top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
    top_block->connect(gr_interleaved_char_to_complex, 0, acquisition->get_left_block(), 0);
    top_block->msg_connect(acquisition->get_left_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

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

    for (unsigned int PRN = 1; PRN < 33; PRN++)
        {
            tmp_gnss_synchro.PRN = PRN;
            acquisition->set_gnss_synchro(&tmp_gnss_synchro);
            acquisition->init();
            acquisition->set_local_code();
            acquisition->reset();
            msg_rx->rx_message = 0;
            top_block->run();
            if (start_msg == true)
                {
                    std::cout << "Reading external signal file: " << FLAGS_signal_file << std::endl;
                    std::cout << "Searching for GPS Satellites in L1 band..." << std::endl;
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
            file_source->seek(0, 0);
            std::cout.flush();
        }
    std::cout << "]" << std::endl;

    // report the elapsed time
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "Total signal acquisition run time "
              << elapsed_seconds.count()
              << " [seconds]" << std::endl;
    return true;
}

TEST_F(GpsL1CADllPllTrackingPullInTest, ValidationOfResults)
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
            ASSERT_EQ(acquire_GPS_L1CA_signal(FLAGS_test_satellite_PRN), true);
            bool found_satellite = doppler_measurements_map.find(FLAGS_test_satellite_PRN) != doppler_measurements_map.end();
            EXPECT_TRUE(found_satellite) << "Error: satellite SV: " << FLAGS_test_satellite_PRN << " is not acquired";
            if (!found_satellite) return;
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
    unsigned long int acq_samplestamp_samples = 0;

    tracking_true_obs_reader true_obs_data;
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
            std::cout << "True Initial Doppler " << true_obs_data.doppler_l1_hz << "[Hz], true Initial code delay [Chips]=" << true_obs_data.prn_delay_chips << "[Chips]" << std::endl;
            true_acq_doppler_hz = true_obs_data.doppler_l1_hz;
            true_acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_obs_data.prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * static_cast<double>(baseband_sampling_freq) * GPS_L1_CA_CODE_PERIOD;
            acq_samplestamp_samples = 0;
        }
    else
        {
            true_acq_doppler_hz = doppler_measurements_map.find(FLAGS_test_satellite_PRN)->second;
            true_acq_delay_samples = code_delay_measurements_map.find(FLAGS_test_satellite_PRN)->second;
            acq_samplestamp_samples = 0;  //acq_samplestamp_map.find(FLAGS_test_satellite_PRN)->second;
            std::cout << "Estimated Initial Doppler " << true_acq_doppler_hz << "[Hz], estimated Initial code delay " << true_acq_delay_samples << " [Samples]" << std::endl;
        }
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
                            std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(config, "Tracking_1C", implementation, 1, 1);
                            std::shared_ptr<TrackingInterface> tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);
                            boost::shared_ptr<GpsL1CADllPllTrackingPullInTest_msg_rx> msg_rx = GpsL1CADllPllTrackingPullInTest_msg_rx_make();


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
                                top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
                                top_block->connect(gr_interleaved_char_to_complex, 0, tracking->get_left_block(), 0);
                                top_block->connect(tracking->get_right_block(), 0, sink, 0);
                                top_block->msg_connect(tracking->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

                                file_source->seek(acq_samplestamp_samples, 0);
                            }) << "Failure connecting the blocks of tracking test.";


                            //********************************************************************
                            //***** STEP 5: Perform the signal tracking and read the results *****
                            //********************************************************************
                            std::cout << "------------ START TRACKING -------------" << std::endl;
                            tracking->start_tracking();
                            std::chrono::time_point<std::chrono::system_clock> start, end;
                            EXPECT_NO_THROW({
                                start = std::chrono::system_clock::now();
                                top_block->run();  // Start threads and wait
                                end = std::chrono::system_clock::now();
                            }) << "Failure running the top_block.";

                            std::chrono::duration<double> elapsed_seconds = end - start;
                            std::cout << "Signal tracking completed in " << elapsed_seconds.count() << " seconds" << std::endl;


                            pull_in_results_v.push_back(msg_rx->rx_message != 3);  //save last asynchronous tracking message in order to detect a loss of lock

                            //********************************
                            //***** STEP 7: Plot results *****
                            //********************************
                            if (FLAGS_plot_detail_level >= 2 and FLAGS_show_plots)
                                {
                                    //load the measured values
                                    tracking_dump_reader trk_dump;
                                    ASSERT_EQ(trk_dump.open_obs_file(std::string("./tracking_ch_0.dat")), true)
                                        << "Failure opening tracking dump file";

                                    long int n_measured_epochs = trk_dump.num_epochs();
                                    //todo: use vectors instead
                                    arma::vec trk_timestamp_s = arma::zeros(n_measured_epochs, 1);
                                    arma::vec trk_acc_carrier_phase_cycles = arma::zeros(n_measured_epochs, 1);
                                    arma::vec trk_Doppler_Hz = arma::zeros(n_measured_epochs, 1);
                                    arma::vec trk_prn_delay_chips = arma::zeros(n_measured_epochs, 1);
                                    std::vector<double> timestamp_s;
                                    std::vector<double> prompt;
                                    std::vector<double> early;
                                    std::vector<double> late;
                                    std::vector<double> promptI;
                                    std::vector<double> promptQ;
                                    std::vector<double> CN0_dBHz;
                                    long int epoch_counter = 0;
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
                                            promptI.push_back(trk_dump.prompt_I);
                                            promptQ.push_back(trk_dump.prompt_Q);
                                            CN0_dBHz.push_back(trk_dump.CN0_SNV_dB_Hz);

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
                                                    boost::filesystem::path p(gnuplot_executable);
                                                    boost::filesystem::path dir = p.parent_path();
                                                    std::string gnuplot_path = dir.native();
                                                    Gnuplot::set_GNUPlotPath(gnuplot_path);
                                                    unsigned int decimate = static_cast<unsigned int>(FLAGS_plot_decimate);

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
                                                                    g1.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips], PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }

                                                            g1.set_grid();
                                                            g1.set_xlabel("Time [s]");
                                                            g1.set_ylabel("Correlators' output");
                                                            //g1.cmd("set key box opaque");
                                                            g1.plot_xy(trk_timestamp_s, prompt, "Prompt", decimate);
                                                            g1.plot_xy(trk_timestamp_s, early, "Early", decimate);
                                                            g1.plot_xy(trk_timestamp_s, late, "Late", decimate);
                                                            g1.set_legend();
                                                            //g1.savetops("Correlators_outputs" + std::to_string(generator_CN0_values.at(current_cn0_idx)));
                                                            //g1.savetopdf("Correlators_outputs" + std::to_string(generator_CN0_values.at(current_cn0_idx)), 18);

                                                            Gnuplot g2("points");
                                                            g2.showonscreen();  // window output
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g2.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz Constellation " + "PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g2.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips], PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }

                                                            g2.set_grid();
                                                            g2.set_xlabel("Inphase");
                                                            g2.set_ylabel("Quadrature");
                                                            //g2.cmd("set size ratio -1");
                                                            g2.plot_xy(promptI, promptQ);
                                                            //g2.savetops("Constellation");
                                                            //g2.savetopdf("Constellation", 18);

                                                            Gnuplot g3("linespoints");
                                                            if (!FLAGS_enable_external_signal_file)
                                                                {
                                                                    g3.set_title(std::to_string(generator_CN0_values.at(current_cn0_idx)) + " dB-Hz, GPS L1 C/A tracking CN0 output (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            else
                                                                {
                                                                    g3.set_title("D_e=" + std::to_string(acq_doppler_error_hz_values.at(current_acq_doppler_error_idx)) + " [Hz] " + "T_e= " + std::to_string(acq_delay_error_chips_values.at(current_acq_doppler_error_idx).at(current_acq_code_error_idx)) + " [Chips] PLL/DLL BW: " + std::to_string(FLAGS_PLL_bw_hz_start) + "," + std::to_string(FLAGS_DLL_bw_hz_start) + " [Hz], GPS L1 C/A (PRN #" + std::to_string(FLAGS_test_satellite_PRN) + ")");
                                                                }
                                                            g3.set_grid();
                                                            g3.set_xlabel("Time [s]");
                                                            g3.set_ylabel("Reported CN0 [dB-Hz]");
                                                            g3.cmd("set key box opaque");

                                                            g3.plot_xy(trk_timestamp_s, CN0_dBHz,
                                                                std::to_string(static_cast<int>(round(generator_CN0_values.at(current_cn0_idx)))) + "[dB-Hz]", decimate);

                                                            g3.set_legend();
                                                            //g3.savetops("CN0_output");
                                                            //g3.savetopdf("CN0_output", 18);
                                                            g3.showonscreen();  // window output
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
            g4.cmd("set palette defined ( 0 \"black\", 1 \"green\" )");
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
