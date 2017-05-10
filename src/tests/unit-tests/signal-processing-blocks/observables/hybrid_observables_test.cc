/*!
 * \file hybrid_observables_test.cc
 * \brief  This class implements a tracking test for Galileo_E5a_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2015  (see AUTHORS file for a list of contributors)
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


#include <exception>
#include <cstring>
#include <ctime>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include <armadillo>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gtest/gtest.h>
#include "GPS_L1_CA.h"
#include "gnss_satellite.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "tracking_interface.h"
#include "telemetry_decoder_interface.h"
#include "in_memory_configuration.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_telemetry_decoder.h"
#include "tracking_true_obs_reader.h"
#include "true_observables_reader.h"
#include "tracking_dump_reader.h"
#include "observables_dump_reader.h"
#include "tlm_dump_reader.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "gps_l1_ca_dll_pll_c_aid_tracking.h"
#include "hybrid_observables.h"
#include "signal_generator_flags.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER FOR TRACKING MESSAGES #########
class HybridObservablesTest_msg_rx;

typedef boost::shared_ptr<HybridObservablesTest_msg_rx> HybridObservablesTest_msg_rx_sptr;

HybridObservablesTest_msg_rx_sptr HybridObservablesTest_msg_rx_make();

class HybridObservablesTest_msg_rx : public gr::block
{
private:
    friend HybridObservablesTest_msg_rx_sptr HybridObservablesTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    HybridObservablesTest_msg_rx();

public:
    int rx_message;
    ~HybridObservablesTest_msg_rx(); //!< Default destructor

};

HybridObservablesTest_msg_rx_sptr HybridObservablesTest_msg_rx_make()
{
    return HybridObservablesTest_msg_rx_sptr(new HybridObservablesTest_msg_rx());
}

void HybridObservablesTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
    {
            long int message = pmt::to_long(msg);
            rx_message = message;
    }
    catch(boost::bad_any_cast& e)
    {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
    }
}

HybridObservablesTest_msg_rx::HybridObservablesTest_msg_rx() :
            gr::block("HybridObservablesTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&HybridObservablesTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

HybridObservablesTest_msg_rx::~HybridObservablesTest_msg_rx()
{}


// ###########################################################


// ######## GNURADIO BLOCK MESSAGE RECEVER FOR TLM MESSAGES #########
class HybridObservablesTest_tlm_msg_rx;

typedef boost::shared_ptr<HybridObservablesTest_tlm_msg_rx> HybridObservablesTest_tlm_msg_rx_sptr;

HybridObservablesTest_tlm_msg_rx_sptr HybridObservablesTest_tlm_msg_rx_make();

class HybridObservablesTest_tlm_msg_rx : public gr::block
{
private:
    friend HybridObservablesTest_tlm_msg_rx_sptr HybridObservablesTest_tlm_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    HybridObservablesTest_tlm_msg_rx();

public:
    int rx_message;
    ~HybridObservablesTest_tlm_msg_rx(); //!< Default destructor

};

HybridObservablesTest_tlm_msg_rx_sptr HybridObservablesTest_tlm_msg_rx_make()
{
    return HybridObservablesTest_tlm_msg_rx_sptr(new HybridObservablesTest_tlm_msg_rx());
}

void HybridObservablesTest_tlm_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
    {
            long int message = pmt::to_long(msg);
            rx_message = message;
    }
    catch(boost::bad_any_cast& e)
    {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
    }
}

HybridObservablesTest_tlm_msg_rx::HybridObservablesTest_tlm_msg_rx() :
            gr::block("HybridObservablesTest_tlm_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&HybridObservablesTest_tlm_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

HybridObservablesTest_tlm_msg_rx::~HybridObservablesTest_tlm_msg_rx()
{}


// ###########################################################


class HybridObservablesTest: public ::testing::Test
{

public:
    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;

    const int baseband_sampling_freq = FLAGS_fs_gen_hz;

    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_filename_raw_data;

    int configure_generator();
    int generate_signal();
    void check_results(
            arma::vec true_ch0_dist_m, arma::vec true_ch1_dist_m,
            arma::vec true_ch0_tow_s,
            arma::vec measuded_ch0_Pseudorange_m,
            arma::vec measuded_ch1_Pseudorange_m,
            arma::vec measuded_ch0_RX_time_s);

    HybridObservablesTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro_ch0 = Gnss_Synchro();
        gnss_synchro_ch1 = Gnss_Synchro();
    }

    ~HybridObservablesTest()
    {}

    void configure_receiver();

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro_ch0;
    Gnss_Synchro gnss_synchro_ch1;
    size_t item_size;
};

int HybridObservablesTest::configure_generator()
{
    // Configure signal generator
    generator_binary = FLAGS_generator_binary;

    p1 = std::string("-rinex_nav_file=") + FLAGS_rinex_nav_file;
    if(FLAGS_dynamic_position.empty())
        {
            p2 = std::string("-static_position=") + FLAGS_static_position + std::string(",") + std::to_string(FLAGS_duration * 10);
        }
    else
        {
            p2 = std::string("-obs_pos_file=") + std::string(FLAGS_dynamic_position);
        }
    p3 = std::string("-rinex_obs_file=") + FLAGS_filename_rinex_obs; // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + FLAGS_filename_raw_data; // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq); //Baseband sampling frequency [MSps]
    return 0;
}


int HybridObservablesTest::generate_signal()
{
    int child_status;

    char *const parmList[] = { &generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], NULL };

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

    std::cout << "Signal and Observables RINEX and RAW files created."  << std::endl;
    return 0;
}


void HybridObservablesTest::configure_receiver()
{
    gnss_synchro_ch0.Channel_ID = 0;
    gnss_synchro_ch0.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro_ch0.Signal, 2, 0);
    gnss_synchro_ch0.PRN = FLAGS_test_satellite_PRN;

    gnss_synchro_ch1.Channel_ID = 1;
    gnss_synchro_ch1.System = 'G';
    signal.copy(gnss_synchro_ch1.Signal, 2, 0);
    gnss_synchro_ch1.PRN = FLAGS_test_satellite_PRN2;


    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(baseband_sampling_freq));

    // Set Tracking
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("Tracking_1C.if", "0");
    config->set_property("Tracking_1C.dump", "true");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");
    config->set_property("Tracking_1C.pll_bw_hz", "20.0");
    config->set_property("Tracking_1C.dll_bw_hz", "1.5");
    config->set_property("Tracking_1C.early_late_space_chips", "0.5");


    config->set_property("TelemetryDecoder_1C.dump","true");
    config->set_property("TelemetryDecoder_1C.decimation_factor","1");

    config->set_property("Observables.history_depth","500");
    config->set_property("Observables.dump","true");


}

void HybridObservablesTest::check_results(
        arma::vec true_ch0_dist_m,
        arma::vec true_ch1_dist_m,
        arma::vec true_ch0_tow_s,
        arma::vec measuded_ch0_Pseudorange_m,
        arma::vec measuded_ch1_Pseudorange_m,
        arma::vec measuded_ch0_RX_time_s)
{
    //1. True value interpolation to match the measurement times

    arma::vec true_ch0_dist_interp;
    arma::vec true_ch1_dist_interp;
    arma::interp1(true_ch0_tow_s, true_ch0_dist_m, measuded_ch0_RX_time_s, true_ch0_dist_interp);
    arma::interp1(true_ch0_tow_s, true_ch1_dist_m, measuded_ch0_RX_time_s, true_ch1_dist_interp);

    // generate delta pseudoranges
    std::cout<<"s1:"<<true_ch0_dist_m.size()<<std::endl;
    std::cout<<"s2:"<<true_ch1_dist_m.size()<<std::endl;
    std::cout<<"s3:"<<measuded_ch0_Pseudorange_m.size()<<std::endl;
    std::cout<<"s4:"<<measuded_ch1_Pseudorange_m.size()<<std::endl;
    arma::vec delta_true_dist_m = true_ch0_dist_interp-true_ch1_dist_interp;
    arma::vec delta_measured_dist_m = measuded_ch0_Pseudorange_m-measuded_ch1_Pseudorange_m;



    //2. RMSE
    arma::vec err;

    err = delta_measured_dist_m - delta_true_dist_m;
    arma::vec err2 = arma::square(err);
    double rmse = sqrt(arma::mean(err2));

    //3. Mean err and variance
    double error_mean = arma::mean(err);
    double error_var = arma::var(err);

    // 4. Peaks
    double max_error = arma::max(err);
    double min_error = arma::min(err);

    //5. report

    std::cout << std::setprecision(10) << "Delta Observables RMSE="
              << rmse << ", mean=" << error_mean
              << ", stdev=" << sqrt(error_var)
              << " (max,min)=" << max_error
              << "," << min_error
              << " [meters]" << std::endl;

    ASSERT_LT(rmse, 10E-3);
    ASSERT_LT(error_mean, 10E-3);
    ASSERT_GT(error_mean, 10E-3);
    ASSERT_LT(error_var, 10E-3);
    ASSERT_LT(max_error, 10E-3);
    ASSERT_GT(min_error, 10E-3);
}

TEST_F(HybridObservablesTest, ValidationOfResults)
{
    // Configure the signal generator
    configure_generator();

    // Generate signal raw signal samples and observations RINEX file
    if (FLAGS_disable_generator==false)
    {
        generate_signal();
    }

    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;

    configure_receiver();

    //open true observables log file written by the simulator
    tracking_true_obs_reader true_obs_data_ch0;
    tracking_true_obs_reader true_obs_data_ch1;
    int test_satellite_PRN = FLAGS_test_satellite_PRN;
    int test_satellite_PRN2 = FLAGS_test_satellite_PRN2;
    std::cout << "Testing satellite PRNs " << test_satellite_PRN <<","<<test_satellite_PRN << std::endl;
    std::string true_obs_file = std::string("./gps_l1_ca_obs_prn");
    true_obs_file.append(std::to_string(test_satellite_PRN));
    true_obs_file.append(".dat");
    ASSERT_NO_THROW({
        if (true_obs_data_ch0.open_obs_file(true_obs_file) == false)
            {
                throw std::exception();
            };
    }) << "Failure opening true observables file" << std::endl;

    true_obs_file = std::string("./gps_l1_ca_obs_prn");
    true_obs_file.append(std::to_string(test_satellite_PRN2));
    true_obs_file.append(".dat");
    ASSERT_NO_THROW({
        if (true_obs_data_ch1.open_obs_file(true_obs_file) == false)
            {
                throw std::exception();
            };
    }) << "Failure opening true observables file" << std::endl;

    top_block = gr::make_top_block("Telemetry_Decoder test");
    std::shared_ptr<TrackingInterface> tracking_ch0 = std::make_shared<GpsL1CaDllPllTracking>(config.get(), "Tracking_1C", 1, 1);
    //std::shared_ptr<TrackingInterface> tracking_ch1 = std::make_shared<GpsL1CaDllPllCAidTracking>(config.get(), "Tracking_1C", 1, 1);
    std::shared_ptr<TrackingInterface> tracking_ch1 = std::make_shared<GpsL1CaDllPllTracking>(config.get(), "Tracking_1C", 1, 1);
    //std::shared_ptr<TrackingInterface> tracking_ch1 = std::make_shared<GpsL1CaDllPllCAidTracking>(config.get(), "Tracking_1C", 1, 1);

    boost::shared_ptr<HybridObservablesTest_msg_rx> msg_rx_ch0 = HybridObservablesTest_msg_rx_make();
    boost::shared_ptr<HybridObservablesTest_msg_rx> msg_rx_ch1 = HybridObservablesTest_msg_rx_make();

    // load acquisition data based on the first epoch of the true observations
    ASSERT_NO_THROW({
        if (true_obs_data_ch0.read_binary_obs() == false)
        {
            throw std::exception();
        };
    })<< "Failure reading true observables file" << std::endl;

    ASSERT_NO_THROW({
        if (true_obs_data_ch1.read_binary_obs() == false)
        {
            throw std::exception();
        };
    })<< "Failure reading true observables file" << std::endl;

    //restart the epoch counter
    true_obs_data_ch0.restart();
    true_obs_data_ch1.restart();

    std::cout << "Initial Doppler [Hz]=" << true_obs_data_ch0.doppler_l1_hz << " Initial code delay [Chips]=" << true_obs_data_ch0.prn_delay_chips << std::endl;

    gnss_synchro_ch0.Acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_obs_data_ch0.prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD;
    gnss_synchro_ch0.Acq_doppler_hz = true_obs_data_ch0.doppler_l1_hz;
    gnss_synchro_ch0.Acq_samplestamp_samples = 0;

    std::cout << "Initial Doppler [Hz]=" << true_obs_data_ch1.doppler_l1_hz << " Initial code delay [Chips]=" << true_obs_data_ch1.prn_delay_chips << std::endl;

    gnss_synchro_ch1.Acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_obs_data_ch1.prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD;
    gnss_synchro_ch1.Acq_doppler_hz = true_obs_data_ch1.doppler_l1_hz;
    gnss_synchro_ch1.Acq_samplestamp_samples = 0;

    //telemetry decoders
    std::shared_ptr<TelemetryDecoderInterface> tlm_ch0(new GpsL1CaTelemetryDecoder(config.get(), "TelemetryDecoder_1C",1, 1));
    std::shared_ptr<TelemetryDecoderInterface> tlm_ch1(new GpsL1CaTelemetryDecoder(config.get(), "TelemetryDecoder_1C",1, 1));



    ASSERT_NO_THROW( {
        tlm_ch0->set_channel(0);
        tlm_ch1->set_channel(1);

        tlm_ch0->set_satellite(Gnss_Satellite(std::string("GPS"),gnss_synchro_ch0.PRN));
        tlm_ch1->set_satellite(Gnss_Satellite(std::string("GPS"),gnss_synchro_ch1.PRN));
    }) << "Failure setting gnss_synchro." << std::endl;

    boost::shared_ptr<HybridObservablesTest_tlm_msg_rx> tlm_msg_rx_ch1 = HybridObservablesTest_tlm_msg_rx_make();
    boost::shared_ptr<HybridObservablesTest_tlm_msg_rx> tlm_msg_rx_ch2 = HybridObservablesTest_tlm_msg_rx_make();

    //Observables
    std::shared_ptr<ObservablesInterface> observables(new HybridObservables(config.get(), "Observables",2, 2));


    ASSERT_NO_THROW( {
        tracking_ch0->set_channel(gnss_synchro_ch0.Channel_ID);
        tracking_ch1->set_channel(gnss_synchro_ch1.Channel_ID);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        tracking_ch0->set_gnss_synchro(&gnss_synchro_ch0);
        tracking_ch1->set_gnss_synchro(&gnss_synchro_ch1);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        tracking_ch0->connect(top_block);
        tracking_ch1->connect(top_block);
    }) << "Failure connecting tracking to the top_block." << std::endl;

    ASSERT_NO_THROW( {
        std::string file =  "./" + filename_raw_data;
        const char * file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);
        gr::blocks::interleaved_char_to_complex::sptr  gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();
        gr::blocks::null_sink::sptr sink_ch0 = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        gr::blocks::null_sink::sptr sink_ch1 = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
        //ch0
        top_block->connect(gr_interleaved_char_to_complex, 0, tracking_ch0->get_left_block(), 0);
        top_block->connect(tracking_ch0->get_right_block(), 0, tlm_ch0->get_left_block(), 0);
        top_block->connect(tlm_ch0->get_right_block(), 0, observables->get_left_block(), 0);
        top_block->msg_connect(tracking_ch0->get_right_block(), pmt::mp("events"), msg_rx_ch0, pmt::mp("events"));
        //ch1
        top_block->connect(gr_interleaved_char_to_complex, 0, tracking_ch1->get_left_block(), 0);
        top_block->connect(tracking_ch1->get_right_block(), 0, tlm_ch1->get_left_block(), 0);
        top_block->connect(tlm_ch1->get_right_block(), 0, observables->get_left_block(), 1);
        top_block->msg_connect(tracking_ch1->get_right_block(), pmt::mp("events"), msg_rx_ch1, pmt::mp("events"));

        top_block->connect(observables->get_right_block(), 0, sink_ch0, 0);
        top_block->connect(observables->get_right_block(), 1, sink_ch1, 0);

    }) << "Failure connecting the blocks." << std::endl;

    tracking_ch0->start_tracking();
    tracking_ch1->start_tracking();

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec * 1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec * 1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    //check results
    //load the true values

    true_observables_reader true_observables;

    ASSERT_NO_THROW({
        if (    true_observables.open_obs_file(std::string("./obs_out.bin")) == false)
            {
                throw std::exception();
            };
    }) << "Failure opening true observables file" << std::endl;

    long int nepoch = true_observables.num_epochs();

    std::cout << "True observation epochs=" << nepoch << std::endl;
    arma::vec true_ch0_dist_m = arma::zeros(nepoch, 1);
    arma::vec true_ch0_acc_carrier_phase_cycles = arma::zeros(nepoch, 1);
    arma::vec true_ch0_Doppler_Hz = arma::zeros(nepoch, 1);
    arma::vec true_ch0_tow_s = arma::zeros(nepoch, 1);
    arma::vec true_ch1_dist_m = arma::zeros(nepoch, 1);
    arma::vec true_ch1_acc_carrier_phase_cycles = arma::zeros(nepoch, 1);
    arma::vec true_ch1_Doppler_Hz = arma::zeros(nepoch, 1);
    arma::vec true_ch1_tow_s = arma::zeros(nepoch, 1);

    true_observables.restart();
    long int epoch_counter = 0;
    ASSERT_NO_THROW({
        while(true_observables.read_binary_obs())
        {
            if (round(true_observables.prn[0])!=gnss_synchro_ch0.PRN)
                {
                    std::cout<<"True observables SV PRN do not match"<<round(true_observables.prn[1])<<std::endl;
                    throw std::exception();
                }
            if (round(true_observables.prn[1])!=gnss_synchro_ch1.PRN)
                {
                    std::cout<<"True observables SV PRN do not match "<<round(true_observables.prn[1])<<std::endl;
                    throw std::exception();
                }
            true_ch0_tow_s(epoch_counter) = true_observables.gps_time_sec[0];
            true_ch0_dist_m(epoch_counter) = true_observables.dist_m[0];
            true_ch0_Doppler_Hz(epoch_counter) = true_observables.doppler_l1_hz[0];
            true_ch0_acc_carrier_phase_cycles(epoch_counter) = true_observables.acc_carrier_phase_l1_cycles[0];

            true_ch1_tow_s(epoch_counter) = true_observables.gps_time_sec[1];
            true_ch1_dist_m(epoch_counter) = true_observables.dist_m[1];
            true_ch1_Doppler_Hz(epoch_counter) = true_observables.doppler_l1_hz[1];
            true_ch1_acc_carrier_phase_cycles(epoch_counter) = true_observables.acc_carrier_phase_l1_cycles[1];

            epoch_counter++;
        }

    });

    //read measured values
    observables_dump_reader estimated_observables(2); //two channels
    ASSERT_NO_THROW({
        if (estimated_observables.open_obs_file(std::string("./observables.dat")) == false)
            {
                throw std::exception();
            };
    }) << "Failure opening dump observables file" << std::endl;

    nepoch = estimated_observables.num_epochs();
    std::cout << "Measured observation epochs=" << nepoch << std::endl;

    arma::vec measuded_ch0_RX_time_s = arma::zeros(nepoch, 1);
    arma::vec measuded_ch0_TOW_at_current_symbol_s = arma::zeros(nepoch, 1);
    arma::vec measuded_ch0_Carrier_Doppler_hz = arma::zeros(nepoch, 1);
    arma::vec measuded_ch0_Acc_carrier_phase_hz = arma::zeros(nepoch, 1);
    arma::vec measuded_ch0_Pseudorange_m = arma::zeros(nepoch, 1);

    arma::vec measuded_ch1_RX_time_s = arma::zeros(nepoch, 1);
    arma::vec measuded_ch1_TOW_at_current_symbol_s = arma::zeros(nepoch, 1);
    arma::vec measuded_ch1_Carrier_Doppler_hz = arma::zeros(nepoch, 1);
    arma::vec measuded_ch1_Acc_carrier_phase_hz = arma::zeros(nepoch, 1);
    arma::vec measuded_ch1_Pseudorange_m = arma::zeros(nepoch, 1);

    estimated_observables.restart();

    epoch_counter = 0;
    while(estimated_observables.read_binary_obs())
    {
        measuded_ch0_RX_time_s(epoch_counter) = estimated_observables.RX_time[0];
        measuded_ch0_TOW_at_current_symbol_s(epoch_counter) =estimated_observables.TOW_at_current_symbol_s[0];
        measuded_ch0_Carrier_Doppler_hz(epoch_counter) = estimated_observables.Carrier_Doppler_hz[0];
        measuded_ch0_Acc_carrier_phase_hz(epoch_counter) = estimated_observables.Acc_carrier_phase_hz[0];
        measuded_ch0_Pseudorange_m(epoch_counter) = estimated_observables.Pseudorange_m[0];

        measuded_ch1_RX_time_s(epoch_counter) = estimated_observables.RX_time[1];
        measuded_ch1_TOW_at_current_symbol_s(epoch_counter) =estimated_observables.TOW_at_current_symbol_s[1];
        measuded_ch1_Carrier_Doppler_hz(epoch_counter) = estimated_observables.Carrier_Doppler_hz[1];
        measuded_ch1_Acc_carrier_phase_hz(epoch_counter) = estimated_observables.Acc_carrier_phase_hz[1];
        measuded_ch1_Pseudorange_m(epoch_counter) = estimated_observables.Pseudorange_m[1];

        epoch_counter++;
    }

    //Cut measurement initial transitory of the measurements
    arma::uvec initial_meas_point = arma::find(measuded_ch0_RX_time_s >= true_ch0_tow_s(0), 1, "first");

    measuded_ch0_RX_time_s = measuded_ch0_RX_time_s.subvec(initial_meas_point(0), measuded_ch0_RX_time_s.size() - 1);
    measuded_ch0_Pseudorange_m = measuded_ch0_Pseudorange_m.subvec(initial_meas_point(0), measuded_ch0_Pseudorange_m.size() - 1);
    measuded_ch1_RX_time_s = measuded_ch1_RX_time_s.subvec(initial_meas_point(0), measuded_ch1_RX_time_s.size() - 1);
    measuded_ch1_Pseudorange_m = measuded_ch1_Pseudorange_m.subvec(initial_meas_point(0), measuded_ch1_Pseudorange_m.size() - 1);

    check_results(true_ch0_dist_m, true_ch1_dist_m, true_ch0_tow_s,
            measuded_ch0_Pseudorange_m,measuded_ch1_Pseudorange_m, measuded_ch0_RX_time_s);

    std::cout <<  "Test completed in " << (end - begin) << " microseconds" << std::endl;
}

