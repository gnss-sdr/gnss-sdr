/*
 * galileo_e5a_pcps_acquisition_gsoc2014_test.cc
 *
 *  Created on: Jun 6, 2014
 *      Author: marc
 */

#include <ctime>
#include <iostream>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "configuration_interface.h"
#include "gnss_synchro.h"
#include "galileo_e5a_pcps_acquisition.h"
#include "signal_generator.h"
#include "signal_generator_c.h"
#include "fir_filter.h"
#include "gen_signal_source.h"
#include "gnss_sdr_valve.h"
#include "boost/shared_ptr.hpp"
#include "pass_through.h"
#include "file_output_filter.h"

class GalileoE5aPcpsAcquisitionGSoC2014GensourceTest: public ::testing::Test
{
protected:
    GalileoE5aPcpsAcquisitionGSoC2014GensourceTest()
    {
        queue = gr::msg_queue::make(0);
        top_block = gr::make_top_block("Acquisition test");
        item_size = sizeof(gr_complex);
        stop = false;
        message = 0;
    }

    ~GalileoE5aPcpsAcquisitionGSoC2014GensourceTest()
    {
    }

    void init();
    void config_1();
    void config_2();
    void start_queue();
    void wait_message();
    void process_message();
    void stop_queue();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    GalileoE5aPcpsAcquisition *acquisition;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
    concurrent_queue<int> channel_internal_queue;
    bool stop;
    int message;
    boost::thread ch_thread;

    unsigned int integration_time_ms;
    unsigned int fs_in;

    double expected_delay_chips;
    double expected_doppler_hz;
    float max_doppler_error_hz;
    float max_delay_error_chips;

    unsigned int num_of_realizations;
    unsigned int realization_counter;
    unsigned int detection_counter;
    unsigned int correct_estimation_counter;
    unsigned int acquired_samples;
    unsigned int mean_acq_time_us;

    double mse_doppler;
    double mse_delay;

    double Pd;
    double Pfa_p;
    double Pfa_a;
};

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::init()
{
    message = 0;
    realization_counter = 0;
    detection_counter = 0;
    correct_estimation_counter = 0;
    acquired_samples = 0;
    mse_doppler = 0;
    mse_delay = 0;
    mean_acq_time_us = 0;
    Pd = 0;
    Pfa_p = 0;
    Pfa_a = 0;
}

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::config_1()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    //std::string signal = "5Q";
    std::string signal = "5X";
    signal.copy(gnss_synchro.Signal,2,0);


    integration_time_ms = 1;
    //fs_in = 10.24e6;
    //fs_in = 12e6;
    fs_in = 12e6;

    //expected_delay_chips = 600;
    //expected_doppler_hz = 750;

    expected_delay_chips = 1000;
    expected_doppler_hz = 250;
    max_doppler_error_hz = 2/(3*integration_time_ms*1e-3);
    max_delay_error_chips = 0.50;

    //max_doppler_error_hz = 1000;
    //max_delay_error_chips = 1;

    num_of_realizations = 1;

    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.fs_hz", std::to_string(fs_in));

    config->set_property("SignalSource.item_type", "gr_complex");

    config->set_property("SignalSource.num_satellites", "1");

    config->set_property("SignalSource.system_0", "E");
    config->set_property("SignalSource.signal_0", "5X");
    config->set_property("SignalSource.PRN_0", "11");
    config->set_property("SignalSource.CN0_dB_0", "44");
    config->set_property("SignalSource.doppler_Hz_0", std::to_string(expected_doppler_hz));
    config->set_property("SignalSource.delay_chips_0", std::to_string(expected_delay_chips));

    config->set_property("SignalSource.noise_flag", "false");
    config->set_property("SignalSource.data_flag", "false");
    config->set_property("SignalSource.BW_BB", "0.97");

    config->set_property("SignalSource.dump", "true");
    config->set_property("SignalSource.dump_filename", "../data/signal_source.dat");

    config->set_property("InputFilter.implementation", "Fir_Filter");
    config->set_property("InputFilter.input_item_type", "gr_complex");
    config->set_property("InputFilter.output_item_type", "gr_complex");
    config->set_property("InputFilter.taps_item_type", "float");
    config->set_property("InputFilter.number_of_taps", "11");
    config->set_property("InputFilter.number_of_bands", "2");
    config->set_property("InputFilter.band1_begin", "0.0");
    config->set_property("InputFilter.band1_end", "0.97");
    config->set_property("InputFilter.band2_begin", "0.98");
    config->set_property("InputFilter.band2_end", "1.0");
    config->set_property("InputFilter.ampl1_begin", "1.0");
    config->set_property("InputFilter.ampl1_end", "1.0");
    config->set_property("InputFilter.ampl2_begin", "0.0");
    config->set_property("InputFilter.ampl2_end", "0.0");
    config->set_property("InputFilter.band1_error", "1.0");
    config->set_property("InputFilter.band2_error", "1.0");
    config->set_property("InputFilter.filter_type", "bandpass");
    config->set_property("InputFilter.grid_density", "16");

    config->set_property("Acquisition.item_type", "gr_complex");
    config->set_property("Acquisition.if", "0");
    config->set_property("Acquisition.coherent_integration_time_ms",
                         std::to_string(integration_time_ms));
    config->set_property("Acquisition.max_dwells", "1");
    config->set_property("Acquisition.implementation", "Galileo_E5a_PCPS_Acquisition");
    config->set_property("Acquisition.threshold", "0.1");
    config->set_property("Acquisition.doppler_max", "10000");
    config->set_property("Acquisition.doppler_step", "250");
    config->set_property("Acquisition.bit_transition_flag", "true");
    config->set_property("Acquisition.dump", "true");
    config->set_property("SignalSource.dump_filename", "../data/acquisition.dat");

}
void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::config_2()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'E';
    //std::string signal = "5Q";
    std::string signal = "5X";
    signal.copy(gnss_synchro.Signal,2,0);


    integration_time_ms = 2;
    //fs_in = 10.24e6;
    //fs_in = 12e6;
    fs_in = 12e6;

    //expected_delay_chips = 600;
    //expected_doppler_hz = 750;

    expected_delay_chips = 1000;
    expected_doppler_hz = 250;
    max_doppler_error_hz = 2/(3*integration_time_ms*1e-3);
    max_delay_error_chips = 0.50;

    //max_doppler_error_hz = 1000;
    //max_delay_error_chips = 1;

    num_of_realizations = 1;

    config = std::make_shared<InMemoryConfiguration>();

    config->set_property("GNSS-SDR.internal_fs_hz", std::to_string(fs_in));

    config->set_property("Acquisition.item_type", "gr_complex");
    config->set_property("Acquisition.if", "0");
    config->set_property("Acquisition.coherent_integration_time_ms",
                         std::to_string(integration_time_ms));
    config->set_property("Acquisition.max_dwells", "1");
    config->set_property("Acquisition.implementation", "Galileo_E5a_PCPS_Acquisition");
    config->set_property("Acquisition.threshold", "0.1");
    config->set_property("Acquisition.doppler_max", "10000");
    config->set_property("Acquisition.doppler_step", "250");
    config->set_property("Acquisition.bit_transition_flag", "true");
    config->set_property("Acquisition.dump", "true");
    config->set_property("SignalSource.dump_filename", "../data/acquisition.dat");

}

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::start_queue()
{
    stop = false;
    ch_thread = boost::thread(&GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::wait_message, this);
}

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::wait_message()
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;

    while (!stop)
        {
            acquisition->reset();

            gettimeofday(&tv, NULL);
            begin = tv.tv_sec *1e6 + tv.tv_usec;

            channel_internal_queue.wait_and_pop(message);

            gettimeofday(&tv, NULL);
            end = tv.tv_sec *1e6 + tv.tv_usec;

            mean_acq_time_us += (end-begin);

            process_message();
        }
}

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::process_message()
{
    if (message == 1)
        {
            detection_counter++;

            // The term -5 is here to correct the additional delay introduced by the FIR filter
            double delay_error_chips = abs((double)expected_delay_chips - (double)(gnss_synchro.Acq_delay_samples-5)*10230.0/((double)fs_in*1e-3));
            double doppler_error_hz = abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

            mse_delay += std::pow(delay_error_chips, 2);
            mse_doppler += std::pow(doppler_error_hz, 2);

            if ((delay_error_chips < max_delay_error_chips) && (doppler_error_hz < max_doppler_error_hz))
                {
                    correct_estimation_counter++;
                }
        }

    realization_counter++;

    std::cout << correct_estimation_counter << "correct estimation counter" << std::endl;
    std::cout << "Progress: " << round((float)realization_counter/num_of_realizations*100) << "% \r" << std::flush;
    std::cout << message << "message" <<std::endl;
    if (realization_counter == num_of_realizations)
        {
            mse_delay /= num_of_realizations;
            mse_doppler /= num_of_realizations;

            Pd = (double)correct_estimation_counter / (double)num_of_realizations;
            Pfa_a = (double)detection_counter / (double)num_of_realizations;
            Pfa_p = (double)(detection_counter - correct_estimation_counter) / (double)num_of_realizations;

            mean_acq_time_us /= num_of_realizations;

            stop_queue();
            top_block->stop();
        }
}

void GalileoE5aPcpsAcquisitionGSoC2014GensourceTest::stop_queue()
{
    stop = true;
}

/*TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, Instantiate)
{
    config_1();
    acquisition = new GalileoE5aPcpsAcquisition(config.get(), "Acquisition", 1, 1, queue);
    delete acquisition;
}
*/
/*
TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, ConnectAndRun)
{
    int nsamples = floor(fs_in*integration_time_ms*1e-3);
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;

    config_1();
    acquisition = new GalileoE5aPcpsAcquisition(config.get(), "Acquisition", 1, 1, queue);

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
    }) << "Failure connecting the blocks of acquisition test."<< std::endl;

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec *1e6 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec *1e6 + tv.tv_usec;
    }) << "Failure running the top_block."<< std::endl;

    std::cout <<  "Processed " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;

    free()
    delete acquisition;
}
*/
/*
TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, SOURCEValidation)
{
    config_1();
    ASSERT_NO_THROW( {
        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);
        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1, queue);
        signal_source.reset(new GenSignalSource(config.get(), signal_generator, filter, "SignalSource", queue));
        signal_source->connect(top_block);


        //top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);

    }) << "Failure generating signal" << std::endl;
}
*/
TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, SOURCEValidationTOFILE)
{
    config_1();
    ASSERT_NO_THROW( {
	std::string filename_ = "../data/Tiered_sinknull.dat";
	boost::shared_ptr<gr::blocks::file_sink> file_sink_;

        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);
        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1, queue);
        signal_source.reset(new GenSignalSource(config.get(), signal_generator, filter, "SignalSource", queue));
        //signal_source->connect(top_block);
        file_sink_=gr::blocks::file_sink::make(sizeof(gr_complex), filename_.c_str());

        top_block->connect(signal_source->get_right_block(),0,file_sink_,0);

        //top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);

    }) << "Failure generating signal" << std::endl;
}

TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, ValidationOfSIM)
{
    config_1();

    int nsamples = floor(fs_in*integration_time_ms*1e-3);
    acquisition = new GalileoE5aPcpsAcquisition(config.get(), "Acquisition", 1, 1, queue);

    ASSERT_NO_THROW( {
        acquisition->set_channel(1);
    }) << "Failure setting channel."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_channel_queue(&channel_internal_queue);
    }) << "Failure setting channel_internal_queue."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_max(config->property("Acquisition.doppler_max", 10000));
    }) << "Failure setting doppler_max."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_step(config->property("Acquisition.doppler_step", 500));
    }) << "Failure setting doppler_step."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_threshold(config->property("Acquisition.threshold", 0.0));
    }) << "Failure setting threshold."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block."<< std::endl;

    acquisition->init();

    ASSERT_NO_THROW( {
	std::string filename_ = "../data/Tiered_sink.dat";
	boost::shared_ptr<gr::blocks::file_sink> file_sink_;

        boost::shared_ptr<GenSignalSource> signal_source;
        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);

        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1, queue);
        signal_source.reset(new GenSignalSource(config.get(), signal_generator, filter, "SignalSource", queue));
        signal_source->connect(top_block);
        //
        file_sink_=gr::blocks::file_sink::make(sizeof(gr_complex), filename_.c_str());

        top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);
        top_block->connect(signal_source->get_right_block(), 0, file_sink_, 0);

    }) << "Failure connecting the blocks of acquisition test." << std::endl;


//    ASSERT_NO_THROW( {
//	//std::string path = std::string(TEST_PATH);
//	//std::string file =  "/home/marc/E5a_acquisitions/signal_source_5X_primary.dat";
//	std::string file =  "/home/marc/E5a_acquisitions/galileo_E5_8M_r2_upsampled_12.dat";
//	const char * file_name = file.c_str();
//	gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
//	top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
//    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    // i = 0 --> satellite in acquisition is visible
    // i = 1 --> satellite in acquisition is not visible
    for (unsigned int i = 0; i < 2; i++)
        {
            init();

            if (i == 0)
                {
                    gnss_synchro.PRN = 11; // This satellite is visible
                }
            else if (i == 1)
                {
                    gnss_synchro.PRN = 20; // This satellite is not visible
                }

            acquisition->set_local_code();

            start_queue();

            EXPECT_NO_THROW( {
                top_block->run(); // Start threads and wait
            }) << "Failure running he top_block."<< std::endl;

            if (i == 0)
            {
                EXPECT_EQ(1, message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";
                if (message == 1)
                    {
                	std::cout << gnss_synchro.Acq_delay_samples << "acq delay" <<std::endl;
                        EXPECT_EQ((unsigned int) 1, correct_estimation_counter) << "Acquisition failure. Incorrect parameters estimation.";
                    }

            }
            else if (i == 1)
            {
                EXPECT_EQ(2, message) << "Acquisition failure. Expected message: 2=ACQ FAIL.";
            }
        }

//    free(acquisition);
    delete acquisition;
}

/*
TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, ValidationOfResults)
{
    config_2();

    int nsamples = floor(fs_in*integration_time_ms*1e-3);
    acquisition = new GalileoE5aPcpsAcquisition(config.get(), "Acquisition", 1, 1, queue);

    ASSERT_NO_THROW( {
        acquisition->set_channel(0);
    }) << "Failure setting channel."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_channel_queue(&channel_internal_queue);
    }) << "Failure setting channel_internal_queue."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_max(config->property("Acquisition.doppler_max", 10000));
    }) << "Failure setting doppler_max."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_step(config->property("Acquisition.doppler_step", 500));
    }) << "Failure setting doppler_step."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_threshold(config->property("Acquisition.threshold", 0.0));
    }) << "Failure setting threshold."<< std::endl;

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block."<< std::endl;

    acquisition->init();

    ASSERT_NO_THROW( {
	//std::string path = std::string(TEST_PATH);
	//std::string file =  "/home/marc/E5a_acquisitions/signal_source_5X_primary.dat";
	std::string file =  "/home/marc/E5a_acquisitions/galileo_E5_8M_r2_upsampled_12.dat";
	const char * file_name = file.c_str();
	gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
	top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    // i = 0 --> satellite in acquisition is visible
    // i = 1 --> satellite in acquisition is not visible
    for (unsigned int i = 0; i < 4; i++)
        {
            init();

            switch (i)
            {
            case 0:
        	gnss_synchro.PRN = 12;
        	break;
            case 1:
        	gnss_synchro.PRN = 11;
        	break;
            case 2:
        	gnss_synchro.PRN = 19;
        	break;
            case 3:
        	gnss_synchro.PRN = 20;
        	break;
            }

            acquisition->set_local_code();

            start_queue();

            EXPECT_NO_THROW( {
                top_block->run(); // Start threads and wait
            }) << "Failure running he top_block."<< std::endl;

            if (i == 0)
            {
                EXPECT_EQ(1, message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";
                if (message == 1)
                    {
                	std::cout << gnss_synchro.Acq_delay_samples << "acq delay" <<std::endl;
                        EXPECT_EQ((unsigned int) 1, correct_estimation_counter) << "Acquisition failure. Incorrect parameters estimation.";
                    }

            }
            else if (i == 1)
            {
                EXPECT_EQ(2, message) << "Acquisition failure. Expected message: 2=ACQ FAIL.";
            }
        }

//    free(acquisition);
    delete acquisition;
}
*/
//TEST_F(GalileoE5aPcpsAcquisitionGSoC2014GensourceTest, ValidationOfResultsProbabilities)
//{
//    config_2();
//
//    acquisition = new GpsL1CaPcpsAcquisition(config.get(), "Acquisition", 1, 1, queue);
//
//    ASSERT_NO_THROW( {
//        acquisition->set_channel(1);
//    }) << "Failure setting channel."<< std::endl;
//
//    ASSERT_NO_THROW( {
//        acquisition->set_gnss_synchro(&gnss_synchro);
//    }) << "Failure setting gnss_synchro."<< std::endl;
//
//    ASSERT_NO_THROW( {
//        acquisition->set_channel_queue(&channel_internal_queue);
//    }) << "Failure setting channel_internal_queue."<< std::endl;
//
//    ASSERT_NO_THROW( {
//        acquisition->set_doppler_max(config->property("Acquisition.doppler_max", 10000));
//    }) << "Failure setting doppler_max."<< std::endl;
//
//    ASSERT_NO_THROW( {
//        acquisition->set_doppler_step(config->property("Acquisition.doppler_step", 500));
//    }) << "Failure setting doppler_step."<< std::endl;
//
//    ASSERT_NO_THROW( {
//        acquisition->set_threshold(config->property("Acquisition.threshold", 0.0));
//    }) << "Failure setting threshold."<< std::endl;
//
//    ASSERT_NO_THROW( {
//        acquisition->connect(top_block);
//    }) << "Failure connecting acquisition to the top_block."<< std::endl;
//
//    acquisition->init();
//
//    ASSERT_NO_THROW( {
//        boost::shared_ptr<GenSignalSource> signal_source;
//        SignalGenerator* signal_generator = new SignalGenerator(config.get(), "SignalSource", 0, 1, queue);
//        FirFilter* filter = new FirFilter(config.get(), "InputFilter", 1, 1, queue);
//        signal_source.reset(new GenSignalSource(config.get(), signal_generator, filter, "SignalSource", queue));
//        signal_source->connect(top_block);
//        top_block->connect(signal_source->get_right_block(), 0, acquisition->get_left_block(), 0);
//    }) << "Failure connecting the blocks of acquisition test." << std::endl;
//
//    std::cout << "Probability of false alarm (target) = " << 0.1 << std::endl;
//
//    // i = 0 --> satellite in acquisition is visible (prob of detection and prob of detection with wrong estimation)
//    // i = 1 --> satellite in acquisition is not visible (prob of false detection)
//    for (unsigned int i = 0; i < 2; i++)
//        {
//            init();
//
//            if (i == 0)
//                {
//                    gnss_synchro.PRN = 10; // This satellite is visible
//                }
//            else if (i == 1)
//                {
//                    gnss_synchro.PRN = 20; // This satellite is not visible
//                }
//
//            acquisition->set_local_code();
//
//            start_queue();
//
//            EXPECT_NO_THROW( {
//                top_block->run(); // Start threads and wait
//            }) << "Failure running the top_block."<< std::endl;
//
//            if (i == 0)
//            {
//                std::cout << "Estimated probability of detection = " << Pd << std::endl;
//                std::cout << "Estimated probability of false alarm (satellite present) = " << Pfa_p << std::endl;
//                std::cout << "Mean acq time = " << mean_acq_time_us << " microseconds." << std::endl;           }
//            else if (i == 1)
//            {
//                std::cout << "Estimated probability of false alarm (satellite absent) = " << Pfa_a << std::endl;
//                std::cout << "Mean acq time = " << mean_acq_time_us << " microseconds." << std::endl;
//            }
//        }
//
//    delete acquisition;
//}
