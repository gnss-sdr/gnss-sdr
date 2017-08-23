#include <ctime>
#include <cstdlib>
#include <iostream>
#include <boost/chrono.hpp>
#include <boost/make_shared.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "glonass_l1_ca_pcps_acquisition.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GlonassL1CaPcpsAcquisitionTest_msg_rx;

typedef boost::shared_ptr<GlonassL1CaPcpsAcquisitionTest_msg_rx> GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr;

GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr GlonassL1CaPcpsAcquisitionTest_msg_rx_make();

class GlonassL1CaPcpsAcquisitionTest_msg_rx : public gr::block
{
private:
    friend GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr GlonassL1CaPcpsAcquisitionTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GlonassL1CaPcpsAcquisitionTest_msg_rx();
public:
    int rx_message;
    ~GlonassL1CaPcpsAcquisitionTest_msg_rx(); //!< Default destructor
};


GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr GlonassL1CaPcpsAcquisitionTest_msg_rx_make()
{
    return GlonassL1CaPcpsAcquisitionTest_msg_rx_sptr(new GlonassL1CaPcpsAcquisitionTest_msg_rx());
}


void GlonassL1CaPcpsAcquisitionTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
    {
            long int message = pmt::to_long(msg);
            rx_message = message;
    }
    catch(boost::bad_any_cast& e)
    {
            std::cout << "msg_handler_telemetry Bad any cast!" << std::endl;
            rx_message = 0;
    }
}


GlonassL1CaPcpsAcquisitionTest_msg_rx::GlonassL1CaPcpsAcquisitionTest_msg_rx() :
    gr::block("GlonassL1CaPcpsAcquisitionTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GlonassL1CaPcpsAcquisitionTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


GlonassL1CaPcpsAcquisitionTest_msg_rx::~GlonassL1CaPcpsAcquisitionTest_msg_rx()
{}


// ###########################################################

class GlonassL1CaPcpsAcquisitionTest: public ::testing::Test
{
protected:
    GlonassL1CaPcpsAcquisitionTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GlonassL1CaPcpsAcquisitionTest()
    {}

    void init();

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


void GlonassL1CaPcpsAcquisitionTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'R';
    std::string signal = "1G";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;
    config->set_property("GNSS-SDR.internal_fs_hz", "62314000");
    config->set_property("Acquisition.item_type", "gr_complex");
    config->set_property("Acquisition.if", "9540000");
    config->set_property("Acquisition.coherent_integration_time_ms", "1");
    config->set_property("Acquisition.dump", "true");
    config->set_property("Acquisition.dump_filename", "./acquisition.dat");
    config->set_property("Acquisition.implementation", "Glonass_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition.threshold", "0.001");
    config->set_property("Acquisition.doppler_max", "5000");
    config->set_property("Acquisition.doppler_step", "500");
    config->set_property("Acquisition.repeat_satellite", "false");
    config->set_property("Acquisition.pfa", "0.0");
}



TEST_F(GlonassL1CaPcpsAcquisitionTest, Instantiate)
{
    init();
    boost::shared_ptr<GlonassL1CaPcpsAcquisition> acquisition = boost::make_shared<GlonassL1CaPcpsAcquisition>(config.get(), "Acquisition", 1, 1);
}

TEST_F(GlonassL1CaPcpsAcquisitionTest, ConnectAndRun)
{
    int fs_in = 62314000;
    int nsamples = 62314;
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    top_block = gr::make_top_block("Acquisition test");
    init();
    boost::shared_ptr<GlonassL1CaPcpsAcquisition> acquisition = boost::make_shared<GlonassL1CaPcpsAcquisition>(config.get(), "Acquisition", 1, 1);
    boost::shared_ptr<GlonassL1CaPcpsAcquisitionTest_msg_rx> msg_rx = GlonassL1CaPcpsAcquisitionTest_msg_rx_make();

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));

    }) << "Failure connecting the blocks of acquisition test." << std::endl;

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec * 1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec * 1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;

    std::cout <<  "Processed " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;
}

TEST_F(GlonassL1CaPcpsAcquisitionTest, ValidationOfResults)
{
    struct timeval tv;
    long long int begin = 0;
    long long int end = 0;
    top_block = gr::make_top_block("Acquisition test");

    double expected_delay_samples = 31874;
    double expected_doppler_hz = -9500;
    init();
    std::shared_ptr<GlonassL1CaPcpsAcquisition> acquisition = std::make_shared<GlonassL1CaPcpsAcquisition>(config.get(), "Acquisition", 1, 1);

    boost::shared_ptr<GlonassL1CaPcpsAcquisitionTest_msg_rx> msg_rx = GlonassL1CaPcpsAcquisitionTest_msg_rx_make();

    ASSERT_NO_THROW( {
        acquisition->set_channel(1);
    }) << "Failure setting channel." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_threshold(0.001);
    }) << "Failure setting threshold." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_max(10000);
    }) << "Failure setting doppler_max." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->set_doppler_step(250);
    }) << "Failure setting doppler_step." << std::endl;

    ASSERT_NO_THROW( {
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block." << std::endl;

    ASSERT_NO_THROW( {
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/Glonass_L1_CA_SIM_Fs_62Msps_4ms.dat";
        const char * file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test." << std::endl;


    acquisition->set_state(1); // Ensure that acquisition starts at the first sample
    acquisition->init();

    EXPECT_NO_THROW( {
        gettimeofday(&tv, NULL);
        begin = tv.tv_sec * 1000000 + tv.tv_usec;
        top_block->run(); // Start threads and wait
        gettimeofday(&tv, NULL);
        end = tv.tv_sec * 1000000 + tv.tv_usec;
    }) << "Failure running the top_block." << std::endl;


    unsigned long int nsamples = gnss_synchro.Acq_samplestamp_samples;
    std::cout <<  "Acquired " << nsamples << " samples in " << (end - begin) << " microseconds" << std::endl;

    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    float delay_error_chips = (float)(delay_error_samples * 511 / 62316);
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    EXPECT_LE(doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";
}
