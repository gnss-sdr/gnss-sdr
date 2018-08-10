/*!
 * \file gps_l1_ca_pcps_acquisition_test_fpga.cc
 * \brief  This class implements an acquisition test for
 * GpsL1CaPcpsAcquisitionFpga class based on some input parameters.
 * \author Marc Majoral, 2017. mmajoral(at)cttc.cat
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
#include <cstdlib>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/throttle.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_pcps_acquisition_fpga.h"

#include <unistd.h>

#define DMA_ACQ_TRANSFER_SIZE 2046               // DMA transfer size for the acquisition
#define RX_SIGNAL_MAX_VALUE 127                  // 2^7 - 1 for 8-bit signed values
#define NTIMES_CYCLE_THROUGH_RX_SAMPLES_FILE 50  // number of times we cycle through the file containing the received samples
#define ONE_SECOND 1000000                       // one second in microseconds
#define FLOAT_SIZE (sizeof(float))               // size of the float variable in characters

// thread that reads the file containing the received samples, scales the samples to the dynamic range of the fixed point values, sends
// the samples to the DMA and finally it stops the top block
void thread_acquisition_send_rx_samples(gr::top_block_sptr top_block,
    const char *file_name)
{
    FILE *rx_signal_file;  // file descriptor
    int file_length;       // length of the file containing the received samples
    int dma_descr;         // DMA descriptor

    // sleep for 1 second to give some time to GNSS-SDR to activate the acquisition module.
    // the acquisition module does not block the RX buffer before activation.
    // If this process starts sending samples straight ahead without waiting it could occur that
    // the first samples are lost. This is normal behaviour in a real receiver but this is not what
    // we want for the test
    usleep(ONE_SECOND);

    char *buffer_float;                         // temporary buffer to convert from binary char to float and from float to char
    signed char *buffer_DMA;                    // temporary buffer to store the samples to be sent to the DMA
    buffer_float = (char *)malloc(FLOAT_SIZE);  // allocate space for the temporary buffer
    if (!buffer_float)
        {
            fprintf(stderr, "Memory error!");
        }

    rx_signal_file = fopen(file_name, "rb");  // file containing the received signal
    if (!rx_signal_file)
        {
            printf("Unable to open file!");
        }

    // determine the length of the file that contains the received signal
    fseek(rx_signal_file, 0, SEEK_END);
    file_length = ftell(rx_signal_file);
    fseek(rx_signal_file, 0, SEEK_SET);

    // first step: check for the maximum value of the received signal

    float max = 0;
    float *pointer_float;
    pointer_float = (float *)&buffer_float[0];
    for (int k = 0; k < file_length; k = k + FLOAT_SIZE)
        {
            fread(buffer_float, FLOAT_SIZE, 1, rx_signal_file);

            if (fabs(pointer_float[0]) > max)
                {
                    max = (pointer_float[0]);
                }
        }

    // go back to the beginning of the file containing the received samples
    fseek(rx_signal_file, 0, SEEK_SET);

    // allocate memory for the samples to be transferred to the DMA

    buffer_DMA = (signed char *)malloc(DMA_ACQ_TRANSFER_SIZE);
    if (!buffer_DMA)
        {
            fprintf(stderr, "Memory error!");
        }

    // open the DMA descriptor
    dma_descr = open("/dev/loop_tx", O_WRONLY);
    if (dma_descr < 0)
        {
            printf("can't open loop device\n");
            exit(1);
        }

    // cycle through the file containing the received samples

    for (int k = 0; k < NTIMES_CYCLE_THROUGH_RX_SAMPLES_FILE; k++)
        {
            fseek(rx_signal_file, 0, SEEK_SET);

            int transfer_size;
            int num_transferred_samples = 0;
            while (num_transferred_samples < static_cast<int>((file_length / FLOAT_SIZE)))
                {
                    if (((file_length / FLOAT_SIZE) - num_transferred_samples) > DMA_ACQ_TRANSFER_SIZE)
                        {
                            transfer_size = DMA_ACQ_TRANSFER_SIZE;
                            num_transferred_samples = num_transferred_samples + DMA_ACQ_TRANSFER_SIZE;
                        }
                    else
                        {
                            transfer_size = file_length / FLOAT_SIZE - num_transferred_samples;
                            num_transferred_samples = file_length / FLOAT_SIZE;
                        }

                    for (int t = 0; t < transfer_size; t++)
                        {
                            fread(buffer_float, FLOAT_SIZE, 1, rx_signal_file);

                            // specify (float) (int) for a quantization maximizing the dynamic range
                            buffer_DMA[t] = static_cast<signed char>((pointer_float[0] * (RX_SIGNAL_MAX_VALUE - 1) / max));
                        }

                    //send_acquisition_gps_input_samples(buffer_DMA, transfer_size, dma_descr);
                    assert(transfer_size == write(dma_descr, &buffer_DMA[0], transfer_size));
                }
        }
    fclose(rx_signal_file);
    free(buffer_float);
    free(buffer_DMA);
    close(dma_descr);

    // when all the samples are sent stop the top block

    top_block->stop();
}


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class GpsL1CaPcpsAcquisitionTestFpga_msg_rx;

typedef boost::shared_ptr<GpsL1CaPcpsAcquisitionTestFpga_msg_rx> GpsL1CaPcpsAcquisitionTest_msg_fpga_rx_sptr;

GpsL1CaPcpsAcquisitionTest_msg_fpga_rx_sptr GpsL1CaPcpsAcquisitionTestFpga_msg_rx_make();

class GpsL1CaPcpsAcquisitionTestFpga_msg_rx : public gr::block
{
private:
    friend GpsL1CaPcpsAcquisitionTest_msg_fpga_rx_sptr GpsL1CaPcpsAcquisitionTestFpga_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL1CaPcpsAcquisitionTestFpga_msg_rx();

public:
    int rx_message;
    ~GpsL1CaPcpsAcquisitionTestFpga_msg_rx();  //!< Default destructor
};


GpsL1CaPcpsAcquisitionTest_msg_fpga_rx_sptr GpsL1CaPcpsAcquisitionTestFpga_msg_rx_make()
{
    return GpsL1CaPcpsAcquisitionTest_msg_fpga_rx_sptr(
        new GpsL1CaPcpsAcquisitionTestFpga_msg_rx());
}


void GpsL1CaPcpsAcquisitionTestFpga_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(msg);
            rx_message = message;
        }
    catch (boost::bad_any_cast &e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}


GpsL1CaPcpsAcquisitionTestFpga_msg_rx::GpsL1CaPcpsAcquisitionTestFpga_msg_rx() : gr::block("GpsL1CaPcpsAcquisitionTestFpga_msg_rx",
                                                                                     gr::io_signature::make(0, 0, 0),
                                                                                     gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
        boost::bind(&GpsL1CaPcpsAcquisitionTestFpga_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


GpsL1CaPcpsAcquisitionTestFpga_msg_rx::~GpsL1CaPcpsAcquisitionTestFpga_msg_rx()
{
}


class GpsL1CaPcpsAcquisitionTestFpga : public ::testing::Test
{
protected:
    GpsL1CaPcpsAcquisitionTestFpga()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL1CaPcpsAcquisitionTestFpga()
    {
    }

    void init();

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


void GpsL1CaPcpsAcquisitionTestFpga::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 1;
    config->set_property("GNSS-SDR.internal_fs_sps", "4000000");
    config->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    config->set_property("Acquisition_1C.item_type", "cshort");
    config->set_property("Acquisition_1C.coherent_integration_time_ms", "1");
    config->set_property("Acquisition_1C.dump", "false");
    config->set_property("Acquisition_1C.threshold", "0.001");
    config->set_property("Acquisition_1C.doppler_max", "5000");
    config->set_property("Acquisition_1C.doppler_step", "500");
    config->set_property("Acquisition_1C.repeat_satellite", "false");
    config->set_property("Acquisition_1C.pfa", "0.0");
    config->set_property("Acquisition_1C.select_queue_Fpga", "0");
    config->set_property("Acquisition_1C.devicename", "/dev/uio0");
}


TEST_F(GpsL1CaPcpsAcquisitionTestFpga, Instantiate)
{
    init();
    boost::shared_ptr<GpsL1CaPcpsAcquisitionFpga> acquisition =
        boost::make_shared<GpsL1CaPcpsAcquisitionFpga>(config.get(), "Acquisition_1C", 0, 1);
}


TEST_F(GpsL1CaPcpsAcquisitionTestFpga, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Acquisition test");

    double expected_delay_samples = 524;
    double expected_doppler_hz = 1680;
    init();

    std::shared_ptr<GpsL1CaPcpsAcquisitionFpga> acquisition =
        std::make_shared<GpsL1CaPcpsAcquisitionFpga>(config.get(), "Acquisition_1C", 0, 1);

    boost::shared_ptr<GpsL1CaPcpsAcquisitionTestFpga_msg_rx> msg_rx = GpsL1CaPcpsAcquisitionTestFpga_msg_rx_make();

    ASSERT_NO_THROW(
        {
            acquisition->set_channel(1);
        })
        << "Failure setting channel.";

    ASSERT_NO_THROW(
        {
            acquisition->set_gnss_synchro(&gnss_synchro);
        })
        << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW(
        {
            acquisition->set_threshold(0.1);
        })
        << "Failure setting threshold.";

    ASSERT_NO_THROW(
        {
            acquisition->set_doppler_max(10000);
        })
        << "Failure setting doppler_max.";

    ASSERT_NO_THROW(
        {
            acquisition->set_doppler_step(250);
        })
        << "Failure setting doppler_step.";

    ASSERT_NO_THROW(
        {
            acquisition->connect(top_block);
        })
        << "Failure connecting acquisition to the top_block.";

    // uncomment the next line to load the file from the current directory
    std::string file = "./GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";

    // uncomment the next two lines to load the file from the signal samples subdirectory
    //std::string path = std::string(TEST_PATH);
    //std::string file = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";

    const char *file_name = file.c_str();

    ASSERT_NO_THROW(
        {
            // for the unit test use dummy blocks to make the flowgraph work and allow the acquisition message to be sent.
            // in the actual system there is a flowchart running in parallel so this is not needed

            gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
            gr::blocks::null_sink::sptr null_sink = gr::blocks::null_sink::make(sizeof(gr_complex));
            gr::blocks::throttle::sptr throttle_block = gr::blocks::throttle::make(sizeof(gr_complex), 1000);

            top_block->connect(file_source, 0, throttle_block, 0);
            top_block->connect(throttle_block, 0, null_sink, 0);
            top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
        })
        << "Failure connecting the blocks of acquisition test.";

    acquisition->set_state(1);  // Ensure that acquisition starts at the first state
    acquisition->init();
    top_block->start();  // Start the top block

    // start thread that sends the DMA samples to the FPGA
    boost::thread t3{thread_acquisition_send_rx_samples, top_block, file_name};

    EXPECT_NO_THROW(
        {
            start = std::chrono::system_clock::now();
            acquisition->reset();  // launch the tracking process
            top_block->wait();
            end = std::chrono::system_clock::now();
            elapsed_seconds = end - start;
        })
        << "Failure running the top_block.";

    t3.join();

    std::cout << "Ran GpsL1CaPcpsAcquisitionTestFpga in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;

    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    float delay_error_chips = static_cast<float>(delay_error_samples * 1023 / 4000);
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    EXPECT_LE(doppler_error_hz, 666) << "Doppler error exceeds the expected value: 666 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";
}
