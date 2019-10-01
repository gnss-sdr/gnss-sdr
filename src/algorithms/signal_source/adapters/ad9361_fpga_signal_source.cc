/*!
 * \file ad9361_fpga_signal_source.cc
 * \brief signal source for Analog Devices front-end AD9361 connected directly to FPGA accelerators.
 * This source implements only the AD9361 control. It is NOT compatible with conventional SDR acquisition and tracking blocks.
 * Please use the fmcomms2 source if conventional SDR acquisition and tracking is selected in the configuration file.
 * \author Javier Arribas, jarribas(at)cttc.es
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

#include "ad9361_fpga_signal_source.h"
#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include "ad9361_manager.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <iio.h>
#include <exception>
#include <fcntl.h>   // for open, O_WRONLY
#include <fstream>   // for std::ifstream
#include <iostream>  // for cout, endl
#include <string>	 // for string manipulation
#include <unistd.h>  // for write
#include <utility>
#include <vector>


void run_DMA_process(const std::string &FreqBand, const std::string &Filename1, const std::string &Filename2, const bool &enable_DMA)
{
    const int MAX_INPUT_SAMPLES_TOTAL = 8192;
    int max_value = 0;
    int tx_fd;  // DMA descriptor
    std::ifstream infile1;
    infile1.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try
        {
            infile1.open(Filename1, std::ios::binary);
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Exception opening file " << Filename1 << std::endl;
            return;
        }

    std::ifstream infile2;
    infile2.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
        {
            infile2.open(Filename2, std::ios::binary);
        }
    catch (const std::ifstream::failure &e)
        {
            // could not exist
        }

    // rx signal
    std::vector<int8_t> input_samples(MAX_INPUT_SAMPLES_TOTAL * 2);
    std::vector<int8_t> input_samples2(MAX_INPUT_SAMPLES_TOTAL * 2);
    std::vector<int8_t> input_samples_dma(MAX_INPUT_SAMPLES_TOTAL * 2 * 2);

    int nread_elements;
    int nread_elements2;
    int file_completed = 0;
    int num_transferred_bytes;

    //**************************************************************************
    // Open DMA device
    //**************************************************************************
    tx_fd = open("/dev/loop_tx", O_WRONLY);
    if (tx_fd < 0)
        {
            std::cout << "Cannot open loop device" << std::endl;
            return;
        }

    //**************************************************************************
    // Open input file
    //**************************************************************************
    int nsamples = 0;

    while ((file_completed == 0) && (enable_DMA == true))
        {
            unsigned int dma_index = 0;

            if (FreqBand == "L1")
                {
                    try
                        {
                            infile1.read(reinterpret_cast<char *>(input_samples.data()), MAX_INPUT_SAMPLES_TOTAL * 2);
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            std::cerr << "Exception reading file " << Filename1 << std::endl;
                        }
                    if (infile1)
                        {
                            nread_elements = MAX_INPUT_SAMPLES_TOTAL * 2;
                        }
                    else
                        {
                            nread_elements = 0;
                        }
                    nsamples += (nread_elements / 2);

                    for (int index0 = 0; index0 < (nread_elements); index0 += 2)
                        {
                            // channel 1 (queue 1)
                            input_samples_dma[dma_index] = 0;
                            input_samples_dma[dma_index + 1] = 0;
                            // channel 0 (queue 0)
                            input_samples_dma[dma_index + 2] = input_samples[index0];
                            input_samples_dma[dma_index + 3] = input_samples[index0 + 1];

                            dma_index += 4;
                        }
                }
            else if (FreqBand == "L2")
                {
                    try
                        {
                            infile1.read(reinterpret_cast<char *>(input_samples.data()), MAX_INPUT_SAMPLES_TOTAL * 2);
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            std::cerr << "Exception reading file " << Filename1 << std::endl;
                        }
                    if (infile1)
                        {
                            nread_elements = MAX_INPUT_SAMPLES_TOTAL * 2;
                        }
                    else
                        {
                            nread_elements = 0;
                        }
                    nsamples += (nread_elements / 2);

                    for (int index0 = 0; index0 < (nread_elements); index0 += 2)
                        {
                            // channel 1 (queue 1)
                            input_samples_dma[dma_index] = input_samples[index0];
                            input_samples_dma[dma_index + 1] = input_samples[index0 + 1];
                            // channel 0 (queue 0)
                            input_samples_dma[dma_index + 2] = 0;
                            input_samples_dma[dma_index + 3] = 0;

                            dma_index += 4;
                        }
                }
            else if (FreqBand == "L1L2")
                {
                    try
                        {
                            infile1.read(reinterpret_cast<char *>(input_samples.data()), MAX_INPUT_SAMPLES_TOTAL * 2);
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            std::cerr << "Exception reading file " << Filename1 << std::endl;
                        }
                    if (infile1)
                        {
                            nread_elements = MAX_INPUT_SAMPLES_TOTAL * 2;
                        }
                    else
                        {
                            nread_elements = 0;
                        }
                    try
                        {
                            infile2.read(reinterpret_cast<char *>(input_samples2.data()), MAX_INPUT_SAMPLES_TOTAL * 2);
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            std::cerr << "Exception reading file " << Filename1 << std::endl;
                        }
                    if (infile2)
                        {
                            nread_elements2 = MAX_INPUT_SAMPLES_TOTAL * 2;
                        }
                    else
                        {
                            nread_elements2 = 0;
                        }

                    if (nread_elements > nread_elements2)
                        {
                            nread_elements = nread_elements2;  // take the smallest
                        }

                    nsamples += (nread_elements / 2);

                    for (int index0 = 0; index0 < (nread_elements); index0 += 2)
                        {
                            input_samples[index0] = input_samples[index0];
                            input_samples[index0 + 1] = input_samples[index0 + 1];

                            if (input_samples[index0] > max_value)
                                {
                                    max_value = input_samples[index0];
                                }
                            else if (-input_samples[index0] > max_value)
                                {
                                    max_value = -input_samples[index0];
                                }

                            if (input_samples[index0 + 1] > max_value)
                                {
                                    max_value = input_samples[index0 + 1];
                                }
                            else if (-input_samples[index0 + 1] > max_value)
                                {
                                    max_value = -input_samples[index0 + 1];
                                }

                            // channel 1 (queue 1)
                            input_samples_dma[dma_index] = input_samples2[index0];
                            input_samples_dma[dma_index + 1] = input_samples2[index0 + 1];
                            // channel 0 (queue 0)
                            input_samples_dma[dma_index + 2] = input_samples[index0];
                            input_samples_dma[dma_index + 3] = input_samples[index0 + 1];

                            dma_index += 4;
                        }
                }

            if (nread_elements > 0)
                {
                    num_transferred_bytes = nread_elements * 2;
					int num_bytes_sent = write(tx_fd, input_samples_dma.data(), nread_elements * 2);
					if (num_bytes_sent != num_transferred_bytes)
						{
							std::cerr << "Error: DMA could not send all the required samples " << std::endl;
						}
                }

            if (nread_elements != MAX_INPUT_SAMPLES_TOTAL * 2)
                {
                    file_completed = 1;
                }
        }

    try
        {
            infile1.close();
            infile2.close();
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Exception closing files " << Filename1 << " and " << Filename2 << std::endl;
        }
}


Ad9361FpgaSignalSource::Ad9361FpgaSignalSource(ConfigurationInterface *configuration,
    const std::string &role, unsigned int in_stream, unsigned int out_stream,
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue) : role_(role), in_stream_(in_stream), out_stream_(out_stream), queue_(std::move(queue))
{
    std::string default_dump_file = "./data/signal_source.dat";
    freq_ = configuration->property(role + ".freq", GPS_L1_FREQ_HZ);
    sample_rate_ = configuration->property(role + ".sampling_frequency", 12500000);
    bandwidth_ = configuration->property(role + ".bandwidth", 12500000);
    rx1_en_ = configuration->property(role + ".rx1_enable", true);
    rx2_en_ = configuration->property(role + ".rx2_enable", true);
    buffer_size_ = configuration->property(role + ".buffer_size", 0xA0000);
    quadrature_ = configuration->property(role + ".quadrature", true);
    rf_dc_ = configuration->property(role + ".rf_dc", true);
    bb_dc_ = configuration->property(role + ".bb_dc", true);
    gain_mode_rx1_ = configuration->property(role + ".gain_mode_rx1", std::string("manual"));
    gain_mode_rx2_ = configuration->property(role + ".gain_mode_rx2", std::string("manual"));
    rf_gain_rx1_ = configuration->property(role + ".gain_rx1", 64.0);
    rf_gain_rx2_ = configuration->property(role + ".gain_rx2", 64.0);
    rf_port_select_ = configuration->property(role + ".rf_port_select", std::string("A_BALANCED"));
    filter_file_ = configuration->property(role + ".filter_file", std::string(""));
    filter_auto_ = configuration->property(role + ".filter_auto", true);
    samples_ = configuration->property(role + ".samples", 0);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    enable_dds_lo_ = configuration->property(role + ".enable_dds_lo", false);
    freq_rf_tx_hz_ = configuration->property(role + ".freq_rf_tx_hz", GPS_L1_FREQ_HZ - GPS_L2_FREQ_HZ - 1000);
    freq_dds_tx_hz_ = configuration->property(role + ".freq_dds_tx_hz", 1000);
    scale_dds_dbfs_ = configuration->property(role + ".scale_dds_dbfs", -3.0);
    phase_dds_deg_ = configuration->property(role + ".phase_dds_deg", 0.0);
    tx_attenuation_db_ = configuration->property(role + ".tx_attenuation_db", 0.0);

    // turn switch to A/D position
    std::string default_device_name = "/dev/uio1";
    std::string device_name = configuration->property(role + ".devicename", default_device_name);
    switch_position = configuration->property(role + ".switch_position", 0);
    switch_fpga = std::make_shared<Fpga_Switch>(device_name);
    switch_fpga->set_switch_position(switch_position);

    item_size_ = sizeof(gr_complex);

    std::cout << "Sample rate: " << sample_rate_ << " Sps" << std::endl;

    if (switch_position == 0)  // Inject file(s) via DMA
        {
    		enable_DMA_ = true;
            std::string empty_string;
            filename_rx1 = configuration->property(role + ".filename_rx1", empty_string);
            filename_rx2 = configuration->property(role + ".filename_rx2", empty_string);
            int l1_band = configuration->property("Channels_1C.count", 0) +
                          configuration->property("Channels_1B.count", 0);

            int l2_band = configuration->property("Channels_L5.count", 0) +
                          configuration->property("Channels_5X.count", 0) +
                          configuration->property("Channels_2S.count", 0);

            if (l1_band != 0)
                {
                    freq_band = "L1";
                }
            if (l2_band != 0 && l1_band == 0)
                {
                    freq_band = "L2";
                }
            if (l1_band != 0 && l2_band != 0)
                {
                    freq_band = "L1L2";
                }

            thread_file_to_dma = std::thread([&] { run_DMA_process(freq_band, filename_rx1, filename_rx2, enable_DMA_); });
        }
    if (switch_position == 2)  // Real-time via AD9361
        {
            std::cout << "LO frequency : " << freq_ << " Hz" << std::endl;
            config_ad9361_rx_local(bandwidth_,
                sample_rate_,
                freq_,
                rf_port_select_,
                gain_mode_rx1_,
                gain_mode_rx2_,
                rf_gain_rx1_,
                rf_gain_rx2_);

            // LOCAL OSCILLATOR DDS GENERATOR FOR DUAL FREQUENCY OPERATION
            if (enable_dds_lo_ == true)
                {
                    config_ad9361_lo_local(bandwidth_,
                        sample_rate_,
                        freq_rf_tx_hz_,
                        tx_attenuation_db_,
                        freq_dds_tx_hz_,
                        scale_dds_dbfs_);
                }
        }
    if (switch_position != 0 && switch_position != 2)
        {
            std::cout << "SignalSource.switch_position configuration parameter must be either 0: read from file(s) via DMA, or 2: read from AD9361" << std::endl;
        }
    if (in_stream_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


Ad9361FpgaSignalSource::~Ad9361FpgaSignalSource()
{
    /* cleanup and exit */

	// std::cout<<"* AD9361 Disabling streaming channels\n";
    // if (rx0_i) { iio_channel_disable(rx0_i); }
    // if (rx0_q) { iio_channel_disable(rx0_q); }

	enable_DMA_ = false; // disable the DMA

    if (enable_dds_lo_)
        {
            try
                {
                    ad9361_disable_lo_local();
                }
            catch (const std::exception &e)
                {
                    LOG(WARNING) << "Problem closing the Ad9361FpgaSignalSource: " << e.what();
                }
        }
    if (switch_position == 0)  // read samples from a file via DMA
        {
            if (thread_file_to_dma.joinable())
                {
                    thread_file_to_dma.join();
                }
        }
    // std::cout<<"* AD9361 Destroying context\n";
    // if (ctx) { iio_context_destroy(ctx); }
}


void Ad9361FpgaSignalSource::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to connect";
}


void Ad9361FpgaSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    DLOG(INFO) << "AD9361 FPGA source nothing to disconnect";
}


gr::basic_block_sptr Ad9361FpgaSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return gr::basic_block_sptr();
}


gr::basic_block_sptr Ad9361FpgaSignalSource::get_right_block()
{
    return gr::basic_block_sptr();
}
