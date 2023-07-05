/*!
 * \file ad936x_iio_source.cc
 * \brief A direct IIO custom front-end gnss-sdr signal gnuradio block for the AD936x AD front-end family with special FPGA custom functionalities.
 * \author Javier Arribas, jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "ad936x_iio_source.h"
#include "INIReader.h"
#include "ad936x_iio_samples.h"
#include "command_event.h"
#include "gnss_sdr_make_unique.h"
#include "pps_samplestamp.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <array>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <utility>


ad936x_iio_source_sptr ad936x_iio_make_source_sptr(
    std::string pluto_uri_,
    std::string board_type_,
    long long bandwidth_,
    long long sample_rate_,
    long long freq_,
    std::string rf_port_select_,
    std::string rf_filter,
    std::string gain_mode_rx0_,
    std::string gain_mode_rx1_,
    double rf_gain_rx0_,
    double rf_gain_rx1_,
    bool enable_ch0,
    bool enable_ch1,
    long long freq_2ch,
    bool ppsmode_,
    bool customsamplesize_,
    std::string fe_ip_,
    int fe_ctlport_,
    int ssize_,
    int bshift_,
    bool spattern_,
    double lo_attenuation_db_,
    bool high_side_lo_,
    int tx_lo_channel_)
{
    return ad936x_iio_source_sptr(new ad936x_iio_source(
        pluto_uri_,
        board_type_,
        bandwidth_,
        sample_rate_,
        freq_,
        rf_port_select_,
        rf_filter,
        gain_mode_rx0_,
        gain_mode_rx1_,
        rf_gain_rx0_,
        rf_gain_rx1_,
        enable_ch0,
        enable_ch1,
        freq_2ch,
        ppsmode_,
        customsamplesize_,
        fe_ip_,
        fe_ctlport_,
        ssize_,
        bshift_,
        spattern_,
        lo_attenuation_db_,
        high_side_lo_,
        tx_lo_channel_));
}

void ad936x_iio_source::ad9361_channel_demux_and_record(ad936x_iio_samples *samples_in, int nchannels, std::vector<std::fstream> *files_out)
{
    int32_t current_byte = 0;
    int16_t ch = 0;
    // std::cout << "nbytes: " << samples_in->n_bytes << " nsamples: " << samples_in->n_samples << " nch: " << nchannels << "\n";
    while (current_byte < samples_in->n_bytes)
        {
            for (ch = 0; ch < nchannels; ch++)
                {
                    // std::cout << current_byte << " of " << samples_in->n_bytes << " test: " << (int)samples_in->buffer[current_byte] << "\n";
                    (*files_out).at(ch).write(&samples_in->buffer[current_byte], 4);  // two bytes I + two bytes Q per channel
                    current_byte += 4;
                }
        }
}

ad936x_iio_source::ad936x_iio_source(
    std::string pluto_uri_,
    std::string board_type_,
    long long bandwidth_,
    long long sample_rate_,
    long long freq_,
    std::string rf_port_select_,
    std::string rf_filter,
    std::string gain_mode_rx0_,
    std::string gain_mode_rx1_,
    double rf_gain_rx0_,
    double rf_gain_rx1_,
    bool enable_ch0,
    bool enable_ch1,
    long long freq_2ch,
    bool ppsmode_,
    bool customsamplesize_,
    std::string fe_ip_,
    int fe_ctlport_,
    int ssize_,
    int bshift_,
    bool spattern_,
    double lo_attenuation_db_,
    bool high_side_lo_,
    int tx_lo_channel_) : gr::block("ad936x_iio_source",
                              gr::io_signature::make(0, 0, 0),
                              gr::io_signature::make(1, 4, sizeof(int16_t)))
{
    ad936x_custom = std::make_unique<ad936x_iio_custom>(0, 0);
    try
        {
            if (ad936x_custom->initialize_device(pluto_uri_, board_type_) == true)
                {
                    // configure channels
                    if (ad936x_custom->init_config_ad9361_rx(bandwidth_,
                            sample_rate_,
                            freq_,
                            rf_port_select_,
                            rf_filter,
                            gain_mode_rx0_,
                            gain_mode_rx1_,
                            rf_gain_rx0_,
                            rf_gain_rx1_,
                            enable_ch0,
                            enable_ch1,
                            freq_2ch,
                            lo_attenuation_db_,
                            high_side_lo_,
                            tx_lo_channel_) == true)
                        {
                            std::cout << "ad936x_iio_source HW configured OK!\n";

                            // PPS FPGA Samplestamp information from TCP server
                            pps_rx = std::make_shared<pps_tcp_rx>();
                            ppsqueue = std::shared_ptr<Concurrent_Queue<PpsSamplestamp>>(new Concurrent_Queue<PpsSamplestamp>());

                            pps_rx->set_pps_samplestamp_queue(ppsqueue);
                            ad936x_custom->set_pps_samplestamp_queue(ppsqueue);

                            // start PPS RX thread
                            if (ppsmode_ == true or customsamplesize_ == true)
                                {
                                    pps_rx_thread = std::thread(&pps_tcp_rx::receive_pps, pps_rx, fe_ip_, fe_ctlport_);
                                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                    // configure custom FPGA options
                                    switch (ssize_)
                                        {
                                        case 16:
                                            {
                                                std::cout << "FPGA sample size set to 16 bits per sample.\n";
                                                if (pps_rx->send_cmd("ssize=16\n") == false) std::cout << "cmd send error!\n";
                                                break;
                                            }
                                        case 8:
                                            {
                                                std::cout << "FPGA sample size set to 8 bits per sample.\n";
                                                if (pps_rx->send_cmd("ssize=8\n") == false) std::cout << "cmd send error!\n";
                                                break;
                                            }
                                        case 4:
                                            {
                                                std::cout << "FPGA sample size set to 4 bits per sample.\n";
                                                if (pps_rx->send_cmd("ssize=4\n") == false) std::cout << "cmd send error!\n";
                                                break;
                                            }
                                        case 2:
                                            {
                                                std::cout << "FPGA sample size set to 2 bits per sample.\n";
                                                if (pps_rx->send_cmd("ssize=2\n") == false) std::cout << "cmd send error!\n";
                                                break;
                                            }
                                        default:
                                            {
                                                std::cout << "WARNING: Unsupported ssize. FPGA sample size set to 16 bits per sample.\n";
                                                if (pps_rx->send_cmd("ssize=16") == false) std::cout << "cmd send error!\n";
                                            }
                                        }

                                    if (bshift_ >= 0 and bshift_ <= 14)
                                        {
                                            std::cout << "FPGA sample bits shift left set to " + std::to_string(bshift_) + " positions.\n";
                                            if (pps_rx->send_cmd("bshift=" + std::to_string(bshift_) + "\n") == false) std::cout << "cmd send error!\n";
                                        }
                                    else
                                        {
                                            std::cout << "WARNING: Unsupported bshift. FPGA sample bits shift left set to 0.\n";
                                            if (pps_rx->send_cmd("bshift=0\n") == false) std::cout << "cmd send error!\n";
                                        }

                                    if (spattern_ == true)
                                        {
                                            std::cout << "FPGA debug sample pattern is active!.\n";
                                            if (pps_rx->send_cmd("spattern=1\n") == false) std::cout << "cmd send error!\n";
                                        }
                                    else
                                        {
                                            std::cout << "FPGA debug sample pattern disabled.\n";
                                            if (pps_rx->send_cmd("spattern=0\n") == false) std::cout << "cmd send error!\n";
                                        }
                                }
                            else
                                {
                                    std::cout << "PPS mode NOT enabled, not configuring PlutoSDR custom timestamping FPGA IP.\n";
                                }
                        }
                    else
                        {
                            std::cerr << "ad936x_iio_source IIO initialization error." << std::endl;
                            exit(1);
                        }
                }
            else
                {
                    std::cerr << "ad936x_iio_source IIO initialization error." << std::endl;
                    exit(1);
                }
        }
    catch (std::exception const &ex)
        {
            std::cerr << "STD exception: " << ex.what() << std::endl;
            exit(1);
        }
    catch (...)
        {
            std::cerr << "Unexpected catch" << std::endl;
            exit(1);
        }

    set_min_noutput_items(IIO_DEFAULTAD936XAPIFIFOSIZE_SAMPLES * 2);  // multiplexed I,Q, so, two samples per complex sample
    set_min_output_buffer(IIO_DEFAULTAD936XAPIFIFOSIZE_SAMPLES * sizeof(int16_t) * 2);
    // std::cout << "max_output_buffer " << min_output_buffer(0) << " min_noutput_items: " << min_noutput_items() << "\n";

    //    for (int n = 0; n < ad936x_custom->n_channels; n++)
    //        {
    //            std::string cap_file_root_name = "./debug_cap_ch";
    //            samplesfile.push_back(std::fstream(cap_file_root_name + std::to_string(n) + ".dat", std::ios::out | std::ios::binary));
    //            // samplesfile.back().exceptions(std::ios_base::badbit | std::ios_base::failbit); //this will enable exceptions for debug
    //
    //            if (samplesfile.back().is_open() == false)
    //                {
    //                    std::cout << "ERROR: Could not open " << cap_file_root_name + "_ch" + std::to_string(n) + ".dat"
    //                              << " for record samples!\n";
    //                }
    //        }
}

ad936x_iio_source::~ad936x_iio_source()
{
    // Terminate PPS thread
    if (pps_rx_thread.joinable())
        {
            pthread_t id = pps_rx_thread.native_handle();
            pps_rx_thread.detach();
            pthread_cancel(id);
        }
}


bool ad936x_iio_source::start()
{
    return ad936x_custom->start_sample_rx(false);
}

bool ad936x_iio_source::stop()
{
    std::cout << "stopping ad936x_iio_source...\n";
    ad936x_custom->stop_record();
    return true;
}

int ad936x_iio_source::general_work(int noutput_items,
    __attribute__((unused)) gr_vector_int &ninput_items,
    __attribute__((unused)) gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    std::shared_ptr<ad936x_iio_samples> current_buffer;
    ad936x_iio_samples *current_samples;
    ad936x_custom->pop_sample_buffer(current_buffer);
    current_samples = current_buffer.get();


    // I and Q samples are interleaved in buffer: IQIQIQ...
    int32_t n_interleaved_iq_samples_per_channel = current_samples->n_bytes / (ad936x_custom->n_channels * 2);
    if (noutput_items < n_interleaved_iq_samples_per_channel)
        {
            std::cout << "ad936x_iio_source output buffer overflow! noutput_items: " << noutput_items << " vs. " << n_interleaved_iq_samples_per_channel << "\n";
            return 0;
        }
    else
        {
            // ad9361_channel_demux_and_record(current_samples, ad936x_custom->n_channels, &samplesfile);
            auto **out = reinterpret_cast<int8_t **>(&output_items[0]);
            uint32_t current_byte = 0;
            uint32_t current_byte_in_gr = 0;
            int16_t ch = 0;
            // std::cout << "nbytes: " << samples_in->n_bytes << " nsamples: " << samples_in->n_samples << " nch: " << nchannels << "\n";
            if (ad936x_custom->n_channels == 1)
                {
                    memcpy(out[0], &current_samples->buffer[current_byte], current_samples->n_bytes);
                }
            else
                {
                    while (current_byte < current_samples->n_bytes)

                        {
                            for (ch = 0; ch < ad936x_custom->n_channels; ch++)
                                {
                                    memcpy(&out[ch][current_byte_in_gr], &current_samples->buffer[current_byte], 4);  // two bytes I + two bytes Q per channel: 4 bytes
                                    current_byte += 4;
                                }
                            current_byte_in_gr += 4;
                        }
                }
            ad936x_custom->push_sample_buffer(current_buffer);
            return n_interleaved_iq_samples_per_channel;  // always int16_t samples interleaved (I,Q,I,Q)

            //    for (size_t n = 0; n < ad936x_custom->n_channels; n++)
            //        {
            //            produce(n, current_samples->n_samples[n]);
            //        }
            //
            //
            //    return this->WORK_CALLED_PRODUCE;
        }
}
