/*!
 * \file ad936x_iio_source.cc
 *
 * \brief Unpacks capture files in the LabSat 2 (ls2), LabSat 3 (ls3), or LabSat
 * 3 Wideband (LS3W) formats.
 * \author Javier Arribas jarribas (at) cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "ad936x_iio_source.h"
#include "INIReader.h"
#include "command_event.h"
#include "gnss_sdr_make_unique.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <array>
#include <bitset>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <utility>


ad936x_iio_source_sptr ad936x_iio_make_source_sptr(Concurrent_Queue<pmt::pmt_t> *queue,
    std::string pluto_device_uri,
    std::string board_type,
    long long bandwidth_,
    long long sample_rate_,
    std::vector<std::string> ch_list,
    std::vector<std::string> ch_gain_mode,
    std::vector<double> ch_gain_db,
    std::vector<long int> ch_freq_hz,
    int ch_sample_size,
    int ch_sample_bits_shift)
{
    return ad936x_iio_source_sptr(new ad936x_iio_source(*queue,
        pluto_device_uri,
        board_type,
        bandwidth_,
        sample_rate_,
        ch_list,
        ch_gain_mode,
        ch_gain_db,
        ch_freq_hz,
        ch_sample_size,
        ch_sample_bits_shift));
}


ad936x_iio_source::ad936x_iio_source(Concurrent_Queue<pmt::pmt_t> *queue,
    std::string pluto_device_uri,
    std::string board_type,
    long long bandwidth_,
    long long sample_rate_,
    std::vector<std::string> ch_list,
    std::vector<std::string> ch_gain_mode,
    std::vector<double> ch_gain_db,
    std::vector<long int> ch_freq_hz,
    int ch_sample_size,
    int ch_sample_bits_shift) : gr::block("ad936x_iio_source",
                                    gr::io_signature::make(0, 0, 0),
                                    gr::io_signature::make(1, 3, sizeof(gr_complex)))
{
}


ad936x_iio_source::~ad936x_iio_source()
{
}


int ad936x_iio_source::general_work(int noutput_items,
    __attribute__((unused)) gr_vector_int &ninput_items,
    __attribute__((unused)) gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    std::vector<gr_complex *> out;
    for (auto &output_item : output_items)
        {
            out.push_back(reinterpret_cast<gr_complex *>(output_item));
        }
    std::cout << "Warning!!\n";
    return 0;
}
