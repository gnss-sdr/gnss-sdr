/*!
 * \file ad936x_iio_source.h
 *
 * \brief signal source to receive samples from the AD936x FE family over libiio, including special custom functionalities in FPGA firmware.
 * \author Javier Arribas jarribas (at) cttc.es
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

#ifndef GNSS_SDR_AD9361_IIO_SOURCE_H
#define GNSS_SDR_AD9361_IIO_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include <gnuradio/block.h>
#include <iio.h>
#include <pmt/pmt.h>
#include <ad9361.h>  //multichip sync and high level functions
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class ad936x_iio_source;

using ad936x_iio_source_sptr = gnss_shared_ptr<ad936x_iio_source>;

ad936x_iio_source_sptr ad936x_iio_make_source_sptr(
    Concurrent_Queue<pmt::pmt_t> *queue,
    std::string pluto_device_uri,
    std::string board_type,
    long long bandwidth_,
    long long sample_rate_,
    std::vector<std::string> ch_list,
    std::vector<std::string> ch_gain_mode,
    std::vector<double> ch_gain_db,
    std::vector<long int> ch_freq_hz,
    int ch_sample_size,
    int ch_sample_bits_shift);

/*!
 * \brief This class implements conversion between Labsat 2, 3 and 3 Wideband
 * formats to gr_complex
 */
class ad936x_iio_source : public gr::block
{
public:
    ~ad936x_iio_source();

    int general_work(int noutput_items,
        gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend ad936x_iio_source_sptr ad936x_iio_make_source_sptr(
        Concurrent_Queue<pmt::pmt_t> *queue,
        std::string pluto_device_uri,
        std::string board_type,
        long long bandwidth_,
        long long sample_rate_,
        std::vector<std::string> ch_list,
        std::vector<std::string> ch_gain_mode,
        std::vector<double> ch_gain_db,
        std::vector<long int> ch_freq_hz,
        int ch_sample_size,
        int ch_sample_bits_shift);

    ad936x_iio_source(Concurrent_Queue<pmt::pmt_t> *queue,
        std::string pluto_device_uri,
        std::string board_type,
        long long bandwidth_,
        long long sample_rate_,
        std::vector<std::string> ch_list,
        std::vector<std::string> ch_gain_mode,
        std::vector<double> ch_gain_db,
        std::vector<long int> ch_freq_hz,
        int ch_sample_size,
        int ch_sample_bits_shift);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_AD9361_IIO_SOURCE_H
