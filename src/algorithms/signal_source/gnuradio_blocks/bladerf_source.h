/*!
 * \file bladerf_source.h
 * \brief GNU Radio source block for Nuand's bladeRF front-ends (bladeRF x40,
 * x115, and bladeRF 2.0 Micro xA4/xA9), implemented directly against
 * libbladeRF.
 * \author Oleksandr Suvorov, 2026.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_BLADERF_SOURCE_H
#define GNSS_SDR_BLADERF_SOURCE_H

#include "gnss_block_interface.h"
#include <gnuradio/sync_block.h>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class bladerf_source;

using bladerf_source_sptr = gnss_shared_ptr<bladerf_source>;

bladerf_source_sptr bladerf_make_source_sptr(
    const std::string &bladerf_args,
    const std::string &device_serial,
    double sample_rate,
    double freq,
    double bandwidth,
    double gain,
    bool agc_enabled,
    bool rx_bias_tee,
    unsigned int num_buffers,
    unsigned int buffer_size,
    unsigned int num_transfers,
    unsigned int stream_timeout_ms);

/*!
 * \brief This class implements a GNU Radio source block that streams RX
 * samples from a bladeRF device (single RX channel) using libbladeRF's
 * synchronous streaming API, converting the device's native
 * BLADERF_FORMAT_SC16_Q11 samples to gr_complex.
 */
class bladerf_source : public gr::sync_block
{
public:
    ~bladerf_source() override = default;

    bool start() override;
    bool stop() override;

    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items) override;

private:
    friend bladerf_source_sptr bladerf_make_source_sptr(
        const std::string &bladerf_args,
        const std::string &device_serial,
        double sample_rate,
        double freq,
        double bandwidth,
        double gain,
        bool agc_enabled,
        bool rx_bias_tee,
        unsigned int num_buffers,
        unsigned int buffer_size,
        unsigned int num_transfers,
        unsigned int stream_timeout_ms);

    bladerf_source(
        const std::string &bladerf_args,
        const std::string &device_serial,
        double sample_rate,
        double freq,
        double bandwidth,
        double gain,
        bool agc_enabled,
        bool rx_bias_tee,
        unsigned int num_buffers,
        unsigned int buffer_size,
        unsigned int num_transfers,
        unsigned int stream_timeout_ms);

    struct bladerf_deleter
    {
        void operator()(struct bladerf *dev) const;
    };

    std::unique_ptr<struct bladerf, bladerf_deleter> dev_;
    std::vector<int16_t> raw_buffer_;
    unsigned int buffer_size_;
    unsigned int stream_timeout_ms_;
    bool streaming_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BLADERF_SOURCE_H
