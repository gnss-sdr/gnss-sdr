/*!
 * \file bladerf_signal_source.h
 * \brief Signal source for the Nuand bladeRF family of front-ends (bladeRF
 * x40, x115, and bladeRF 2.0 Micro xA4/xA9), using libbladeRF directly.
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

#ifndef GNSS_SDR_BLADERF_SIGNAL_SOURCE_H
#define GNSS_SDR_BLADERF_SIGNAL_SOURCE_H

#include "bladerf_source.h"
#include "concurrent_queue.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/file_sink.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class instantiates the bladeRF signal source, streaming RX
 * samples directly through libbladeRF (no gr-osmosdr or other intermediate
 * driver required).
 */
class BladerfSignalSource : public SignalSourceBase
{
public:
    BladerfSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~BladerfSignalSource() = default;

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    bladerf_source_sptr bladerf_source_;
    gnss_shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;

    std::string item_type_;
    std::string dump_filename_;
    std::string bladerf_args_;
    std::string device_serial_;

    double sample_rate_;
    double freq_;
    double bandwidth_;
    double gain_;
    size_t item_size_;
    int64_t samples_;

    unsigned int in_stream_;
    unsigned int out_stream_;
    unsigned int num_buffers_;
    unsigned int buffer_size_;
    unsigned int num_transfers_;
    unsigned int stream_timeout_ms_;

    bool agc_enabled_;
    bool rx_bias_tee_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BLADERF_SIGNAL_SOURCE_H
