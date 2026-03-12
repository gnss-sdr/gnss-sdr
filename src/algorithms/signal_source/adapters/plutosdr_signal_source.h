/*!
 * \file plutosdr_signal_source.h
 * \brief Signal source for PlutoSDR
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H
#define GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H

#include "signal_source_base.h"
#include <gnuradio/blocks/file_sink.h>
#if GRIIO_INCLUDE_HAS_GNURADIO
#include <gnuradio/iio/fmcomms2_source.h>
#else
#include <iio/fmcomms2_source.h>
#endif
#include "concurrent_queue.h"
#include <pmt/pmt.h>
#include <cstdint>
#include <string>

class ConfigurationInterface;

class PlutosdrSignalSource : public SignalSourceBase
{
public:
    PlutosdrSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~PlutosdrSignalSource() = default;

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    const std::string default_gain_mode = std::string("manual");

#if GNURADIO_API_IIO
#if GR_IIO_TEMPLATIZED_API
    gr::iio::fmcomms2_source<gr_complex>::sptr plutosdr_source_;
#else
    gr::iio::fmcomms2_source::sptr plutosdr_source_;
#endif
#else
    gr::iio::fmcomms2_source_f32c::sptr plutosdr_source_;
#endif

    gnss_shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;

    std::string item_type_;
    std::string dump_filename_;
    std::string uri_;
    std::string gain_mode_;
    std::string filter_file_;
    std::string filter_filename_;
    std::string filter_source_;

    int64_t samples_;
    size_t item_size_;

    double rf_gain_;
    double Fpass_;
    double Fstop_;

    uint64_t freq_;
    uint64_t sample_rate_;
    uint64_t bandwidth_;
    uint64_t buffer_size_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    int RF_channels_;

    bool quadrature_;
    bool rf_dc_;
    bool bb_dc_;
    bool filter_auto_;
    bool dump_;
};

#endif  // GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H
