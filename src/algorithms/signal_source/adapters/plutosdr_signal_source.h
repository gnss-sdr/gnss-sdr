/*!
 * \file plutosdr_signal_source.h
 * \brief Signal source for PlutoSDR
 * \author Rodrigo Muñoz, 2017, rmunozl(at)inacap.cl
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
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H
#define GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H

#include "gnss_block_interface.h"
#include <boost/shared_ptr.hpp>
#include <gnuradio/blocks/file_sink.h>
#if GRIIO_INCLUDE_HAS_GNURADIO
#include <gnuradio/iio/pluto_source.h>
#else
#include <iio/pluto_source.h>
#endif
#include "concurrent_queue.h"
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <string>


class ConfigurationInterface;

/*!
 */
class PlutosdrSignalSource : public GNSSBlockInterface
{
public:
    PlutosdrSignalSource(ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    ~PlutosdrSignalSource() = default;

    std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Plutosdr_Signal_Source"
     */
    std::string implementation() override
    {
        return "Plutosdr_Signal_Source";
    }
    size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    std::string role_;

    // Front-end settings
    std::string uri_;  // device direction
    uint64_t freq_;    // frequency of local oscilator
    uint64_t sample_rate_;
    uint64_t bandwidth_;
    uint64_t buffer_size_;  // reception buffer
    bool quadrature_;
    bool rf_dc_;
    bool bb_dc_;
    std::string gain_mode_;
    double rf_gain_;
    std::string filter_file_;
    bool filter_auto_;
    std::string filter_source_;
    std::string filter_filename_;
    float Fpass_;
    float Fstop_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    std::string item_type_;
    size_t item_size_;
    int64_t samples_;
    bool dump_;
    std::string dump_filename_;

    gr::iio::pluto_source::sptr plutosdr_source_;

    boost::shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue_;
};

#endif  // GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H
