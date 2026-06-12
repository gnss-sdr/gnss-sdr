/*!
 * \file pocket_sdr_signal_source.h
 * \brief Signal source for Pocket SDR FE front-ends
 * \author Minhaj Ahmad, 2026. mahmad12(at)crimson.ua.edu
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

#ifndef GNSS_SDR_POCKET_SDR_SIGNAL_SOURCE_H
#define GNSS_SDR_POCKET_SDR_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/pocketsdr/pocketsdr_source.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class instantiates the gr-pocketsdr signal source for the
 * Pocket SDR FE 2CH/4CH/8CH GNSS RF front-ends.
 */
class PocketSdrSignalSource : public SignalSourceBase
{
public:
    PocketSdrSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~PocketSdrSignalSource() = default;

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    gr::pocketsdr::pocketsdr_source::sptr pocketsdr_source_;
    gnss_shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;

    std::string item_type_;
    std::string dump_filename_;
    std::string conf_file_;

    double sampling_frequency_;
    double freq_;
    size_t item_size_;
    int64_t samples_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    int channel_;

    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_POCKET_SDR_SIGNAL_SOURCE_H
