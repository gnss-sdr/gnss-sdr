/*!
 * \file osmosdr_signal_source.h
 * \brief Signal source wrapper for OsmoSDR-compatible front-ends, such as
 * HackRF or Realtek's RTL2832U-based USB dongle DVB-T receivers
 * (see https://osmocom.org/projects/rtl-sdr/wiki for more information)
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_OSMOSDR_SIGNAL_SOURCE_H
#define GNSS_SDR_OSMOSDR_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/file_sink.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <osmosdr/source.h>
#include <stdexcept>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class reads samples OsmoSDR-compatible front-ends, such as
 * HackRF or Realtek's RTL2832U-based USB dongle DVB-T receivers
 * (see https://osmocom.org/projects/rtl-sdr/wiki)
 */
class OsmosdrSignalSource : public SignalSourceBase
{
public:
    OsmosdrSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~OsmosdrSignalSource() = default;

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    void driver_instance();

    osmosdr::source::sptr osmosdr_source_;
    gnss_shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;

    std::string item_type_;
    std::string dump_filename_;
    std::string osmosdr_args_;
    std::string antenna_;

    // Front-end settings
    double sample_rate_;
    double freq_;
    double gain_;
    double if_gain_;
    double rf_gain_;

    size_t item_size_;
    int64_t samples_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    bool AGC_enabled_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_OSMOSDR_SIGNAL_SOURCE_H
