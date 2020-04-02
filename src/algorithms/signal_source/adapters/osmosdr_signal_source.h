/*!
 * \file osmosdr_signal_source.h
 * \brief Signal source wrapper for OsmoSDR-compatible front-ends, such as
 * HackRF or Realtek's RTL2832U-based USB dongle DVB-T receivers
 * (see https://osmocom.org/projects/rtl-sdr/wiki for more information)
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_OSMOSDR_SIGNAL_SOURCE_H
#define GNSS_SDR_OSMOSDR_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include <gnuradio/blocks/file_sink.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <osmosdr/source.h>
#include <stdexcept>
#include <string>

class ConfigurationInterface;

/*!
 * \brief This class reads samples OsmoSDR-compatible front-ends, such as
 * HackRF or Realtek's RTL2832U-based USB dongle DVB-T receivers
 * (see https://osmocom.org/projects/rtl-sdr/wiki)
 */
class OsmosdrSignalSource : public GNSSBlockInterface
{
public:
    OsmosdrSignalSource(ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    ~OsmosdrSignalSource() = default;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Osmosdr_Signal_Source"
     */
    inline std::string implementation() override
    {
        return "Osmosdr_Signal_Source";
    }

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
    std::string role_;

    // Front-end settings
    bool AGC_enabled_;
    double sample_rate_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    double freq_;
    double gain_;
    double if_gain_;
    double rf_gain_;

    std::string item_type_;
    size_t item_size_;
    int64_t samples_;
    bool dump_;
    std::string dump_filename_;

    osmosdr::source::sptr osmosdr_source_;
    std::string osmosdr_args_;

    std::string antenna_;

    std::shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue_;
};

#endif  // GNSS_SDR_OSMOSDR_SIGNAL_SOURCE_H
