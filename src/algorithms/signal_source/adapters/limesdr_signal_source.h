/*!
 * \file limesdr_signal_source.cc
 * \brief Signal source for LimeSDR front-end
 * \author Javier Arribas, 2021. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_LIMESDR_SIGNAL_SOURCE_H
#define GNSS_SDR_LIMESDR_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/file_sink.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <limesdr/source.h>
#include <memory>
#include <stdexcept>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class instantiates the LimeSDR gnuradio signal source.
 * It has support also for a customized LimeSDR firmware and signal source to support PPS samplestamp reading.
 */
class LimesdrSignalSource : public SignalSourceBase
{
public:
    LimesdrSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~LimesdrSignalSource() = default;

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    gr::limesdr::source::sptr limesdr_source_;
    gnss_shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;

    std::string item_type_;
    std::string dump_filename_;
    std::string limesdr_serial_;
    std::string limesdr_file_;

    // Front-end settings
    double sample_rate_;
    double freq_;
    double gain_;
    double analog_bw_hz_;
    double digital_bw_hz_;
    double ext_clock_MHz_;
    size_t item_size_;
    int64_t samples_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    int limechannel_mode_;
    int antenna_;
    int channel_;

    bool PPS_mode_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_LIMESDR_SIGNAL_SOURCE_H
