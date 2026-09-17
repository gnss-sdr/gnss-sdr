/*!
 * \file evk1029_signal_source.h
 * \brief SAPHYRION EVK1029 signal source. Reads a continuous, header-less
 * raw capture file (OBA-encoded 4-bit samples) directly, without going
 * through the generic XML-metadata-driven ION_GSMS_Signal_Source path.
 *
 * Successor of the old EVK10x9_signal_source: same overall structure
 * (dedicated file reader + optional throttle + optional dump), updated
 * for the current EVK1029 capture format (see evk1029_source.h).
 *
 * Set role + ".RF_channels" to N > 1 (e.g. for a multi-band setup sharing a
 * single IF capture) to feed N Signal Conditioners from ONE underlying
 * Evk1029Source instance/read, instead of instantiating N separate
 * SignalSourceN blocks each reading the file independently. Evk1029Source
 * has a single output port (see its own header); gnss_flowgraph.cc's
 * generic signal-source-to-conditioner wiring detects this
 * (output_signature()->max_streams() == 1) and connects each of the N
 * Signal Conditioners to that same port, so all of them read the same
 * sample stream and GNU Radio keeps them within one output buffer of each
 * other. The optional throttle sits between the source and that shared
 * port, so .enable_throttle_control works for any RF_channels value.
 * Set role + ".seconds_to_skip" to start reading at a given time into the
 * capture (converted to a byte position using the raw sampling frequency).
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

#ifndef GNSS_SDR_EVK1029_SIGNAL_SOURCE_H
#define GNSS_SDR_EVK1029_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "evk1029_source.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/throttle.h>
#include <cstdint>
#include <string>

class ConfigurationInterface;

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


/*!
 * \brief This class reads a SAPHYRION EVK1029 raw capture file.
 */
class Evk1029SignalSource : public SignalSourceBase
{
public:
    Evk1029SignalSource(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_stream,
        unsigned int out_stream,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~Evk1029SignalSource() = default;

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_right_block() override;

private:
    Evk1029Source_sptr evk1029_source_;
    gr::blocks::throttle::sptr throttle_;
    gr::blocks::file_sink::sptr file_sink_;

    std::string dump_filename_;
    bool dump_;

    size_t item_size_;
    unsigned int rf_channels_;

    bool enable_throttle_control_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_EVK1029_SIGNAL_SOURCE_H
