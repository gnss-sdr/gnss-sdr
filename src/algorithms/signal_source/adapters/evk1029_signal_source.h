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
 * Set role + ".RF_channels" to N > 1 (e.g. for a dual-band setup sharing a
 * single IF capture) to get N identical, sample-locked output streams from
 * ONE underlying Evk1029Source instance/read, instead of instantiating N
 * separate SignalSourceN blocks each reading the file independently -- the
 * latter has no shared clock forcing the two chains to progress at the same
 * rate, and can drift apart over a long run if their downstream processing
 * costs differ. In RF_channels > 1 mode, .enable_throttle_control is not
 * supported (a single throttle block can't sit across N ports); it is
 * ignored with a warning. Backpressure from the slower of the two
 * downstream chains naturally keeps both output ports of the shared block
 * in lockstep instead.
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
