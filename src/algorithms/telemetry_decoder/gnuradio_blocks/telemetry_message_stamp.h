/*!
 * \file telemetry_message_stamp.h
 * \brief Stamps the navigation data that telemetry decoders send to the PVT
 * \author Carles Fernandez Prades, 2026 cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_TELEMETRY_MESSAGE_STAMP_H
#define GNSS_SDR_TELEMETRY_MESSAGE_STAMP_H

#include "deterministic_clock.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <exception>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks telemetry_decoder_gr_blocks
 * \{ */

/*!
 * \brief With GNSS-SDR.deterministic=true, wraps \p msg as
 * ((sample stamp . channel) . msg), where the stamp is that of the symbol
 * being decoded (see DeterministicClock::Scope), so that the PVT applies it at
 * the first epoch past that sample. Otherwise, returns \p msg.
 */
inline pmt::pmt_t stamp_telemetry_message(const pmt::pmt_t& msg, int32_t channel)
{
    if (!DeterministicClock::instance().enabled())
        {
            return msg;
        }
    return pmt::cons(pmt::cons(pmt::from_uint64(DeterministicClock::scope_stamp()), pmt::from_long(channel)), msg);
}


/*!
 * \brief Scope of a call to the general_work() of a telemetry decoder.
 * Messages published inside it carry \p stamp, the sample stamp of the symbol
 * being decoded (see stamp_telemetry_message()). With
 * GNSS-SDR.deterministic=true, the tracking block upstream counts the symbols
 * as consumed only when the scope ends, once the messages that they cause
 * (e.g., telemetry faults) have been published, so that it cannot run past
 * the sample at which they take effect. Construct it before consume_each().
 */
class TelemetryWorkScope
{
public:
    TelemetryWorkScope(gr::block* block, uint64_t stamp)
        : d_stamp(stamp),
          d_block(DeterministicClock::instance().enabled() ? block : nullptr)
    {
        if (d_block)
            {
                DeterministicClock::instance().set_consumed(d_block, d_block->nitems_read(0));
            }
    }

    ~TelemetryWorkScope()
    {
        // A decoder that throws may not have published its messages
        if (d_block && !unwinding())
            {
                DeterministicClock::instance().set_consumed(d_block, d_block->nitems_read(0));
            }
    }

    TelemetryWorkScope(const TelemetryWorkScope&) = delete;
    TelemetryWorkScope& operator=(const TelemetryWorkScope&) = delete;

private:
#if defined(__cpp_lib_uncaught_exceptions)
    bool unwinding() const { return std::uncaught_exceptions() > d_exceptions; }
    const int d_exceptions{std::uncaught_exceptions()};
#else
    bool unwinding() const { return std::uncaught_exception(); }
#endif
    const DeterministicClock::Scope d_stamp;
    gr::block* d_block;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_TELEMETRY_MESSAGE_STAMP_H
