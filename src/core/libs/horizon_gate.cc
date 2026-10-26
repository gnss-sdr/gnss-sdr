/*!
 * \file horizon_gate.cc
 * \brief Block that releases a stream up to the deterministic horizon
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

#include "horizon_gate.h"
#include "deterministic_clock.h"
#include <gnuradio/block_detail.h>
#include <gnuradio/buffer.h>
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cstring>
#include <limits>

#if GNURADIO_HAS_BUFFER_READER_H
#include <gnuradio/buffer_reader.h>
#endif

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
// Waiting is bounded so that the scheduler can stop the block
constexpr std::chrono::milliseconds kHorizonWait(10);
constexpr std::chrono::seconds kStallReport(10);
}  // namespace


horizon_gate_sptr make_horizon_gate(size_t item_size, uint64_t batch_items)
{
    return horizon_gate_sptr(new horizon_gate(item_size, batch_items));
}


horizon_gate::horizon_gate(size_t item_size, uint64_t batch_items)
    : gr::block("horizon_gate",
          gr::io_signature::make(1, 1, static_cast<int>(item_size)),
          gr::io_signature::make(1, 1, static_cast<int>(item_size))),
      item_size_(item_size)
{
    // Wake the readers once per batch, not whenever one of them frees space
    set_output_multiple(static_cast<int>(std::min<uint64_t>(std::max<uint64_t>(batch_items, 1),
        static_cast<uint64_t>(std::numeric_limits<int>::max() / 2))));
}


bool horizon_gate::input_finished() const
{
    const auto state = detail();
    if (!state || state->ninputs() == 0)
        {
            return false;
        }
    const auto input = state->input(0);
    gr::thread::scoped_lock lock(*input->mutex());
    return input->done();
}


void horizon_gate::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    // A whole batch of input, except for the tail of the stream
    ninput_items_required[0] = input_finished() ? 1 : noutput_items;
}


int horizon_gate::general_work(int noutput_items, gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items, gr_vector_void_star& output_items)
{
    auto& clock = DeterministicClock::instance();
    const uint64_t released = nitems_written(0);
    if (clock.horizon() <= released && !clock.wait_beyond(released, kHorizonWait))
        {
            // Nothing consumed: the scheduler calls again, after the wait
            const auto now = std::chrono::steady_clock::now();
            if (!stalled_)
                {
                    stalled_ = true;
                    stalled_since_ = now;
                }
            else if (!stall_reported_ && now - stalled_since_ > kStallReport)
                {
                    stall_reported_ = true;
                    LOG(WARNING) << "Deterministic horizon stalled at sample " << released
                                 << " (lowest producer progress " << clock.min_progress() << ")";
                }
            return 0;
        }
    stalled_ = false;
    stall_reported_ = false;
    const uint64_t allowance = clock.horizon() - released;
    const int count = static_cast<int>(std::min<uint64_t>(allowance, static_cast<uint64_t>(std::min(noutput_items, ninput_items[0]))));
    std::memcpy(output_items[0], input_items[0], static_cast<size_t>(count) * item_size_);
    consume_each(count);
    return count;
}
