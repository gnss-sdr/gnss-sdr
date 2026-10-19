/*!
 * \file stream_batcher.cc
 * \brief Publish stream items to many readers in batches
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


#include "stream_batcher.h"
#include "stream_batcher_deadline.h"
#include <gnuradio/block_detail.h>
#include <gnuradio/buffer.h>
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#if GNURADIO_HAS_BUFFER_READER_H
#include <gnuradio/buffer_reader.h>
#endif

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


stream_batcher_sptr make_stream_batcher(size_t item_size, uint64_t batch_items, uint64_t reader_window_items,
    double max_latency_ms, std::shared_ptr<StreamBatcherDeadline> deadlines)
{
    if (!std::isfinite(max_latency_ms) || max_latency_ms <= 0.0 || max_latency_ms > STREAM_BATCHER_MAX_LATENCY_MS)
        {
            throw std::invalid_argument("SignalConditioner maximum batching latency must be positive and not above " +
                                        std::to_string(static_cast<int64_t>(STREAM_BATCHER_MAX_LATENCY_MS)) + " ms");
        }
    // GNU Radio allocates at least two batches on each side of this block.
    const auto max_items = static_cast<uint64_t>(std::numeric_limits<int>::max());
    if (item_size == 0 || item_size > max_items || batch_items == 0 || batch_items > max_items / 4)
        {
            throw std::invalid_argument("SignalConditioner batch size must be between 1 and INT_MAX/4, with a valid item size");
        }
    if (batch_items > std::numeric_limits<size_t>::max() / item_size / 4)
        {
            throw std::invalid_argument("SignalConditioner batch size exceeds the addressable buffer size");
        }
    if (reader_window_items > max_items)
        {
            throw std::invalid_argument("SignalConditioner reader window exceeds supported buffering");
        }
    return stream_batcher_sptr(new stream_batcher(item_size, batch_items, reader_window_items, max_latency_ms, std::move(deadlines)));
}


stream_batcher::stream_batcher(size_t item_size, uint64_t batch_items, uint64_t reader_window_items,
    double max_latency_ms, std::shared_ptr<StreamBatcherDeadline> deadlines)
    : gr::block("stream_batcher",
          gr::io_signature::make(1, 1, static_cast<int>(item_size)),
          gr::io_signature::make(1, 1, static_cast<int>(item_size))),
      deadlines_(deadlines ? std::move(deadlines) : std::make_shared<StreamBatcherDeadline>()),
      max_latency_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double, std::milli>(max_latency_ms))),
      item_size_(item_size),
      batch_items_(static_cast<size_t>(batch_items)),
      reader_window_items_(static_cast<size_t>(reader_window_items)),
      wanted_items_(static_cast<int>(batch_items))
{
    // A batch waits in the input buffer and is copied once. The output
    // multiple makes GNU Radio size that buffer, and the output buffer, for
    // two batches at least, and offer output space in whole batches only: when
    // a reader falls behind, this block sleeps until a batch fits instead of
    // publishing every code period that the reader releases. Stream tags keep
    // their offsets because each call consumes exactly what it produces.
    set_output_multiple(static_cast<int>(batch_items_));
    const auto port = pmt::mp("batch_deadline");
    message_port_register_in(port);
    set_msg_handler(port, [this](const pmt::pmt_t& message) { deadline_expired(message); });
    deadline_id_ = deadlines_->add([this, port](uint64_t generation) {
        this->_post(port, pmt::from_uint64(generation));
    });
}


stream_batcher::~stream_batcher()
{
    deadlines_->remove(deadline_id_);
}


bool stream_batcher::check_topology(int /*ninputs*/, int /*noutputs*/)
{
    // GNU Radio up to 3.9 records the allocated output buffer size as a
    // maximum, and a maximum cancels the requested minimum. This runs before
    // each allocation, so a restarted block gets its buffers again, sized for
    // whole batches even if the previous run had to settle for less.
    set_max_output_buffer(0, -1);
    set_output_multiple(static_cast<int>(batch_items_));
    return true;
}


bool stream_batcher::start()
{
    armed_ = false;
    flush_ = false;
    ++generation_;
    auto quantum = static_cast<int64_t>(batch_items_);
    auto wanted = quantum;
    const auto state = detail();
    if (state && state->ninputs() > 0 && state->noutputs() > 0)
        {
            // The scheduler ends a block that forecasts more than its input
            // buffer can hold, and an upstream block can always fill half.
            wanted = std::min<int64_t>(wanted, (state->input(0)->max_possible_items_available() + 1) / 2);
            // A reader needing W items and a writer needing a whole batch
            // cannot wait for each other if batch + W fits in the buffer.
            const auto capacity = static_cast<int64_t>(state->output(0)->bufsize());
            quantum = std::min(quantum, std::min(capacity / 2, capacity - static_cast<int64_t>(reader_window_items_)));
        }
    if (is_set_max_noutput_items())
        {
            quantum = std::min<int64_t>(quantum, max_noutput_items());
        }
    quantum = std::max<int64_t>(quantum, 1);
    wanted_items_ = static_cast<int>(std::max<int64_t>(std::min(wanted, quantum), 1));
    if (static_cast<size_t>(quantum) != batch_items_ || static_cast<size_t>(wanted_items_) != batch_items_)
        {
            LOG(WARNING) << "stream_batcher: stream buffers are smaller than planned for batches of " << batch_items_
                         << " items; collecting " << wanted_items_ << " items and publishing at least " << quantum << " at a time";
        }
    set_output_multiple(static_cast<int>(quantum));
    deadlines_->enable(deadline_id_, true);
    return true;
}


bool stream_batcher::stop()
{
    deadlines_->enable(deadline_id_, false);
    return true;
}


void stream_batcher::deadline_expired(const pmt::pmt_t& message)
{
    deadlines_->acknowledge(deadline_id_);
    if (armed_ && pmt::is_uint64(message) && pmt::to_uint64(message) == generation_)
        {
            flush_ = true;
            // GNU Radio 3.7 dispatches messages inside its input wait. A
            // notification alone does not make it reevaluate the forecast.
            const auto state = detail();
            if (state)
                {
                    gr::thread::scoped_lock lock(state->d_tpb.mutex);
                    state->d_tpb.input_changed = true;
                    state->d_tpb.input_cond.notify_one();
                }
        }
}


bool stream_batcher::input_finished(int* available) const
{
    const auto state = detail();
    if (!state || state->ninputs() == 0)
        {
            *available = 0;
            return false;
        }
    const auto input = state->input(0);
    gr::thread::scoped_lock lock(*input->mutex());
    *available = input->items_available();
    return input->done();
}


void stream_batcher::forecast(int /*noutput_items*/, gr_vector_int& ninput_items_required)
{
    // Wait for a whole batch, unless its deadline expired or upstream
    // finished: then whatever is buffered is released. Requiring more than is
    // left at the end of the stream would end this block before its tail.
    int available = 0;
    if (flush_ || input_finished(&available))
        {
            ninput_items_required[0] = 1;
            return;
        }
    if (!armed_ && available > 0 && available < wanted_items_)
        {
            // The scheduler evaluates this block when input arrives. The first
            // item of a batch starts the clock that bounds its waiting time.
            armed_ = true;
            deadlines_->arm(deadline_id_, ++generation_, std::chrono::steady_clock::now() + max_latency_);
        }
    ninput_items_required[0] = wanted_items_;
}


int stream_batcher::general_work(int noutput_items, gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items, gr_vector_void_star& output_items)
{
    const int count = std::min(noutput_items, ninput_items[0]);
    if (count <= 0)
        {
            // A work allowance below one batch leaves no whole batch to offer.
            // Returning nothing makes the scheduler call again at once, so
            // give up the batch size rather than spin.
            set_output_multiple(1);
            return 0;
        }
    std::memcpy(output_items[0], input_items[0], static_cast<size_t>(count) * item_size_);
    consume_each(count);
    // An expired batch is released whole, in as many calls as it takes.
    flush_ = flush_ && count < ninput_items[0];
    if (armed_)
        {
            armed_ = false;
            ++generation_;
            deadlines_->cancel(deadline_id_);
        }
    return count;
}
