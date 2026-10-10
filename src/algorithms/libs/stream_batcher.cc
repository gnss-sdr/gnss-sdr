/*!
 * \file stream_batcher.cc
 * \brief Batch stream items while preserving finite tails and stream tags
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
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

#if GNURADIO_HAS_BUFFER_READER_H
#include <gnuradio/buffer_reader.h>
#endif


stream_batcher_sptr make_stream_batcher(size_t item_size, uint64_t batch_items, uint64_t max_buffer_items,
    double max_latency_ms, std::shared_ptr<StreamBatcherDeadline> deadlines)
{
    if (!std::isfinite(max_latency_ms) || max_latency_ms <= 0.0 || max_latency_ms > STREAM_BATCHER_MAX_LATENCY_MS)
        {
            throw std::invalid_argument("SignalConditioner maximum batching latency must be in (0, 20] ms");
        }
    const auto max_items = static_cast<uint64_t>(std::numeric_limits<int>::max());
    if (item_size == 0 || item_size > max_items || batch_items == 0 || batch_items > max_items / 2)
        {
            throw std::invalid_argument("SignalConditioner batch size must be between 1 and INT_MAX/2, with a valid item size");
        }
    if (max_buffer_items > max_items || max_buffer_items == 1)
        {
            throw std::invalid_argument("GNSS-SDR.max_source_buffer_samples must be 0 or between 2 and INT_MAX");
        }
    if (max_buffer_items > 0)
        {
            batch_items = std::min(batch_items, max_buffer_items / 2);
        }
    if (batch_items > std::numeric_limits<size_t>::max() / item_size / 2 ||
        max_buffer_items > std::numeric_limits<size_t>::max() / item_size)
        {
            throw std::invalid_argument("SignalConditioner batch size exceeds the addressable buffer size");
        }
    return stream_batcher_sptr(new stream_batcher(item_size, batch_items, max_buffer_items, max_latency_ms, std::move(deadlines)));
}


stream_batcher::stream_batcher(size_t item_size, uint64_t batch_items, uint64_t max_buffer_items,
    double max_latency_ms, std::shared_ptr<StreamBatcherDeadline> deadlines)
    : gr::block("stream_batcher",
          gr::io_signature::make(1, 1, static_cast<int>(item_size)),
          gr::io_signature::make(1, 1, static_cast<int>(item_size))),
      item_size_(item_size),
      batch_items_(static_cast<size_t>(batch_items)),
      deadlines_(deadlines ? std::move(deadlines) : std::make_shared<StreamBatcherDeadline>()),
      max_latency_ms_(max_latency_ms)
{
    pending_.resize(batch_items_ * item_size_);

    // A target size, not a hard output_multiple: a long-code consumer may need
    // more input while leaving less than one batch of free output space.
    set_min_output_buffer(static_cast<long>(2 * batch_items_));
    if (max_buffer_items > 0)
        {
            set_max_output_buffer(static_cast<long>(max_buffer_items));
        }
    // Consumption can precede publication by a batch. Publish each tag only
    // with its corresponding output item, retaining the original 1:1 offset.
    set_tag_propagation_policy(TPP_DONT);
    const auto port = pmt::mp("batch_deadline");
    message_port_register_in(port);
    set_msg_handler(port, [this](pmt::pmt_t message) { deadline_expired(std::move(message)); });
    deadline_id_ = deadlines_->add([this, port](uint64_t generation) {
        this->_post(port, pmt::from_uint64(generation));
    });
}


stream_batcher::~stream_batcher()
{
    deadlines_->remove(deadline_id_);
}


bool stream_batcher::start()
{
    ++generation_;
    filled_ = 0;
    emitted_ = 0;
    draining_ = false;
    tags_.clear();
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
    if (pmt::is_uint64(message) && pmt::to_uint64(message) == generation_ && filled_ > 0)
        {
            draining_ = true;
        }
}


bool stream_batcher::input_finished(bool require_empty) const
{
    const auto state = detail();
    if (!state || state->ninputs() == 0)
        {
            return false;
        }
    const auto input = state->input(0);
    gr::thread::scoped_lock lock(*input->mutex());
    return input->done() && (!require_empty || input->items_available() == 0);
}


void stream_batcher::forecast(int /*noutput_items*/, gr_vector_int& ninput_items_required)
{
    // Requiring even one input item at EOF would let the scheduler terminate
    // this block before its staged tail has been published.
    ninput_items_required[0] = (draining_ || input_finished()) ? 0 : 1;
}


void stream_batcher::collect_tags(size_t count)
{
    std::vector<gr::tag_t> incoming;
    const uint64_t offset = nitems_read(0);
    get_tags_in_range(incoming, 0, offset, offset + count);
    std::stable_sort(incoming.begin(), incoming.end(),
        [](const gr::tag_t& a, const gr::tag_t& b) { return a.offset < b.offset; });
    tags_.insert(tags_.end(), incoming.begin(), incoming.end());
}


int stream_batcher::general_work(int noutput_items, gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items, gr_vector_void_star& output_items)
{
    const bool finished = input_finished();
    if (!draining_ && filled_ == 0 && (static_cast<size_t>(ninput_items[0]) >= batch_items_ || finished))
        {
            // Already coalesced input needs only one copy. Do not split large
            // source deliveries merely because our accumulation target is smaller.
            const size_t count = std::min(static_cast<size_t>(ninput_items[0]), static_cast<size_t>(noutput_items));
            if (count > 0)
                {
                    collect_tags(count);
                    std::memcpy(output_items[0], input_items[0], count * item_size_);
                    consume_each(static_cast<int>(count));
                    publish_tags(count);
                    return static_cast<int>(count);
                }
            return finished && input_finished(true) ? WORK_DONE : 0;
        }
    if (!draining_)
        {
            const size_t count = std::min(batch_items_ - filled_, static_cast<size_t>(ninput_items[0]));
            if (count > 0)
                {
                    if (filled_ == 0)
                        {
                            ++generation_;
                            // Arm once per partial batch, not once per source delivery.
                            const auto delay = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                std::chrono::duration<double, std::milli>(max_latency_ms_));
                            deadlines_->arm(deadline_id_, generation_, std::chrono::steady_clock::now() + delay);
                        }
                    std::memcpy(pending_.data() + filled_ * item_size_, input_items[0], count * item_size_);
                    collect_tags(count);
                    consume_each(static_cast<int>(count));
                    filled_ += count;
                }
            // If upstream finishes between the scheduler's input snapshot and
            // this call, flushing a shorter batch is harmless: later items are
            // still read on subsequent calls before WORK_DONE is returned.
            draining_ = (filled_ == batch_items_) || finished;
        }
    if (!draining_)
        {
            return 0;
        }
    if (emitted_ == 0)
        {
            deadlines_->cancel(deadline_id_);
        }
    const size_t count = std::min(filled_ - emitted_, static_cast<size_t>(noutput_items));
    if (count == 0)
        {
            draining_ = false;
            return finished && ninput_items[0] == 0 && input_finished(true) ? WORK_DONE : 0;
        }
    std::memcpy(output_items[0], pending_.data() + emitted_ * item_size_, count * item_size_);
    publish_tags(count);
    emitted_ += count;
    if (emitted_ == filled_)
        {
            ++generation_;
            filled_ = 0;
            emitted_ = 0;
            draining_ = false;
        }
    return static_cast<int>(count);
}


void stream_batcher::publish_tags(size_t count)
{
    const uint64_t end = nitems_written(0) + count;
    while (!tags_.empty() && tags_.front().offset < end)
        {
            add_item_tag(0, tags_.front());
            tags_.pop_front();
        }
}
