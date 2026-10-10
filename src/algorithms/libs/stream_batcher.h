/*!
 * \file stream_batcher.h
 * \brief Batch stream items without imposing a scheduler output multiple
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


#ifndef GNSS_SDR_STREAM_BATCHER_H
#define GNSS_SDR_STREAM_BATCHER_H

#include "gnss_block_interface.h"
#include "stream_batcher_limits.h"
#include <gnuradio/block.h>
#include <gnuradio/tags.h>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */

class stream_batcher;
class StreamBatcherDeadline;
using stream_batcher_sptr = gnss_shared_ptr<stream_batcher>;

stream_batcher_sptr make_stream_batcher(size_t item_size, uint64_t batch_items, uint64_t max_buffer_items,
    double max_latency_ms = STREAM_BATCHER_DEFAULT_LATENCY_MS, std::shared_ptr<StreamBatcherDeadline> deadlines = {});

class stream_batcher : public gr::block
{
public:
    ~stream_batcher() override;
    bool start() override;
    bool stop() override;
    void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items) override;
    size_t batch_items() const { return batch_items_; }

private:
    friend stream_batcher_sptr make_stream_batcher(size_t item_size, uint64_t batch_items, uint64_t max_buffer_items,
        double max_latency_ms, std::shared_ptr<StreamBatcherDeadline> deadlines);
    stream_batcher(size_t item_size, uint64_t batch_items, uint64_t max_buffer_items,
        double max_latency_ms, std::shared_ptr<StreamBatcherDeadline> deadlines);
    bool input_finished(bool require_empty = false) const;
    void collect_tags(size_t count);
    void publish_tags(size_t count);
    void deadline_expired(const pmt::pmt_t& message);

    std::vector<uint8_t> pending_;
    std::deque<gr::tag_t> tags_;
    size_t item_size_;
    size_t batch_items_;
    size_t filled_ = 0;
    size_t emitted_ = 0;
    bool draining_ = false;
    std::shared_ptr<StreamBatcherDeadline> deadlines_;
    uint64_t deadline_id_ = 0;
    uint64_t generation_ = 0;
    double max_latency_ms_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_STREAM_BATCHER_H
