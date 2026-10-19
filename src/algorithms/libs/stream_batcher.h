/*!
 * \file stream_batcher.h
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


#ifndef GNSS_SDR_STREAM_BATCHER_H
#define GNSS_SDR_STREAM_BATCHER_H

#include "gnss_block_interface.h"
#include "stream_batcher_limits.h"
#include <gnuradio/block.h>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */

class stream_batcher;
class StreamBatcherDeadline;
using stream_batcher_sptr = gnss_shared_ptr<stream_batcher>;

/*!
 * \brief Makes a block that hands a stream to its readers one batch at a time.
 *
 * \param item_size Size of a stream item, in bytes.
 * \param batch_items Items per batch.
 * \param reader_window_items Largest number of items that a reader needs
 * available before it consumes any (forecast or history). The batch size is
 * reduced at start if the allocated output buffer cannot hold one batch on top
 * of that window.
 * \param max_latency_ms Longest time that an item waits for its batch to fill.
 * \param deadlines Timer shared by the batching blocks of a receiver.
 */
stream_batcher_sptr make_stream_batcher(size_t item_size, uint64_t batch_items, uint64_t reader_window_items = 0,
    double max_latency_ms = STREAM_BATCHER_DEFAULT_LATENCY_MS, std::shared_ptr<StreamBatcherDeadline> deadlines = {});

/*!
 * \brief Hands a stream to its readers one batch at a time.
 *
 * Items wait in the input buffer of the block, which copies them once. A batch
 * is published when it is complete, when its first item has waited for the
 * maximum latency, or when upstream finishes. Output space is taken in whole
 * batches, so a reader that falls behind delays the next batch instead of
 * shrinking it.
 */
class stream_batcher : public gr::block
{
public:
    ~stream_batcher() override;
    bool check_topology(int ninputs, int noutputs) override;
    bool start() override;
    bool stop() override;
    void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items) override;
    size_t batch_items() const { return batch_items_; }

private:
    friend stream_batcher_sptr make_stream_batcher(size_t item_size, uint64_t batch_items, uint64_t reader_window_items,
        double max_latency_ms, std::shared_ptr<StreamBatcherDeadline> deadlines);
    stream_batcher(size_t item_size, uint64_t batch_items, uint64_t reader_window_items,
        double max_latency_ms, std::shared_ptr<StreamBatcherDeadline> deadlines);
    bool input_finished(int* available) const;
    void deadline_expired(const pmt::pmt_t& message);

    std::shared_ptr<StreamBatcherDeadline> deadlines_;
    std::chrono::steady_clock::duration max_latency_;
    size_t item_size_;
    size_t batch_items_;
    size_t reader_window_items_;
    uint64_t deadline_id_ = 0;
    uint64_t generation_ = 0;
    int wanted_items_;
    bool armed_ = false;
    bool flush_ = false;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_STREAM_BATCHER_H
