/*!
 * \file shared_memory_stream_reader.h
 * \brief RawStreamReader backed by the live shared-memory ring EVK1029_CtrlApp
 * publishes (see shm_ring.h for the wire protocol).
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

#ifndef GNSS_SDR_SHARED_MEMORY_STREAM_READER_H
#define GNSS_SDR_SHARED_MEMORY_STREAM_READER_H

#include "raw_stream_reader.h"
#include "shm_ring.h"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


class SharedMemoryStreamReader : public RawStreamReader
{
public:
    // Retries for up to connect_timeout_seconds if the producer (EVK1029_CtrlApp)
    // hasn't created the ring yet -- a late-starting producer is recoverable.
    // stop_on_overflow: stop (as EOF) on a reported overflow instead of just logging it (see read()).
    // stall_timeout_seconds: end the stream if the producer publishes nothing for that long while
    // the stream is running (a hung, crashed or restarted producer); 0 waits forever.
    explicit SharedMemoryStreamReader(const std::string& shm_name, double connect_timeout_seconds = 30.0, bool stop_on_overflow = false, double stall_timeout_seconds = 5.0);
    ~SharedMemoryStreamReader() override;

    std::size_t read(uint8_t* destination, std::size_t requested_bytes) override;
    bool end_of_stream() const override { return ended_; }
    void request_stop() override;

private:
    shm_ring::MappedRing ring_;
    std::vector<uint8_t> pending_;  // the current SHM block, not yet fully consumed by read() calls
    std::size_t pending_pos_ = 0;
    std::atomic<bool> stop_requested_{false};
    bool stop_on_overflow_ = false;
    double stall_timeout_seconds_ = 5.0;
    std::chrono::steady_clock::time_point last_block_time_;  // see read()
    bool started_ = false;                                   // read() was called
    bool ended_ = false;

    // overflow_event_count as of the last check; initialized at the first read(), not 0 (see read()).
    uint64_t last_seen_overflow_count_ = 0;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SHARED_MEMORY_STREAM_READER_H
