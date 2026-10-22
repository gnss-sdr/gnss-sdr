/*!
 * \file shared_memory_stream_reader.cc
 * \brief RawStreamReader backed by the live shared-memory ring EVK1029_CtrlApp
 * publishes.
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

#include "shared_memory_stream_reader.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

SharedMemoryStreamReader::SharedMemoryStreamReader(const std::string& shm_name, double connect_timeout_seconds, bool stop_on_overflow)
    : stop_on_overflow_(stop_on_overflow)
{
    LOG(INFO) << "EVK1029_Source: waiting for shared-memory producer on '" << shm_name << "' ...";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(connect_timeout_seconds);
    auto last_log = std::chrono::steady_clock::now();
    while (!shm_ring::Attach(ring_, shm_name))
        {
            if (std::chrono::steady_clock::now() >= deadline)
                {
                    std::cerr << "EVK1029_Source: timed out waiting for shared-memory producer on '" << shm_name
                              << "' after " << connect_timeout_seconds
                              << "s -- is EVK1029_CtrlApp running with \"Shared-memory Stream Start\" active?\n";
                    exit(1);
                }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_log).count() >= 2.0)
                {
                    LOG(WARNING) << "EVK1029_Source: still waiting for shared-memory producer on '" << shm_name << "' ...";
                    last_log = now;
                }
        }
    LOG(INFO) << "EVK1029_Source: attached to shared-memory stream '" << shm_name
              << "' (block_size=" << ring_.header->block_size
              << ", block_count=" << ring_.header->block_count << ")";
    // Only count overflows from here on -- one from before this attach isn't our discontinuity.
    last_seen_overflow_count_ = ring_.header->overflow_event_count.load(std::memory_order_relaxed);
}


SharedMemoryStreamReader::~SharedMemoryStreamReader()
{
    // Unblock a producer that may be stuck waiting for space we'll never free now.
    if (ring_.header)
        {
            shm_ring::FutexWakeAll(ring_.header->space_event);
        }
    shm_ring::Destroy(ring_);
}


void SharedMemoryStreamReader::request_stop()
{
    // The flag alone won't wake a ReadBlock() already asleep in FUTEX_WAIT.
    stop_requested_.store(true, std::memory_order_release);
    if (ring_.header)
        {
            shm_ring::FutexWakeAll(ring_.header->data_event);
        }
}


std::size_t SharedMemoryStreamReader::read(uint8_t* destination, std::size_t requested_bytes)
{
    if (pending_pos_ >= pending_.size())
        {
            pending_.clear();
            pending_pos_ = 0;
            if (!shm_ring::ReadBlock(ring_, pending_, &stop_requested_))
                {
                    return 0;  // producer ended the stream, or we were asked to stop -- both mean EOF here
                }

            // No byte count is available (the hardware only has a sticky overflow bit), so this can't be zero-filled.
            const uint64_t overflow_now = ring_.header->overflow_event_count.load(std::memory_order_relaxed);
            if (overflow_now != last_seen_overflow_count_)
                {
                    const uint64_t new_events = overflow_now - last_seen_overflow_count_;
                    last_seen_overflow_count_ = overflow_now;
                    if (stop_on_overflow_)
                        {
                            LOG(ERROR) << "EVK1029_Source: " << new_events
                                       << " IF_TO_AXIS hardware overflow(s) reported; stopping (shm_stop_on_buffer_overflow=true)";
                            return 0;
                        }
                    LOG(WARNING) << "EVK1029_Source: " << new_events
                                 << " IF_TO_AXIS hardware overflow(s) reported; sample count will jump in place";
                }
        }

    const std::size_t available = pending_.size() - pending_pos_;
    const std::size_t to_copy = std::min(available, requested_bytes);
    std::memcpy(destination, pending_.data() + pending_pos_, to_copy);
    pending_pos_ += to_copy;
    return to_copy;
}
