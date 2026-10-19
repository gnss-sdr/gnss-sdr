/*!
 * \file stream_batcher_deadline.h
 * \brief Shared cancellable deadline service for stream publication
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

#ifndef GNSS_SDR_STREAM_BATCHER_DEADLINE_H
#define GNSS_SDR_STREAM_BATCHER_DEADLINE_H

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */

class StreamBatcherDeadline
{
public:
    StreamBatcherDeadline();
    ~StreamBatcherDeadline();
    StreamBatcherDeadline(const StreamBatcherDeadline&) = delete;
    StreamBatcherDeadline& operator=(const StreamBatcherDeadline&) = delete;

    uint64_t add(std::function<void(uint64_t)> notify);
    void remove(uint64_t id);
    void enable(uint64_t id, bool enabled);
    void arm(uint64_t id, uint64_t generation, std::chrono::steady_clock::time_point deadline);
    void cancel(uint64_t id);
    void acknowledge(uint64_t id);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_STREAM_BATCHER_DEADLINE_H
