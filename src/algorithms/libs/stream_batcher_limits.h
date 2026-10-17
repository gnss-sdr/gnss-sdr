/*!
 * \file stream_batcher_limits.h
 * \brief Shared limits for stream batching and receiver scheduling
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

#ifndef GNSS_SDR_STREAM_BATCHER_LIMITS_H
#define GNSS_SDR_STREAM_BATCHER_LIMITS_H

#include <algorithm>
#include <cstdint>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */

//! Hard ceiling for fixed batch durations and batch deadlines (one hour). It
//! only keeps the timer arithmetic finite: the limit for a configuration comes
//! from the observables timing budget and STREAM_BATCHER_MAX_BATCH_BYTES.
constexpr double STREAM_BATCHER_MAX_LATENCY_MS = 3600000.0;

//! Default batch deadline: long enough to amortize reader fan-out, short
//! enough not to slow down the acquisition search cadence of idle channels.
constexpr double STREAM_BATCHER_DEFAULT_LATENCY_MS = 10.0;

//! Largest automatic batch, in bytes. GNU Radio allocates twice this much in
//! front of a batching block.
constexpr uint64_t STREAM_BATCHER_AUTO_BATCH_BYTES = 512U * 1024U;

//! Largest configured batch, in bytes. GNU Radio allocates two batches on each
//! side of a batching block, which keeps each of those buffers within the
//! limit of the automatic buffer of a conditioned stream.
constexpr uint64_t STREAM_BATCHER_MAX_BATCH_BYTES = 4U * 1024U * 1024U;

//! Largest automatic buffer of a conditioned stream, in bytes. It stays below
//! 16 MiB, where the shared memory segments that GNU Radio maps twice start to
//! exceed the default limits of old kernels.
constexpr uint64_t CONDITIONED_STREAM_AUTO_BUFFER_BYTES = 8U * 1024U * 1024U;

//! Longest telemetry symbol, in ms: a channel reports once per symbol.
constexpr uint32_t CONDITIONED_STREAM_SYMBOL_MS = 20U;

//! Signal time left for tracking data to reach the observables block, in ms,
//! when the slowest channel is a full buffer behind the receiver clock.
constexpr uint32_t CONDITIONED_STREAM_GUARD_MS = 60U;

/*!
 * \brief Receiver clock ticks kept by the observables block. An epoch is
 * computed when it is this many ticks old, less one.
 */
inline uint32_t observables_clock_ticks(uint32_t observable_interval_ms)
{
    return std::min(std::max(300U / std::max(observable_interval_ms, 1U), 3U), 20U);
}

/*!
 * \brief Age of an epoch, in ms of signal, when the observables block computes
 * it. A channel whose samples are processed later than this after they reach
 * the receiver clock has no observable for that epoch.
 */
inline uint32_t observables_clock_delay_ms(uint32_t observable_interval_ms)
{
    return (observables_clock_ticks(observable_interval_ms) - 1U) * observable_interval_ms;
}

/** \} */
/** \} */
#endif  // GNSS_SDR_STREAM_BATCHER_LIMITS_H
