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

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */

//! Hard ceiling for fixed batch durations and staging deadlines.
constexpr double STREAM_BATCHER_MAX_LATENCY_MS = 20.0;

//! Default staging deadline: long enough to amortize reader fan-out, short
//! enough not to slow down the acquisition search cadence of idle channels.
constexpr double STREAM_BATCHER_DEFAULT_LATENCY_MS = 10.0;

/** \} */
/** \} */
#endif  // GNSS_SDR_STREAM_BATCHER_LIMITS_H
