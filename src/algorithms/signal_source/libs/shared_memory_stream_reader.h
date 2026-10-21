/*!
 * \file shared_memory_stream_reader.h
 * \brief RawStreamReader backed by the live shared-memory ring EVK1029_CtrlApp
 * publishes (see shm_ring.h / EVK1029_shared_memory_input_spec(2).md).
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
    explicit SharedMemoryStreamReader(const std::string& shm_name, double connect_timeout_seconds = 30.0);
    ~SharedMemoryStreamReader() override;

    std::size_t read(uint8_t* destination, std::size_t requested_bytes) override;
    void request_stop() override;

private:
    shm_ring::MappedRing ring_;
    std::vector<uint8_t> pending_;  // the current SHM block, not yet fully consumed by read() calls
    std::size_t pending_pos_ = 0;
    std::atomic<bool> stop_requested_{false};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SHARED_MEMORY_STREAM_READER_H
