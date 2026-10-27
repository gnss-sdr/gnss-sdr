/*!
 * \file raw_stream_reader.h
 * \brief Transport-agnostic source of raw EVK1029 bytes. Evk1029Source's
 * existing OBA-decode/FIR/CUDA pipeline reads through this interface instead
 * of knowing whether the bytes come from a recorded file (FileStreamReader)
 * or a live shared-memory stream (SharedMemoryStreamReader, Linux only --
 * see shm_ring.h for its wire protocol).
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

#ifndef GNSS_SDR_RAW_STREAM_READER_H
#define GNSS_SDR_RAW_STREAM_READER_H

#include <cstddef>
#include <cstdint>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


class RawStreamReader
{
public:
    virtual ~RawStreamReader() = default;

    //! A short read (> 0, < requested_bytes) is normal, like POSIX read().
    //! Returns 0 once the stream has permanently ended or, for a live
    //! stream, if nothing arrived within a short poll interval, so that the
    //! caller can check whether it has to stop; end_of_stream() tells which.
    virtual std::size_t read(uint8_t* destination, std::size_t requested_bytes) = 0;

    //! After read() returned 0: true if the stream has ended, false if it
    //! just had nothing new yet. The default suits readers that block until
    //! there is data or the stream ends (FileStreamReader).
    virtual bool end_of_stream() const { return true; }

    //! Ends the stream: a concurrent or later read() returns 0 promptly.
    //! Safe to call from another thread than read()'s. No-op default for a
    //! reader that can't block (FileStreamReader).
    virtual void request_stop() {}
};


/** \} */
/** \} */
#endif  // GNSS_SDR_RAW_STREAM_READER_H
