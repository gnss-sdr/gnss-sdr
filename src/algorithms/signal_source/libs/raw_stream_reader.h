/*!
 * \file raw_stream_reader.h
 * \brief Transport-agnostic source of raw EVK1029 bytes -- see
 * EVK1029_shared_memory_input_spec(2).md. Evk1029Source's existing OBA-
 * decode/FIR/CUDA pipeline reads through this interface instead of knowing
 * whether the bytes come from a recorded file (FileStreamReader) or a live
 * shared-memory stream (SharedMemoryStreamReader).
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
    //! May block unboundedly (e.g. waiting on a live producer); returns 0
    //! only once the stream has truly and permanently ended.
    virtual std::size_t read(uint8_t* destination, std::size_t requested_bytes) = 0;

    //! Unblocks a concurrent read(); no-op default for a reader that can't
    //! block (FileStreamReader). Called from Evk1029Source's destructor.
    virtual void request_stop() {}
};


/** \} */
/** \} */
#endif  // GNSS_SDR_RAW_STREAM_READER_H
