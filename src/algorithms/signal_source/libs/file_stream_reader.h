/*!
 * \file file_stream_reader.h
 * \brief RawStreamReader backed by a recorded EVK1029 raw capture file --
 * preserves Evk1029Source's original file-reading behavior verbatim.
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

#ifndef GNSS_SDR_FILE_STREAM_READER_H
#define GNSS_SDR_FILE_STREAM_READER_H

#include "raw_stream_reader.h"
#include <cstdint>
#include <fstream>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


class FileStreamReader : public RawStreamReader
{
public:
    // bytes_to_skip is rounded up to the next 64-bit word, matching the
    // capture's native packing -- same convention Evk1029Source's own
    // constructor has always documented for this parameter.
    explicit FileStreamReader(const std::string& filename, uint64_t bytes_to_skip = 0);

    std::size_t read(uint8_t* destination, std::size_t requested_bytes) override;

private:
    std::ifstream file_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FILE_STREAM_READER_H
