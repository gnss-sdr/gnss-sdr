/*!
 * \file file_stream_reader.cc
 * \brief RawStreamReader backed by a recorded EVK1029 raw capture file.
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

#include "file_stream_reader.h"
#include <cstdlib>
#include <iostream>

FileStreamReader::FileStreamReader(const std::string& filename, uint64_t bytes_to_skip)
{
    file_.open(filename.c_str(), std::ios::in | std::ios::binary);
    if (file_.is_open())
        {
            std::cout << "EVK1029_Source: file '" << filename << "' successfully opened.\n";
        }
    else
        {
            std::cerr << "EVK1029_Source: failed to open file: " << filename << '\n';
            exit(1);
        }

    if (bytes_to_skip > 0)
        {
            // Round up to the next 64-bit (8-byte) word boundary -- see this
            // parameter's doc comment for why.
            constexpr uint64_t kWordBytes = 8;
            const uint64_t aligned_offset = ((bytes_to_skip + kWordBytes - 1) / kWordBytes) * kWordBytes;

            file_.seekg(0, std::ios::end);
            const auto file_size = static_cast<uint64_t>(file_.tellg());
            if (aligned_offset >= file_size)
                {
                    std::cerr << "EVK1029_Source: seconds_to_skip lands at byte " << aligned_offset
                              << " (rounded up to a 64-bit word), at or beyond the end of '" << filename
                              << "' (" << file_size << " bytes) -- nothing to read.\n";
                    exit(1);
                }

            file_.seekg(static_cast<std::streamoff>(aligned_offset), std::ios::beg);
            std::cout << "EVK1029_Source: skipping the first " << aligned_offset << " bytes of the capture (requested "
                      << bytes_to_skip << ", rounded up to a 64-bit word boundary)" << std::endl;
        }
}


std::size_t FileStreamReader::read(uint8_t* destination, std::size_t requested_bytes)
{
    file_.read(reinterpret_cast<char*>(destination), static_cast<std::streamsize>(requested_bytes));
    return static_cast<std::size_t>(file_.gcount());
}
