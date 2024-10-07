/*!
 * \file extra_data_file.h
 * \brief  Provides a simple abstraction for reading contiguous binary data from a file
 * \author Victor Castillo, 2024. victorcastilloaguero(at).gmail.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_EXTRA_DATA_FILE_H
#define GNSS_SDR_EXTRA_DATA_FILE_H

#include <cstddef>  // for size_t
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


class ExtraDataFile
{
    static constexpr std::size_t IO_BUFFER_CAPACITY = 1024;

public:
    ExtraDataFile(
        const std::string& path,
        const std::size_t& offset_in_file,
        const std::size_t& item_size,
        const bool& repeat);


    void reset();

    std::vector<uint8_t> read_item();

private:
    void read_into_io_buffer();

    void read_into_item_buffer(std::vector<uint8_t>& item_buf);

private:
    std::string path_;
    std::ifstream file_;
    std::size_t offset_in_file_;
    std::size_t item_size_;
    bool repeat_;
    bool done_;

    std::vector<uint8_t> io_buffer_;
    std::size_t io_buffer_size_;
    std::size_t offset_in_io_buffer_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_EXTRA_DATA_FILE_H
