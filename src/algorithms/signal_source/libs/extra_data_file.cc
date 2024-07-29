/*!
 * \file extra_data_file.cc
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

#include "extra_data_file.h"
#include <cstring>

ExtraDataFile::ExtraDataFile(
    const std::string& path,
    const std::size_t& offset_in_file,
    const std::size_t& item_size,
    const bool& repeat)
    : path_(path),
      file_(path_),
      offset_in_file_(offset_in_file),
      item_size_(item_size),
      repeat_(repeat),
      done_(false),
      io_buffer_size_(item_size * IO_BUFFER_CAPACITY),
      offset_in_io_buffer_(io_buffer_size_)  // Set to end of buffer so that first look up will trigger a read.
{
    file_.seekg(offset_in_file_, std::ios_base::beg);

    io_buffer_.resize(io_buffer_size_);
}

void ExtraDataFile::reset()
{
    file_.seekg(offset_in_file_, std::ios_base::beg);
    offset_in_io_buffer_ = io_buffer_size_;
    done_ = false;
}

std::vector<uint8_t> ExtraDataFile::read_item()
{
    if (offset_in_io_buffer_ >= io_buffer_size_)
        {
            if (done_)
                {
                    return {};
                }
            else
                {
                    read_into_io_buffer();
                }
        }

    std::vector<uint8_t> item_buf{};
    read_into_item_buffer(item_buf);

    return item_buf;
}

void ExtraDataFile::read_into_io_buffer()
{
    file_.read(reinterpret_cast<char*>(&io_buffer_[0]), io_buffer_size_);
    const std::size_t bytes_read = file_.gcount();

    if (bytes_read < io_buffer_size_)
        {
            if (repeat_)
                {
                    reset();
                    file_.read(reinterpret_cast<char*>(&io_buffer_[bytes_read]), io_buffer_size_ - bytes_read);
                }
            else
                {
                    io_buffer_size_ = bytes_read;
                    done_ = true;
                }
        }

    offset_in_io_buffer_ = 0;
}

void ExtraDataFile::read_into_item_buffer(std::vector<uint8_t>& item_buf)
{
    item_buf.resize(item_size_);
    std::memcpy(item_buf.data(), &io_buffer_[offset_in_io_buffer_], item_size_);
    offset_in_io_buffer_ += item_size_;
}
