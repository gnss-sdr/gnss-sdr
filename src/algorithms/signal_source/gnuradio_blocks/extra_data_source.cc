/*!
 * \file extra_data_source.cc
 * \brief  GNURadio block that adds extra data to the sample stream.
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

#include "extra_data_source.h"
#include <pmt/pmt.h>


ExtraDataSource::ExtraDataSource(
    const std::string& path,
    const std::size_t& offset_in_file,
    const std::size_t& item_size,
    const bool& repeat,
    const std::size_t& offset_in_samples,
    const std::size_t& sample_period,
    const gr::io_signature::sptr& io_signature
    )
    : gr::sync_block("Extra Data Source",
          io_signature, io_signature),
      extra_data_file_(
          path,
          offset_in_file,
          item_size,
          repeat),
      offset_in_samples_(offset_in_samples),
      sample_period_(sample_period),
      next_tagged_sample_(offset_in_samples_)
{
    if (io_signature->min_streams() != 1 and io_signature->max_streams() != 1)
        {
            std::cout << "ERROR: This block only supports adding data to a single stream." << "\n";
        }
}

std::vector<uint8_t> ExtraDataSource::get_next_item()
{
    return extra_data_file_.read_item();
}

std::size_t ExtraDataSource::get_offset_in_samples() const
{
    return offset_in_samples_;
}

std::size_t ExtraDataSource::get_sample_period() const
{
    return sample_period_;
}

int ExtraDataSource::work(int noutput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    const std::size_t ch = 0;
    const int item_size = input_signature()->sizeof_stream_item(ch);
    std::memcpy(output_items[ch], input_items[ch], noutput_items * item_size);

    const uint64_t total_items_written = nitems_written(ch) + noutput_items;
    if (total_items_written >= next_tagged_sample_)
        {
            for (uint64_t sample = next_tagged_sample_; sample < total_items_written; sample += sample_period_)
                {
                    auto extra_data_item = get_next_item();
                    add_item_tag(ch, sample, pmt::mp("extra_data"), pmt::init_u8vector(extra_data_item.size(), extra_data_item));
                    next_tagged_sample_ += sample_period_;
                }
        }

    return noutput_items;
}
