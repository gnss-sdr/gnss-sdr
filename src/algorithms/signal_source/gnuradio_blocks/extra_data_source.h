/*!
 * \file extra_data_source.h
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


#ifndef GNSS_SDR_EXTRA_DATA_SOURCE_H
#define GNSS_SDR_EXTRA_DATA_SOURCE_H

#include "extra_data_file.h"
#include "gnss_block_interface.h"
#include <gnuradio/sync_block.h>  // for sync_block
#include <gnuradio/types.h>       // for gr_vector_const_void_star
#include <cstddef>                // for size_t
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class ExtraDataSource : public gr::sync_block
{
public:
    using sptr = gnss_shared_ptr<ExtraDataSource>;

    ExtraDataSource(
        const std::string& path,
        const std::size_t& offset_in_file,
        const std::size_t& item_size,
        const bool& repeat,
        const std::size_t& offset_in_samples,
        const std::size_t& sample_period,
        const gr::io_signature::sptr& io_signature);

private:
    std::vector<uint8_t> get_next_item();

    std::size_t get_offset_in_samples() const;

    std::size_t get_sample_period() const;

public:
    int work(int noutput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items) override;

private:
    ExtraDataFile extra_data_file_;
    std::size_t offset_in_samples_;
    std::size_t sample_period_;

    std::size_t next_tagged_sample_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_EXTRA_DATA_SOURCE_H
