/*!
 * \file ion_gsms.h
 * \brief GNU Radio block that reads a Block from a file following ION's GNSS-SDR metadata standard
 * \author Víctor Castillo Agüero, 2024. victorcastilloaguero(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ION_GNSS_SDR_METADATA_STANDARD_H
#define GNSS_SDR_ION_GNSS_SDR_METADATA_STANDARD_H

#include "gnss_block_interface.h"
#include "ion_gsms_chunk_data.h"
#include <gnuradio/block.h>
#include <gnuradio/sync_block.h>
#include <filesystem>
#include <string>


class IONGSMSFileSource : public gr::sync_block
{
public:
    using sptr = gnss_shared_ptr<IONGSMSFileSource>;

    IONGSMSFileSource(
        const std::filesystem::path& metadata_filepath,
        const GnssMetadata::File& file,
        const GnssMetadata::Block& block,
        const std::vector<std::string>& stream_ids);

    ~IONGSMSFileSource() override;

    int work(
        int noutput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items) override;

    std::size_t output_stream_count() const;
    std::size_t output_stream_item_size(std::size_t stream_index) const;

private:
    static gr::io_signature::sptr make_output_signature(const GnssMetadata::Block& block, const std::vector<std::string>& stream_ids);

private:
    const GnssMetadata::File& file_metadata_;
    const GnssMetadata::Block& block_metadata_;
    FILE* fd_;
    std::size_t output_stream_count_;
    std::vector<std::size_t> output_stream_item_sizes_;
    std::vector<std::shared_ptr<IONGSMSChunkData>> chunk_data_;
};

#include "ion_gsms_metadata_handler.h"


#endif  // GNSS_SDR_ION_GNSS_SDR_METADATA_STANDARD_H
