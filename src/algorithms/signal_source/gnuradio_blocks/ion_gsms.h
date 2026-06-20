/*!
 * \file ion_gsms.h
 * \brief GNU Radio block that reads a Block from a file following ION's GNSS-SDR metadata standard
 * \author Víctor Castillo Agüero, 2024. victorcastilloaguero(at)gmail.com
 * \author Carles Fernandez, 2026 carles.fernandez(at)cttc.es
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

#ifndef GNSS_SDR_ION_GSMS_H
#define GNSS_SDR_ION_GSMS_H

#include "gnss_block_interface.h"
#include "gnss_sdr_filesystem.h"
#include "ion_gsms_chunk_data.h"
#include <gnuradio/block.h>
#include <gnuradio/sync_block.h>
#include <cstddef>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */

class IONGSMSFileSource : public gr::sync_block
{
public:
    using sptr = gnss_shared_ptr<IONGSMSFileSource>;

    IONGSMSFileSource(
        const fs::path& metadata_filepath,
        const GnssMetadata::File& file,
        const GnssMetadata::Block& block,
        std::size_t block_start_offset,
        const std::vector<std::string>& stream_ids);

    int work(
        int noutput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items) override;

    std::size_t output_stream_count() const;
    std::size_t output_stream_item_size(std::size_t stream_index) const;
    std::size_t output_stream_total_sample_count(std::size_t stream_index) const;

private:
    static gr::io_signature::sptr make_output_signature(const GnssMetadata::Block& block, const std::vector<std::string>& stream_ids);
    static bool block_contains_stream_id(const GnssMetadata::Block& block, const std::string& stream_id);
    static std::vector<std::string> block_output_stream_ids(const GnssMetadata::Block& block, const std::vector<std::string>& stream_ids);
    static int output_item_size_for_stream_id(const GnssMetadata::Block& block, const std::string& stream_id);
    static std::size_t infer_cycle_count_from_file(
        const fs::path& data_filepath,
        const GnssMetadata::Block& block,
        std::size_t block_start_offset,
        std::size_t chunk_cycle_length);

    std::ifstream file_stream_;
    std::vector<char> io_buffer_;
    std::size_t io_buffer_offset_;
    std::vector<int> items_produced_;
    std::size_t output_stream_count_;
    std::vector<std::size_t> output_stream_item_sizes_;
    std::vector<std::size_t> output_stream_item_rates_;
    std::vector<std::size_t> output_stream_total_sample_counts_;
    std::size_t maximum_item_rate_;
    std::vector<std::shared_ptr<IONGSMSChunkData>> chunk_data_;
    std::size_t chunk_cycle_length_;
    std::size_t cycles_remaining_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_ION_GSMS_H
