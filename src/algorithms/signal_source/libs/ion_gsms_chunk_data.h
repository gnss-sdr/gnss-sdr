/*!
 * \file ion_gsms_chunk_data.h
 * \brief Holds logic for reading and decoding samples from a chunk
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

#ifndef ION_GSM_CHUNK_DATA_H
#define ION_GSM_CHUNK_DATA_H

#include "GnssMetadata.h"
#include "ion_gsms_stream_encodings.h"
#include "ion_gsms_chunk_unpacking_ctx.h"
#include <gnuradio/block.h>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

inline std::size_t bits_to_item_size(const std::size_t bit_count)
{
    if (bit_count <= 8)
        {
            return 1;
        }
    if (bit_count <= 16)
        {
            return 2;
        }
    if (bit_count <= 32)
        {
            return 4;
        }
    if (bit_count <= 64)
        {
            return 8;
        }

    // You are asking too much of this humble processor
    LOG(ERROR) << "Item size too large (" << std::to_string(bit_count) << "), returning nonsense.";
    return 1;
}

void with_word_type(const uint8_t word_size, auto&& callback)
{
    switch (word_size)
        {
        case 1:
            callback.template operator()<int8_t>();
            break;
        case 2:
            callback.template operator()<int16_t>();
            break;
        case 4:
            callback.template operator()<int32_t>();
            break;
        case 8:
            callback.template operator()<int64_t>();
            break;
        default:
            LOG(ERROR) << "Unknown word size (" << std::to_string(word_size) << "), returning nonsense.";
            break;
        }
}

class IONGSMSChunkData
{
public:
    IONGSMSChunkData(const GnssMetadata::Chunk& chunk, const std::vector<std::string>& stream_ids, std::size_t output_stream_offset);

    ~IONGSMSChunkData();

    IONGSMSChunkData(const IONGSMSChunkData& rhl) = delete;
    IONGSMSChunkData& operator=(const IONGSMSChunkData& rhl) = delete;

    IONGSMSChunkData(IONGSMSChunkData&& rhl) = delete;
    IONGSMSChunkData& operator=(IONGSMSChunkData&& rhl) = delete;

    std::size_t read_from_buffer(uint8_t* buffer, std::size_t offset);

    void write_to_output(gr_vector_void_star& outputs, std::vector<int>& output_items);

    std::size_t output_stream_count() const;
    std::size_t output_stream_item_size(std::size_t stream_index) const;

private:
    template <typename WT>
    void unpack_words(gr_vector_void_star& outputs, std::vector<int>& output_items);

    template <typename WT>
    std::size_t write_stream_samples(
        IONGSMSChunkUnpackingCtx<WT>& ctx,
        const GnssMetadata::Lump& lump,
        const GnssMetadata::IonStream& stream,
        GnssMetadata::StreamEncoding stream_encoding,
        void*& out);

    template <typename WT, typename OT>
    void write_n_samples(
        IONGSMSChunkUnpackingCtx<WT>& ctx,
        GnssMetadata::Lump::LumpShift lump_shift,
        uint8_t sample_bitsize,
        std::size_t sample_count,
        GnssMetadata::StreamEncoding stream_encoding,
        void*& out);

    static void decode_sample(uint8_t sample_bitsize, auto* sample, GnssMetadata::StreamEncoding encoding);

    static void dump_sample(auto value);

private:
    const GnssMetadata::Chunk& chunk_;
    uint8_t sizeword_;
    uint8_t countwords_;
    uint8_t padding_bitsize_;
    std::size_t output_stream_count_;
    std::vector<std::size_t> output_stream_item_size_;

    struct stream_metadata_t
    {
        const GnssMetadata::Lump& lump;
        const GnssMetadata::IonStream& stream;
        GnssMetadata::StreamEncoding stream_encoding;
        int output_index = -1;
    };
    std::vector<stream_metadata_t> streams_;

    void* buffer_;
};

#endif //ION_GSM_CHUNK_DATA_H
