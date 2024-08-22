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
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ION_GSMS_CHUNK_DATA_H
#define GNSS_SDR_ION_GSMS_CHUNK_DATA_H

#include "ion_gsms_chunk_unpacking_ctx.h"
#include "ion_gsms_stream_encodings.h"
#include <gnuradio/block.h>
#include <GnssMetadata.h>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>


inline std::size_t bits_to_item_size(std::size_t bit_count)
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
    std::cerr << "Item size too large (" << std::to_string(bit_count) << "), returning nonsense.\n";
    return 1;
}


// Define a functor that has a templated operator()
struct Allocator
{
    size_t countwords_;
    void*& buffer_;  // Using void* to hold any type of pointer

    Allocator(size_t countwords, void*& buffer)
        : countwords_(countwords), buffer_(buffer) {}

    template <typename WordType>
    void operator()() const
    {
        buffer_ = new WordType[countwords_];
    }
};


// Define a functor to delete the allocated memory
struct Deleter
{
    void* buffer_;

    explicit Deleter(void* buffer)
        : buffer_(buffer) {}

    template <typename WordType>
    void operator()() const
    {
        delete[] static_cast<WordType*>(buffer_);
    }
};


template <typename Callback>
void with_word_type(uint8_t word_size, Callback callback)
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
            std::cerr << "Unknown word size (" << std::to_string(word_size) << "), returning nonsense.\n";
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
    std::size_t output_stream_item_rate(std::size_t stream_index) const;

private:
    template <typename WT>
    void unpack_words(gr_vector_void_star& outputs, std::vector<int>& output_items);

    template <typename WT>
    std::size_t write_stream_samples(
        IONGSMSChunkUnpackingCtx<WT>& ctx,
        const GnssMetadata::Lump& lump,
        const GnssMetadata::IonStream& stream,
        GnssMetadata::StreamEncoding stream_encoding,
        void** out);

    template <typename WT, typename OT>
    void write_n_samples(
        IONGSMSChunkUnpackingCtx<WT>& ctx,
        GnssMetadata::Lump::LumpShift lump_shift,
        uint8_t sample_bitsize,
        std::size_t sample_count,
        GnssMetadata::StreamEncoding stream_encoding,
        OT** out);

    template <typename Sample>
    static void decode_sample(uint8_t sample_bitsize, Sample* sample, GnssMetadata::StreamEncoding encoding);

    const GnssMetadata::Chunk& chunk_;
    uint8_t sizeword_;
    uint8_t countwords_;
    uint8_t padding_bitsize_;
    std::size_t output_stream_count_;
    std::vector<std::size_t> output_stream_item_size_;
    std::vector<std::size_t> output_stream_item_rate_;

    struct stream_metadata_t
    {
        const GnssMetadata::Lump& lump;
        const GnssMetadata::IonStream& stream;
        GnssMetadata::StreamEncoding stream_encoding;
        int output_index = -1;

        stream_metadata_t(
            const GnssMetadata::Lump& lump_,
            const GnssMetadata::IonStream& stream_,
            GnssMetadata::StreamEncoding stream_encoding_,
            int output_index_ = -1) : lump(lump_),
                                      stream(stream_),
                                      stream_encoding(stream_encoding_),
                                      output_index(output_index_)
        {
        }
    };
    std::vector<stream_metadata_t> streams_;

    void* buffer_;
};

#endif  // GNSS_SDR_ION_GSMS_CHUNK_DATA_H
