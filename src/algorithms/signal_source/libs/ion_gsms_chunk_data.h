/*!
 * \file ion_gsms_chunk_data.h
 * \brief Holds logic for reading and decoding samples from a chunk
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

#ifndef GNSS_SDR_ION_GSMS_CHUNK_DATA_H
#define GNSS_SDR_ION_GSMS_CHUNK_DATA_H

#include "ion_gsms_chunk_unpacking_ctx.h"
#include "ion_gsms_stream_encodings.h"
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <GnssMetadata.h>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
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

    throw std::runtime_error("ION_GSMS_Signal_Source item size is larger than 64 bits");
}


// Define a functor that has a templated operator()
struct Allocator
{
    std::size_t countwords_;
    void*& buffer_;  // Using void* to hold any type of pointer

    Allocator(std::size_t countwords, void*& buffer)
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
void with_word_type(std::size_t word_size, Callback callback)
{
    switch (word_size)
        {
        case 1:
            callback.template operator()<uint8_t>();
            break;
        case 2:
            callback.template operator()<uint16_t>();
            break;
        case 4:
            callback.template operator()<uint32_t>();
            break;
        case 8:
            callback.template operator()<uint64_t>();
            break;
        default:
            throw std::runtime_error("ION_GSMS_Signal_Source unsupported chunk word size: " + std::to_string(word_size));
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

    static bool stream_is_complex(GnssMetadata::IonStream::SampleFormat format);
    static uint8_t stream_output_item_bits(const GnssMetadata::IonStream& stream);
    static std::size_t stream_output_item_size(const GnssMetadata::IonStream& stream);
    static std::size_t stream_output_item_rate(const GnssMetadata::IonStream& stream);

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
    std::size_t write_n_samples(
        IONGSMSChunkUnpackingCtx<WT>& ctx,
        GnssMetadata::Lump::LumpShift lump_shift,
        const GnssMetadata::IonStream& stream,
        GnssMetadata::StreamEncoding stream_encoding,
        OT** out);

    template <typename WT>
    std::size_t write_fp32_samples(
        IONGSMSChunkUnpackingCtx<WT>& ctx,
        GnssMetadata::Lump::LumpShift lump_shift,
        const GnssMetadata::IonStream& stream,
        void** out);

    template <typename Sample>
    static Sample decode_sample(uint8_t sample_bitsize, uint64_t raw_sample, GnssMetadata::StreamEncoding encoding);

    template <typename WT>
    static float read_fp32_sample(IONGSMSChunkUnpackingCtx<WT>& ctx);

    static gr_complex complex_sample_from_format(GnssMetadata::IonStream::SampleFormat format, float first, float second);
    static bool stream_outputs_gr_complex(const GnssMetadata::IonStream& stream);
    static bool samples_are_reversed(GnssMetadata::IonStream::StreamShift stream_shift, GnssMetadata::Lump::LumpShift lump_shift);
    static std::size_t stream_padding_bits(const GnssMetadata::IonStream& stream);
    static uint64_t low_bits_mask(std::size_t bits);

    template <typename Word>
    static Word byte_swap_word(Word value);

    static bool host_is_little_endian();
    static bool source_endianness_is_different(GnssMetadata::Chunk::WordEndian endian);

    const GnssMetadata::Chunk& chunk_;
    std::size_t sizeword_;
    std::size_t countwords_;
    std::size_t padding_bitsize_;
    std::size_t output_stream_offset_;
    std::size_t output_stream_count_;
    std::vector<std::size_t> output_stream_item_size_;
    std::vector<std::size_t> output_stream_item_rate_;

    struct stream_metadata_t
    {
        const GnssMetadata::Lump& lump;
        const GnssMetadata::IonStream& stream;
        GnssMetadata::StreamEncoding stream_encoding;
        int output_index = -1;
        std::size_t output_item_offset = 0;
        std::size_t output_item_size = 0;

        stream_metadata_t(
            const GnssMetadata::Lump& lump_,
            const GnssMetadata::IonStream& stream_,
            GnssMetadata::StreamEncoding stream_encoding_,
            int output_index_ = -1,
            std::size_t output_item_offset_ = 0,
            std::size_t output_item_size_ = 0) : lump(lump_),
                                                 stream(stream_),
                                                 stream_encoding(stream_encoding_),
                                                 output_index(output_index_),
                                                 output_item_offset(output_item_offset_),
                                                 output_item_size(output_item_size_)
        {
        }
    };
    std::vector<stream_metadata_t> streams_;
    std::vector<int> output_stream_indices_;

    void* buffer_ = nullptr;
};

#endif  // GNSS_SDR_ION_GSMS_CHUNK_DATA_H
