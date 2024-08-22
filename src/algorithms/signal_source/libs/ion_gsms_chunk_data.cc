/*!
 * \file ion_gsms_chunk_data.cc
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

#include "ion_gsms_chunk_data.h"
#include <cstring>
#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


IONGSMSChunkData::IONGSMSChunkData(const GnssMetadata::Chunk& chunk, const std::vector<std::string>& stream_ids, std::size_t output_stream_offset)
    : chunk_(chunk),
      sizeword_(chunk_.SizeWord()),
      countwords_(chunk_.CountWords())
{
    // Instantiate the Allocator functor
    Allocator allocator(countwords_, buffer_);
    // Call with_word_type with the Allocator functor
    with_word_type(sizeword_, allocator);

    const std::size_t total_bitsize = sizeword_ * countwords_ * 8;
    std::size_t used_bitsize = 0;
    std::size_t output_streams = 0;
    for (const auto& lump : chunk.Lumps())
        {
            for (const auto& stream : lump.Streams())
                {
                    used_bitsize += stream.Packedbits();

                    bool found = false;
                    for (const auto& stream_id : stream_ids)
                        {
                            if (stream_id == stream.Id())
                                {
                                    found = true;
                                    break;
                                }
                        }
                    if (found)
                        {
                            streams_.emplace_back(lump, stream, GnssMetadata::encoding_from_string(stream.Encoding()), output_streams + output_stream_offset);
                            ++output_streams;
                            std::size_t sample_bitsize = stream.Packedbits() / stream.RateFactor();
                            std::size_t sample_rate = stream.RateFactor();
                            if (stream.Packedbits() >= 2 * stream.RateFactor() * stream.Quantization())
                                {
                                    // Samples have 'Complex' format
                                    sample_bitsize /= 2;
                                    sample_rate *= 2;
                                }
                            output_stream_item_size_.push_back(bits_to_item_size(sample_bitsize));
                            output_stream_item_rate_.push_back(sample_rate);
                        }
                    else
                        {
                            streams_.emplace_back(lump, stream, GnssMetadata::encoding_from_string(stream.Encoding()), -1);
                        }
                }
        }

    output_stream_count_ = output_streams;
    padding_bitsize_ = total_bitsize - used_bitsize;
}


IONGSMSChunkData::~IONGSMSChunkData()
{
    Deleter deleter(static_cast<void*>(buffer_));
    with_word_type(sizeword_, deleter);
}


std::size_t IONGSMSChunkData::read_from_buffer(uint8_t* buffer, std::size_t offset)
{
    memset(buffer_, 0, sizeword_ * countwords_);
    memcpy(buffer_, &buffer[offset], sizeword_ * countwords_);
    return sizeword_ * countwords_;
}


void IONGSMSChunkData::write_to_output(gr_vector_void_star& outputs, std::vector<int>& output_items)
{
    switch (sizeword_)
        {
        case 1:
            unpack_words<int8_t>(outputs, output_items);
            break;
        case 2:
            unpack_words<int16_t>(outputs, output_items);
            break;
        case 4:
            unpack_words<int32_t>(outputs, output_items);
            break;
        case 8:
            unpack_words<int64_t>(outputs, output_items);
            break;
        default:
            LOG(ERROR) << "Unknown word size (" << std::to_string(sizeword_) << "), unpacking nothing.";
            break;
        }
}


std::size_t IONGSMSChunkData::output_stream_count() const
{
    return output_stream_count_;
}


std::size_t IONGSMSChunkData::output_stream_item_size(std::size_t stream_index) const
{
    return output_stream_item_size_[stream_index];
}


std::size_t IONGSMSChunkData::output_stream_item_rate(std::size_t stream_index) const
{
    return output_stream_item_rate_[stream_index];
}


template <typename WT>
void IONGSMSChunkData::unpack_words(gr_vector_void_star& outputs, std::vector<int>& output_items)
{
    WT* data = static_cast<WT*>(buffer_);
    // TODO - Swap endiannes if needed

    IONGSMSChunkUnpackingCtx<WT> ctx{
        chunk_.Shift(),
        data,
        countwords_,
    };

    // Head padding
    if (padding_bitsize_ > 0 && chunk_.Padding() == GnssMetadata::Chunk::Head)
        {
            ctx.shift_padding(padding_bitsize_);
        }

    // Samples
    for (const auto& [lump, stream, encoding, output_index] : streams_)
        {
            if (output_index == -1)
                {
                    // skip stream
                    ctx.shift_padding(stream.Packedbits());
                }
            else
                {
                    output_items[output_index] += write_stream_samples(ctx, lump, stream, encoding, &outputs[output_index]);
                }
        }
}


template <typename WT>
std::size_t IONGSMSChunkData::write_stream_samples(
    IONGSMSChunkUnpackingCtx<WT>& ctx,
    const GnssMetadata::Lump& lump,
    const GnssMetadata::IonStream& stream,
    const GnssMetadata::StreamEncoding stream_encoding,
    void** out)
{
    std::size_t sample_bitsize = stream.Packedbits() / stream.RateFactor();
    std::size_t sample_count = stream.RateFactor();

    if (stream.Packedbits() >= 2 * stream.RateFactor() * stream.Quantization())
        {
            // Samples have 'Complex' format
            sample_bitsize /= 2;
            sample_count *= 2;
        }

    if (sample_bitsize <= 8)
        {
            write_n_samples<WT, int8_t>(ctx, lump.Shift(), sample_bitsize, sample_count, stream_encoding, reinterpret_cast<int8_t**>(out));
        }
    else if (sample_bitsize <= 16)
        {
            write_n_samples<WT, int16_t>(ctx, lump.Shift(), sample_bitsize, sample_count, stream_encoding, reinterpret_cast<int16_t**>(out));
        }
    else if (sample_bitsize <= 32)
        {
            write_n_samples<WT, int32_t>(ctx, lump.Shift(), sample_bitsize, sample_count, stream_encoding, reinterpret_cast<int32_t**>(out));
        }
    else if (sample_bitsize <= 64)
        {
            write_n_samples<WT, int64_t>(ctx, lump.Shift(), sample_bitsize, sample_count, stream_encoding, reinterpret_cast<int64_t**>(out));
        }

    return sample_count;
}


template <typename WT, typename OT>
void IONGSMSChunkData::write_n_samples(
    IONGSMSChunkUnpackingCtx<WT>& ctx,
    GnssMetadata::Lump::LumpShift lump_shift,
    uint8_t sample_bitsize,
    std::size_t sample_count,
    GnssMetadata::StreamEncoding stream_encoding,
    OT** out)
{
    if (lump_shift == GnssMetadata::Lump::shiftRight)
        {
            auto* sample = static_cast<OT*>(*out);
            sample += sample_count;
            for (std::size_t i = 0; i < sample_count; ++i)
                {
                    *sample = 0;
                    ctx.shift_sample(sample_bitsize, sample);
                    decode_sample(sample_bitsize, sample, stream_encoding);
                    --sample;
                }
        }
    else  // if (lump_shift == GnssMetadata::Lump::shiftLeft || lump_shift == GnssMetadata::Lump::shiftUndefined)
        {
            auto* sample = static_cast<OT*>(*out);
            for (std::size_t i = 0; i < sample_count; ++i)
                {
                    *sample = 0;
                    ctx.shift_sample(sample_bitsize, sample);
                    decode_sample(sample_bitsize, sample, stream_encoding);
                    ++sample;
                }
        }

    (*out) += sample_count;
}


// Static utilities
template <typename Sample>
void IONGSMSChunkData::decode_sample(const uint8_t sample_bitsize, Sample* sample, const GnssMetadata::StreamEncoding encoding)
{
    // using SampleType = std::remove_pointer_t<decltype(sample)>;
    switch (sample_bitsize)
        {
        case 2:
            *sample = GnssMetadata::two_bit_look_up<Sample>[encoding][*sample];
            break;
        case 3:
            *sample = GnssMetadata::three_bit_look_up<Sample>[encoding][*sample];
            break;
        case 4:
            *sample = GnssMetadata::four_bit_look_up<Sample>[encoding][*sample];
            break;
        case 5:
            *sample = GnssMetadata::five_bit_look_up<Sample>[encoding][*sample];
            break;
        default:
            // TODO - Is this an error that can happen?
            // for now we'll just do nothing, if the sample is this wide it may need no decoding
            break;
        }
}
