/*!
 * \file ion_gsms_chunk_data.cc
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

#include "ion_gsms_chunk_data.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


IONGSMSChunkData::stream_metadata_t::stream_metadata_t(
    const GnssMetadata::Lump& lump,
    const GnssMetadata::IonStream& stream,
    GnssMetadata::StreamEncoding stream_encoding_,
    int output_index_,
    std::size_t output_item_offset_,
    std::size_t output_item_size_)
    : stream_encoding(stream_encoding_),
      format(stream.Format()),
      alignment(stream.Alignment()),
      output_index(output_index_),
      output_item_offset(output_item_offset_),
      output_item_size(output_item_size_),
      packed_bits(stream.Packedbits())
{
    if (output_index == -1)
        {
            // Skipped streams only need their packed size
            return;
        }

    padding_bits = stream_padding_bits(stream);
    if (padding_bits > 0 && alignment == GnssMetadata::IonStream::Undefined)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source stream has packed padding bits but no alignment metadata");
        }
    if (stream.Quantization() == 0 || stream.Quantization() > 64)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source unsupported sample quantization");
        }
    rate_factor = stream.RateFactor();
    sample_bitsize = static_cast<uint8_t>(stream.Quantization());
    output_item_bits = stream_output_item_bits(stream);
    is_complex = stream_is_complex(format);
    reversed = samples_are_reversed(stream.Shift(), lump.Shift());
}


IONGSMSChunkData::IONGSMSChunkData(const GnssMetadata::Chunk& chunk, const std::vector<std::string>& stream_ids, std::size_t output_stream_offset)
    : chunk_(chunk),
      sizeword_(chunk_.SizeWord()),
      countwords_(chunk_.CountWords()),
      head_padding_(chunk_.Padding() == GnssMetadata::Chunk::Head),
      read_lsb_first_(chunk_.Shift() == GnssMetadata::Chunk::Right),
      swap_endianness_(source_endianness_is_different(chunk_.Endian())),
      output_stream_offset_(output_stream_offset)
{
    // Instantiate the Allocator functor
    Allocator allocator(countwords_, buffer_);
    // Call with_word_type with the Allocator functor
    with_word_type(sizeword_, allocator);

    const std::size_t total_bitsize = sizeword_ * countwords_ * 8;
    std::size_t pattern_bitsize = 0;
    for (const auto& lump : chunk.Lumps())
        {
            std::size_t lump_bitsize = 0;
            for (const auto& stream : lump.Streams())
                {
                    lump_bitsize += stream.Packedbits();
                }

            if (lump_bitsize == 0)
                {
                    throw std::runtime_error("ION_GSMS_Signal_Source lump must occupy at least one bit");
                }
            pattern_bitsize += lump_bitsize;
        }

    if (pattern_bitsize == 0)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source chunk must occupy at least one bit");
        }
    if (pattern_bitsize > total_bitsize)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source metadata describes a lump pattern larger than its chunk");
        }

    output_stream_count_ = stream_ids.size();
    output_stream_item_size_.assign(output_stream_count_, 0);
    output_stream_item_rate_.assign(output_stream_count_, 0);
    std::vector<bool> output_stream_seen(output_stream_count_, false);
    for (const auto& lump : chunk.Lumps())
        {
            for (const auto& stream : lump.Streams())
                {
                    const auto stream_id = std::find(stream_ids.begin(), stream_ids.end(), stream.Id());
                    const bool found = stream_id != stream_ids.end();
                    const auto stream_encoding = GnssMetadata::encoding_from_string(stream.Encoding());
                    int output_index = -1;
                    std::size_t output_item_offset = 0;
                    std::size_t output_item_rate = 0;
                    std::size_t output_item_size = 0;
                    if (found)
                        {
                            const auto relative_output_index = static_cast<std::size_t>(std::distance(stream_ids.begin(), stream_id));
                            output_index = static_cast<int>(relative_output_index + output_stream_offset);
                            output_item_rate = stream_output_item_rate(stream);
                            output_item_size = stream_output_item_size(stream);
                            output_item_offset = output_stream_item_rate_[relative_output_index];
                            output_stream_item_rate_[relative_output_index] += output_item_rate;

                            if (output_stream_item_size_[relative_output_index] != 0 &&
                                output_stream_item_size_[relative_output_index] != output_item_size)
                                {
                                    throw std::runtime_error("ION_GSMS_Signal_Source stream appears with inconsistent output item sizes");
                                }
                            output_stream_item_size_[relative_output_index] = output_item_size;

                            if (!output_stream_seen[relative_output_index])
                                {
                                    output_stream_indices_.push_back(output_index);
                                    output_stream_seen[relative_output_index] = true;
                                }
                        }

                    streams_.emplace_back(lump, stream, stream_encoding, output_index, output_item_offset, output_item_size);
                }
        }

    padding_bitsize_ = total_bitsize - pattern_bitsize;

    build_byte_lut();
}


void IONGSMSChunkData::build_byte_lut()
{
    if (sizeword_ * countwords_ != 1 || output_stream_indices_.empty())
        {
            return;
        }

    byte_lut_output_offset_.assign(output_stream_count_, 0);
    byte_lut_row_size_ = 0;
    for (std::size_t i = 0; i < output_stream_count_; ++i)
        {
            if (output_stream_item_rate_[i] > 0 && output_stream_item_size_[i] != sizeof(int8_t))
                {
                    byte_lut_output_offset_.clear();
                    byte_lut_row_size_ = 0;
                    return;
                }
            byte_lut_output_offset_[i] = byte_lut_row_size_;
            byte_lut_row_size_ += output_stream_item_rate_[i];
        }

    // Run the generic decoder once for every possible chunk value
    byte_lut_.assign(256 * byte_lut_row_size_, 0);
    gr_vector_void_star outputs(output_stream_offset_ + output_stream_count_, nullptr);
    std::vector<int> output_items(outputs.size(), 0);
    auto* chunk_byte = static_cast<uint8_t*>(buffer_);
    for (std::size_t value = 0; value < 256; ++value)
        {
            *chunk_byte = static_cast<uint8_t>(value);
            for (std::size_t i = 0; i < output_stream_count_; ++i)
                {
                    // An output without items in this chunk may point one past the end
                    outputs[output_stream_offset_ + i] = byte_lut_.data() + value * byte_lut_row_size_ + byte_lut_output_offset_[i];
                }
            unpack_words<uint8_t>(outputs, output_items);
        }
}


void IONGSMSChunkData::write_from_byte_lut(gr_vector_void_star& outputs, std::vector<int>& output_items) const
{
    const int8_t* row = byte_lut_.data() + static_cast<const uint8_t*>(buffer_)[0] * byte_lut_row_size_;
    for (int output_index : output_stream_indices_)
        {
            const auto relative_output_index = static_cast<std::size_t>(output_index) - output_stream_offset_;
            const auto item_count = output_stream_item_rate_[relative_output_index];
            const int8_t* items = row + byte_lut_output_offset_[relative_output_index];
            auto* out = static_cast<int8_t*>(outputs[output_index]);
            std::copy(items, items + item_count, out);
            outputs[output_index] = out + item_count;
            output_items[output_index] += static_cast<int>(item_count);
        }
}


IONGSMSChunkData::~IONGSMSChunkData()
{
    Deleter deleter(static_cast<void*>(buffer_));
    with_word_type(sizeword_, deleter);
}


uint64_t IONGSMSChunkData::low_bits_mask(const std::size_t bits)
{
    if (bits >= 64)
        {
            return std::numeric_limits<uint64_t>::max();
        }
    return (uint64_t{1} << bits) - 1U;
}


template <typename Word>
Word IONGSMSChunkData::byte_swap_word(Word value)
{
    if constexpr (sizeof(Word) == 1)
        {
            return value;
        }

    Word swapped = 0;
    for (std::size_t byte = 0; byte < sizeof(Word); ++byte)
        {
            swapped <<= 8U;
            swapped |= (value >> (byte * 8U)) & static_cast<Word>(0xFFU);
        }
    return swapped;
}


bool IONGSMSChunkData::host_is_little_endian()
{
    const uint16_t one = 1;
    return *reinterpret_cast<const uint8_t*>(&one) == 1U;
}


bool IONGSMSChunkData::source_endianness_is_different(const GnssMetadata::Chunk::WordEndian endian)
{
    if (endian == GnssMetadata::Chunk::Undefined)
        {
            return false;
        }

    return (endian == GnssMetadata::Chunk::Little) != host_is_little_endian();
}


std::size_t IONGSMSChunkData::read_from_buffer(uint8_t* buffer, std::size_t offset)
{
    memcpy(buffer_, &buffer[offset], sizeword_ * countwords_);
    return sizeword_ * countwords_;
}


void IONGSMSChunkData::write_to_output(gr_vector_void_star& outputs, std::vector<int>& output_items)
{
    if (!byte_lut_.empty())
        {
            write_from_byte_lut(outputs, output_items);
            return;
        }

    switch (sizeword_)
        {
        case 1:
            unpack_words<uint8_t>(outputs, output_items);
            break;
        case 2:
            unpack_words<uint16_t>(outputs, output_items);
            break;
        case 4:
            unpack_words<uint32_t>(outputs, output_items);
            break;
        case 8:
            unpack_words<uint64_t>(outputs, output_items);
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
    if (swap_endianness_)
        {
            for (std::size_t i = 0; i < countwords_; ++i)
                {
                    data[i] = byte_swap_word(data[i]);
                }
        }

    IONGSMSChunkUnpackingCtx<WT> ctx{data, countwords_, read_lsb_first_};

    // Head padding
    if (padding_bitsize_ > 0 && head_padding_)
        {
            ctx.shift_padding(padding_bitsize_);
        }

    // Samples
    for (const auto& stream : streams_)
        {
            if (stream.output_index == -1)
                {
                    // skip stream
                    ctx.shift_padding(stream.packed_bits);
                }
            else
                {
                    void* output = static_cast<char*>(outputs[stream.output_index]) +
                                   stream.output_item_offset * stream.output_item_size;
                    output_items[stream.output_index] += write_stream_samples(ctx, stream, &output);
                }
        }

    // Each chunk writes exactly output_stream_item_rate_ items to each of its outputs
    for (int output_index : output_stream_indices_)
        {
            const auto relative_output_index = static_cast<std::size_t>(output_index) - output_stream_offset_;
            outputs[output_index] = static_cast<char*>(outputs[output_index]) +
                                    output_stream_item_rate_[relative_output_index] * output_stream_item_size_[relative_output_index];
        }
}


template <typename WT>
std::size_t IONGSMSChunkData::write_stream_samples(
    IONGSMSChunkUnpackingCtx<WT>& ctx,
    const stream_metadata_t& stream,
    void** out)
{
    if (stream.padding_bits > 0 && stream.alignment == GnssMetadata::IonStream::Right)
        {
            ctx.shift_padding(stream.padding_bits);
        }

    std::size_t items_written = 0;
    if (stream.stream_encoding == GnssMetadata::StreamEncodings::FP)
        {
            items_written = write_fp32_samples(ctx, stream, out);
        }
    else
        {
            if (stream.output_item_bits <= 8)
                {
                    items_written = write_n_samples<WT, int8_t>(ctx, stream, reinterpret_cast<int8_t**>(out));
                }
            else if (stream.output_item_bits <= 16)
                {
                    items_written = write_n_samples<WT, int16_t>(ctx, stream, reinterpret_cast<int16_t**>(out));
                }
            else if (stream.output_item_bits <= 32)
                {
                    items_written = write_n_samples<WT, int32_t>(ctx, stream, reinterpret_cast<int32_t**>(out));
                }
            else if (stream.output_item_bits <= 64)
                {
                    items_written = write_n_samples<WT, int64_t>(ctx, stream, reinterpret_cast<int64_t**>(out));
                }
        }

    if (stream.padding_bits > 0 && stream.alignment == GnssMetadata::IonStream::Left)
        {
            ctx.shift_padding(stream.padding_bits);
        }

    return items_written;
}


template <typename WT, typename OT>
std::size_t IONGSMSChunkData::write_n_samples(
    IONGSMSChunkUnpackingCtx<WT>& ctx,
    const stream_metadata_t& stream,
    OT** out)
{
    const std::size_t sample_count = stream.rate_factor;
    const std::size_t items_per_sample = stream.is_complex ? 2U : 1U;
    auto* sample_out = static_cast<OT*>(*out);

    // Samples are read in packing order and written straight to their chronological position
    for (std::size_t i = 0; i < sample_count; ++i)
        {
            OT* sample = sample_out + (stream.reversed ? sample_count - 1 - i : i) * items_per_sample;
            const auto first = decode_sample<OT>(stream.sample_bitsize, ctx.read_bits(stream.sample_bitsize), stream.stream_encoding);
            if (!stream.is_complex)
                {
                    sample[0] = (stream.format == GnssMetadata::IonStream::IFn) ? static_cast<OT>(-first) : first;
                    continue;
                }

            const auto second = decode_sample<OT>(stream.sample_bitsize, ctx.read_bits(stream.sample_bitsize), stream.stream_encoding);
            switch (stream.format)
                {
                case GnssMetadata::IonStream::IQ:
                case GnssMetadata::IonStream::Int8IQ:
                case GnssMetadata::IonStream::Int16IQ:
                    sample[0] = first;
                    sample[1] = second;
                    break;
                case GnssMetadata::IonStream::IQn:
                    sample[0] = first;
                    sample[1] = static_cast<OT>(-second);
                    break;
                case GnssMetadata::IonStream::InQ:
                    sample[0] = static_cast<OT>(-first);
                    sample[1] = second;
                    break;
                case GnssMetadata::IonStream::InQn:
                    sample[0] = static_cast<OT>(-first);
                    sample[1] = static_cast<OT>(-second);
                    break;
                case GnssMetadata::IonStream::QI:
                    sample[0] = second;
                    sample[1] = first;
                    break;
                case GnssMetadata::IonStream::QIn:
                    sample[0] = static_cast<OT>(-second);
                    sample[1] = first;
                    break;
                case GnssMetadata::IonStream::QnI:
                    sample[0] = second;
                    sample[1] = static_cast<OT>(-first);
                    break;
                case GnssMetadata::IonStream::QnIn:
                    sample[0] = static_cast<OT>(-second);
                    sample[1] = static_cast<OT>(-first);
                    break;
                default:
                    throw std::runtime_error("ION_GSMS_Signal_Source unsupported complex stream format");
                }
        }

    *out = sample_out + sample_count * items_per_sample;
    return sample_count * items_per_sample;
}


template <typename WT>
std::size_t IONGSMSChunkData::write_fp32_samples(
    IONGSMSChunkUnpackingCtx<WT>& ctx,
    const stream_metadata_t& stream,
    void** out)
{
    if (stream.sample_bitsize != 32)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source only supports FP32 stream encoding");
        }

    const std::size_t sample_count = stream.rate_factor;
    if (!stream.is_complex)
        {
            auto* sample_out = static_cast<float*>(*out);
            for (std::size_t i = 0; i < sample_count; ++i)
                {
                    auto sample = read_fp32_sample(ctx);
                    if (stream.format == GnssMetadata::IonStream::IFn)
                        {
                            sample = -sample;
                        }
                    sample_out[stream.reversed ? sample_count - 1 - i : i] = sample;
                }

            *out = sample_out + sample_count;
            return sample_count;
        }

    auto* sample_out = static_cast<gr_complex*>(*out);
    for (std::size_t i = 0; i < sample_count; ++i)
        {
            const auto first = read_fp32_sample(ctx);
            const auto second = read_fp32_sample(ctx);
            sample_out[stream.reversed ? sample_count - 1 - i : i] = complex_sample_from_format(stream.format, first, second);
        }

    *out = sample_out + sample_count;
    return sample_count;
}


// Static utilities
template <typename Sample>
Sample IONGSMSChunkData::decode_sample(const uint8_t sample_bitsize, const uint64_t raw_sample, const GnssMetadata::StreamEncoding encoding)
{
    if (sample_bitsize == 0 || sample_bitsize > 64)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source unsupported sample quantization");
        }

    const auto raw = raw_sample & low_bits_mask(sample_bitsize);
    const auto magnitude_mask = low_bits_mask(sample_bitsize - 1U);
    int64_t decoded = 0;

    switch (encoding)
        {
        case GnssMetadata::StreamEncodings::SIGN:
            decoded = (raw & 0x01U) ? -1 : 1;
            break;
        case GnssMetadata::StreamEncodings::OB:
            decoded = static_cast<int64_t>(raw) - static_cast<int64_t>(uint64_t{1} << (sample_bitsize - 1U));
            break;
        case GnssMetadata::StreamEncodings::SM:
            decoded = (raw & (uint64_t{1} << (sample_bitsize - 1U))) ? -static_cast<int64_t>(raw & magnitude_mask) : static_cast<int64_t>(raw & magnitude_mask);
            break;
        case GnssMetadata::StreamEncodings::MS:
            decoded = (raw & 0x01U) ? -static_cast<int64_t>((raw >> 1U) & magnitude_mask) : static_cast<int64_t>((raw >> 1U) & magnitude_mask);
            break;
        case GnssMetadata::StreamEncodings::TC:
            if ((raw & (uint64_t{1} << (sample_bitsize - 1U))) != 0)
                {
                    decoded = static_cast<int64_t>(raw | ~low_bits_mask(sample_bitsize));
                }
            else
                {
                    decoded = static_cast<int64_t>(raw);
                }
            break;
        case GnssMetadata::StreamEncodings::OG:
            {
                uint64_t binary = raw;
                for (uint64_t mask = binary >> 1U; mask != 0; mask >>= 1U)
                    {
                        binary ^= mask;
                    }
                decoded = static_cast<int64_t>(binary) - static_cast<int64_t>(uint64_t{1} << (sample_bitsize - 1U));
                break;
            }
        case GnssMetadata::StreamEncodings::OBA:
            decoded = static_cast<int64_t>(raw << 1U) - static_cast<int64_t>((uint64_t{1} << sample_bitsize) - 1U);
            break;
        case GnssMetadata::StreamEncodings::SMA:
            decoded = (raw & (uint64_t{1} << (sample_bitsize - 1U))) ? -static_cast<int64_t>(((raw & magnitude_mask) << 1U) | 0x01U) : static_cast<int64_t>(((raw & magnitude_mask) << 1U) | 0x01U);
            break;
        case GnssMetadata::StreamEncodings::MSA:
            decoded = (raw & 0x01U) ? -static_cast<int64_t>((((raw >> 1U) & magnitude_mask) << 1U) | 0x01U) : static_cast<int64_t>((((raw >> 1U) & magnitude_mask) << 1U) | 0x01U);
            break;
        case GnssMetadata::StreamEncodings::TCA:
            {
                const auto shifted = raw << 1U;
                if ((raw & (uint64_t{1} << (sample_bitsize - 1U))) != 0)
                    {
                        decoded = static_cast<int64_t>(shifted | ~low_bits_mask(sample_bitsize + 1U)) + 1;
                    }
                else
                    {
                        decoded = static_cast<int64_t>(shifted) + 1;
                    }
                break;
            }
        case GnssMetadata::StreamEncodings::OGA:
            {
                uint64_t binary = raw;
                for (uint64_t mask = binary >> 1U; mask != 0; mask >>= 1U)
                    {
                        binary ^= mask;
                    }
                decoded = static_cast<int64_t>(binary << 1U) - static_cast<int64_t>((uint64_t{1} << sample_bitsize) - 1U);
                break;
            }
        default:
            throw std::runtime_error("ION_GSMS_Signal_Source unsupported stream encoding");
        }

    return static_cast<Sample>(decoded);
}


template <typename WT>
float IONGSMSChunkData::read_fp32_sample(IONGSMSChunkUnpackingCtx<WT>& ctx)
{
    const auto raw_sample = static_cast<uint32_t>(ctx.read_bits(32));
    float sample = 0.0F;
    static_assert(sizeof(sample) == sizeof(raw_sample), "FP32 sample storage must be 32 bits");
    std::memcpy(&sample, &raw_sample, sizeof(sample));
    return sample;
}


gr_complex IONGSMSChunkData::complex_sample_from_format(const GnssMetadata::IonStream::SampleFormat format, const float first, const float second)
{
    switch (format)
        {
        case GnssMetadata::IonStream::IQ:
        case GnssMetadata::IonStream::Int8IQ:
        case GnssMetadata::IonStream::Int16IQ:
            return {first, second};
        case GnssMetadata::IonStream::IQn:
            return {first, -second};
        case GnssMetadata::IonStream::InQ:
            return {-first, second};
        case GnssMetadata::IonStream::InQn:
            return {-first, -second};
        case GnssMetadata::IonStream::QI:
            return {second, first};
        case GnssMetadata::IonStream::QIn:
            return {-second, first};
        case GnssMetadata::IonStream::QnI:
            return {second, -first};
        case GnssMetadata::IonStream::QnIn:
            return {-second, -first};
        default:
            throw std::runtime_error("ION_GSMS_Signal_Source unsupported complex stream format");
        }
}


bool IONGSMSChunkData::stream_is_complex(const GnssMetadata::IonStream::SampleFormat format)
{
    return format != GnssMetadata::IonStream::IF && format != GnssMetadata::IonStream::IFn;
}


uint8_t IONGSMSChunkData::stream_output_item_bits(const GnssMetadata::IonStream& stream)
{
    std::size_t output_bits = stream.Quantization();
    const auto encoding = GnssMetadata::encoding_from_string(stream.Encoding());
    if (encoding == GnssMetadata::StreamEncodings::UNKNOWN)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source unknown stream encoding: " + stream.Encoding());
        }
    if (encoding == GnssMetadata::StreamEncodings::FP)
        {
            if (stream.Quantization() != 32)
                {
                    throw std::runtime_error("ION_GSMS_Signal_Source only supports FP32 stream encoding");
                }
            return stream_is_complex(stream.Format()) ? static_cast<uint8_t>(sizeof(gr_complex) * 8U) : 32U;
        }
    if ((encoding == GnssMetadata::StreamEncodings::OBA ||
            encoding == GnssMetadata::StreamEncodings::SMA ||
            encoding == GnssMetadata::StreamEncodings::MSA ||
            encoding == GnssMetadata::StreamEncodings::TCA ||
            encoding == GnssMetadata::StreamEncodings::OGA) &&
        output_bits >= 8)
        {
            ++output_bits;
        }

    if (output_bits == 0 || output_bits > 64)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source unsupported stream quantization");
        }

    return static_cast<uint8_t>(output_bits);
}


std::size_t IONGSMSChunkData::stream_output_item_size(const GnssMetadata::IonStream& stream)
{
    if (stream_outputs_gr_complex(stream))
        {
            stream_output_item_bits(stream);
            return sizeof(gr_complex);
        }

    return bits_to_item_size(stream_output_item_bits(stream));
}


std::size_t IONGSMSChunkData::stream_output_item_rate(const GnssMetadata::IonStream& stream)
{
    const std::size_t output_items_per_sample = stream_outputs_gr_complex(stream) ? 1U : (stream_is_complex(stream.Format()) ? 2U : 1U);
    return stream.RateFactor() * output_items_per_sample;
}


bool IONGSMSChunkData::stream_outputs_gr_complex(const GnssMetadata::IonStream& stream)
{
    return GnssMetadata::encoding_from_string(stream.Encoding()) == GnssMetadata::StreamEncodings::FP &&
           stream_is_complex(stream.Format());
}


bool IONGSMSChunkData::samples_are_reversed(const GnssMetadata::IonStream::StreamShift stream_shift, const GnssMetadata::Lump::LumpShift lump_shift)
{
    return stream_shift == GnssMetadata::IonStream::shiftRight ||
           (stream_shift == GnssMetadata::IonStream::shiftUndefined && lump_shift == GnssMetadata::Lump::shiftRight);
}


std::size_t IONGSMSChunkData::stream_padding_bits(const GnssMetadata::IonStream& stream)
{
    if (stream.RateFactor() == 0)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source stream ratefactor must be greater than zero");
        }

    const std::size_t components_per_sample = stream_is_complex(stream.Format()) ? 2U : 1U;
    const std::size_t used_bits = stream.RateFactor() * components_per_sample * stream.Quantization();
    if (stream.Packedbits() < used_bits)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source stream packedbits is smaller than its sample payload");
        }

    return stream.Packedbits() - used_bits;
}
