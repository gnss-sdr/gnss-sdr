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

namespace GnssMetadata
{
    using StreamEncoding = unsigned char;

    namespace StreamEncodings
    {
        constexpr unsigned char SIGN = 0;
        constexpr unsigned char OB = 1;
        constexpr unsigned char SM = 2;
        constexpr unsigned char MS = 3;
        constexpr unsigned char TC = 4;
        constexpr unsigned char OG = 5;
        constexpr unsigned char OBA = 6;
        constexpr unsigned char SMA = 7;
        constexpr unsigned char MSA = 8;
        constexpr unsigned char TCA = 9;
        constexpr unsigned char OGA = 10;
        constexpr unsigned char FP = 11;
    }

    inline StreamEncoding encoding_from_string(const std::string& str)
    {
        if (str == "SIGN")
            {
                return StreamEncodings::SIGN;
            }
        if (str == "OB")
            {
                return StreamEncodings::OB;
            }
        if (str == "SM")
            {
                return StreamEncodings::SM;
            }
        if (str == "MS")
            {
                return StreamEncodings::MS;
            }
        if (str == "TC")
            {
                return StreamEncodings::TC;
            }
        if (str == "OG")
            {
                return StreamEncodings::OG;
            }
        if (str == "OBA")
            {
                return StreamEncodings::OBA;
            }
        if (str == "SMA")
            {
                return StreamEncodings::SMA;
            }
        if (str == "MSA")
            {
                return StreamEncodings::MSA;
            }
        if (str == "TCA")
            {
                return StreamEncodings::TCA;
            }
        if (str == "OGA")
            {
                return StreamEncodings::OGA;
            }
        if (str == "FP")
            {
                return StreamEncodings::FP;
            }
        return 0;
    }

    template <typename T>
    inline T two_bit_look_up[11][4]
    {
        [0] = {},
        [1   /*OB*/] = {-2, -1, 0, 1},
        [2   /*SM*/] = {0, 1, 0, -1},
        [3   /*MS*/] = {0, 0, 1, -1},
        [4   /*TC*/] = {0, 1, -2, -1},
        [5   /*OG*/] = {-2, -1, 1, 0},
        [6  /*OBA*/] = {-3, -1, 1, 3},
        [7  /*SMA*/] = {1, 3, -1, -3},
        [8  /*MSA*/] = {1, -1, 3, -3},
        [9  /*TCA*/] = {1, 3, -3, -1},
        [10 /*OGA*/] = {-3, -1, 3, 1},
    };

    template <typename T>
    inline T three_bit_look_up[11][8]
    {
        [0] = {},
        [1   /*OB*/] = {-4, -3, -2, -1, 0, 1, 2, 3},
        [2   /*SM*/] = {0, 1, 2, 3, 0, -1, -2, -3},
        [3   /*MS*/] = {0, 0, 1, -1, 0, 0, 1, -1},
        [4   /*TC*/] = {0, 1, 2, 3, -4, -3, -2, -1},
        [5   /*OG*/] = {-4, -3, -1, -2, 3, 2, 0, 1},
        [6  /*OBA*/] = {-7, -5, -3, -1, 1, 3, 5, 7},
        [7  /*SMA*/] = {1, 3, 5, 7, -1, -3, -5, -7},
        [8  /*MSA*/] = {1, -1, 3, -3, 5, -5, 7, -7},
        [9  /*TCA*/] = {1, 3, 5, 7, -7, -5, -3, -1},
        [10 /*OGA*/] = {-7, -5, -1, -3, 7, 5, 1, 3},
    };

    template <typename T>
    inline T four_bit_look_up[11][16]
    {
        [0] = {},
        [1   /*OB*/] = {-8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7},
        [2   /*SM*/] = {0, 1, 2, 3, 4, 5, 6, 7, 0, -1, -2, -3, -4, -5, -6, -7},
        [3   /*MS*/] = {0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1},
        [4   /*TC*/] = {0, 1, 2, 3, 4, 5, 6, 7, -8, -7, -6, -5, -4, -3, -2, -1},
        [5   /*OG*/] = {-8, -7, -5, -6, -1, -2, -4, -3, 7, 6, 4, 5, 0, 1, 3, 2},
        [6  /*OBA*/] = {-15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15},
        [7  /*SMA*/] = {1, 3, 5, 7, 9, 11, 13, 15, -1, -3, -5, -7, -9, -11, -13, -15},
        [8  /*MSA*/] = {1, -1, 3, -3, 5, -5, 7, -7, 9, -9, 11, -11, 13, -13, 15, -15},
        [9  /*TCA*/] = {1, 3, 5, 7, 9, 11, 13, 15, -15, -13, -11, -9, -7, -5, -3, -1},
        [10 /*OGA*/] = {-15, -13, -9, -11, -1, -3, -7, -5, 15, 13, 9, 11, 1, 3, 7, 5},
    };

    template <typename T>
    inline T five_bit_look_up[11][32]
    {
        [0] = {},
        [1   /*OB*/] = {-16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
        [2   /*SM*/] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -15},
        [3   /*MS*/] = {0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, -1},
        [4   /*TC*/] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1},
        [5   /*OG*/] = {-16, -15, -13, -14, -9, -10, -12, -11, -1, -2, -4, -3, -8, -7, -5, -6, 15, 14, 12, 13, 8, 9, 11, 10, 0, 1, 3, 2, 7, 6, 4, 5},
        [6  /*OBA*/] = {-31, -29, -27, -25, -23, -21, -19, -17, -15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31},
        [7  /*SMA*/] = {1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, -1, -3, -5, -7, -9, -11, -13, -15, -17, -19, -21, -23, -25, -27, -29, -31},
        [8  /*MSA*/] = {1, -1, 3, -3, 5, -5, 7, -7, 9, -9, 11, -11, 13, -13, 15, -15, 17, -17, 19, -19, 21, -21, 23, -23, 25, -25, 27, -27, 29, -29, 31, -31},
        [9  /*TCA*/] = {1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, -31, -29, -27, -25, -23, -21, -19, -17, -15, -13, -11, -9, -7, -5, -3, -1},
        [10 /*OGA*/] = {-31, -29, -25, -27, -17, -19, -23, -21, -1, -3, -7, -5, -15, -13, -9, -11, 31, 29, 25, 27, 17, 19, 23, 21, 1, 3, 7, 5, 15, 13, 9, 11},
    };
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

    void read_from_file(FILE* fd);

    void write_to_output(gr_vector_void_star& outputs, const std::function<void(int output, int nitems)>& produce);

    std::size_t output_stream_count() const;
    std::size_t output_stream_item_size(std::size_t stream_index) const;

private:
    template <typename WT>
    struct unpacking_context_t
    {
        WT* iterator_;
        WT current_word_;
        uint8_t bitshift_ = 0;
    };

    template <typename WT>
    void unpack_words(gr_vector_void_star& outputs, const std::function<void(int output, int nitems)>& produce)
    {
        WT* data = static_cast<WT*>(buffer_);
        // TODO - Swap endiannes if needed

        unpacking_context_t<WT> ctx{};
        if (chunk_.Shift() == GnssMetadata::Chunk::Left)
            {
                ctx.iterator_ = data;
            }
        else if (chunk_.Shift() == GnssMetadata::Chunk::Right)
            {
                ctx.iterator_ = &data[countwords_];
            }
        advance_word(ctx);  // Initializes ctx.current_word_

        // Head padding
        if (padding_bitsize_ > 0 && chunk_.Padding() == GnssMetadata::Chunk::Head)
            {
                shift_padding(ctx, padding_bitsize_);
            }

        // Samples
        for (const auto& [lump, stream, encoding, output_index] : streams_)
            {
                if (output_index == -1)
                    {
                        skip_stream(ctx, lump, stream);
                    }
                else
                    {
                        produce(output_index, write_stream_samples(ctx, lump, stream, encoding, outputs[output_index]));
                    }
            }
    }

    template <typename WT>
    void skip_stream(
        unpacking_context_t<WT>& ctx,
        const GnssMetadata::Lump& lump,
        const GnssMetadata::IonStream& stream)
    {
        shift_padding(ctx, stream.Packedbits());
    }

    template <typename WT>
    std::size_t write_stream_samples(
        unpacking_context_t<WT>& ctx,
        const GnssMetadata::Lump& lump,
        const GnssMetadata::IonStream& stream,
        const GnssMetadata::StreamEncoding stream_encoding,
        void*& out)
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
                write_n_samples<WT, int8_t>(ctx, lump.Shift(), sample_bitsize, sample_count, stream_encoding, out);
            }
        else if (sample_bitsize <= 16)
            {
                write_n_samples<WT, int16_t>(ctx, lump.Shift(), sample_bitsize, sample_count, stream_encoding, out);
            }
        else if (sample_bitsize <= 32)
            {
                write_n_samples<WT, int32_t>(ctx, lump.Shift(), sample_bitsize, sample_count, stream_encoding, out);
            }
        else if (sample_bitsize <= 64)
            {
                write_n_samples<WT, int64_t>(ctx, lump.Shift(), sample_bitsize, sample_count, stream_encoding, out);
            }

        return sample_count;
    }

    template <typename WT, typename OT>
    void write_n_samples(
        unpacking_context_t<WT>& ctx,
        GnssMetadata::Lump::LumpShift lump_shift,
        uint8_t sample_bitsize,
        std::size_t sample_count,
        GnssMetadata::StreamEncoding stream_encoding,
        void*& out)
    {
        if (lump_shift == GnssMetadata::Lump::shiftRight)
            {
                auto* sample = static_cast<OT*>(out);
                sample += sample_count;
                for (std::size_t i = 0; i < sample_count; ++i)
                    {
                        shift_sample(ctx, sample_bitsize, sample);
                        decode_sample(sample_bitsize, sample, stream_encoding);
                        --sample;
                    }
            }
        else // if (lump_shift == GnssMetadata::Lump::shiftLeft || lump_shift == GnssMetadata::Lump::shiftUndefined)
            {
                auto* sample = static_cast<OT*>(out);
                for (std::size_t i = 0; i < sample_count; ++i)
                    {
                        shift_sample(ctx, sample_bitsize, sample);
                        decode_sample(sample_bitsize, sample, stream_encoding);
                        ++sample;
                    }
            }
    }

    template <typename WT, typename OT>
    void shift_sample(unpacking_context_t<WT>& ctx, uint8_t sample_bitsize, OT* output, uint8_t output_bit_offset = 0)
    {
        const uint8_t word_bitsize = sizeword_ * 8;

        if ((sample_bitsize + (ctx.bitshift_ % word_bitsize)) > word_bitsize)
            {
                uint8_t bits_shifted = word_bitsize - (ctx.bitshift_ % word_bitsize);

                if (chunk_.Shift() == GnssMetadata::Chunk::Left)
                    {
                        WT mask = ~((1 << (word_bitsize - bits_shifted)) - 1);
                        dump_sample(ctx.current_word_ & mask);
                        *output |= ((ctx.current_word_ & mask) >> output_bit_offset);
                        ctx.current_word_ <<= bits_shifted;
                    }
                else if (chunk_.Shift() == GnssMetadata::Chunk::Right)
                    {
                        WT mask = ((1 << (bits_shifted)) - 1);
                        dump_sample(ctx.current_word_ & mask);
                        *output |= (ctx.current_word_ & mask) << output_bit_offset;
                        // TODO - reverse bit order of sample? maybe?
                        ctx.current_word_ >>= bits_shifted;
                    }

                advance_word(ctx);
                ctx.bitshift_ += bits_shifted;
                shift_sample(ctx, sample_bitsize - bits_shifted, output, bits_shifted);
            }
        else
            {
                if (chunk_.Shift() == GnssMetadata::Chunk::Left)
                    {
                        WT mask = ~((1 << (word_bitsize - sample_bitsize)) - 1);
                        OT sample = (ctx.current_word_ & mask) >> (word_bitsize - sample_bitsize);
                        dump_sample(sample);
                        *output |= (sample) >> output_bit_offset;
                        ctx.current_word_ <<= sample_bitsize;
                    }
                else if (chunk_.Shift() == GnssMetadata::Chunk::Right)
                    {
                        WT mask = ((1 << (sample_bitsize)) - 1);
                        dump_sample(ctx.current_word_ & mask);
                        *output |= (ctx.current_word_ & mask) << output_bit_offset;
                        // TODO - reverse bit order of sample? maybe?
                        ctx.current_word_ >>= sample_bitsize;
                    }

                ctx.bitshift_ += sample_bitsize;
            }
    }

    template <typename WT>
    void shift_padding(unpacking_context_t<WT>& ctx, uint8_t n_bits)
    {
        if(n_bits == 0) return;

        const uint8_t word_bitsize = sizeword_ * 8;

        if ((n_bits + (ctx.bitshift_ % word_bitsize)) >= word_bitsize)
            {
                uint8_t bits_shifted = word_bitsize - (ctx.bitshift_ % word_bitsize);

                if (chunk_.Shift() == GnssMetadata::Chunk::Left)
                    {
                        ctx.current_word_ <<= bits_shifted;
                    }
                else if (chunk_.Shift() == GnssMetadata::Chunk::Right)
                    {
                        ctx.current_word_ >>= bits_shifted;
                    }

                advance_word(ctx);
                ctx.bitshift_ += bits_shifted;
                shift_padding(ctx, n_bits - bits_shifted);
            }
        else
            {
                if (chunk_.Shift() == GnssMetadata::Chunk::Left)
                    {
                        ctx.current_word_ <<= n_bits;
                    }
                else if (chunk_.Shift() == GnssMetadata::Chunk::Right)
                    {
                        ctx.current_word_ >>= n_bits;
                    }

                ctx.bitshift_ += n_bits;
            }
    }

    template <typename WT>
    void advance_word(unpacking_context_t<WT>& ctx)
    {
        WT word = *ctx.iterator_;
        if (chunk_.Shift() == GnssMetadata::Chunk::Left)
            {
                ++ctx.iterator_;
            }
        else if (chunk_.Shift() == GnssMetadata::Chunk::Right)
            {
                --ctx.iterator_;
            }

        ctx.current_word_ = word;
    }

    template <typename ST>
    static void decode_sample(const uint8_t sample_bitsize, ST* sample, const GnssMetadata::StreamEncoding encoding)
    {
        switch (sample_bitsize)
            {
            case 2:
                *sample =  GnssMetadata::two_bit_look_up<ST>[encoding][*sample];
                break;
            case 3:
                *sample =  GnssMetadata::three_bit_look_up<ST>[encoding][*sample];
                break;
            case 4:
                *sample =  GnssMetadata::four_bit_look_up<ST>[encoding][*sample];
                break;
            case 5:
                *sample =  GnssMetadata::five_bit_look_up<ST>[encoding][*sample];
                break;
            default:
                // TODO - Is this an error that can happen?
                break;
            }
    }

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
