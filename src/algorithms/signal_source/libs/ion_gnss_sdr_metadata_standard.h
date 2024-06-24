//
// Created by castle on 6/24/24.
//

#ifndef GNSS_SDR_ION_GNSS_SDR_METADATA_STANDARD_H
#define GNSS_SDR_ION_GNSS_SDR_METADATA_STANDARD_H

#include "GnssMetadata.h"
#include "gnss_block_interface.h"
#include <gnuradio/sync_block.h>
#include <string>
#include <filesystem>

class chunk_data_t
{
public:
    chunk_data_t(const GnssMetadata::Chunk& chunk, const std::vector<std::string>& stream_ids, std::size_t output_stream_offset);

    ~chunk_data_t();

    chunk_data_t(const chunk_data_t& rhl) = delete;
    chunk_data_t& operator=(const chunk_data_t& rhl) = delete;

    chunk_data_t(chunk_data_t&& rhl) = delete;
    chunk_data_t& operator=(chunk_data_t&& rhl) = delete;

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
        for (const auto& [lump, stream, output_index] : streams_)
            {
                if (output_index == -1)
                    {
                        skip_stream(ctx, lump, stream);
                    }
                else
                    {
                        produce(output_index, write_stream_samples(ctx, lump, stream, outputs[output_index]));
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
        void*& out)
    {
        std::size_t sample_bitsize = stream.Packedbits() / stream.RateFactor();
        std::size_t sample_count = stream.RateFactor();
        if (sample_bitsize <= 8)
            {
                write_n_samples<WT, uint8_t>(ctx, lump.Shift(), sample_bitsize, sample_count, out);
            }
        else if (sample_bitsize <= 16)
            {
                write_n_samples<WT, uint16_t>(ctx, lump.Shift(), sample_bitsize, sample_count, out);
            }
        else if (sample_bitsize <= 32)
            {
                write_n_samples<WT, uint32_t>(ctx, lump.Shift(), sample_bitsize, sample_count, out);
            }
        else if (sample_bitsize <= 64)
            {
                write_n_samples<WT, uint64_t>(ctx, lump.Shift(), sample_bitsize, sample_count, out);
            }

        return sample_count;
    }

    template <typename WT, typename OT>
    void write_n_samples(
        unpacking_context_t<WT>& ctx,
        GnssMetadata::Lump::LumpShift lump_shift,
        uint8_t sample_bitsize,
        std::size_t sample_count,
        void*& out)
    {
        if (lump_shift == GnssMetadata::Lump::shiftLeft)
            {
                auto* sample = static_cast<OT*>(out);
                for (int i = 0; i < sample_count; ++i)
                    {
                        shift_sample(ctx, sample_bitsize, sample);
                        ++sample;
                    }
            }
        else if (lump_shift == GnssMetadata::Lump::shiftRight)
            {
                auto* sample = static_cast<OT*>(out);
                sample += sample_count;
                for (int i = 0; i < sample_count; ++i)
                    {
                        shift_sample(ctx, sample_bitsize, sample);
                        --sample;
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
                        *output |= ((ctx.current_word_ & mask) >> output_bit_offset);
                        ctx.current_word_ <<= bits_shifted;
                    }
                else if (chunk_.Shift() == GnssMetadata::Chunk::Right)
                    {
                        WT mask = ((1 << (bits_shifted)) - 1);
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
                        *output |= (ctx.current_word_ & mask) >> output_bit_offset;
                        ctx.current_word_ <<= sample_bitsize;
                    }
                else if (chunk_.Shift() == GnssMetadata::Chunk::Right)
                    {
                        WT mask = ((1 << (sample_bitsize)) - 1);
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
        const uint8_t word_bitsize = sizeword_ * 8;

        if ((n_bits + (ctx.bitshift_ % word_bitsize)) > word_bitsize)
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
        int output_index = -1;
    };
    std::vector<stream_metadata_t> streams_;

    void* buffer_;
};

class IONMetadataStdFileSource : public gr::sync_block
{
public:
    using sptr = gnss_shared_ptr<IONMetadataStdFileSource>;

    IONMetadataStdFileSource(
        const std::filesystem::path& metadata_filepath,
        const GnssMetadata::File& file,
        const GnssMetadata::Block& block,
        const std::vector<std::string>& stream_ids);

    ~IONMetadataStdFileSource() override;

    int work(
        int noutput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items) override;

    std::size_t output_stream_count() const;
    std::size_t output_stream_item_size(std::size_t stream_index) const;

private:
    void read_chunk_pattern(gr_vector_void_star& output_items);

private:
    static gr::io_signature::sptr make_output_signature(const GnssMetadata::Block& block, const std::vector<std::string>& stream_ids);

private:
    const GnssMetadata::File& file_metadata_;
    const GnssMetadata::Block& block_metadata_;
    FILE* fd_;
    std::size_t output_stream_count_;
    std::vector<std::size_t> output_stream_item_sizes_;
    std::vector<std::shared_ptr<chunk_data_t>> chunk_data_;
};

class GnssMetadataHandler
{
public:
    explicit GnssMetadataHandler(const std::string& metadata_filepath);

    std::vector<IONMetadataStdFileSource::sptr> make_stream_sources(const std::vector<std::string>& stream_ids) const;

public:  // Getters
    const std::string& metadata_filepath() const;

private:
    void load_metadata();

private:
    std::string metadata_filepath_;
    GnssMetadata::Metadata metadata_;
};


#endif  // GNSS_SDR_ION_GNSS_SDR_METADATA_STANDARD_H
