/*!
 * \file ion_gsms_chunk_unpacking_ctx.h
 * \brief Holds state and provides utilities for unpacking samples from a chunk
 * \author Víctor Castillo Agüero, 2024. victorcastilloaguero(at)gmail.com
 *
 * This is a template class, and thus, its member functions must be defined in the header file.
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

#ifndef GNSS_SDR_ION_GSMS_CHUNK_UNPACKING_CTX_H
#define GNSS_SDR_ION_GSMS_CHUNK_UNPACKING_CTX_H

#include <gnuradio/block.h>
#include <GnssMetadata.h>
#include <cstdint>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */

template <typename WT>
struct IONGSMSChunkUnpackingCtx
{
    static constexpr uint8_t word_bitsize_ = sizeof(WT) * 8;

    const GnssMetadata::Chunk::WordShift word_shift_direction_;
    WT* iterator_ = nullptr;  // Not owned by this class, MUST NOT destroy
    WT current_word_{};
    uint8_t bitshift_ = 0;

    IONGSMSChunkUnpackingCtx(
        const GnssMetadata::Chunk::WordShift word_shift,
        WT* data_buffer,
        uint8_t data_buffer_word_count) : word_shift_direction_(word_shift)
    {
        if (word_shift_direction_ == GnssMetadata::Chunk::Left)
            {
                iterator_ = data_buffer;
            }
        else if (word_shift_direction_ == GnssMetadata::Chunk::Right)
            {
                iterator_ = &data_buffer[data_buffer_word_count];
            }
        if (iterator_)
            {
                advance_word();  // Initializes current_word_
            }
    }

    void advance_word()
    {
        WT word = *iterator_;
        if (word_shift_direction_ == GnssMetadata::Chunk::Left)
            {
                ++iterator_;
            }
        else if (word_shift_direction_ == GnssMetadata::Chunk::Right)
            {
                --iterator_;
            }

        current_word_ = word;
    }

    void shift_current_word(uint8_t n)
    {
        if ((n % word_bitsize_) == 0)
            {
                for (uint8_t i = 0; i < (n / word_bitsize_); ++i)
                    {
                        advance_word();
                    }
                return;
            }

        if (word_shift_direction_ == GnssMetadata::Chunk::Left)
            {
                current_word_ <<= n;
            }
        else if (word_shift_direction_ == GnssMetadata::Chunk::Right)
            {
                current_word_ >>= n;
            }

        bitshift_ += n;
        if (bitshift_ >= word_bitsize_)
            {
                advance_word();
                bitshift_ -= word_bitsize_;
            }
    }

    void shift_padding(uint8_t n_bits)
    {
        if (n_bits == 0)
            {
                return;
            }

        if ((n_bits + (bitshift_ % word_bitsize_)) >= word_bitsize_)
            {
                const uint8_t bits_shifted = word_bitsize_ - (bitshift_ % word_bitsize_);

                shift_current_word(bits_shifted);
                shift_padding(n_bits - bits_shifted);
            }
        else
            {
                shift_current_word(n_bits);
            }
    }

    template <typename OT>
    void shift_sample(uint8_t sample_bitsize, OT* output, uint8_t output_bit_offset = 0)
    {
        if (sample_bitsize % word_bitsize_ == 0)
            {
                const uint8_t words_per_sample = sample_bitsize / word_bitsize_;
                for (uint8_t i = 0; i < words_per_sample; ++i)
                    {
                        if (word_shift_direction_ == GnssMetadata::Chunk::Left)
                            {
                                *output |= (current_word_ << ((words_per_sample - 1 - i) * word_bitsize_));
                            }
                        else if (word_shift_direction_ == GnssMetadata::Chunk::Right)
                            {
                                *output |= (current_word_ << (i * word_bitsize_));
                                // TODO - reverse bit order of sample? maybe?
                            }
                        advance_word();
                    }
            }
        else if ((sample_bitsize + (bitshift_ % word_bitsize_)) > word_bitsize_)
            {
                const uint8_t bits_shifted = word_bitsize_ - (bitshift_ % word_bitsize_);

                if (word_shift_direction_ == GnssMetadata::Chunk::Left)
                    {
                        WT mask = ~((1 << (word_bitsize_ - bits_shifted)) - 1);
                        *output |= ((current_word_ & mask) >> output_bit_offset);
                    }
                else if (word_shift_direction_ == GnssMetadata::Chunk::Right)
                    {
                        WT mask = ((1 << (bits_shifted)) - 1);
                        *output |= (current_word_ & mask) << output_bit_offset;
                        // TODO - reverse bit order of sample? maybe?
                    }

                shift_current_word(bits_shifted);
                shift_sample(sample_bitsize - bits_shifted, output, bits_shifted);
            }
        else
            {
                if (word_shift_direction_ == GnssMetadata::Chunk::Left)
                    {
                        WT mask = ~((1 << (word_bitsize_ - sample_bitsize)) - 1);
                        OT sample = (current_word_ & mask) >> (word_bitsize_ - sample_bitsize);
                        *output |= (sample) >> output_bit_offset;
                    }
                else if (word_shift_direction_ == GnssMetadata::Chunk::Right)
                    {
                        WT mask = ((1 << (sample_bitsize)) - 1);
                        *output |= (current_word_ & mask) << output_bit_offset;
                        // TODO - reverse bit order of sample? maybe?
                    }

                shift_current_word(sample_bitsize);
            }
    }
};

/** \} */
/** \} */
#endif  // GNSS_SDR_ION_GSMS_CHUNK_UNPACKING_CTX_H
