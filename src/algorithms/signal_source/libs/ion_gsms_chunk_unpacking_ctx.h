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

#include <cstddef>
#include <cstdint>
#include <stdexcept>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */

template <typename WT>
struct IONGSMSChunkUnpackingCtx
{
    static constexpr uint8_t word_bitsize_ = sizeof(WT) * 8;

    const WT* data_ = nullptr;  // Not owned by this class, MUST NOT destroy.
    std::size_t word_count_ = 0;
    std::size_t bit_offset_ = 0;
    bool read_lsb_first_ = false;

    IONGSMSChunkUnpackingCtx(
        WT* data_buffer,
        std::size_t data_buffer_word_count,
        bool read_lsb_first) : data_(data_buffer),
                               word_count_(data_buffer_word_count),
                               read_lsb_first_(read_lsb_first)
    {
    }

    void shift_padding(std::size_t n_bits)
    {
        bit_offset_ += n_bits;
    }

    uint64_t read_bits(std::size_t bit_count)
    {
        // Fast path: the field lies within a single word
        const std::size_t first_word_index = bit_offset_ / word_bitsize_;
        const std::size_t first_bit_index = bit_offset_ % word_bitsize_;
        if (bit_count > 0 && first_bit_index + bit_count <= word_bitsize_ && first_word_index < word_count_)
            {
                const std::size_t shift = read_lsb_first_ ? first_bit_index : word_bitsize_ - first_bit_index - bit_count;
                const uint64_t mask = bit_count >= 64 ? ~uint64_t{0} : (uint64_t{1} << bit_count) - 1U;
                bit_offset_ += bit_count;
                return (static_cast<uint64_t>(data_[first_word_index]) >> shift) & mask;
            }

        uint64_t value = 0;
        for (std::size_t bit = 0; bit < bit_count; ++bit)
            {
                const std::size_t absolute_bit = bit_offset_ + bit;
                const std::size_t word_index = absolute_bit / word_bitsize_;
                if (word_index >= word_count_)
                    {
                        throw std::runtime_error("ION_GSMS_Signal_Source tried to read past the chunk boundary");
                    }
                const std::size_t bit_index = absolute_bit % word_bitsize_;
                const std::size_t word_bit = read_lsb_first_ ? bit_index : word_bitsize_ - 1 - bit_index;
                const uint64_t bit_value = (static_cast<uint64_t>(data_[word_index]) >> word_bit) & 0x01U;
                if (read_lsb_first_)
                    {
                        value |= bit_value << bit;
                    }
                else
                    {
                        value <<= 1;
                        value |= bit_value;
                    }
            }

        bit_offset_ += bit_count;
        return value;
    }
};

/** \} */
/** \} */
#endif  // GNSS_SDR_ION_GSMS_CHUNK_UNPACKING_CTX_H
