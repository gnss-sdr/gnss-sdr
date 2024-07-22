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
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "ion_gsms_chunk_data.h"
#include <bitset>

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
    with_word_type(sizeword_, [&]<typename WordType>
    {
            buffer_ = new WordType[countwords_];
    });

    const std::size_t total_bitsize = sizeword_ * countwords_ * 8;
    std::size_t used_bitsize = 0;
    std::size_t output_streams = 0;
    for (const auto& lump : chunk.Lumps())
        {
            for (const auto& stream : lump.Streams())
                {
                    used_bitsize += stream.Packedbits();

                    if (std::ranges::any_of(stream_ids.begin(), stream_ids.end(), [&](const std::string& it) {
                            return stream.Id() == it;
                        }))
                        {
                            streams_.emplace_back(lump, stream, GnssMetadata::encoding_from_string(stream.Encoding()),output_streams + output_stream_offset);
                            ++output_streams;
                            const std::size_t sample_bitsize = stream.Packedbits() / stream.RateFactor();
                            output_stream_item_size_.push_back(bits_to_item_size(sample_bitsize));
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
    with_word_type(sizeword_, [&]<typename WordType>
    {
            delete[] static_cast<WordType*>(buffer_);
    });
}

void IONGSMSChunkData::read_from_file(FILE* fd)
{
    std::fread(buffer_, sizeword_, countwords_, fd);
}

void IONGSMSChunkData::write_to_output(gr_vector_void_star& outputs, const std::function<void(int output, int nitems)>& produce)
{
    with_word_type(sizeword_, [&]<typename WordType>
    {
        unpack_words<WordType>(outputs, produce);
    });
}

std::size_t IONGSMSChunkData::output_stream_count() const
{
    return output_stream_count_;
}

std::size_t IONGSMSChunkData::output_stream_item_size(std::size_t stream_index) const
{
    return output_stream_item_size_[stream_index];
}



void IONGSMSChunkData::dump_sample(auto value)
{
    static int count = 100;
    if (count > 0)
        {
            --count;
            std::cout << "SAMPLE: " << std::bitset<32>(value) << std::endl;
        }
}

