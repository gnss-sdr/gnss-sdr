//
// Created by castle on 6/24/24.
//

#include "ion_gnss_sdr_metadata_standard.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GnssMetadataHandler::GnssMetadataHandler(const std::string& metadata_filepath)
    : metadata_filepath_(metadata_filepath)
{
    load_metadata();
}

const std::string& GnssMetadataHandler::metadata_filepath() const
{
    return metadata_filepath_;
}

void GnssMetadataHandler::load_metadata()
{
    try
        {
            GnssMetadata::XmlProcessor xml_proc;
            if (!xml_proc.Load(metadata_filepath_.c_str(), false, metadata_))
                {
                    LOG(ERROR) << "Could not load XML metadata file:";
                }
        }
    catch (GnssMetadata::ApiException& e)
        {
            LOG(ERROR) << "API Exception while loadind XML metadata file: " << e.what();
        }
    catch (std::exception& e)
        {
            LOG(ERROR) << "Exception while loading XML metadata file: " << e.what();
        }
}

std::vector<IONMetadataStdFileSource::sptr> GnssMetadataHandler::make_stream_sources(const std::vector<std::string>& stream_ids) const
{
    std::vector<IONMetadataStdFileSource::sptr> sources{};
    for (const auto& file : metadata_.Files())
        {
            for (const auto& block : file.Lane().Blocks())
                {
                    for (const auto& chunk : block.Chunks())
                        {
                            for (const auto& lump : chunk.Lumps())
                                {
                                    for (const auto& stream : lump.Streams())
                                        {
                                            if (std::ranges::any_of(stream_ids.begin(), stream_ids.end(), [&](const std::string& it) {
                                                    return stream.Id() == it;
                                                }))
                                                {
                                                    auto source = gnss_make_shared<IONMetadataStdFileSource>(
                                                        file,
                                                        block,
                                                        stream_ids);

                                                    sources.push_back(source);

                                                    // This file source will take care of any other matching streams in this block
                                                    // We can skip the rest of this block
                                                    goto next_block;
                                                }
                                        }
                                }
                        }
                next_block:
                }
        }

    return sources;
}


IONMetadataStdFileSource::IONMetadataStdFileSource(
    const GnssMetadata::File& file,
    const GnssMetadata::Block& block,
    const std::vector<std::string>& stream_ids)
    : gr::sync_block(
          "ion_metadata_standard_source",
          gr::io_signature::make(0, 0, 0),
          make_output_signature(block, stream_ids)),
      file_metadata_(file),
      block_metadata_(block)
{
    fd_ = std::fopen(file.Url().Value().c_str(), "rb");
    std::size_t block_offset = file.Offset();
    std::fseek(fd_, file.Offset() + block_offset + block.SizeHeader(), SEEK_SET);

    std::size_t output_stream_offset = 0;
    for (const auto& chunk : block.Chunks())
        {
            chunk_data_.emplace_back(std::make_shared<chunk_data_t>(chunk, stream_ids, output_stream_offset));
            const std::size_t out_count = chunk_data_.back()->output_stream_count();
            output_stream_offset += out_count;
            for (std::size_t i = 0; i < out_count; ++i)
                {
                    output_stream_item_sizes_.push_back(chunk_data_.back()->output_stream_item_size(i));
                }
        }
    output_stream_count_ = output_stream_offset;
}

IONMetadataStdFileSource::~IONMetadataStdFileSource()
{
    std::fclose(fd_);
}

int IONMetadataStdFileSource::work(
    int noutput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    for (int i = 0; i < noutput_items; ++i)
        {
            read_chunk_pattern(output_items);
        }
    return WORK_CALLED_PRODUCE;
}

std::size_t IONMetadataStdFileSource::output_stream_count() const
{
    return output_stream_count_;
}

std::size_t IONMetadataStdFileSource::output_stream_item_size(std::size_t stream_index) const
{
    return output_stream_item_sizes_[stream_index];
}


void IONMetadataStdFileSource::read_chunk_pattern(gr_vector_void_star& output_items)
{
    gr_vector_void_star chunk_outputs = output_items;
    for (auto& c : chunk_data_)
        {
            c->read_from_file(fd_);
            c->write_to_output(output_items, [this](int output, int nitems) {
                produce(output, nitems);
            });
        }
}

gr::io_signature::sptr IONMetadataStdFileSource::make_output_signature(const GnssMetadata::Block& block, const std::vector<std::string>& stream_ids)
{
    int nstreams = 0;
    std::vector<size_t> item_sizes{};

    for (const auto& chunk : block.Chunks())
        {
            for (const auto& lump : chunk.Lumps())
                {
                    for (const auto& stream : lump.Streams())
                        {
                            if (std::ranges::any_of(stream_ids.begin(), stream_ids.end(), [&](const std::string& it) {
                                    return stream.Id() == it;
                                }))
                                {
                                    ++nstreams;
                                    std::size_t sample_bitsize = stream.Packedbits() / stream.RateFactor();
                                    if (sample_bitsize <= 8)
                                        {
                                            item_sizes.push_back(1);
                                        }
                                    else if (sample_bitsize <= 16)
                                        {
                                            item_sizes.push_back(2);
                                        }
                                    else if (sample_bitsize <= 32)
                                        {
                                            item_sizes.push_back(4);
                                        }
                                    else if (sample_bitsize <= 64)
                                        {
                                            item_sizes.push_back(8);
                                        }
                                    else
                                        {
                                            // This shouldn't happen
                                            item_sizes.push_back(1);
                                        }
                                }
                        }
                }
        }

    return gr::io_signature::make(
        nstreams,
        nstreams,
        item_sizes);
}


chunk_data_t::chunk_data_t(const GnssMetadata::Chunk& chunk, const std::vector<std::string>& stream_ids, std::size_t output_stream_offset)
    : chunk_(chunk),
      sizeword_(chunk_.SizeWord()),
      countwords_(chunk_.CountWords())
{
    switch (sizeword_)
        {
        case 1:
            buffer_ = new uint8_t[countwords_];
            break;
        case 2:
            buffer_ = new uint16_t[countwords_];
            break;
        case 4:
            buffer_ = new uint32_t[countwords_];
            break;
        case 8:
            buffer_ = new uint64_t[countwords_];
            break;
        default:
            LOG(ERROR) << "Unknown word size: " << std::to_string(sizeword_);
            break;
        }


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
                            streams_.emplace_back(lump, stream, output_streams + output_stream_offset);
                            ++output_streams;
                            std::size_t sample_bitsize = stream.Packedbits() / stream.RateFactor();
                            if (sample_bitsize <= 8)
                                {
                                    output_stream_item_size_.push_back(1);
                                }
                            else if (sample_bitsize <= 16)
                                {
                                    output_stream_item_size_.push_back(2);
                                }
                            else if (sample_bitsize <= 32)
                                {
                                    output_stream_item_size_.push_back(4);
                                }
                            else if (sample_bitsize <= 64)
                                {
                                    output_stream_item_size_.push_back(8);
                                }
                            else
                                {
                                    // This shouldn't happen
                                    output_stream_item_size_.push_back(1);
                                }
                        }
                    else
                        {
                            streams_.emplace_back(lump, stream, -1);
                        }
                }
        }

    output_stream_count_ = output_streams;
    padding_bitsize_ = total_bitsize - used_bitsize;
}
chunk_data_t::~chunk_data_t()
{
    switch (sizeword_)
        {
        case 1:
            delete[] static_cast<uint8_t*>(buffer_);
            break;
        case 2:
            delete[] static_cast<uint16_t*>(buffer_);
            break;
        case 4:
            delete[] static_cast<uint32_t*>(buffer_);
            break;
        case 8:
            delete[] static_cast<uint64_t*>(buffer_);
            break;
        default:
            break;
        }
}

void chunk_data_t::read_from_file(FILE* fd)
{
    std::fread(buffer_, sizeword_, countwords_, fd);
}

void chunk_data_t::write_to_output(gr_vector_void_star& outputs, const std::function<void(int output, int nitems)>& produce)
{
    switch (sizeword_)
        {
        case 1:
            unpack_words<uint8_t>(outputs, produce);
            break;
        case 2:
            unpack_words<uint16_t>(outputs, produce);
            break;
        case 4:
            unpack_words<uint32_t>(outputs, produce);
            break;
        case 8:
            unpack_words<uint64_t>(outputs, produce);
            break;
        default:
            break;
        }
}

std::size_t chunk_data_t::output_stream_count() const
{
    return output_stream_count_;
}

std::size_t chunk_data_t::output_stream_item_size(std::size_t stream_index) const
{
    return output_stream_item_size_[stream_index];
}
