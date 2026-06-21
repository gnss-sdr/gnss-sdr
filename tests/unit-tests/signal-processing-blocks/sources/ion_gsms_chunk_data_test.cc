/*!
 * \file ion_gsms_chunk_data_test.cc
 * \brief Unit tests for ION GNSS Metadata Standard chunk unpacking
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
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

#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_flags.h"
#include "ion_gsms.h"
#include "ion_gsms_chunk_data.h"
#include <gnuradio/blocks/vector_sink.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
GnssMetadata::IonStream make_stream(
    const GnssMetadata::IonStream::SampleFormat format,
    const std::string& encoding,
    const std::size_t quantization,
    const std::size_t packed_bits,
    const std::size_t rate_factor,
    const GnssMetadata::IonStream::StreamAlignment alignment = GnssMetadata::IonStream::Undefined,
    const GnssMetadata::IonStream::StreamShift shift = GnssMetadata::IonStream::shiftUndefined,
    const std::string& id = "L1")
{
    GnssMetadata::IonStream stream(id);
    stream.Format(format);
    stream.Encoding(encoding);
    stream.Quantization(quantization);
    stream.Packedbits(packed_bits);
    stream.RateFactor(rate_factor);
    stream.Alignment(alignment);
    stream.Shift(shift);
    return stream;
}


GnssMetadata::Chunk make_chunk(
    const GnssMetadata::IonStream& stream,
    const std::size_t size_word,
    const std::size_t count_words,
    const GnssMetadata::Chunk::WordEndian endian = GnssMetadata::Chunk::Undefined,
    const GnssMetadata::Chunk::WordPadding padding = GnssMetadata::Chunk::Tail,
    const GnssMetadata::Chunk::WordShift shift = GnssMetadata::Chunk::Left,
    const GnssMetadata::Lump::LumpShift lump_shift = GnssMetadata::Lump::shiftUndefined)
{
    GnssMetadata::Lump lump;
    lump.Shift(lump_shift);
    lump.Streams().push_back(stream);

    GnssMetadata::Chunk chunk;
    chunk.SizeWord(size_word);
    chunk.CountWords(count_words);
    chunk.Endian(endian);
    chunk.Padding(padding);
    chunk.Shift(shift);
    chunk.Lumps().push_back(lump);
    return chunk;
}


std::vector<std::vector<int8_t>> decode_int8_chunk(
    const GnssMetadata::Chunk& chunk,
    const std::vector<std::string>& stream_ids,
    const std::vector<uint8_t>& bytes)
{
    IONGSMSChunkData chunk_data(chunk, stream_ids, 0);

    auto input = bytes;
    chunk_data.read_from_buffer(input.data(), 0);

    std::vector<std::vector<int8_t>> output(chunk_data.output_stream_count());
    gr_vector_void_star outputs(chunk_data.output_stream_count());
    for (std::size_t i = 0; i < output.size(); ++i)
        {
            output[i].resize(chunk_data.output_stream_item_rate(i), 0);
            outputs[i] = output[i].data();
        }
    std::vector<int> output_items(chunk_data.output_stream_count(), 0);

    chunk_data.write_to_output(outputs, output_items);
    for (std::size_t i = 0; i < output.size(); ++i)
        {
            output[i].resize(static_cast<std::size_t>(output_items[i]));
        }
    return output;
}


std::vector<int8_t> decode_int8_stream(
    const GnssMetadata::IonStream& stream,
    const std::vector<uint8_t>& bytes,
    const std::size_t size_word = 1,
    const std::size_t count_words = 1,
    const GnssMetadata::Chunk::WordEndian endian = GnssMetadata::Chunk::Undefined,
    const GnssMetadata::Chunk::WordPadding padding = GnssMetadata::Chunk::Tail,
    const GnssMetadata::Chunk::WordShift chunk_shift = GnssMetadata::Chunk::Left,
    const GnssMetadata::Lump::LumpShift lump_shift = GnssMetadata::Lump::shiftUndefined)
{
    auto chunk = make_chunk(stream, size_word, count_words, endian, padding, chunk_shift, lump_shift);
    return decode_int8_chunk(chunk, {stream.Id()}, bytes).front();
}


std::vector<uint8_t> fp32_little_endian_bytes(const std::vector<float>& samples)
{
    std::vector<uint8_t> bytes;
    bytes.reserve(samples.size() * sizeof(uint32_t));
    for (const auto sample : samples)
        {
            uint32_t raw = 0;
            static_assert(sizeof(sample) == sizeof(raw), "FP32 sample storage must be 32 bits");
            std::memcpy(&raw, &sample, sizeof(raw));
            bytes.push_back(static_cast<uint8_t>(raw & 0xFFU));
            bytes.push_back(static_cast<uint8_t>((raw >> 8U) & 0xFFU));
            bytes.push_back(static_cast<uint8_t>((raw >> 16U) & 0xFFU));
            bytes.push_back(static_cast<uint8_t>((raw >> 24U) & 0xFFU));
        }
    return bytes;
}


std::vector<float> decode_float_stream(
    const GnssMetadata::IonStream& stream,
    const std::vector<uint8_t>& bytes,
    const std::size_t size_word = 4,
    const std::size_t count_words = 1,
    const GnssMetadata::Chunk::WordEndian endian = GnssMetadata::Chunk::Little,
    const GnssMetadata::Chunk::WordPadding padding = GnssMetadata::Chunk::Tail,
    const GnssMetadata::Chunk::WordShift chunk_shift = GnssMetadata::Chunk::Left,
    const GnssMetadata::Lump::LumpShift lump_shift = GnssMetadata::Lump::shiftUndefined)
{
    auto chunk = make_chunk(stream, size_word, count_words, endian, padding, chunk_shift, lump_shift);
    IONGSMSChunkData chunk_data(chunk, {stream.Id()}, 0);

    auto input = bytes;
    chunk_data.read_from_buffer(input.data(), 0);

    std::vector<float> output(chunk_data.output_stream_item_rate(0), 0.0F);
    gr_vector_void_star outputs(chunk_data.output_stream_count());
    outputs[0] = output.data();
    std::vector<int> output_items(chunk_data.output_stream_count(), 0);

    chunk_data.write_to_output(outputs, output_items);
    output.resize(static_cast<std::size_t>(output_items[0]));
    return output;
}


std::vector<gr_complex> decode_complex_stream(
    const GnssMetadata::IonStream& stream,
    const std::vector<uint8_t>& bytes,
    const std::size_t size_word = 4,
    const std::size_t count_words = 2,
    const GnssMetadata::Chunk::WordEndian endian = GnssMetadata::Chunk::Little,
    const GnssMetadata::Chunk::WordPadding padding = GnssMetadata::Chunk::Tail,
    const GnssMetadata::Chunk::WordShift chunk_shift = GnssMetadata::Chunk::Left,
    const GnssMetadata::Lump::LumpShift lump_shift = GnssMetadata::Lump::shiftUndefined)
{
    auto chunk = make_chunk(stream, size_word, count_words, endian, padding, chunk_shift, lump_shift);
    IONGSMSChunkData chunk_data(chunk, {stream.Id()}, 0);

    auto input = bytes;
    chunk_data.read_from_buffer(input.data(), 0);

    std::vector<gr_complex> output(chunk_data.output_stream_item_rate(0));
    gr_vector_void_star outputs(chunk_data.output_stream_count());
    outputs[0] = output.data();
    std::vector<int> output_items(chunk_data.output_stream_count(), 0);

    chunk_data.write_to_output(outputs, output_items);
    output.resize(static_cast<std::size_t>(output_items[0]));
    return output;
}


GnssMetadata::Block make_file_block(
    const std::size_t cycles,
    const std::size_t size_header = 0,
    const std::size_t size_footer = 0)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IF, "TC", 2, 2, 1);
    auto chunk = make_chunk(stream, 1, 1);

    GnssMetadata::Block block;
    block.Cycles(cycles);
    block.SizeHeader(size_header);
    block.SizeFooter(size_footer);
    block.Chunks().push_back(chunk);
    return block;
}


void write_binary_file(const fs::path& path, const std::vector<uint8_t>& bytes)
{
    std::ofstream file(path.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    for (const auto byte : bytes)
        {
            file.put(static_cast<char>(byte));
        }
}


GnssMetadata::File make_data_file(const fs::path& data_path, const std::size_t offset)
{
    GnssMetadata::File file;
    file.Url(GnssMetadata::AnyUri(data_path.filename().string()));
    file.Offset(offset);
    return file;
}


std::vector<int8_t> run_file_source(
    const fs::path& metadata_path,
    const GnssMetadata::File& file,
    const GnssMetadata::Block& block,
    const std::size_t block_start_offset)
{
    auto source = gnss_make_shared<IONGSMSFileSource>(metadata_path, file, block, block_start_offset, std::vector<std::string>{"L1"});
    auto sink = gr::blocks::vector_sink_b::make();
    auto top_block = gr::make_top_block("IONGSMSFileSourceTest");
    top_block->connect(source, 0, sink, 0);
    top_block->run();

    const auto sink_data = sink->data();
    std::vector<int8_t> output;
    output.reserve(sink_data.size());
    for (const auto item : sink_data)
        {
            output.push_back(static_cast<int8_t>(item));
        }
    return output;
}


std::vector<gr_complex> run_complex_file_source(
    const fs::path& metadata_path,
    const GnssMetadata::File& file,
    const GnssMetadata::Block& block,
    const std::size_t block_start_offset)
{
    auto source = gnss_make_shared<IONGSMSFileSource>(metadata_path, file, block, block_start_offset, std::vector<std::string>{"L1"});
    auto sink = gr::blocks::vector_sink_c::make();
    auto top_block = gr::make_top_block("IONGSMSFileSourceTest");
    top_block->connect(source, 0, sink, 0);
    top_block->run();

    return sink->data();
}
}  // namespace


TEST(IONGSMSChunkDataTest, DecodesSignIqNFromDeclaredFormat)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IQn, "SIGN", 1, 8, 1, GnssMetadata::IonStream::Left);

    const auto output = decode_int8_stream(stream, {0b01000000});

    const std::vector<int8_t> expected{1, 1};
    EXPECT_EQ(expected, output);
}


TEST(IONGSMSChunkDataTest, DecodesInQnFromDeclaredFormat)
{
    const auto stream = make_stream(GnssMetadata::IonStream::InQn, "SIGN", 1, 8, 1, GnssMetadata::IonStream::Left);

    const auto output = decode_int8_stream(stream, {0b01000000});

    const std::vector<int8_t> expected{-1, 1};
    EXPECT_EQ(expected, output);
}


TEST(IONGSMSChunkDataTest, HonorsRightAlignedStreamPadding)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IF, "TC", 2, 8, 1, GnssMetadata::IonStream::Right);

    const auto output = decode_int8_stream(stream, {0b00000011});

    const std::vector<int8_t> expected{-1};
    EXPECT_EQ(expected, output);
}


TEST(IONGSMSChunkDataTest, HonorsStreamShiftRightWithoutOverrun)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IF, "TC", 2, 8, 2, GnssMetadata::IonStream::Left, GnssMetadata::IonStream::shiftRight);

    const auto output = decode_int8_stream(stream, {0b01100000});

    const std::vector<int8_t> expected{-2, 1};
    EXPECT_EQ(expected, output);
}


TEST(IONGSMSChunkDataTest, DecodesBigEndianWordsAsUnsigned)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IF, "OB", 4, 16, 1, GnssMetadata::IonStream::Left);

    const auto output = decode_int8_stream(stream, {0x12, 0x34}, 2, 1, GnssMetadata::Chunk::Big);

    const std::vector<int8_t> expected{-7};
    EXPECT_EQ(expected, output);
}


TEST(IONGSMSChunkDataTest, RepeatsSingleLumpPayloadToFillChunk)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IF, "TC", 2, 2, 1);

    const auto output = decode_int8_stream(stream, {0b01101100});

    const std::vector<int8_t> expected{1, -2, -1, 0};
    EXPECT_EQ(expected, output);
}


TEST(IONGSMSChunkDataTest, DecodesMultipleLumpsInDeclaredOrder)
{
    const auto first_stream = make_stream(GnssMetadata::IonStream::IF, "TC", 4, 4, 1, GnssMetadata::IonStream::Undefined, GnssMetadata::IonStream::shiftUndefined, "L1");
    const auto second_stream = make_stream(GnssMetadata::IonStream::IF, "TC", 4, 4, 1, GnssMetadata::IonStream::Undefined, GnssMetadata::IonStream::shiftUndefined, "L2");

    GnssMetadata::Lump first_lump;
    first_lump.Streams().push_back(first_stream);
    GnssMetadata::Lump second_lump;
    second_lump.Streams().push_back(second_stream);

    GnssMetadata::Chunk chunk;
    chunk.SizeWord(1);
    chunk.CountWords(1);
    chunk.Endian(GnssMetadata::Chunk::Undefined);
    chunk.Padding(GnssMetadata::Chunk::Tail);
    chunk.Shift(GnssMetadata::Chunk::Left);
    chunk.Lumps().push_back(first_lump);
    chunk.Lumps().push_back(second_lump);

    const auto output = decode_int8_chunk(chunk, {"L1", "L2"}, {0b00011111});

    ASSERT_EQ(2U, output.size());
    const std::vector<int8_t> expected_first{1};
    const std::vector<int8_t> expected_second{-1};
    EXPECT_EQ(expected_first, output[0]);
    EXPECT_EQ(expected_second, output[1]);
}


TEST(IONGSMSChunkDataTest, RepeatsOrderedMultiLumpPatternToFillChunk)
{
    const auto first_stream = make_stream(GnssMetadata::IonStream::IF, "TC", 2, 2, 1, GnssMetadata::IonStream::Undefined, GnssMetadata::IonStream::shiftUndefined, "L1");
    const auto second_stream = make_stream(GnssMetadata::IonStream::IF, "TC", 2, 2, 1, GnssMetadata::IonStream::Undefined, GnssMetadata::IonStream::shiftUndefined, "L2");

    GnssMetadata::Lump first_lump;
    first_lump.Streams().push_back(first_stream);
    GnssMetadata::Lump second_lump;
    second_lump.Streams().push_back(second_stream);

    GnssMetadata::Chunk chunk;
    chunk.SizeWord(1);
    chunk.CountWords(1);
    chunk.Endian(GnssMetadata::Chunk::Undefined);
    chunk.Padding(GnssMetadata::Chunk::Tail);
    chunk.Shift(GnssMetadata::Chunk::Left);
    chunk.Lumps().push_back(first_lump);
    chunk.Lumps().push_back(second_lump);

    const auto output = decode_int8_chunk(chunk, {"L1", "L2"}, {0b01101100});

    ASSERT_EQ(2U, output.size());
    const std::vector<int8_t> expected_first{1, -1};
    const std::vector<int8_t> expected_second{-2, 0};
    EXPECT_EQ(expected_first, output[0]);
    EXPECT_EQ(expected_second, output[1]);
}


TEST(IONGSMSChunkDataTest, CollapsesRepeatedStreamIdIntoOneOutput)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IF, "TC", 2, 2, 1);

    GnssMetadata::Lump first_lump;
    first_lump.Streams().push_back(stream);
    GnssMetadata::Lump second_lump;
    second_lump.Streams().push_back(stream);

    GnssMetadata::Chunk chunk;
    chunk.SizeWord(1);
    chunk.CountWords(1);
    chunk.Endian(GnssMetadata::Chunk::Undefined);
    chunk.Padding(GnssMetadata::Chunk::Tail);
    chunk.Shift(GnssMetadata::Chunk::Left);
    chunk.Lumps().push_back(first_lump);
    chunk.Lumps().push_back(second_lump);

    const auto output = decode_int8_chunk(chunk, {"L1"}, {0b01101100});

    ASSERT_EQ(1U, output.size());
    const std::vector<int8_t> expected{1, -2, -1, 0};
    EXPECT_EQ(expected, output[0]);
}


TEST(IONGSMSChunkDataTest, DecodesFp32IfSamples)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IF, "FP", 32, 64, 2);

    const auto output = decode_float_stream(stream, fp32_little_endian_bytes({1.25F, -0.5F}), 4, 2);

    ASSERT_EQ(2U, output.size());
    EXPECT_FLOAT_EQ(1.25F, output[0]);
    EXPECT_FLOAT_EQ(-0.5F, output[1]);
}


TEST(IONGSMSChunkDataTest, NegatesFp32IfnSamples)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IFn, "FP", 32, 32, 1);

    const auto output = decode_float_stream(stream, fp32_little_endian_bytes({2.0F}));

    ASSERT_EQ(1U, output.size());
    EXPECT_FLOAT_EQ(-2.0F, output[0]);
}


TEST(IONGSMSChunkDataTest, DecodesFp32IqAsGrComplex)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IQ, "FP", 32, 64, 1);
    const auto chunk = make_chunk(stream, 4, 2, GnssMetadata::Chunk::Little);
    IONGSMSChunkData chunk_data(chunk, {stream.Id()}, 0);

    EXPECT_EQ(sizeof(gr_complex), chunk_data.output_stream_item_size(0));
    EXPECT_EQ(1U, chunk_data.output_stream_item_rate(0));

    const auto output = decode_complex_stream(stream, fp32_little_endian_bytes({1.5F, -2.25F}));

    ASSERT_EQ(1U, output.size());
    EXPECT_FLOAT_EQ(1.5F, output[0].real());
    EXPECT_FLOAT_EQ(-2.25F, output[0].imag());
}


TEST(IONGSMSChunkDataTest, AppliesFp32ComplexFormatSignAndOrder)
{
    const auto stream = make_stream(GnssMetadata::IonStream::QIn, "FP", 32, 64, 1);

    const auto output = decode_complex_stream(stream, fp32_little_endian_bytes({3.0F, 4.0F}));

    ASSERT_EQ(1U, output.size());
    EXPECT_FLOAT_EQ(-4.0F, output[0].real());
    EXPECT_FLOAT_EQ(3.0F, output[0].imag());
}


TEST(IONGSMSChunkDataTest, RejectsUnsupportedFloatingPointQuantization)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IF, "FP", 64, 64, 1);
    const auto chunk = make_chunk(stream, 8, 1);

    EXPECT_THROW(IONGSMSChunkData(chunk, {stream.Id()}, 0), std::runtime_error);
}


TEST(IONGSMSChunkDataTest, RejectsUnsupportedChunkWordSizeAtConstruction)
{
    const auto stream = make_stream(GnssMetadata::IonStream::IF, "TC", 2, 2, 1);
    const auto chunk = make_chunk(stream, 3, 1);

    EXPECT_THROW(IONGSMSChunkData(chunk, {stream.Id()}, 0), std::runtime_error);
}


TEST(IONGSMSFileSourceTest, DecodesOnlyCompleteCyclesReadFromFile)
{
    const fs::path temp_dir(GetTempDir());
    const fs::path data_path = temp_dir / "ion_gsms_file_source_short_read.bin";
    const fs::path metadata_path = temp_dir / "ion_gsms_file_source_short_read.sdrx";
    write_binary_file(data_path, {static_cast<uint8_t>('H'), static_cast<uint8_t>(0b01101100U), static_cast<uint8_t>(0b00011011U)});

    const auto file = make_data_file(data_path, 0);
    const auto block = make_file_block(3, 1);

    const auto output = run_file_source(metadata_path, file, block, 0);

    const std::vector<int8_t> expected{1, -2, -1, 0, 0, 1, -2, -1};
    EXPECT_EQ(expected, output);

    fs::remove(data_path);
}


TEST(IONGSMSFileSourceTest, CollapsesRepeatedStreamIdAcrossChunksIntoOneOutput)
{
    const fs::path temp_dir(GetTempDir());
    const fs::path data_path = temp_dir / "ion_gsms_file_source_repeated_stream.bin";
    const fs::path metadata_path = temp_dir / "ion_gsms_file_source_repeated_stream.sdrx";
    write_binary_file(data_path, {static_cast<uint8_t>(0b01101100U), static_cast<uint8_t>(0b00011011U)});

    const auto stream = make_stream(GnssMetadata::IonStream::IF, "TC", 2, 2, 1);
    auto first_chunk = make_chunk(stream, 1, 1);
    auto second_chunk = make_chunk(stream, 1, 1);

    GnssMetadata::Block block;
    block.Cycles(1);
    block.Chunks().push_back(first_chunk);
    block.Chunks().push_back(second_chunk);

    const auto file = make_data_file(data_path, 0);
    const auto output = run_file_source(metadata_path, file, block, 0);

    const std::vector<int8_t> expected{1, -2, -1, 0, 0, 1, -2, -1};
    EXPECT_EQ(expected, output);

    fs::remove(data_path);
}


TEST(IONGSMSFileSourceTest, StartsReadingAtProvidedBlockOffset)
{
    const fs::path temp_dir(GetTempDir());
    const fs::path data_path = temp_dir / "ion_gsms_file_source_block_offset.bin";
    const fs::path metadata_path = temp_dir / "ion_gsms_file_source_block_offset.sdrx";
    write_binary_file(data_path, {static_cast<uint8_t>('P'), static_cast<uint8_t>('P'), static_cast<uint8_t>('A'), static_cast<uint8_t>(0b11111111U), static_cast<uint8_t>('F'), static_cast<uint8_t>('B'), static_cast<uint8_t>(0b01101100U), static_cast<uint8_t>('G')});

    const auto file = make_data_file(data_path, 2);
    const auto block = make_file_block(1, 1, 1);
    const std::size_t second_block_offset = 2 + 1 + 1 + 1;

    const auto output = run_file_source(metadata_path, file, block, second_block_offset);

    const std::vector<int8_t> expected{1, -2, -1, 0};
    EXPECT_EQ(expected, output);

    fs::remove(data_path);
}


TEST(IONGSMSFileSourceTest, EmitsFp32IqAsComplexItems)
{
    const fs::path temp_dir(GetTempDir());
    const fs::path data_path = temp_dir / "ion_gsms_file_source_fp32_iq.bin";
    const fs::path metadata_path = temp_dir / "ion_gsms_file_source_fp32_iq.sdrx";
    write_binary_file(data_path, fp32_little_endian_bytes({1.5F, -2.25F}));

    const auto stream = make_stream(GnssMetadata::IonStream::IQ, "FP", 32, 64, 1);
    auto chunk = make_chunk(stream, 4, 2, GnssMetadata::Chunk::Little);

    GnssMetadata::Block block;
    block.Cycles(1);
    block.Chunks().push_back(chunk);

    const auto file = make_data_file(data_path, 0);
    const auto output = run_complex_file_source(metadata_path, file, block, 0);

    ASSERT_EQ(1U, output.size());
    EXPECT_FLOAT_EQ(1.5F, output[0].real());
    EXPECT_FLOAT_EQ(-2.25F, output[0].imag());

    fs::remove(data_path);
}
