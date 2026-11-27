/*!
 * \file labsat23_source_test.cc
 * \brief Unit tests for the LabSat 2, 3, and 3 Wideband source block.
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
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

#include "concurrent_queue.h"
#include "gnss_sdr_filesystem.h"
#include "labsat23_source.h"
#if GR_GREATER_38
#include <gnuradio/blocks/vector_sink.h>
#else
#include <gnuradio/blocks/vector_sink_c.h>
#endif
#include <gnuradio/top_block.h>
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

struct Section
{
    uint16_t id;
    std::vector<uint8_t> payload;
};

struct LabSatRunResult
{
    std::vector<std::vector<gr_complex>> outputs;
};

fs::path temp_path(const std::string& name)
{
    const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    return fs::temp_directory_path() / (name + "_" + std::to_string(stamp));
}

void expect_remove(const fs::path& path)
{
    const auto filename = path.string();
    EXPECT_EQ(0, std::remove(filename.c_str())) << "Failure deleting temporary file " << filename;
}

void append_le16(std::vector<uint8_t>& bytes, uint16_t value)
{
    bytes.push_back(static_cast<uint8_t>(value & 0xFF));
    bytes.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void append_le32(std::vector<uint8_t>& bytes, uint32_t value)
{
    bytes.push_back(static_cast<uint8_t>(value & 0xFF));
    bytes.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    bytes.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    bytes.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

void append_le64(std::vector<uint8_t>& bytes, uint64_t value)
{
    for (int i = 0; i < 8; ++i)
        {
            bytes.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
        }
}

void set_msb_bit(uint16_t& word, int bit_offset, bool value)
{
    if (value)
        {
            word |= static_cast<uint16_t>(1U << (15 - bit_offset));
        }
}

void set_msb_bit(uint32_t& word, int bit_offset, bool value)
{
    if (value)
        {
            word |= static_cast<uint32_t>(1UL << (31 - bit_offset));
        }
}

void set_msb_bit(uint64_t& word, int bit_offset, bool value)
{
    if (value)
        {
            word |= static_cast<uint64_t>(1ULL << (63 - bit_offset));
        }
}

void write_bytes(const fs::path& path, const std::vector<uint8_t>& bytes)
{
    std::ofstream output(path.string().c_str(), std::ios::binary);
    ASSERT_TRUE(output.is_open());
    output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

void append_header(std::vector<uint8_t>& bytes, const std::array<char, 3>& version, const std::vector<Section>& sections)
{
    uint32_t header_size = 16;
    for (const auto& section : sections)
        {
            header_size += 6 + static_cast<uint32_t>(section.payload.size());
        }

    for (int i = 0; i < 8; ++i)
        {
            bytes.push_back(0);
        }
    bytes.push_back(static_cast<uint8_t>(version[0]));
    bytes.push_back(static_cast<uint8_t>(version[1]));
    bytes.push_back(static_cast<uint8_t>(version[2]));
    bytes.push_back(1);
    append_le32(bytes, header_size);

    for (const auto& section : sections)
        {
            append_le16(bytes, section.id);
            append_le32(bytes, static_cast<uint32_t>(section.payload.size()));
            bytes.insert(bytes.end(), section.payload.begin(), section.payload.end());
        }
}

std::vector<uint8_t> section2_payload(uint8_t bits_per_sample, uint8_t channel_selection)
{
    std::vector<uint8_t> payload(6, 0);
    payload[0] = 1;  // TCXO
    payload[1] = bits_per_sample;
    payload[2] = channel_selection;
    payload[3] = channel_selection == 3 || channel_selection == 4 ? 1 : 0;
    payload[4] = 0;  // Channel A GPS
    payload[5] = channel_selection == 1 || channel_selection == 3 ? 0xFF : 1;
    return payload;
}

std::vector<uint8_t> section2_payload_ls3_triple()
{
    std::vector<uint8_t> payload(21, 0);
    payload[0] = 1;   // TCXO
    payload[1] = 6;   // 3 channels * I/Q
    payload[2] = 0;   // ignored by LabSat 3 triple constellation
    payload[3] = 0;   // 1 bit
    payload[4] = 0;   // A GPS
    payload[5] = 1;   // B GLONASS
    payload[19] = 1;  // triple constellation
    payload[20] = 2;  // C BDS
    return payload;
}

uint16_t pack_single_1bit(const std::vector<gr_complex>& samples)
{
    uint16_t word = 0;
    for (size_t i = 0; i < samples.size(); ++i)
        {
            set_msb_bit(word, static_cast<int>(2 * i), samples[i].real() > 0);
            set_msb_bit(word, static_cast<int>(2 * i + 1), samples[i].imag() > 0);
        }
    return word;
}

std::pair<bool, bool> bits_for_unsigned_2bit(float value)
{
    if (value > 1.5F)
        {
            return std::make_pair(true, true);
        }
    if (value > 0.0F)
        {
            return std::make_pair(true, false);
        }
    if (value > -1.5F)
        {
            return std::make_pair(false, true);
        }
    return std::make_pair(false, false);
}

uint16_t pack_single_2bit(const std::vector<gr_complex>& samples)
{
    uint16_t word = 0;
    for (size_t i = 0; i < samples.size(); ++i)
        {
            const auto i_bits = bits_for_unsigned_2bit(samples[i].real());
            const auto q_bits = bits_for_unsigned_2bit(samples[i].imag());
            set_msb_bit(word, static_cast<int>(4 * i), i_bits.first);
            set_msb_bit(word, static_cast<int>(4 * i + 1), q_bits.first);
            set_msb_bit(word, static_cast<int>(4 * i + 2), i_bits.second);
            set_msb_bit(word, static_cast<int>(4 * i + 3), q_bits.second);
        }
    return word;
}

uint64_t pack_ls4_2bit(const std::vector<std::pair<uint8_t, uint8_t>>& samples)
{
    uint64_t word = 0;
    for (size_t i = 0; i < samples.size(); ++i)
        {
            const int offset = static_cast<int>(4 * i);
            set_msb_bit(word, offset, (samples[i].first & 0x2U) != 0U);
            set_msb_bit(word, offset + 1, (samples[i].first & 0x1U) != 0U);
            set_msb_bit(word, offset + 2, (samples[i].second & 0x2U) != 0U);
            set_msb_bit(word, offset + 3, (samples[i].second & 0x1U) != 0U);
        }
    return word;
}

std::array<uint64_t, 3> pack_ls4_12bit(const std::vector<std::pair<uint16_t, uint16_t>>& samples)
{
    std::array<uint64_t, 3> words{};
    const auto write_component = [&words](int bit_offset, uint16_t value) {
        for (int bit = 0; bit < 12; ++bit)
            {
                const int absolute_bit_offset = bit_offset + bit;
                const bool is_set = (value & static_cast<uint16_t>(1U << (11 - bit))) != 0U;
                set_msb_bit(words[absolute_bit_offset / 64], absolute_bit_offset % 64, is_set);
            }
    };

    for (size_t i = 0; i < samples.size(); ++i)
        {
            const int bit_offset = static_cast<int>(24 * i);
            write_component(bit_offset, samples[i].first);
            write_component(bit_offset + 12, samples[i].second);
        }
    return words;
}

uint16_t pack_dual_1bit(const std::vector<gr_complex>& channel_a, const std::vector<gr_complex>& channel_b)
{
    uint16_t word = 0;
    for (size_t i = 0; i < channel_a.size(); ++i)
        {
            set_msb_bit(word, static_cast<int>(4 * i), channel_a[i].real() > 0);
            set_msb_bit(word, static_cast<int>(4 * i + 1), channel_a[i].imag() > 0);
            set_msb_bit(word, static_cast<int>(4 * i + 2), channel_b[i].real() > 0);
            set_msb_bit(word, static_cast<int>(4 * i + 3), channel_b[i].imag() > 0);
        }
    return word;
}

uint32_t pack_triple_1bit(const std::vector<gr_complex>& channel_a, const std::vector<gr_complex>& channel_b, const std::vector<gr_complex>& channel_c)
{
    uint32_t word = 0;
    for (size_t i = 0; i < channel_a.size(); ++i)
        {
            const int offset = 2 + static_cast<int>(6 * i);
            set_msb_bit(word, offset, channel_a[i].real() > 0);
            set_msb_bit(word, offset + 1, channel_a[i].imag() > 0);
            set_msb_bit(word, offset + 2, channel_b[i].real() > 0);
            set_msb_bit(word, offset + 3, channel_b[i].imag() > 0);
            set_msb_bit(word, offset + 4, channel_c[i].real() > 0);
            set_msb_bit(word, offset + 5, channel_c[i].imag() > 0);
        }
    return word;
}

std::pair<bool, bool> bits_for_ls3w_2bit(float value)
{
    if (value > 0.75F)
        {
            return std::make_pair(false, true);
        }
    if (value > 0.0F)
        {
            return std::make_pair(false, false);
        }
    if (value < -0.75F)
        {
            return std::make_pair(true, false);
        }
    return std::make_pair(true, true);
}

uint64_t pack_ls3w_3ch_2bit(const std::vector<gr_complex>& channel_a, const std::vector<gr_complex>& channel_b, const std::vector<gr_complex>& channel_c)
{
    uint64_t word = 0;
    for (size_t i = 0; i < channel_a.size(); ++i)
        {
            const int sample_offset = 4 + static_cast<int>(12 * i);
            const std::array<gr_complex, 3> channels = {{channel_a[i], channel_b[i], channel_c[i]}};
            for (size_t channel = 0; channel < channels.size(); ++channel)
                {
                    const auto i_bits = bits_for_ls3w_2bit(channels[channel].real());
                    const auto q_bits = bits_for_ls3w_2bit(channels[channel].imag());
                    const int offset = sample_offset + static_cast<int>(4 * channel);
                    set_msb_bit(word, offset, i_bits.first);
                    set_msb_bit(word, offset + 1, i_bits.second);
                    set_msb_bit(word, offset + 2, q_bits.first);
                    set_msb_bit(word, offset + 3, q_bits.second);
                }
        }
    return word;
}

LabSatRunResult run_labsat_source(const std::string& file_base, const std::vector<int>& selected_channels)
{
    Concurrent_Queue<pmt::pmt_t> queue;
    auto source = labsat23_make_source_sptr(file_base.c_str(), selected_channels, &queue, false, 0.0);
    auto top_block = gr::make_top_block("labsat23_source_test");

    std::vector<gr::blocks::vector_sink_c::sptr> sinks;
    for (size_t i = 0; i < selected_channels.size(); ++i)
        {
            auto sink = gr::blocks::vector_sink_c::make();
            top_block->connect(source, static_cast<int>(i), sink, 0);
            sinks.push_back(std::move(sink));
        }

    top_block->run();

    LabSatRunResult result;
    for (const auto& sink : sinks)
        {
            result.outputs.push_back(sink->data());
        }
    return result;
}

void expect_complex_vector_eq(const std::vector<gr_complex>& expected, const std::vector<gr_complex>& actual)
{
    ASSERT_GE(actual.size(), expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        {
            ASSERT_FLOAT_EQ(expected[i].real(), actual[i].real()) << "sample " << i;
            ASSERT_FLOAT_EQ(expected[i].imag(), actual[i].imag()) << "sample " << i;
        }
}

}  // namespace


TEST(LabSat23SourceTest, FindsSectionTwoAfterEarlierHeaderSection)
{
    fs::path path = temp_path("labsat23_header_scan");
    path.replace_extension(".ls2");
    const std::vector<gr_complex> samples = {
        gr_complex(1, -1), gr_complex(-1, 1), gr_complex(1, 1), gr_complex(-1, -1),
        gr_complex(1, -1), gr_complex(-1, 1), gr_complex(1, 1), gr_complex(-1, -1)};

    std::vector<uint8_t> bytes;
    append_header(bytes, {{'L', 'S', '2'}}, {{0x0001, {0xAA, 0xBB}}, {0x0002, section2_payload(2, 1)}});
    append_le16(bytes, pack_single_1bit(samples));
    write_bytes(path, bytes);

    const auto result = run_labsat_source(path.string(), {1});

    ASSERT_EQ(1U, result.outputs.size());
    expect_complex_vector_eq(samples, result.outputs[0]);

    expect_remove(path);
}


TEST(LabSat23SourceTest, DecodesSingleChannelTwoBitUnsignedBinary)
{
    fs::path path = temp_path("labsat23_single_2bit");
    path.replace_extension(".ls2");
    const std::vector<gr_complex> samples = {
        gr_complex(2, -2), gr_complex(1, -1), gr_complex(-1, 1), gr_complex(-2, 2)};

    std::vector<uint8_t> bytes;
    append_header(bytes, {{'L', 'S', '2'}}, {{0x0002, section2_payload(4, 3)}});
    append_le16(bytes, pack_single_2bit(samples));
    write_bytes(path, bytes);

    const auto result = run_labsat_source(path.string(), {1});

    ASSERT_EQ(1U, result.outputs.size());
    expect_complex_vector_eq(samples, result.outputs[0]);

    expect_remove(path);
}


TEST(LabSat23SourceTest, RejectsUnknownSectionTwoQuantization)
{
    fs::path path = temp_path("labsat23_bad_quantization");
    path.replace_extension(".ls2");
    const std::vector<gr_complex> samples = {
        gr_complex(1, -1), gr_complex(-1, 1), gr_complex(1, 1), gr_complex(-1, -1),
        gr_complex(1, -1), gr_complex(-1, 1), gr_complex(1, 1), gr_complex(-1, -1)};
    auto payload = section2_payload(2, 1);
    payload[3] = 2;

    std::vector<uint8_t> bytes;
    append_header(bytes, {{'L', 'S', '2'}}, {{0x0002, std::move(payload)}});
    append_le16(bytes, pack_single_1bit(samples));
    write_bytes(path, bytes);

    const auto result = run_labsat_source(path.string(), {1});

    ASSERT_EQ(1U, result.outputs.size());
    EXPECT_TRUE(result.outputs[0].empty());

    expect_remove(path);
}


TEST(LabSat23SourceTest, DecodesDualChannelOneBitSelectedOutputs)
{
    fs::path path = temp_path("labsat23_dual");
    path.replace_extension(".ls2");
    const std::vector<gr_complex> channel_a = {
        gr_complex(1, -1), gr_complex(-1, 1), gr_complex(1, 1), gr_complex(-1, -1)};
    const std::vector<gr_complex> channel_b = {
        gr_complex(-1, -1), gr_complex(1, 1), gr_complex(-1, 1), gr_complex(1, -1)};

    std::vector<uint8_t> bytes;
    append_header(bytes, {{'L', 'S', '2'}}, {{0x0002, section2_payload(4, 0)}});
    append_le16(bytes, pack_dual_1bit(channel_a, channel_b));
    write_bytes(path, bytes);

    const auto result = run_labsat_source(path.string(), {1, 2});

    ASSERT_EQ(2U, result.outputs.size());
    expect_complex_vector_eq(channel_a, result.outputs[0]);
    expect_complex_vector_eq(channel_b, result.outputs[1]);

    expect_remove(path);
}


TEST(LabSat23SourceTest, DecodesLs3TripleConstellation)
{
    const fs::path base = temp_path("labsat23_triple");
    const fs::path path(base.string() + "_0000.LS3");
    const std::vector<gr_complex> channel_a = {
        gr_complex(1, -1), gr_complex(-1, 1), gr_complex(1, 1), gr_complex(-1, -1), gr_complex(1, -1)};
    const std::vector<gr_complex> channel_b = {
        gr_complex(-1, -1), gr_complex(1, 1), gr_complex(-1, 1), gr_complex(1, -1), gr_complex(-1, -1)};
    const std::vector<gr_complex> channel_c = {
        gr_complex(1, 1), gr_complex(1, -1), gr_complex(-1, -1), gr_complex(-1, 1), gr_complex(1, 1)};

    std::vector<uint8_t> bytes;
    append_header(bytes, {{'L', 'S', '3'}}, {{0x0002, section2_payload_ls3_triple()}});
    append_le32(bytes, pack_triple_1bit(channel_a, channel_b, channel_c));
    write_bytes(path, bytes);

    const auto result = run_labsat_source(base.string(), {1, 2, 3});

    ASSERT_EQ(3U, result.outputs.size());
    expect_complex_vector_eq(channel_a, result.outputs[0]);
    expect_complex_vector_eq(channel_b, result.outputs[1]);
    expect_complex_vector_eq(channel_c, result.outputs[2]);

    expect_remove(path);
}


TEST(LabSat23SourceTest, DecodesLs3WidebandFromCorrectMsbOffset)
{
    fs::path path = temp_path("labsat23_wideband");
    path.replace_extension(".LS3W");
    fs::path ini_path(path);
    ini_path.replace_extension(".ini");
    const std::vector<gr_complex> channel_a = {
        gr_complex(1, 0.5F), gr_complex(-1, -0.5F), gr_complex(0.5F, 1), gr_complex(-0.5F, -1), gr_complex(1, -1)};
    const std::vector<gr_complex> channel_b = {
        gr_complex(0.5F, -1), gr_complex(1, -0.5F), gr_complex(-1, 0.5F), gr_complex(-0.5F, 1), gr_complex(0.5F, -0.5F)};
    const std::vector<gr_complex> channel_c = {
        gr_complex(-0.5F, 1), gr_complex(0.5F, -1), gr_complex(1, -0.5F), gr_complex(-1, 0.5F), gr_complex(-0.5F, 0.5F)};

    std::vector<uint8_t> bytes;
    const uint64_t packed_register = pack_ls3w_3ch_2bit(channel_a, channel_b, channel_c);
    for (int register_index = 0; register_index < 2048; ++register_index)
        {
            append_le64(bytes, packed_register);
        }
    write_bytes(path, bytes);

    std::ofstream ini(ini_path.string().c_str());
    ASSERT_TRUE(ini.is_open());
    ini << "[config]\n"
        << "OSC=TCXO\n"
        << "SMP=33000000\n"
        << "QUA=2\n"
        << "CHN=3\n"
        << "SFT=12\n"
        << "[channel A]\n"
        << "CFA=1575420000\n"
        << "BWA=30000000\n"
        << "[channel B]\n"
        << "CFB=1227600000\n"
        << "BWB=30000000\n"
        << "[channel C]\n"
        << "CFC=1540000000\n"
        << "BWC=30000000\n";
    ini.close();

    const auto result = run_labsat_source(path.string(), {1, 2, 3});

    ASSERT_EQ(3U, result.outputs.size());
    expect_complex_vector_eq(channel_a, result.outputs[0]);
    expect_complex_vector_eq(channel_b, result.outputs[1]);
    expect_complex_vector_eq(channel_c, result.outputs[2]);

    expect_remove(path);
    expect_remove(ini_path);
}


TEST(LabSat23SourceTest, DecodesLs4TwoBitSamples)
{
    fs::path path = temp_path("labsat4_two_bit");
    path.replace_extension(".LS4");
    fs::path ini_path(path);
    ini_path.replace_extension(".ini");

    const std::vector<std::pair<uint8_t, uint8_t>> codes = {
        {0, 0}, {1, 2}, {2, 3}, {3, 1}, {0, 2}, {1, 3}, {2, 0}, {3, 1},
        {0, 3}, {1, 0}, {2, 1}, {3, 2}, {0, 1}, {1, 1}, {2, 2}, {3, 3}};
    const std::vector<gr_complex> expected = {
        gr_complex(0.5F, 0.5F), gr_complex(1.0F, -1.0F), gr_complex(-1.0F, -0.5F), gr_complex(-0.5F, 1.0F),
        gr_complex(0.5F, -1.0F), gr_complex(1.0F, -0.5F), gr_complex(-1.0F, 0.5F), gr_complex(-0.5F, 1.0F),
        gr_complex(0.5F, -0.5F), gr_complex(1.0F, 0.5F), gr_complex(-1.0F, 1.0F), gr_complex(-0.5F, -1.0F),
        gr_complex(0.5F, 1.0F), gr_complex(1.0F, 1.0F), gr_complex(-1.0F, -1.0F), gr_complex(-0.5F, -0.5F)};

    std::vector<uint8_t> bytes;
    const uint64_t packed_register = pack_ls4_2bit(codes);
    for (int register_index = 0; register_index < 2048; ++register_index)
        {
            append_le64(bytes, packed_register);
        }
    write_bytes(path, bytes);

    std::ofstream ini(ini_path.string().c_str());
    ASSERT_TRUE(ini.is_open());
    ini << "[config]\n"
        << "OSC=TCXO\n"
        << "SMP=4000000\n"
        << "QUA=2\n"
        << "CHN=1\n"
        << "SFT=4\n"
        << "BW_MAX=4000000\n"
        << "BW_DIV_A=1\n"
        << "BW_DIV_B=1\n"
        << "BW_DIV_C=1\n"
        << "[channel A]\n"
        << "CFA=1575420000\n"
        << "BWA=4000000\n"
        << "BUF_SIZE_A=16384\n"
        << "[channel B]\n"
        << "CFB=1227600000\n"
        << "BWB=0\n"
        << "BUF_SIZE_B=0\n"
        << "[channel C]\n"
        << "CFC=1176450000\n"
        << "BWC=0\n"
        << "BUF_SIZE_C=0\n";
    ini.close();

    const auto result = run_labsat_source(path.string(), {1});

    ASSERT_EQ(1U, result.outputs.size());
    expect_complex_vector_eq(expected, result.outputs[0]);

    expect_remove(path);
    expect_remove(ini_path);
}


TEST(LabSat23SourceTest, DecodesLs4TwelveBitSamplesAcrossRegisters)
{
    fs::path path = temp_path("labsat4_twelve_bit");
    path.replace_extension(".LS4");
    fs::path ini_path(path);
    ini_path.replace_extension(".ini");

    constexpr float quantization_step = 1.0F / 2048.0F;
    const std::vector<std::pair<uint16_t, uint16_t>> codes = {
        {0, 0}, {1, 2047}, {2048, 4095}, {4095, 2048},
        {2047, 1}, {0, 4095}, {2048, 2047}, {1, 0}};
    const std::vector<gr_complex> expected = {
        gr_complex(quantization_step, quantization_step), gr_complex(2.0F * quantization_step, 1.0F),
        gr_complex(-1.0F, -quantization_step), gr_complex(-quantization_step, -1.0F),
        gr_complex(1.0F, 2.0F * quantization_step), gr_complex(quantization_step, -quantization_step),
        gr_complex(-1.0F, 1.0F), gr_complex(2.0F * quantization_step, quantization_step)};

    std::vector<uint8_t> bytes;
    const auto packed_registers = pack_ls4_12bit(codes);
    for (int register_index = 0; register_index < 2048; ++register_index)
        {
            for (const auto packed_register : packed_registers)
                {
                    append_le64(bytes, packed_register);
                }
        }
    write_bytes(path, bytes);

    std::ofstream ini(ini_path.string().c_str());
    ASSERT_TRUE(ini.is_open());
    ini << "[config]\n"
        << "OSC=OCXO\n"
        << "SMP=20460000\n"
        << "QUA=12\n"
        << "CHN=1\n"
        << "SFT=24\n"
        << "BW_MAX=20000000\n"
        << "BW_DIV_A=2\n"
        << "BW_DIV_B=2\n"
        << "BW_DIV_C=1\n"
        << "[channel A]\n"
        << "CFA=1575420000\n"
        << "BWA=10000000\n"
        << "BUF_SIZE_A=49152\n"
        << "[channel B]\n"
        << "CFB=1593322500\n"
        << "BWB=0\n"
        << "BUF_SIZE_B=0\n"
        << "[channel C]\n"
        << "CFC=1190516250\n"
        << "BWC=0\n"
        << "BUF_SIZE_C=0\n";
    ini.close();

    const auto result = run_labsat_source(path.string(), {1});

    ASSERT_EQ(1U, result.outputs.size());
    expect_complex_vector_eq(expected, result.outputs[0]);

    expect_remove(path);
    expect_remove(ini_path);
}


TEST(LabSat23SourceTest, DecodesLs4TwoChannelTwelveBitCombinedLayout)
{
    fs::path path = temp_path("labsat4_combined_twelve_bit");
    path.replace_extension(".LS4");
    fs::path ini_path(path);
    ini_path.replace_extension(".ini");

    constexpr float quantization_step = 1.0F / 2048.0F;
    const std::vector<std::pair<uint16_t, uint16_t>> channel_a_codes = {
        {0, 2047}, {1, 2048}, {2048, 0}, {4095, 4095},
        {2047, 1}, {0, 2048}, {1, 4095}, {2048, 2047}};
    const std::vector<std::pair<uint16_t, uint16_t>> channel_b_codes = {
        {4095, 0}, {2048, 1}, {1, 2047}, {0, 2048},
        {2047, 4095}, {1, 0}, {2048, 2047}, {4095, 2048}};
    const std::vector<gr_complex> channel_a_expected = {
        gr_complex(quantization_step, 1.0F), gr_complex(2.0F * quantization_step, -1.0F),
        gr_complex(-1.0F, quantization_step), gr_complex(-quantization_step, -quantization_step),
        gr_complex(1.0F, 2.0F * quantization_step), gr_complex(quantization_step, -1.0F),
        gr_complex(2.0F * quantization_step, -quantization_step), gr_complex(-1.0F, 1.0F)};
    const std::vector<gr_complex> channel_b_expected = {
        gr_complex(-quantization_step, quantization_step), gr_complex(-1.0F, 2.0F * quantization_step),
        gr_complex(2.0F * quantization_step, 1.0F), gr_complex(quantization_step, -1.0F),
        gr_complex(1.0F, -quantization_step), gr_complex(2.0F * quantization_step, quantization_step),
        gr_complex(-1.0F, 1.0F), gr_complex(-quantization_step, -1.0F)};

    std::vector<uint8_t> bytes;
    const auto channel_a_registers = pack_ls4_12bit(channel_a_codes);
    const auto channel_b_registers = pack_ls4_12bit(channel_b_codes);
    for (const auto& channel_registers : {channel_a_registers, channel_b_registers})
        {
            for (int register_index = 0; register_index < 2048; ++register_index)
                {
                    for (const auto packed_register : channel_registers)
                        {
                            append_le64(bytes, packed_register);
                        }
                }
        }
    write_bytes(path, bytes);

    std::ofstream ini(ini_path.string().c_str());
    ASSERT_TRUE(ini.is_open());
    ini << "[config]\n"
        << "OSC=OCXO\n"
        << "OP_MODE=LS4\n"
        << "BW_MAX=20000000\n"
        << "QUA=12\n"
        << "CHN=2\n"
        << "SFT=48\n"
        << "BW_DIV_A=1\n"
        << "BW_DIV_B=1\n"
        << "BW_DIV_C=1\n"
        << "SMP=20460000\n"
        << "[channel A]\n"
        << "CFA=1584000000\n"
        << "BWA=20000000\n"
        << "BUF_SIZE_A=49152\n"
        << "[channel B]\n"
        << "CFB=1190516250\n"
        << "BWB=20000000\n"
        << "BUF_SIZE_B=49152\n"
        << "[channel C]\n"
        << "CFC=-1\n"
        << "BWC=-1\n"
        << "BUF_SIZE_C=0\n";
    ini.close();

    const auto result = run_labsat_source(path.string(), {1, 2});

    ASSERT_EQ(2U, result.outputs.size());
    expect_complex_vector_eq(channel_a_expected, result.outputs[0]);
    expect_complex_vector_eq(channel_b_expected, result.outputs[1]);

    const auto duplicate_channel_result = run_labsat_source(path.string(), {1, 1, 2});

    ASSERT_EQ(3U, duplicate_channel_result.outputs.size());
    expect_complex_vector_eq(channel_a_expected, duplicate_channel_result.outputs[0]);
    expect_complex_vector_eq(channel_a_expected, duplicate_channel_result.outputs[1]);
    expect_complex_vector_eq(channel_b_expected, duplicate_channel_result.outputs[2]);

    expect_remove(path);
    expect_remove(ini_path);
}
