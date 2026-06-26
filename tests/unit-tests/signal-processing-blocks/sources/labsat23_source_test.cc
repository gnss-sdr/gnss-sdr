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
#include <gnuradio/blocks/vector_sink.h>
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
            sinks.push_back(sink);
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
            EXPECT_FLOAT_EQ(expected[i].real(), actual[i].real()) << "sample " << i;
            EXPECT_FLOAT_EQ(expected[i].imag(), actual[i].imag()) << "sample " << i;
        }
}

}  // namespace


TEST(LabSat23SourceTest, FindsSectionTwoAfterEarlierHeaderSection)
{
    const auto path = temp_path("labsat23_header_scan").replace_extension(".ls2");
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

    std::remove(path.string().c_str());
}


TEST(LabSat23SourceTest, DecodesSingleChannelTwoBitUnsignedBinary)
{
    const auto path = temp_path("labsat23_single_2bit").replace_extension(".ls2");
    const std::vector<gr_complex> samples = {
        gr_complex(2, -2), gr_complex(1, -1), gr_complex(-1, 1), gr_complex(-2, 2)};

    std::vector<uint8_t> bytes;
    append_header(bytes, {{'L', 'S', '2'}}, {{0x0002, section2_payload(4, 3)}});
    append_le16(bytes, pack_single_2bit(samples));
    write_bytes(path, bytes);

    const auto result = run_labsat_source(path.string(), {1});

    ASSERT_EQ(1U, result.outputs.size());
    expect_complex_vector_eq(samples, result.outputs[0]);

    std::remove(path.string().c_str());
}


TEST(LabSat23SourceTest, DecodesDualChannelOneBitSelectedOutputs)
{
    const auto path = temp_path("labsat23_dual").replace_extension(".ls2");
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

    std::remove(path.string().c_str());
}


TEST(LabSat23SourceTest, DecodesLs3TripleConstellation)
{
    const auto base = temp_path("labsat23_triple");
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

    std::remove(path.string().c_str());
}


TEST(LabSat23SourceTest, DecodesLs3WidebandFromCorrectMsbOffset)
{
    const auto path = temp_path("labsat23_wideband").replace_extension(".LS3W");
    const auto ini_path = fs::path(path).replace_extension(".ini");
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

    std::remove(path.string().c_str());
    std::remove(ini_path.string().c_str());
}
