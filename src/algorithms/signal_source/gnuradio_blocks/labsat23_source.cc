/*!
 * \file labsat23_source.cc
 *
 * \brief Unpacks capture files in the LabSat 2 (ls2), LabSat 3 (ls3), LabSat 3
 * Wideband (LS3W), and Labsat 4 (ls4) formats.
 * \author Javier Arribas jarribas (at) cttc.es
 *         Mathieu Favreau favreau.mathieu (at) hotmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "labsat23_source.h"
#include "INIReader.h"
#include "command_event.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_make_unique.h"  // for std::make_unique in C++11
#include <bitset>
#include <set>
#include <unordered_set>

#if HAS_BOOST_ENDIAN
#include <boost/endian/conversion.hpp>
#endif

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{

std::vector<double> generate_mapping(int qua)
{
    const int levels = 1 << qua;
    const int half = levels / 2;
    std::vector<double> mapping(levels);

    // Positive half
    for (int i = 0; i < half; i++)
        {
            mapping[i] = static_cast<double>(i + 1) / static_cast<double>(half);
        }

    // Negative half (mirror reversed)
    for (int i = 0; i < half; i++)
        {
            mapping[half + i] = -mapping[half - 1 - i];
        }

    return mapping;
}


void write_samples_from_bitset(const std::bitset<64> &bs, int bit_offset, int qua, gr_complex &out)
{
    const auto extract_bits = [&](int start, int count) {
        unsigned val = 0;
        for (int i = 0; i < count; ++i)
            {
                val = (val << 1) | bs[63 - start - i];
            }
        return val;
    };

    const auto i_bits = extract_bits(bit_offset, qua);
    const auto q_bits = extract_bits(bit_offset + qua, qua);

    // Cache mapping per qua (so we only build it once), possible qua are 1,2,3,4,8,12
    static std::vector<std::vector<double>> cache(12);
    auto &mapping = cache[qua - 1];

    if (mapping.empty())
        {
            mapping = generate_mapping(qua);
        }

    out = gr_complex(mapping[i_bits], mapping[q_bits]);
}


void write_samples_ls4(uint64_t data_index, int out_index, int sample_count, int32_t qua, std::vector<uint64_t> &data, gr_complex *out_samples)
{
    const auto buffer_index = data_index % data.size();

    if (qua == 12)
        {
            // For 12 bit QUA, write 48 (MSBs) bits (2x IQ samples) and then shift left and add new data for next write (read 3 registers of 64 bits)

            const auto write_value = [qua, out_samples](uint64_t value, uint64_t index) {
                std::bitset<64> bs(value);
                write_samples_from_bitset(bs, 0, qua, out_samples[index]);
                write_samples_from_bitset(bs, 24, qua, out_samples[index + 1]);
            };

            uint64_t value = data[buffer_index];
            write_value(value, out_index);

            value = value << 48;
            value = value | (data[buffer_index + 1] >> 16);
            write_value(value, out_index + 2);

            value = value << 48;
            value = value | ((data[buffer_index + 1] & 0xFFFF) << 32);
            value = value | (data[buffer_index + 2] >> 32);
            write_value(value, out_index + 4);

            value = value << 48;
            value = value | ((data[buffer_index + 2] & 0xFFFFFFFF) << 16);
            write_value(value, out_index + 6);
        }
    else
        {
            const auto buffer_index = data_index % data.size();
            const std::bitset<64> bs(data[buffer_index]);

            for (int32_t i = 0; i < sample_count; ++i)
                {
                    write_samples_from_bitset(bs, i * qua * 2, qua, out_samples[out_index + i]);
                }
        }
}


void read_file_register_to_local_endian(std::ifstream &binary_input_file, uint64_t &read_register)
{
#if HAS_BOOST_ENDIAN
    binary_input_file.read(reinterpret_cast<char *>(&read_register), sizeof(read_register));
    boost::endian::little_to_native_inplace(read_register);
#else
    std::array<char, 8> memory_block{};
    binary_input_file.read(memory_block.data(), 8);
    for (int k = 7; k >= 0; --k)
        {
            read_register <<= 8;
            read_register |= uint64_t(memory_block[k]);  // This is buggy if the MSB of the char is set.
        }
#endif
}

bool are_equal_ignore_nonpositive(const std::vector<int32_t> &values)
{
    std::vector<int32_t> positives;
    for (const auto &v : values)
        {
            if (v > 0)
                {
                    positives.push_back(v);
                }
        }

    if (positives.size() <= 1)
        {
            return true;
        }

    return std::all_of(positives.begin(), positives.end(),
        [&](int32_t v) { return v == positives[0]; });
}

uint8_t as_u8(char value)
{
    return static_cast<uint8_t>(value);
}

uint16_t read_le_u16(const char *data)
{
    return static_cast<uint16_t>(as_u8(data[0])) |
           static_cast<uint16_t>(as_u8(data[1]) << 8);
}

uint32_t read_le_u32(const char *data)
{
    return static_cast<uint32_t>(as_u8(data[0])) |
           (static_cast<uint32_t>(as_u8(data[1])) << 8) |
           (static_cast<uint32_t>(as_u8(data[2])) << 16) |
           (static_cast<uint32_t>(as_u8(data[3])) << 24);
}

}  // namespace


labsat23_source_sptr labsat23_make_source_sptr(
    const char *signal_file_basename,
    const std::vector<int> &channel_selector,
    Concurrent_Queue<pmt::pmt_t> *queue,
    bool digital_io_enabled,
    double seconds_to_skip)
{
    return labsat23_source_sptr(new labsat23_source(signal_file_basename, channel_selector, queue, digital_io_enabled, seconds_to_skip));
}


labsat23_source::labsat23_source(
    const char *signal_file_basename,
    const std::vector<int> &channel_selector,
    Concurrent_Queue<pmt::pmt_t> *queue,
    bool digital_io_enabled,
    double seconds_to_skip) : gr::block("labsat23_source",
                                  gr::io_signature::make(0, 0, 0),
                                  gr::io_signature::make(1, 3, sizeof(gr_complex))),
                              d_queue(queue),
                              d_channel_selector_config(channel_selector),
                              d_current_file_number(0),
                              d_labsat_version(0),
                              d_channel_selector(0),
                              d_ref_clock(0),
                              d_bits_per_sample(0),
                              d_header_parsed(false),
                              d_ls3w_digital_io_enabled(digital_io_enabled)
{
    d_signal_file_basename = std::string(signal_file_basename);
    std::string signal_file;
    this->set_output_multiple(8);
    signal_file = generate_filename();

    if (d_is_ls3w || d_is_ls4)
        {
            if (d_is_ls3w)
                {
                    d_labsat_version = 3;
                    std::cout << "LabSat file version 3 Wideband detected.\n";
                }
            else if (d_is_ls4)
                {
                    d_labsat_version = 4;
                    std::cout << "LabSat file version 4 detected.\n";
                }

            fs::path file_path(signal_file);
            file_path.replace_extension(".ini");

            // Read ini file
            if (read_ls3w_ini(file_path.string()) != 0)
                {
                    exit(1);
                }
        }

    binary_input_file.open(signal_file.c_str(), std::ios::in | std::ios::binary);

    if (binary_input_file.is_open())
        {
            std::cout << "LabSat file source is reading samples from " << signal_file;

            if (d_is_ls4)
                {
                    d_number_register_per_output = d_ls3w_QUA == 12 ? 3 : 1;
                    d_number_sample_per_output = (d_number_register_per_output * 64) / (d_ls3w_QUA * 2);
                    this->set_output_multiple(d_number_sample_per_output);

                    const auto size = fs::file_size(d_signal_file_basename);
                    const auto bits = size * CHAR_BIT;

                    double bw_factor = 0;

                    for (const auto &channel_map_it : d_channel_map)
                        {
                            const auto &channel = channel_map_it.second;

                            if (channel.buff_size > 0)
                                {
                                    bw_factor += 1.0 / channel.bw_div;
                                }
                        }

                    const auto bits_per_sample = d_ls3w_QUA * 2;  // I and Q
                    const auto samples = bits / bits_per_sample;
                    const auto signal_duration_s = bits / (bits_per_sample * bw_factor * d_ls3w_SMP);
                    std::cout << ", which contains " << samples << " samples (" << size << " bytes)\n";
                    std::cout << "GNSS signal recorded time to be processed: " << (signal_duration_s - seconds_to_skip) << " [s]\n";

                    if (seconds_to_skip > 0)
                        {
                            if (signal_duration_s < seconds_to_skip)
                                {
                                    std::cout << "File duration is smaller then the seconds to skip!\n";
                                    exit(1);
                                }

                            LOG(INFO) << "Skipping " << seconds_to_skip << " seconds of the input file";

                            int64_t total_bytes_to_seek = 0;

                            for (auto &channel_map_it : d_channel_map)
                                {
                                    auto &channel = channel_map_it.second;

                                    if (channel.buff_size > 0)
                                        {
                                            const auto samples_to_skip = static_cast<int64_t>(seconds_to_skip * (static_cast<double>(d_ls3w_SMP) / channel.bw_div));
                                            const auto bytes_to_skip = (samples_to_skip * bits_per_sample) / CHAR_BIT;
                                            const auto buffers_to_skip = bytes_to_skip / channel.buff_size;
                                            const auto bytes_to_seek = buffers_to_skip * channel.buff_size;
                                            const auto bytes_to_read = bytes_to_skip - bytes_to_seek;
                                            total_bytes_to_seek += bytes_to_seek;
                                            channel.data_index = (bytes_to_read / sizeof(uint64_t));  // Can be set before read is called
                                        }
                                }

                            // Advance in the file by a multiple of buffers, then read one buffer of each channel
                            if (!binary_input_file.seekg(total_bytes_to_seek, std::ios::beg) || !read_ls4_data())
                                {
                                    LOG(ERROR) << "Error skipping bytes!";
                                    exit(1);
                                }
                        }
                }
            else
                {
                    std::cout << '\n';
                }
        }
    else
        {
            std::cout << "LabSat file " << signal_file << " could not be opened!\n";
            exit(1);
        }
}


labsat23_source::~labsat23_source()
{
    try
        {
            if (binary_input_file.is_open())
                {
                    binary_input_file.close();
                }
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem closing input file.\n";
        }
    catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
}


std::string labsat23_source::generate_filename()
{
    fs::path file_path(d_signal_file_basename);
    const auto extension = file_path.extension();

    if (extension == ".ls2" or extension == ".LS2")
        {
            if (d_current_file_number == 0)
                {
                    return d_signal_file_basename;
                }
            return {"donotexist"};  // just to stop processing
        }
    if (extension == ".ls3w" or extension == ".LS3W")
        {
            d_is_ls3w = true;
            return d_signal_file_basename;
        }
    if (extension == ".ls4" or extension == ".LS4")
        {
            d_is_ls4 = true;
            return d_signal_file_basename;
        }

    std::ostringstream ss;
    ss << std::setw(4) << std::setfill('0') << d_current_file_number;
    return d_signal_file_basename + "_" + ss.str() + ".LS3";
}


int labsat23_source::parse_header()
{
    if (binary_input_file.eof() == false)
        {
            const int fixed_header_bytes = 16;
            std::array<char, fixed_header_bytes> header_prefix{};
            binary_input_file.read(header_prefix.data(), fixed_header_bytes);
            if (binary_input_file.gcount() != fixed_header_bytes)
                {
                    std::cout << "LabSat file read error: header is incomplete.\n";
                    return -1;
                }

            // parse Labsat header
            // check preamble
            int byte_counter = 0;
            bool preamble_ok = true;
            for (int i = 0; i < 8; i++)
                {
                    if (header_prefix[byte_counter] != 0x00)
                        {
                            preamble_ok = false;
                        }
                    byte_counter++;
                }

            if (preamble_ok == false)
                {
                    std::cout << "LabSat source do not detect the preamble in the selected file\n";
                    return -1;
                }

            // check Labsat version
            if (header_prefix[byte_counter] == 0x4C and header_prefix[byte_counter + 1] == 0x53 and header_prefix[byte_counter + 2] == 0x32)
                {
                    d_labsat_version = 2;
                    std::cout << "LabSat file version 2 detected\n";
                }

            if (header_prefix[byte_counter] == 0x4C and header_prefix[byte_counter + 1] == 0x53 and header_prefix[byte_counter + 2] == 0x33)
                {
                    d_labsat_version = 3;
                    std::cout << "LabSat file version 3 detected\n";
                }

            if (d_labsat_version == 0)
                {
                    std::cout << "LabSat source do not detect the version number in the file header\n";
                    return -1;
                }

            byte_counter += 3;

            int sub_version = static_cast<int>(as_u8(header_prefix[byte_counter]));

            std::cout << "LabSat file sub version " << sub_version << '\n';

            byte_counter++;

            const uint32_t header_bytes = read_le_u32(&header_prefix[byte_counter]);
            if (header_bytes < fixed_header_bytes)
                {
                    std::cout << "LabSat file header error: header length is invalid.\n";
                    return -1;
                }

            std::vector<char> memblock(header_bytes);
            for (int i = 0; i < fixed_header_bytes; i++)
                {
                    memblock[i] = header_prefix[i];
                }
            binary_input_file.read(memblock.data() + fixed_header_bytes, header_bytes - fixed_header_bytes);
            if (static_cast<uint32_t>(binary_input_file.gcount()) != header_bytes - fixed_header_bytes)
                {
                    std::cout << "LabSat file read error: header is incomplete.\n";
                    return -1;
                }

            byte_counter = fixed_header_bytes;
            while (static_cast<uint32_t>(byte_counter) + 6 <= header_bytes)
                {
                    const uint16_t section_id = read_le_u16(&memblock[byte_counter]);
                    byte_counter += 2;

                    const uint32_t section_length_bytes = read_le_u32(&memblock[byte_counter]);
                    byte_counter += 4;

                    const uint32_t section_end = static_cast<uint32_t>(byte_counter) + section_length_bytes;
                    if (section_end > header_bytes)
                        {
                            std::cout << "LabSat file header error: section length exceeds header length.\n";
                            return -1;
                        }

                    if (section_id == 2)
                        {
                            if (section_length_bytes < 6)
                                {
                                    std::cout << "LabSat file header error: section 2 is incomplete.\n";
                                    return -1;
                                }

                            d_ref_clock = static_cast<uint8_t>(memblock[byte_counter]);
                            switch (d_ref_clock)
                                {
                                case 0:
                                    std::cout << "LabSat reference clock: internal OCXO\n";
                                    break;
                                case 1:
                                    std::cout << "LabSat reference clock: internal TCXO\n";
                                    break;
                                case 2:
                                    std::cout << "LabSat reference clock: external 10 MHz\n";
                                    break;
                                case 3:
                                    std::cout << "LabSat reference clock: external 16.386 MHz\n";
                                    break;
                                default:
                                    std::cout << "LabSat Unknown reference clock ID " << static_cast<int>(d_ref_clock) << '\n';
                                }
                            byte_counter++;
                            d_bits_per_sample = static_cast<uint8_t>(memblock[byte_counter]);
                            switch (d_bits_per_sample)
                                {
                                case 2:
                                    std::cout << "LabSat is using 2 bits per sample\n";
                                    break;
                                case 4:
                                    std::cout << "LabSat is using 4 bits per sample\n";
                                    break;
                                case 6:
                                    if (d_labsat_version != 3)
                                        {
                                            std::cout << "LabSat 2 does not support 6 bits per sample\n";
                                            return -1;
                                        }
                                    std::cout << "LabSat is using 6 bits per sample\n";
                                    break;
                                default:
                                    std::cout << "LabSat Unknown bits per sample ID " << static_cast<int>(d_bits_per_sample) << '\n';
                                    return -1;
                                }

                            byte_counter++;
                            d_channel_selector = static_cast<uint8_t>(memblock[byte_counter]);
                            if (d_bits_per_sample == 6)
                                {
                                    std::cout << "Available channels: Channel A + B + C, 1 bit quantisation (I & Q)\n";
                                }
                            else
                                {
                                    switch (d_channel_selector)
                                        {
                                        case 0:
                                            std::cout << "Available channels: Channel A + B, 1 bit quantisation (I & Q)\n";
                                            break;
                                        case 1:
                                            std::cout << "Available channels: Channel A, 1 bit quantisation (I & Q)\n";
                                            break;
                                        case 2:
                                            std::cout << "Available channels: Channel B, 1 bit quantisation (I & Q)\n";
                                            break;
                                        case 3:
                                            std::cout << "Available channels: Channel A, 2 bit quantisation (I & Q)\n";
                                            break;
                                        case 4:
                                            std::cout << "Available channels: Channel B, 2 bit quantisation (I & Q)\n";
                                            break;
                                        default:
                                            std::cout << "Unknown channel selection ID " << static_cast<int>(d_channel_selector) << '\n';
                                            return -1;
                                        }

                                    for (const auto selected_channel : d_channel_selector_config)
                                        {
                                            if (selected_channel == 3)
                                                {
                                                    std::cout << "LabSat source channel config inconsistency: channel C is selected but the file has only channels A and B.\n";
                                                    return -1;
                                                }
                                            if (d_channel_selector != 0 and selected_channel == 1 and d_channel_selector != 1 and d_channel_selector != 3)
                                                {
                                                    std::cout << "LabSat source channel config inconsistency: channel A is selected but the file has only channel B.\n";
                                                    return -1;
                                                }
                                            if (d_channel_selector != 0 and selected_channel == 2 and d_channel_selector != 2 and d_channel_selector != 4)
                                                {
                                                    std::cout << "LabSat source channel config inconsistency: channel B is selected but the file has only channel A.\n";
                                                    return -1;
                                                }
                                        }
                                }
                            byte_counter++;
                            auto quantization = static_cast<uint8_t>(memblock[byte_counter]);
                            switch (quantization)
                                {
                                case 0:
                                    break;
                                case 1:
                                    std::cout << "1 bit per sample\n";
                                    break;
                                case 2:
                                    std::cout << "2 bit per sample\n";
                                    break;
                                default:
                                    std::cout << "Unknown quantization ID " << static_cast<int>(quantization) << '\n';
                                }
                            byte_counter++;
                            auto channel_a_constellation = static_cast<uint8_t>(memblock[byte_counter]);
                            switch (channel_a_constellation)
                                {
                                case 0:
                                    std::cout << "LabSat Channel A is GPS\n";
                                    break;
                                case 1:
                                    std::cout << "LabSat Channel A is GLONASS\n";
                                    break;
                                case 2:
                                    std::cout << "LabSat Channel A is BDS\n";
                                    break;
                                default:
                                    std::cout << "Unknown channel A constellation ID " << static_cast<int>(channel_a_constellation) << '\n';
                                }
                            byte_counter++;
                            auto channel_b_constellation = static_cast<uint8_t>(memblock[byte_counter]);
                            switch (channel_b_constellation)
                                {
                                case 0:
                                    std::cout << "LabSat Channel B is GPS\n";
                                    break;
                                case 1:
                                    std::cout << "LabSat Channel B is GLONASS\n";
                                    break;
                                case 2:
                                    std::cout << "LabSat Channel B is BDS\n";
                                    break;
                                case 255:
                                    // No channel B
                                    break;
                                default:
                                    std::cout << "Unknown channel B constellation ID " << static_cast<int>(channel_b_constellation) << '\n';
                                }
                            if (d_bits_per_sample == 6)
                                {
                                    if (section_length_bytes < 21)
                                        {
                                            std::cout << "LabSat file header error: section 2 is incomplete for LabSat 3 triple constellation.\n";
                                            return -1;
                                        }
                                    const auto channel_c_constellation = static_cast<uint8_t>(memblock[byte_counter + 15]);
                                    switch (channel_c_constellation)
                                        {
                                        case 0:
                                            std::cout << "LabSat Channel C is GPS\n";
                                            break;
                                        case 1:
                                            std::cout << "LabSat Channel C is GLONASS\n";
                                            break;
                                        case 2:
                                            std::cout << "LabSat Channel C is BDS\n";
                                            break;
                                        case 255:
                                            // No channel C
                                            break;
                                        default:
                                            std::cout << "Unknown channel C constellation ID " << static_cast<int>(channel_c_constellation) << '\n';
                                        }
                                }

                            // end of header
                            d_header_parsed = true;
                            // seek file to the first signal sample
                            binary_input_file.clear();
                            binary_input_file.seekg(header_bytes, binary_input_file.beg);
                            if (d_channel_selector_config.size() > 1 and d_bits_per_sample != 6 and d_channel_selector != 0)
                                {
                                    std::cerr << "Multiple RF source is not implemented for LabSat version " << d_labsat_version << "files.\n";
                                    std::cerr << "The Multiple RF source feature is only available for LabSat 3 Wideband format files.\n";
                                    std::cerr << "Selecting channel";
                                    if (d_channel_selector_config[0] == 1)
                                        {
                                            std::cerr << " A.";
                                        }
                                    if (d_channel_selector_config[0] == 2)
                                        {
                                            std::cerr << " B.";
                                        }
                                    if (d_channel_selector_config[0] == 3)
                                        {
                                            std::cerr << " C.";
                                        }
                                    std::cerr << '\n';
                                }
                            return 0;
                        }

                    byte_counter = static_cast<int>(section_end);
                }
            std::cout << "LabSat file header error: section 2 is not available.\n";
            return -1;
        }
    std::cout << "LabSat file read error: file is empty.\n";
    return -1;
}


void labsat23_source::decode_samples_one_channel(int16_t input_short, gr_complex *out, int type)
{
    std::bitset<16> bs(input_short);
    switch (type)
        {
        case 2:
            // two bits per sample, 8 samples per int16
            for (int i = 0; i < 8; i++)
                {
                    out[i] = gr_complex(static_cast<float>(bs[15 - (2 * i)]),
                        static_cast<float>(bs[14 - (2 * i)]));
                    out[i] = out[i] * gr_complex(2, 0) - gr_complex(1, 1);
                }
            break;
        case 4:
            {
                // four bits per sample, 4 samples per int16
                const auto decode_unsigned_binary_2bit = [](bool high_bit, bool low_bit) -> float {
                    if (high_bit)
                        {
                            return low_bit ? 2.0F : 1.0F;
                        }
                    return low_bit ? -1.0F : -2.0F;
                };

                for (int i = 0; i < 4; i++)
                    {
                        out[i] = gr_complex(
                            decode_unsigned_binary_2bit(bs[15 - 4 * i], bs[13 - 4 * i]),
                            decode_unsigned_binary_2bit(bs[14 - 4 * i], bs[12 - 4 * i]));
                    }
                break;
            }
        default:
            break;
        }
}


void labsat23_source::decode_samples_two_channels(uint16_t input_word, std::vector<gr_complex *> &out, std::size_t output_pointer) const
{
    std::bitset<16> bs(input_word);
    const auto decode_1bit = [](bool value) -> float {
        return value ? 1.0F : -1.0F;
    };

    for (std::size_t output_channel = 0; output_channel < d_channel_selector_config.size(); output_channel++)
        {
            gr_complex *aux = out[output_channel];
            const int channel_offset = (d_channel_selector_config[output_channel] - 1) * 2;
            for (int sample = 0; sample < 4; sample++)
                {
                    const int bit_offset = sample * 4 + channel_offset;
                    aux[output_pointer + sample] = gr_complex(
                        decode_1bit(bs[15 - bit_offset]),
                        decode_1bit(bs[15 - bit_offset - 1]));
                }
        }
}


void labsat23_source::decode_samples_three_channels(uint32_t input_word, std::vector<gr_complex *> &out, std::size_t output_pointer) const
{
    std::bitset<32> bs(input_word);
    const auto decode_1bit = [](bool value) -> float {
        return value ? 1.0F : -1.0F;
    };

    for (std::size_t output_channel = 0; output_channel < d_channel_selector_config.size(); output_channel++)
        {
            gr_complex *aux = out[output_channel];
            const int channel_offset = (d_channel_selector_config[output_channel] - 1) * 2;
            for (int sample = 0; sample < 5; sample++)
                {
                    const int bit_offset = 2 + sample * 6 + channel_offset;
                    aux[output_pointer + sample] = gr_complex(
                        decode_1bit(bs[31 - bit_offset]),
                        decode_1bit(bs[31 - bit_offset - 1]));
                }
        }
}


int labsat23_source::read_ls3w_ini(const std::string &filename)
{
    std::cout << "Reading " << filename << " file ...\n";
    auto ini_reader = std::make_unique<INIReader>(filename);
    int error_ = ini_reader->ParseError();

    if (error_ > 0)
        {
            std::cerr << "Warning: LabSat ini file " << filename
                      << " contains a syntax error in line " << error_ << ", continuing anyway.\n";
        }
    if (error_ < 0)
        {
            std::cerr << "Error: LabSat ini file " << filename << " cannot be opened.\n";
            return 1;
        }
    const std::string empty_string("");
    if (ini_reader->HasSection("config"))
        {
            // Reference clock
            d_ls3w_OSC = ini_reader->Get("config", "OSC", empty_string);
            if (d_ls3w_OSC.empty())
                {
                    std::cerr << "LabSat reference clock: not found.\n";
                }
            else
                {
                    // Sanity check
                    if ((d_ls3w_OSC != "OCXO") and (d_ls3w_OSC != "TCXO") and (d_ls3w_OSC != "EXT"))
                        {
                            std::cerr << "LabSat reference clock is unknown.\n";
                        }
                    else
                        {
                            std::cout << "LabSat reference clock: " << d_ls3w_OSC << '\n';
                        }
                }

            // Sample rate
            std::string ls3w_SMP_aux = ini_reader->Get("config", "SMP", empty_string);
            if (!ls3w_SMP_aux.empty())
                {
                    std::stringstream smp_ss(ls3w_SMP_aux);
                    smp_ss >> d_ls3w_SMP;
                    std::cout << "LabSat sample rate: " << d_ls3w_SMP << " Sps\n";
                }

            // Quantization
            std::string ls3w_QUA_aux = ini_reader->Get("config", "QUA", empty_string);
            if (!ls3w_QUA_aux.empty())
                {
                    std::stringstream qua_ss(ls3w_QUA_aux);
                    qua_ss >> d_ls3w_QUA;

                    // Sanity check
                    if ((d_is_ls3w && d_ls3w_QUA > 3) || (d_is_ls4 && std::unordered_set<int>{1, 2, 4, 8, 12}.count(d_ls3w_QUA) == 0))
                        {
                            std::cerr << "LabSat sample quantization of " << d_ls3w_QUA << " bits is not supported.\n";
                            return -1;
                        }
                    else
                        {
                            std::cout << "LabSat sample quantization: " << d_ls3w_QUA << " bits for I + " << d_ls3w_QUA << " bits for Q.\n";
                        }
                }
            else
                {
                    // Look for LS4 new QUAN_A _B _C fields
                    std::string ls3w_QUA_aux = ini_reader->Get("config", "QUAN_A", empty_string);
                    if (!ls3w_QUA_aux.empty())
                        {
                            std::stringstream qua_ss(ls3w_QUA_aux);
                            qua_ss >> d_ls3w_QUA;

                            // Sanity check
                            if ((d_is_ls3w && d_ls3w_QUA > 3) || (d_is_ls4 && std::unordered_set<int>{1, 2, 4, 8, 12}.count(d_ls3w_QUA) == 0))
                                {
                                    std::cerr << "LabSat sample quantization of " << d_ls3w_QUA << " bits is not supported.\n";
                                    return -1;
                                }
                            else
                                {
                                    std::cout << "LabSat sample quantization: " << d_ls3w_QUA << " bits for I + " << d_ls3w_QUA << " bits for Q.\n";
                                }
                        }
                    else
                        {
                            std::cerr << "LabSat sample quantization QUAN_A field not found.\n";
                            return -1;
                        }
                }

            // Number of RF channels
            std::string ls3w_CHN_aux = ini_reader->Get("config", "CHN", empty_string);
            if (!ls3w_CHN_aux.empty())
                {
                    std::stringstream chn_ss(ls3w_CHN_aux);
                    chn_ss >> d_ls3w_CHN;

                    // Sanity check
                    if (d_ls3w_CHN > 3)
                        {
                            std::cerr << "LabSat files with " << d_ls3w_CHN << " RF channels are not supported.\n";
                            return -1;
                        }
                    else
                        {
                            std::cout << "LabSat data file contains " << d_ls3w_CHN << " RF channels.\n";
                        }
                }

            // Number of bits shifted per channel
            std::string ls3w_SFT_aux = ini_reader->Get("config", "SFT", empty_string);
            if (!ls3w_SFT_aux.empty())
                {
                    std::stringstream sft_ss(ls3w_SFT_aux);
                    sft_ss >> d_ls3w_SFT;

                    // Sanity check
                    if (d_ls3w_SFT != d_ls3w_CHN * d_ls3w_QUA * 2)
                        {
                            std::cerr << "SFT parameter value in the .ini file is not valid.\n";
                            d_ls3w_SFT = d_ls3w_CHN * d_ls3w_QUA * 2;
                        }
                }

            if (d_is_ls4)
                {
                    // Max bandwidth
                    const auto ls4_bw_max = ini_reader->Get("config", "BW_MAX", empty_string);
                    if (!ls4_bw_max.empty())
                        {
                            std::stringstream ls4_bw_max_ss(ls4_bw_max);
                            ls4_bw_max_ss >> d_ls4_BW_MAX;
                            std::cout << "LabSat max bandwidth : " << d_ls4_BW_MAX << " Hz.\n";
                        }

                    for (auto &channel_map_it : d_channel_map)
                        {
                            auto &channel = channel_map_it.second;
                            const auto bw_div_str = ini_reader->Get("config", "BW_DIV_" + channel.identifier, empty_string);

                            if (!bw_div_str.empty())
                                {
                                    std::stringstream bw_div_ss(bw_div_str);
                                    bw_div_ss >> channel.bw_div;
                                    std::cout << "LabSat bandwidth div " << channel.identifier << " : " << channel.bw_div << ".\n";
                                }
                        }
                }
        }

    for (auto &channel_map_it : d_channel_map)
        {
            auto &channel = channel_map_it.second;
            const auto channel_name = "channel " + channel.identifier;

            if (ini_reader->HasSection(channel_name))
                {
                    const auto cf_str = ini_reader->Get(channel_name, "CF" + channel.identifier, empty_string);
                    if (!cf_str.empty())
                        {
                            std::stringstream cf_ss(cf_str);
                            cf_ss >> channel.center_freq;
                            std::cout << "LabSat center frequency for RF " << channel_name << ": " << channel.center_freq << " Hz\n";
                        }

                    const auto bw_str = ini_reader->Get(channel_name, "BW" + channel.identifier, empty_string);
                    if (!bw_str.empty())
                        {
                            std::stringstream bw_ss(bw_str);
                            bw_ss >> channel.bandwidth;
                            std::cout << "LabSat RF filter bandwidth for RF " << channel_name << ": " << channel.bandwidth << " Hz\n";
                        }

                    if (d_is_ls4)
                        {
                            const auto buffer_size_str = ini_reader->Get(channel_name, "BUF_SIZE_" + channel.identifier, empty_string);
                            if (!buffer_size_str.empty())
                                {
                                    std::stringstream buff_size_ss(buffer_size_str);
                                    buff_size_ss >> channel.buff_size;

                                    if (channel.buff_size > 0)
                                        {
                                            if (channel.buff_size % sizeof(uint64_t) != 0)
                                                {
                                                    std::cerr << "\nConfiguration error: RF " << channel_name << " is BUFF SIZE is not a multiple of " << sizeof(uint64_t) << ".\n";
                                                    std::cerr << "Exiting the program.\n";
                                                    return -1;
                                                }

                                            channel.data.resize(channel.buff_size / sizeof(uint64_t));
                                        }
                                    std::cout << "LabSat RF BUFFER SIZE for RF " << channel_name << ": " << channel.buff_size << " bytes\n";
                                }
                        }
                }
        }

    if (d_is_ls4)
        {
            d_number_register_per_output = d_ls3w_QUA == 12 ? 3 : 1;
            d_number_sample_per_output = (d_number_register_per_output * 64) / (d_ls3w_QUA * 2);

            std::vector<int32_t> bandwidths = {d_ls4_BW_MAX};
            std::vector<int32_t> relative_buff_sizes;

            for (const auto &channel_map_it : d_channel_map)
                {
                    const auto &channel = channel_map_it.second;
                    bandwidths.emplace_back(channel.bandwidth * channel.bw_div + d_ls4_BW_MAX % channel.bw_div);
                    relative_buff_sizes.emplace_back(channel.buff_size * channel.bw_div);
                }

            // LS4 bandwidths may be different between channels!
            //     if (!are_equal_ignore_nonpositive(bandwidths))
            //     {
            //         std::cerr << "\nConfiguration error: Bandwidth configuration is invalid.\n";
            //         std::cerr << "Exiting the program.\n";
            //         return -1;
            //     }
            if (!are_equal_ignore_nonpositive(relative_buff_sizes))
                {
                    std::cerr << "\nConfiguration error: Buffer size configuration is invalid.\n";
                    std::cerr << "Exiting the program.\n";
                    return -1;
                }

            std::vector<int32_t> selected_bw_divs;
            selected_bw_divs.reserve(d_channel_selector_config.size());

            for (const auto channel_id : d_channel_selector_config)
                {
                    selected_bw_divs.push_back(d_channel_map.at(channel_id).bw_div);
                }

            if (!are_equal_ignore_nonpositive(selected_bw_divs))
                {
                    if (d_channel_selector_config.size() > 1)
                        {
                            std::cerr << "\nConfiguration error: Selecting multiple channels with different bandwidths is not supported.\n";
                            std::cerr << "Exiting the program.\n";
                            return -1;
                        }
                }

            const auto selected_bw_div = static_cast<double>(selected_bw_divs[0]);

            for (auto &channel_map_it : d_channel_map)
                {
                    auto &channel = channel_map_it.second;

                    if (channel.buff_size > 0)
                        {
                            // This ensures we increment the indices correctly when all channels do not have the same bandwidth
                            channel.number_sample_per_output = d_number_register_per_output * (selected_bw_div / channel.bw_div);
                        }
                }
        }

    const std::set<int> channel_selector_config_set(d_channel_selector_config.begin(), d_channel_selector_config.end());
    const std::vector<int> unique_channel_selector_config(channel_selector_config_set.begin(), channel_selector_config_set.end());
    const auto selected_channel_count = static_cast<int>(unique_channel_selector_config.size());
    std::cout << "LabSat selected channel" << ((selected_channel_count > 1) ? "s" : "") << ": ";

    for (int i = 0; i < selected_channel_count; ++i)
        {
            const auto channel_id = unique_channel_selector_config.at(i);
            const auto &channel_char = d_channel_map.at(channel_id).identifier;

            if (channel_id > d_ls3w_CHN)
                {
                    std::cerr << "\nConfiguration error: RF channel " << channel_char << " is selected but not found in data file.\n";
                    std::cerr << "Exiting the program.\n";
                    return -1;
                }

            std::cout << channel_char << (i + 1 < selected_channel_count ? ", " : "");
        }

    std::cout << '\n';

    d_ls3w_samples_per_register = this->number_of_samples_per_ls3w_register();
    d_ls3w_spare_bits = 64 - d_ls3w_samples_per_register * d_ls3w_CHN * d_ls3w_QUA * 2;
    for (auto ch_select : d_channel_selector_config)
        {
            d_ls3w_selected_channel_offset.push_back((ch_select - 1) * d_ls3w_QUA * 2);
        }
    return 0;
}


int labsat23_source::number_of_samples_per_ls3w_register() const
{
    int number_samples = 0;
    switch (d_ls3w_QUA)
        {
        case 1:
            if (d_ls3w_CHN == 1)
                {
                    if (!d_ls3w_digital_io_enabled)
                        {
                            number_samples = 32;
                        }
                    else
                        {
                            number_samples = 30;
                        }
                }
            if (d_ls3w_CHN == 2)
                {
                    if (!d_ls3w_digital_io_enabled)
                        {
                            number_samples = 16;
                        }
                    else
                        {
                            number_samples = 15;
                        }
                }
            if (d_ls3w_CHN == 3)
                {
                    number_samples = 10;
                }
            break;
        case 2:
            if (d_ls3w_CHN == 1)
                {
                    if (!d_ls3w_digital_io_enabled)
                        {
                            number_samples = 16;
                        }
                    else
                        {
                            number_samples = 15;
                        }
                }
            if (d_ls3w_CHN == 2)
                {
                    if (!d_ls3w_digital_io_enabled)
                        {
                            number_samples = 8;
                        }
                    else
                        {
                            number_samples = 7;
                        }
                }
            if (d_ls3w_CHN == 3)
                {
                    number_samples = 5;
                }
            break;
        case 3:
            if (d_ls3w_CHN == 1)
                {
                    number_samples = 10;
                }
            if (d_ls3w_CHN == 2)
                {
                    number_samples = 5;
                }
            if (d_ls3w_CHN == 3)
                {
                    number_samples = 3;
                }
            break;
        default:
            number_samples = 0;
            break;
        }
    return number_samples;
}


void labsat23_source::decode_ls3w_register(uint64_t input, std::vector<gr_complex *> &out, size_t output_pointer) const
{
    std::bitset<64> bs(input);  // Earlier samples are written in the MSBs of the register

    int output_chan = 0;
    for (const auto channel_offset : d_ls3w_selected_channel_offset)
        {
            gr_complex *aux = out[output_chan];

            for (int i = 0; i < d_ls3w_samples_per_register; i++)
                {
                    const int bit_offset = d_ls3w_spare_bits + i * d_ls3w_SFT + channel_offset;
                    write_samples_from_bitset(bs, bit_offset, d_ls3w_QUA, aux[output_pointer + i]);
                }
            output_chan++;
        }
}

int labsat23_source::parse_ls23_data(int noutput_items, std::vector<gr_complex *> out)
{
    if (d_header_parsed == false)
        {
            return parse_header();
        }

    // ready to start reading samples
    switch (d_bits_per_sample)
        {
        case 2:
            switch (d_channel_selector)
                {
                case 0:
                    std::cout << "LabSat source channel config inconsistency: dual channel files use 4 bits per sample.\n";
                    return -1;
                default:
                    // single channel 2 bits per complex sample (1 bit I + 1 bit Q, 8 samples per int16)
                    int n_int16_to_read = noutput_items / 8;
                    if (n_int16_to_read > 0)
                        {
                            std::vector<int16_t> memblock(n_int16_to_read);
                            binary_input_file.read(reinterpret_cast<char *>(memblock.data()), n_int16_to_read * 2);
                            n_int16_to_read = static_cast<int>(binary_input_file.gcount()) / 2;  // from bytes to int16
                            if (n_int16_to_read > 0)
                                {
                                    int output_pointer = 0;
                                    for (int i = 0; i < n_int16_to_read; i++)
                                        {
                                            decode_samples_one_channel(memblock[i], out[0] + output_pointer, d_bits_per_sample);
                                            output_pointer += 8;
                                        }
                                    return output_pointer;
                                }

                            // trigger the read of the next file in the sequence
                            d_current_file_number++;
                            if (d_labsat_version == 3)
                                {
                                    std::cout << "End of current file, reading the next LabSat file in sequence: " << generate_filename() << '\n';
                                }
                            binary_input_file.close();
                            binary_input_file.open(generate_filename().c_str(), std::ios::in | std::ios::binary);
                            if (binary_input_file.is_open())
                                {
                                    std::cout << "LabSat file source is reading samples from " << generate_filename() << '\n';
                                    return 0;
                                }

                            if (d_labsat_version == 3)
                                {
                                    std::cout << "Last file reached, LabSat source stop\n";
                                }
                            else
                                {
                                    std::cout << "End of file reached, LabSat source stop\n";
                                }

                            d_queue->push(pmt::make_any(command_event_make(200, 0)));
                            return -1;
                        }
                    else
                        {
                            return 0;
                        }
                }
            break;
        case 4:
            switch (d_channel_selector)
                {
                case 0:
                    {
                        // dual channel: 4 bits per sample (A I/Q + B I/Q), 4 samples per int16
                        int n_words_to_read = noutput_items / 4;
                        if (n_words_to_read > 0)
                            {
                                std::vector<char> memblock(n_words_to_read * 2);
                                binary_input_file.read(memblock.data(), n_words_to_read * 2);
                                n_words_to_read = static_cast<int>(binary_input_file.gcount()) / 2;
                                if (n_words_to_read > 0)
                                    {
                                        std::size_t output_pointer = 0;
                                        for (int i = 0; i < n_words_to_read; i++)
                                            {
                                                decode_samples_two_channels(read_le_u16(&memblock[i * 2]), out, output_pointer);
                                                output_pointer += 4;
                                            }
                                        return static_cast<int>(output_pointer);
                                    }

                                // trigger the read of the next file in the sequence
                                d_current_file_number++;
                                if (d_labsat_version == 3)
                                    {
                                        std::cout << "End of current file, reading the next LabSat file in sequence: " << generate_filename() << '\n';
                                    }
                                binary_input_file.close();
                                binary_input_file.open(generate_filename().c_str(), std::ios::in | std::ios::binary);
                                if (binary_input_file.is_open())
                                    {
                                        std::cout << "LabSat file source is reading samples from " << generate_filename() << '\n';
                                        return 0;
                                    }

                                if (d_labsat_version == 3)
                                    {
                                        std::cout << "Last file reached, LabSat source stop\n";
                                    }
                                else
                                    {
                                        std::cout << "End of file reached, LabSat source stop\n";
                                    }

                                d_queue->push(pmt::make_any(command_event_make(200, 0)));
                                return -1;
                            }
                        else
                            {
                                return 0;
                            }
                    }
                default:
                    // single channel 4 bits per complex sample (2 bit I + 2 bit Q, 4 samples per int16)
                    int n_int16_to_read = noutput_items / 4;
                    if (n_int16_to_read > 0)
                        {
                            std::vector<int16_t> memblock(n_int16_to_read);
                            binary_input_file.read(reinterpret_cast<char *>(memblock.data()), n_int16_to_read * 2);
                            n_int16_to_read = static_cast<int>(binary_input_file.gcount()) / 2;  // from bytes to int16
                            if (n_int16_to_read > 0)
                                {
                                    int output_pointer = 0;
                                    for (int i = 0; i < n_int16_to_read; i++)
                                        {
                                            decode_samples_one_channel(memblock[i], out[0] + output_pointer, d_bits_per_sample);
                                            output_pointer += 4;
                                        }
                                    return output_pointer;
                                }

                            // trigger the read of the next file in the sequence
                            d_current_file_number++;
                            if (d_labsat_version == 3)
                                {
                                    std::cout << "End of current file, reading the next Labsat file in sequence: " << generate_filename() << '\n';
                                }
                            binary_input_file.close();
                            binary_input_file.open(generate_filename().c_str(), std::ios::in | std::ios::binary);
                            if (binary_input_file.is_open())
                                {
                                    std::cout << "Labsat file source is reading samples from " << generate_filename() << '\n';
                                    return 0;
                                }

                            if (d_labsat_version == 3)
                                {
                                    std::cout << "Last file reached, LabSat source stop\n";
                                }
                            else
                                {
                                    std::cout << "End of file reached, LabSat source stop\n";
                                }
                            d_queue->push(pmt::make_any(command_event_make(200, 0)));
                            return -1;
                        }
                    else
                        {
                            return 0;
                        }
                }
            break;
        case 6:
            // LabSat 3 triple constellation: 3 channels, 1 bit I + 1 bit Q each,
            // packed as 5 samples in each 32-bit little-endian word after two MSB padding bits.
            if (d_labsat_version != 3)
                {
                    return -1;
                }
            else
                {
                    int n_words_to_read = noutput_items / 5;
                    if (n_words_to_read > 0)
                        {
                            std::vector<char> memblock(n_words_to_read * 4);
                            binary_input_file.read(memblock.data(), n_words_to_read * 4);
                            n_words_to_read = static_cast<int>(binary_input_file.gcount()) / 4;
                            if (n_words_to_read > 0)
                                {
                                    std::size_t output_pointer = 0;
                                    for (int i = 0; i < n_words_to_read; i++)
                                        {
                                            decode_samples_three_channels(read_le_u32(&memblock[i * 4]), out, output_pointer);
                                            output_pointer += 5;
                                        }
                                    return static_cast<int>(output_pointer);
                                }

                            d_current_file_number++;
                            std::cout << "End of current file, reading the next LabSat file in sequence: " << generate_filename() << '\n';
                            binary_input_file.close();
                            binary_input_file.open(generate_filename().c_str(), std::ios::in | std::ios::binary);
                            if (binary_input_file.is_open())
                                {
                                    std::cout << "LabSat file source is reading samples from " << generate_filename() << '\n';
                                    return 0;
                                }

                            std::cout << "Last file reached, LabSat source stop\n";
                            d_queue->push(pmt::make_any(command_event_make(200, 0)));
                            return -1;
                        }
                    else
                        {
                            return 0;
                        }
                }
        default:
            return -1;
        }

    return -1;
}

int labsat23_source::parse_ls3w_data(int noutput_items, std::vector<gr_complex *> out)
{
    if (binary_input_file.eof() == false)
        {
            // Integer division, any fractional part of the answer is discarded
            int registers_to_read = noutput_items / d_ls3w_samples_per_register;
            if (registers_to_read < 1)
                {
                    return 0;
                }
            std::size_t output_pointer = 0;
            for (int i = 0; i < registers_to_read; i++)
                {
                    uint64_t read_register = 0ULL;
                    // Labsat3W writes its 64-bit shift register to files in little endian. Read and convert to host endianness.
                    read_file_register_to_local_endian(binary_input_file, read_register);

                    if (binary_input_file.gcount() == 8)
                        {
                            decode_ls3w_register(read_register, out, output_pointer);
                            output_pointer += d_ls3w_samples_per_register;
                        }
                    else
                        {
                            std::cout << "End of file reached, LabSat source stop.\n";
                            d_queue->push(pmt::make_any(command_event_make(200, 0)));
                            return -1;
                        }
                }
            return output_pointer;
        }
    else
        {
            std::cout << "End of file reached, LabSat source stop.\n";
            d_queue->push(pmt::make_any(command_event_make(200, 0)));
            return -1;
        }
}

int labsat23_source::parse_ls4_data(int noutput_items, std::vector<gr_complex *> out)
{
    if (binary_input_file.eof() == false)
        {
            int output_index = 0;
            const int registers_to_read = noutput_items / d_number_sample_per_output;

            for (int j = 0; j < registers_to_read; j++)
                {
                    uint64_t total_data_index = 0;

                    for (const auto &channel_map_it : d_channel_map)
                        {
                            const auto &channel = channel_map_it.second;
                            total_data_index += channel.data_index;
                        }

                    if (total_data_index >= d_read_index)
                        {
                            if (!read_ls4_data())
                                {
                                    std::cout << "End of file reached, LabSat source stop.\n";
                                    d_queue->push(pmt::make_any(command_event_make(200, 0)));
                                    return -1;
                                }
                        }

                    for (size_t channel_index = 0; channel_index < d_channel_selector_config.size(); ++channel_index)
                        {
                            gr_complex *aux = out[channel_index];
                            auto &channel = d_channel_map.at(d_channel_selector_config[channel_index]);
                            write_samples_ls4(channel.data_index, output_index, d_number_sample_per_output, d_ls3w_QUA, channel.data, aux);
                        }

                    output_index += d_number_sample_per_output;

                    for (auto &channel_map_it : d_channel_map)
                        {
                            auto &channel = channel_map_it.second;
                            if (channel.buff_size > 0)
                                {
                                    channel.data_index += channel.number_sample_per_output;
                                }
                        }
                }

            return output_index;
        }
    else
        {
            std::cout << "End of file reached, LabSat source stop.\n";
            d_queue->push(pmt::make_any(command_event_make(200, 0)));
            return -1;
        }
}


bool labsat23_source::read_ls4_data()
{
    for (auto &channel_map_it : d_channel_map)
        {
            auto &channel = channel_map_it.second;

            if (channel.buff_size > 0)
                {
                    binary_input_file.read(reinterpret_cast<char *>(channel.data.data()), channel.buff_size);
                    d_read_index += channel.data.size();
                    if (binary_input_file.gcount() != channel.buff_size)
                        {
                            return false;
                        }
                }
        }

    return true;
}


int labsat23_source::general_work(int noutput_items,
    __attribute__((unused)) gr_vector_int &ninput_items,
    __attribute__((unused)) gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    std::vector<gr_complex *> out;
    for (auto &output_item : output_items)
        {
            out.push_back(reinterpret_cast<gr_complex *>(output_item));
        }

    if (!d_is_ls3w && !d_is_ls4)
        {
            return parse_ls23_data(noutput_items, std::move(out));
        }
    else if (d_is_ls3w)  // Labsat 3 Wideband
        {
            return parse_ls3w_data(noutput_items, std::move(out));
        }
    else  // Labsat 4
        {
            return parse_ls4_data(noutput_items, std::move(out));
        }
}
