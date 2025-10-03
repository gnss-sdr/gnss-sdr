/*!
 * \file labsat23_source.cc
 *
 * \brief Unpacks capture files in the LabSat 2 (ls2), LabSat 3 (ls3), or LabSat
 * 3 Wideband (LS3W) formats.
 * \author Javier Arribas jarribas (at) cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
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


void write_iq_from_bitset(const std::bitset<64> &bs, int bit_offset, int qua, gr_complex &out)
{
    const auto extract_bits = [&](int start, int count) {
        unsigned val = 0;
        for (int i = 0; i < count; ++i)
            {
                val = (val << 1) | bs[start + i];
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


void invert_bitset(std::bitset<64> &bs)
{
    for (size_t i = 0; i < 32; ++i)
        {
            const auto t = bs[i];
            bs[i] = bs[64 - i - 1];
            bs[64 - i - 1] = t;
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

bool are_equal_ignore_nonpositive(std::initializer_list<int32_t> values)
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
                    this->set_output_multiple(16);
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
                    const auto size = fs::file_size(d_signal_file_basename);
                    const auto samples = (size * CHAR_BIT) / d_ls3w_SFT;
                    const auto samples_to_skip = static_cast<size_t>(seconds_to_skip * d_ls3w_SMP);
                    const auto samples_to_read = static_cast<int64_t>(samples - samples_to_skip);

                    std::cout << ", which contains " << samples << " samples (" << size << " bytes)\n";

                    if (samples_to_read < 0)
                        {
                            std::cout << "File duration is smaller then the seconds to skip!\n";
                            exit(1);
                        }

                    const auto signal_duration_s = static_cast<double>(samples_to_read) / d_ls3w_SMP;

                    std::cout << "GNSS signal recorded time to be processed: " << signal_duration_s << " [s]\n";

                    if (samples_to_skip > 0)
                        {
                            LOG(INFO) << "Skipping " << samples_to_skip << " samples of the input file";

                            // We assume all buffers have same size and take advantage of integer rounding
                            const auto bytes_to_skip = (samples_to_skip * d_ls3w_SFT) / CHAR_BIT;
                            const auto bytes_to_skip_per_channel = bytes_to_skip / d_ls3w_CHN;
                            const auto nb_of_channel_buffer_to_skip = bytes_to_skip_per_channel / d_ls4_BUFF_SIZE;
                            const auto bytes_to_seek_per_channel = nb_of_channel_buffer_to_skip * d_ls4_BUFF_SIZE;
                            const auto bytes_to_seek = bytes_to_seek_per_channel * d_ls3w_CHN;
                            const auto bytes_to_read_per_channel = bytes_to_skip_per_channel - bytes_to_seek_per_channel;

                            // Advance in the file by a multiple of buffers, then read one buffer of each channel
                            if (!binary_input_file.seekg(bytes_to_seek, std::ios::beg) || !read_ls4_data())
                                {
                                    LOG(ERROR) << "Error skipping bytes!";
                                    exit(1);
                                }

                            // Advances the indices to start at the correct index in the buffer we just read
                            if (d_ls4_BUFF_SIZE_A > 0)
                                {
                                    d_data_index_a += (bytes_to_read_per_channel / sizeof(uint64_t));
                                }
                            if (d_ls4_BUFF_SIZE_B > 0)
                                {
                                    d_data_index_b += (bytes_to_read_per_channel / sizeof(uint64_t));
                                }
                            if (d_ls4_BUFF_SIZE_C > 0)
                                {
                                    d_data_index_c += (bytes_to_read_per_channel / sizeof(uint64_t));
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
            std::array<char, 1024> memblock{};
            binary_input_file.read(memblock.data(), 1024);
            // parse Labsat header
            // check preamble
            int byte_counter = 0;
            bool preamble_ok = true;
            for (int i = 0; i < 8; i++)
                {
                    if (memblock[byte_counter] != 0x00)
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
            if (memblock[byte_counter] == 0x4C and memblock[byte_counter + 1] == 0x53 and memblock[byte_counter + 2] == 0x32)
                {
                    d_labsat_version = 2;
                    std::cout << "LabSat file version 2 detected\n";
                }

            if (memblock[byte_counter] == 0x4C and memblock[byte_counter + 1] == 0x53 and memblock[byte_counter + 2] == 0x33)
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

            int sub_version = static_cast<int>(memblock[byte_counter]);

            std::cout << "LabSat file sub version " << sub_version << '\n';

            byte_counter++;

            int header_bytes = 0;
            header_bytes += memblock[byte_counter] | (memblock[byte_counter + 1] << 8) | (memblock[byte_counter + 2] << 16) | (memblock[byte_counter + 3] << 24);

            byte_counter += 4;

            // read first section
            // section ID (little-endian)
            uint8_t section_id = static_cast<int>(memblock[byte_counter]) + static_cast<int>(memblock[byte_counter + 1]) * 256;
            byte_counter += 2;

            // uint8_t section_lenght_bytes = 0;
            // section_lenght_bytes += memblock[byte_counter] | (memblock[byte_counter + 1] << 8) | (memblock[byte_counter + 2] << 16) | (memblock[byte_counter + 3] << 24);

            byte_counter += 4;
            if (section_id == 2)
                {
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
                        default:
                            std::cout << "LabSat Unknown bits per sample ID " << static_cast<int>(d_bits_per_sample) << '\n';
                            return -1;
                        }

                    byte_counter++;
                    d_channel_selector = static_cast<uint8_t>(memblock[byte_counter]);
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

                    // check if the selected channel in config file match the file encoding
                    if (d_channel_selector_config[0] == 2 and d_channel_selector != 0)
                        {
                            std::cout << "LabSat source channel config inconsistency: channel 2 is selected but the file has only one channel.\n";
                            return -1;
                        }

                    // todo: Add support for dual channel files
                    if (d_channel_selector == 0)
                        {
                            std::cout << "ERROR: LabSat file contains more than one channel and this is not currently supported for LabSat version " << d_labsat_version << ".\n";
                            return -1;
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

                    // end of header
                    d_header_parsed = true;
                    // seek file to the first signal sample
                    binary_input_file.clear();
                    binary_input_file.seekg(header_bytes, binary_input_file.beg);
                    if (d_channel_selector_config.size() > 1)
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
            //  bits per sample, 4 samples per int16
            for (int i = 0; i < 4; i++)
                {
                    // out[i] = gr_complex(0.0, 0.0);
                    // In-Phase
                    if (bs[15 - 4 * i])
                        {
                            if (bs[13 - 4 * i])  // 11
                                {
                                    out[i] = gr_complex(-1, 0);
                                }
                            else  // 10
                                {
                                    out[i] = gr_complex(-2, 0);
                                }
                        }
                    else
                        {
                            if (bs[13 - 4 * i])  // 01
                                {
                                    out[i] = gr_complex(2, 0);
                                }
                            else
                                {
                                    out[i] = gr_complex(1, 0);
                                }
                        }

                    // Quadrature
                    if (bs[14 - 4 * i])
                        {
                            if (bs[12 - 4 * i])  // 11
                                {
                                    out[i] += gr_complex(0, -1);
                                }
                            else  // 10
                                {
                                    out[i] += gr_complex(0, -2);
                                }
                        }
                    else
                        {
                            if (bs[12 - 4 * i])  // 01
                                {
                                    out[i] += gr_complex(0, 2);
                                }
                            else
                                {
                                    out[i] += gr_complex(0, 1);
                                }
                        }
                    // out[i] += gr_complex(0.5, 0.5);
                }
            break;
        default:
            break;
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
                    if ((d_is_ls3w && d_ls3w_QUA > 3) || (d_is_ls4 && std::unordered_set<int>{1, 2, 4, 8 /*, 12*/}.count(d_ls3w_QUA) == 0))  // 12 currently not working
                        {
                            std::cerr << "LabSat sample quantization of " << d_ls3w_QUA << " bits is not supported.\n";
                            return -1;
                        }
                    else
                        {
                            std::cout << "LabSat sample quantization: " << d_ls3w_QUA << " bits for I + " << d_ls3w_QUA << " bits for Q.\n";
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
                            std::cout << "LabSat max bandwidth : " << d_ls4_BW_MAX << "Hz.\n";
                        }
                }
        }

    const auto channel_handler = [this, &ini_reader, &empty_string](const std::string &channel_char, int32_t &cf, int32_t &bw, int32_t &buffer_size, std::vector<uint64_t> &buffer) {
        const auto channel_name = "channel " + channel_char;

        if (ini_reader->HasSection(channel_name))
            {
                const auto cf_str = ini_reader->Get(channel_name, "CF" + channel_char, empty_string);
                if (!cf_str.empty())
                    {
                        std::stringstream cf_ss(cf_str);
                        cf_ss >> cf;
                        std::cout << "LabSat center frequency for RF " << channel_name << ": " << cf << " Hz\n";
                    }

                const auto bw_str = ini_reader->Get(channel_name, "BW" + channel_char, empty_string);
                if (!bw_str.empty())
                    {
                        std::stringstream bw_ss(bw_str);
                        bw_ss >> bw;
                        std::cout << "LabSat RF filter bandwidth for RF " << channel_name << ": " << bw << " Hz\n";
                    }

                if (d_is_ls4)
                    {
                        const auto buffer_size_str = ini_reader->Get(channel_name, "BUF_SIZE_" + channel_char, empty_string);
                        if (!buffer_size_str.empty())
                            {
                                std::stringstream buff_size_ss(buffer_size_str);
                                buff_size_ss >> buffer_size;

                                if (buffer_size > 0)
                                    {
                                        d_ls4_BUFF_SIZE = buffer_size;

                                        if (buffer_size % sizeof(uint64_t) != 0)
                                            {
                                                std::cerr << "\nConfiguration error: RF " << channel_name << " is BUFF SIZE is not a multiple of " << sizeof(uint64_t) << ".\n";
                                                std::cerr << "Exiting the program.\n";
                                                return false;
                                            }

                                        buffer.resize(buffer_size / sizeof(uint64_t));
                                    }
                                std::cout << "LabSat RF BUFFER SIZE for RF " << channel_name << ": " << buffer_size << " bytes\n";
                            }
                    }
            }
        return true;
    };

    if (!channel_handler("A", d_ls3w_CFA, d_ls3w_BWA, d_ls4_BUFF_SIZE_A, d_ls4_data_a) ||
        !channel_handler("B", d_ls3w_CFB, d_ls3w_BWB, d_ls4_BUFF_SIZE_B, d_ls4_data_b) ||
        !channel_handler("C", d_ls3w_CFC, d_ls3w_BWC, d_ls4_BUFF_SIZE_C, d_ls4_data_c))
        {
            return -1;
        }

    if (d_is_ls4)
        {
            if (!are_equal_ignore_nonpositive({d_ls4_BW_MAX, d_ls3w_BWA, d_ls3w_BWB, d_ls3w_BWC}))
                {
                    std::cerr << "\nConfiguration error: Currently does not support channels with different bandwidths or different from max bandwidth for LS4 files.\n";
                    std::cerr << "Exiting the program.\n";
                    return -1;
                }
            if (!are_equal_ignore_nonpositive({d_ls4_BUFF_SIZE_A, d_ls4_BUFF_SIZE_B, d_ls4_BUFF_SIZE_C}))
                {
                    std::cerr << "\nConfiguration error: Currently does not support channels with different buffer sizes for LS4 files.\n";
                    std::cerr << "Exiting the program.\n";
                    return -1;
                }
        }

    std::cout << "LabSat selected channel" << ((d_channel_selector_config.size() > 1) ? "s" : "") << ": ";
    if (std::find(d_channel_selector_config.begin(), d_channel_selector_config.end(), 1) != d_channel_selector_config.end())
        {
            std::cout << "A";
        }
    if (std::find(d_channel_selector_config.begin(), d_channel_selector_config.end(), 2) != d_channel_selector_config.end())
        {
            if (d_ls3w_CHN > 1)
                {
                    if (d_channel_selector_config.size() == 1)
                        {
                            std::cout << "B";
                        }
                    else
                        {
                            std::cout << ", B";
                        }
                }
            else
                {
                    std::cerr << "\nConfiguration error: RF channel B is selected but not found in data file.\n";
                    std::cerr << "Exiting the program.\n";
                    return -1;
                }
        }
    if (std::find(d_channel_selector_config.begin(), d_channel_selector_config.end(), 3) != d_channel_selector_config.end())
        {
            if (d_ls3w_CHN > 2)
                {
                    if (d_channel_selector_config.size() == 1)
                        {
                            std::cout << "C";
                        }
                    else
                        {
                            std::cout << ", C";
                        }
                }
            else
                {
                    std::cerr << "\nConfiguration error: RF channel C is selected but not found in data file.\n";
                    std::cerr << "Exiting the program.\n";
                    return -1;
                }
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
    std::bitset<64> bs(input);

    // Earlier samples are written in the MSBs of the register. Bit-reverse the register
    // for easier indexing. Note this bit-reverses individual samples as well for quant > 1 bit
    invert_bitset(bs);

    int output_chan = 0;
    for (const auto channel_offset : d_ls3w_selected_channel_offset)
        {
            gr_complex *aux = out[output_chan];

            for (int i = 0; i < d_ls3w_samples_per_register; i++)
                {
                    const int bit_offset = d_ls3w_spare_bits + i * d_ls3w_SFT + channel_offset;
                    write_iq_from_bitset(bs, bit_offset, d_ls3w_QUA, aux[output_pointer + i]);
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
                    // dual channel 2 bits per complex sample
                    // todo: implement dual channel reader
                    break;
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
                    // dual channel
                    // todo: implement dual channel reader
                    break;
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
            const auto shift = d_ls3w_QUA * 2;
            const auto item_count = 64 / shift;
            int registers_to_read = noutput_items / item_count;

            for (int j = 0; j < registers_to_read; j++)
                {
                    if ((d_data_index_a + d_data_index_b + d_data_index_c) >= d_read_index)
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

                            const auto data_parser = [shift, item_count, output_index](uint64_t data_index, int32_t qua, std::vector<uint64_t> &data, gr_complex *out_samples) {
                                std::bitset<64> bs(data[data_index % data.size()]);
                                invert_bitset(bs);

                                for (int i = 0; i < item_count; ++i)
                                    {
                                        write_iq_from_bitset(bs, i * shift, qua, out_samples[output_index + i]);
                                    }
                            };

                            switch (d_channel_selector_config[channel_index])
                                {
                                case 1:
                                    data_parser(d_data_index_a, d_ls3w_QUA, d_ls4_data_a, aux);
                                    break;
                                case 2:
                                    data_parser(d_data_index_b, d_ls3w_QUA, d_ls4_data_b, aux);
                                    break;
                                case 3:
                                    data_parser(d_data_index_c, d_ls3w_QUA, d_ls4_data_c, aux);
                                    break;
                                }
                        }

                    output_index += item_count;

                    if (d_ls4_BUFF_SIZE_A > 0)
                        {
                            ++d_data_index_a;
                        }
                    if (d_ls4_BUFF_SIZE_B > 0)
                        {
                            ++d_data_index_b;
                        }
                    if (d_ls4_BUFF_SIZE_C > 0)
                        {
                            ++d_data_index_c;
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
    const auto read_file = [this](int size, std::vector<uint64_t> &data) {
        if (size > 0)
            {
                binary_input_file.read(reinterpret_cast<char *>(data.data()), size);
                d_read_index += data.size();
                if (binary_input_file.gcount() != size)
                    {
                        return false;
                    }
            }
        return true;
    };

    return read_file(d_ls4_BUFF_SIZE_A, d_ls4_data_a) &&
           read_file(d_ls4_BUFF_SIZE_B, d_ls4_data_b) &&
           read_file(d_ls4_BUFF_SIZE_C, d_ls4_data_c);
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
            return parse_ls23_data(noutput_items, out);
        }
    else if (d_is_ls3w)  // Labsat 3 Wideband
        {
            return parse_ls3w_data(noutput_items, out);
        }
    else  // Labsat 4
        {
            return parse_ls4_data(noutput_items, out);
        }

    std::cout << "Warning!!\n";
    return 0;
}
