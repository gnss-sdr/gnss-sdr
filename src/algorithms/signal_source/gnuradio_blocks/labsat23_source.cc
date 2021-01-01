/*!
 * \file labsat23_source.cc
 *
 * \brief Unpacks the Labsat 2 (ls2) and (ls3) capture files
 * \author Javier Arribas jarribas (at) cttc.es
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


#include "labsat23_source.h"
#include "command_event.h"
#include <boost/any.hpp>
#include <gnuradio/io_signature.h>
#include <array>
#include <exception>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>


labsat23_source_sptr labsat23_make_source_sptr(const char *signal_file_basename, int channel_selector, Concurrent_Queue<pmt::pmt_t> *queue)
{
    return labsat23_source_sptr(new labsat23_source(signal_file_basename, channel_selector, queue));
}


labsat23_source::labsat23_source(const char *signal_file_basename,
    int channel_selector,
    Concurrent_Queue<pmt::pmt_t> *queue) : gr::block("labsat23_source",
                                               gr::io_signature::make(0, 0, 0),
                                               gr::io_signature::make(1, 1, sizeof(gr_complex))),
                                           d_queue(queue)
{
    if (channel_selector < 1 or channel_selector > 2)
        {
            std::cout << "Labsat source config error: channel selection out of bounds, check gnss-sdr config file\n";
            exit(1);
        }
    d_channel_selector_config = channel_selector;
    d_header_parsed = false;
    d_bits_per_sample = 0;
    d_current_file_number = 0;
    d_labsat_version = 0;
    d_ref_clock = 0;
    d_channel_selector = 0;
    d_signal_file_basename = std::string(signal_file_basename);

    std::string signal_file;
    this->set_output_multiple(8);
    signal_file = generate_filename();
    binary_input_file.open(signal_file.c_str(), std::ios::in | std::ios::binary);

    if (binary_input_file.is_open())
        {
            std::cout << "Labsat file source is reading samples from " << signal_file << '\n';
        }
    else
        {
            std::cout << "Labsat file " << signal_file << " could not be opened!\n";
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
            std::cerr << "Problem closing input file" << '\n';
        }
    catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
}


std::string labsat23_source::generate_filename()
{
    if (d_signal_file_basename.substr(d_signal_file_basename.length() - 4, 4) == ".ls2" or d_signal_file_basename.substr(d_signal_file_basename.length() - 4, 4) == ".LS2")
        {
            if (d_current_file_number == 0)
                {
                    return d_signal_file_basename;
                }
            return std::string("donotexist");  // just to stop processing
        }
    std::ostringstream ss;
    ss << std::setw(4) << std::setfill('0') << d_current_file_number;
    return d_signal_file_basename + "_" + ss.str() + ".LS3";
}


int labsat23_source::getBit(uint8_t byte, int position)
{
    return (byte >> position) & 0x01;
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


int labsat23_source::general_work(int noutput_items,
    __attribute__((unused)) gr_vector_int &ninput_items,
    __attribute__((unused)) gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    auto *out = reinterpret_cast<gr_complex *>(output_items[0]);

    if (d_header_parsed == false)
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
                            std::cout << "Labsat source do not detect the preamble in the selected file\n";
                            return -1;
                        }

                    // check Labsat version
                    if (memblock[byte_counter] == 0x4C and memblock[byte_counter + 1] == 0x53 and memblock[byte_counter + 2] == 0x32)
                        {
                            d_labsat_version = 2;
                            std::cout << "Labsat file version 2 detected\n";
                        }

                    if (memblock[byte_counter] == 0x4C and memblock[byte_counter + 1] == 0x53 and memblock[byte_counter + 2] == 0x33)
                        {
                            d_labsat_version = 3;
                            std::cout << "Labsat file version 3 detected\n";
                        }

                    if (d_labsat_version == 0)
                        {
                            std::cout << "Labsat source do not detect the labsat version in file header\n";
                            return -1;
                        }

                    byte_counter += 3;

                    int sub_version = static_cast<int>(memblock[byte_counter]);

                    std::cout << "Labsat file sub version " << sub_version << '\n';

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
                                    std::cout << "Labsat reference clock: internal OCXO\n";
                                    break;
                                case 1:
                                    std::cout << "Labsat reference clock: internal TCXO\n";
                                    break;
                                case 2:
                                    std::cout << "Labsat reference clock: external 10 MHz\n";
                                    break;
                                case 3:
                                    std::cout << "Labsat reference clock: external 16.386 MHz\n";
                                    break;
                                default:
                                    std::cout << "Labsat Unknown reference clock ID " << static_cast<int>(d_ref_clock) << '\n';
                                }
                            byte_counter++;
                            d_bits_per_sample = static_cast<uint8_t>(memblock[byte_counter]);
                            switch (d_bits_per_sample)
                                {
                                case 2:
                                    std::cout << "Labsat is using 2 bits per sample\n";
                                    break;
                                case 4:
                                    std::cout << "Labsat is using 4 bits per sample\n";
                                    break;
                                default:
                                    std::cout << "Labsat Unknown bits per sample ID " << static_cast<int>(d_bits_per_sample) << '\n';
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
                            if (d_channel_selector_config == 2 and d_channel_selector != 0)
                                {
                                    std::cout << "Labsat source channel config inconsistency: channel 2 is selected but the file has only one channel\n";
                                    return -1;
                                }

                            // todo: Add support for dual channel files
                            if (d_channel_selector == 0)
                                {
                                    std::cout << "ERROR: Labsat file contains more than one channel and it is not currently supported by Labsat signal source.\n";
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
                                    std::cout << "Labsat Channel A is GPS\n";
                                    break;
                                case 1:
                                    std::cout << "Labsat Channel A is GLONASS\n";
                                    break;
                                case 2:
                                    std::cout << "Labsat Channel A is BDS\n";
                                    break;
                                default:
                                    std::cout << "Unknown channel A constellation ID " << static_cast<int>(channel_a_constellation) << '\n';
                                }
                            byte_counter++;
                            auto channel_b_constellation = static_cast<uint8_t>(memblock[byte_counter]);
                            switch (channel_b_constellation)
                                {
                                case 0:
                                    std::cout << "Labsat Channel B is GPS\n";
                                    break;
                                case 1:
                                    std::cout << "Labsat Channel B is GLONASS\n";
                                    break;
                                case 2:
                                    std::cout << "Labsat Channel B is BDS\n";
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
                            return 0;
                        }
                    std::cout << "Labsat file header error: section 2 is not available.\n";
                    return -1;
                }
            std::cout << "Labsat file read error: file is empty.\n";
            return -1;
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
                                            decode_samples_one_channel(memblock[i], &out[output_pointer], d_bits_per_sample);
                                            output_pointer += 8;
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
                                            decode_samples_one_channel(memblock[i], &out[output_pointer], d_bits_per_sample);
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

    std::cout << "Warning!!\n";
    return 0;
}
