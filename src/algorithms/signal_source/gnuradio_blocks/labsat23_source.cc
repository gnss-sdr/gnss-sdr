/*!
 * \file labsat23_source.cc
 *
 * \brief Unpacks the Labsat 2 (ls2) and (ls3) capture files
 * \author Javier Arribas jarribas (at) cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "labsat23_source.h"
#include <gnuradio/io_signature.h>
#include <sstream>


labsat23_source_sptr labsat23_make_source(const char *signal_file_basename, int channel_selector)
{
    return labsat23_source_sptr(new labsat23_source(signal_file_basename, channel_selector));
}


std::string labsat23_source::generate_filename()
{
    std::ostringstream ss;
    ss << std::setw(4) << std::setfill('0') << d_current_file_number;
    return d_signal_file_basename + "_" + ss.str()+".LS3";
}


labsat23_source::labsat23_source(const char *signal_file_basename, int channel_selector) : gr::block("labsat23_source",
        gr::io_signature::make(0, 0, 0),
        gr::io_signature::make(1, 1, sizeof(gr_complex)))
{
    if (channel_selector < 1 or channel_selector > 2)
        {
            std::cout << "Labsat source config error: channel selection out of bounds, check gnss-sdr config file" << std::endl;
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
    binary_input_file = new std::ifstream (signal_file.c_str(), std::ios::in|std::ios::binary);

    if (binary_input_file->is_open())
        {
            std::cout << "Labsat file source is reading samples from " << signal_file << std::endl;
        }
    else
        {
            std::cout << "Labsat file " << signal_file << " could not be opened!" << std::endl;
            delete binary_input_file;
            exit(1);
        }
}


labsat23_source::~labsat23_source()
{
    if (binary_input_file->is_open())
        {
            binary_input_file->close();
        }
    delete binary_input_file;
}


int labsat23_source::getBit(uint8_t byte, int position)
{
    return (byte >> position) & 0x01;
}


void labsat23_source::decode_samples_one_channel(int16_t input_short, gr_complex* out, int type)
{
    std::bitset<16> bs(input_short);
    switch(type)
    {
    case 2:
        //two bits per sample, 8 samples per int16
        for (int i = 0; i < 8; i++)
            {
                out[i] = gr_complex(static_cast<float>(bs[15-(2*i)]),
                        static_cast<float>(bs[14-(2*i)]));
                out[i] = out[i]*gr_complex(2,0)-gr_complex(1,1);
            }
        break;
    case 4:
        //four bits per sample, 4 samples per int16
        for (int i = 0; i < 4; i++)
            {
                out[i] = gr_complex(0.0,0.0);
                //In-Phase
                if (bs[15-4*i])
                    {
                        if (bs[13-4*i]) //11
                            {
                                out[i] += gr_complex(-1,0);
                            }
                        else //10
                            {
                                out[i] += gr_complex(-2,0);
                            }
                    }
                else
                    {
                        if (bs[13-4*i]) //01
                            {
                                out[i] += gr_complex(1,0);
                            }
                    }

                //Quadrature
                if (bs[14-4*i])
                    {
                        if (bs[12-4*i]) //11
                            {
                                out[i] += gr_complex(0,-1);
                            }
                        else //10
                            {
                                out[i] += gr_complex(0,-2);
                            }
                    }
                else
                    {
                        if (bs[12-4*i]) //01
                            {
                                out[i] += gr_complex(0,1);
                            }
                    }
                out[i] += gr_complex(0.5,0.5);
            }
        break;
    }
}


int labsat23_source::general_work(int noutput_items,
        __attribute__((unused)) gr_vector_int &ninput_items,
        __attribute__((unused)) gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    gr_complex *out = reinterpret_cast<gr_complex *>(output_items[0]);

    if (d_header_parsed == false)
        {
            if (binary_input_file->eof() == false)
                {
                    char memblock[1024];
                    binary_input_file->read(memblock, 1024);
                    //parse Labsat header
                    //check preamble
                    int byte_counter = 0;
                    bool preamble_ok = true;
                    for (int i = 0; i < 8; i++)
                        {
                            if (memblock[byte_counter] != 0x00) preamble_ok = false;
                            //std::cout << "H[" << i << "]:" << (int)memblock[byte_counter] << std::endl;
                            byte_counter++;
                        }

                    if (preamble_ok == false)
                        {
                            std::cout << "Labsat source do not detect the preamble in the selected file" << std::endl;
                            return -1;
                        }

                    // check Labsat version
                    if (memblock[byte_counter] == 0x4C and memblock[byte_counter+1] == 0x53 and memblock[byte_counter+2] == 0x32)
                        {
                            d_labsat_version = 2;
                            std::cout << "Labsat file version 2 detected" << std::endl;
                        }

                    if (memblock[byte_counter] == 0x4C and memblock[byte_counter+1] == 0x53 and memblock[byte_counter+2] == 0x33)
                        {
                            d_labsat_version = 3;
                            std::cout << "Labsat file version 3 detected" << std::endl;
                        }

                    if (d_labsat_version == 0)
                        {
                            std::cout << "Labsat source do not detect the labsat version in file header" << std::endl;
                            return -1;
                        }

                    byte_counter += 3;

                    int sub_version = static_cast<int>(memblock[byte_counter]);

                    std::cout << "Labsat file sub version " << sub_version << std::endl;

                    byte_counter++;

                    int header_bytes = 0;
                    header_bytes += memblock[byte_counter] | (memblock[byte_counter+1] << 8) | (memblock[byte_counter+2] << 16) | (memblock[byte_counter+3] << 24);

                    byte_counter += 4;
                    //std::cout << "header_bytes=" << header_bytes << std::endl;

                    // read first section
                    // section ID (little-endian)
                    uint8_t section_id = static_cast<int>(memblock[byte_counter]) + static_cast<int>(memblock[byte_counter+1]) * 256;
                    //std::cout << "Section ID: " << (int)section_id << std::endl;
                    byte_counter += 2;

                    uint8_t section_lenght_bytes = 0;
                    section_lenght_bytes += memblock[byte_counter] | (memblock[byte_counter+1] << 8) | (memblock[byte_counter+2] << 16) | (memblock[byte_counter+3] << 24);
                    //std::cout << "section_lenght_bytes=" << (int)section_lenght_bytes << std::endl;

                    byte_counter += 4;
                    if (section_id == 2)
                        {
                            d_ref_clock = static_cast<uint8_t>(memblock[byte_counter]);
                            switch(d_ref_clock)
                            {
                            case 0:
                                std::cout << "Labsat reference clock: internal OXCO" << std::endl;
                                break;
                            case 1:
                                std::cout << "Labsat reference clock: internal TXCO" << std::endl;
                                break;
                            case 2:
                                std::cout << "Labsat reference clock: external 10 MHz" << std::endl;
                                break;
                            case 3:
                                std::cout << "Labsat reference clock: external 16.386 MHz" << std::endl;
                                break;
                            default:
                                std::cout << "Labsat Unknown reference clock ID " << static_cast<int>(d_ref_clock) << std::endl;
                            }
                            byte_counter++;
                            d_bits_per_sample = static_cast<uint8_t>(memblock[byte_counter]);
                            switch(d_bits_per_sample)
                            {
                            case 2:
                                std::cout << "Labsat is using 2 bits per sample" << std::endl;
                                break;
                            case 4:
                                std::cout << "Labsat is using 4 bits per sample" << std::endl;
                                break;
                            default:
                                std::cout << "Labsat Unknown bits per sample ID " << static_cast<int>(d_bits_per_sample) << std::endl;
                                return -1;
                            }

                            byte_counter++;
                            d_channel_selector = static_cast<uint8_t>(memblock[byte_counter]);
                            switch(d_channel_selector)
                            {
                            case 0:
                                std::cout << "Available channels: Channel A + B, 1 bit quantisation" << std::endl;
                                break;
                            case 1:
                                std::cout << "Available channels: Channel A, 1 bit quantisation" << std::endl;
                                break;
                            case 2:
                                std::cout << "Available channels: Channel B, 1 bit quantisation" << std::endl;
                                break;
                            case 3:
                                std::cout << "Available channels: Channel A, 2 bit quantisation" << std::endl;
                                break;
                            case 4:
                                std::cout << "Available channels: Channel B, 2 bit quantisation" << std::endl;
                                break;
                            default:
                                std::cout << "Unknown channel selection ID " << static_cast<int>(d_channel_selector) << std::endl;
                                return -1;
                            }

                            //check if the selected channel in config file match the file encoding
                            if (d_channel_selector_config == 2 and d_channel_selector != 0)
                                {
                                    std::cout << "Labsat source channel config inconsistency: channel 2 is selected but the file has only one channel" << std::endl;
                                    return -1;
                                }

                            //todo: Add support for dual channel files
                            if (d_channel_selector == 0)
                                {
                                    std::cout << "ERROR: Labsat file contains more than one channel and it is not currently supported by Labsat signal source." << std::endl;
                                    return -1;
                                }
                            byte_counter++;
                            uint8_t quantization = static_cast<uint8_t>(memblock[byte_counter]);
                            switch(quantization)
                            {
                            case 1:
                                std::cout << "1 bit per sample" << std::endl;
                                break;
                            case 2:
                                std::cout << "2 bit per sample" << std::endl;
                                break;
                            default:
                                std::cout << "Unknown quantization ID " << static_cast<int>(quantization) << std::endl;
                            }
                            byte_counter++;
                            uint8_t channel_a_constellation = static_cast<uint8_t>(memblock[byte_counter]);
                            switch(channel_a_constellation)
                            {
                            case 0:
                                std::cout << "Labsat Channel A is GPS" << std::endl;
                                break;
                            case 1:
                                std::cout << "Labsat Channel A is GLONASS" << std::endl;
                                break;
                            case 2:
                                std::cout << "Labsat Channel A is BDS" << std::endl;
                                break;
                            default:
                                std::cout << "Unknown channel A constellation ID " << static_cast<int>(channel_a_constellation) << std::endl;
                            }
                            byte_counter++;
                            uint8_t channel_b_constellation = static_cast<uint8_t>(memblock[byte_counter]);
                            switch(channel_b_constellation)
                            {
                            case 0:
                                std::cout << "Labsat Channel B is GPS" << std::endl;
                                break;
                            case 1:
                                std::cout << "Labsat Channel B is GLONASS" << std::endl;
                                break;
                            case 2:
                                std::cout << "Labsat Channel B is BDS" << std::endl;
                                break;
                            default:
                                std::cout << "Unknown channel B constellation ID " << static_cast<int>(channel_b_constellation) << std::endl;
                            }

                            //end of header
                            d_header_parsed = true;
                            //seek file to the first signal sample
                            binary_input_file->clear();
                            binary_input_file->seekg(header_bytes, binary_input_file->beg);
                            return 0;
                        }
                    else
                        {
                            std::cout << "Labsat file header error: section 2 is not available." << std::endl;
                            return -1;
                        }
                }
            else
                {
                    std::cout << "Labsat file read error: file is empty." << std::endl;
                    return -1;
                }
        }
    else
        {
            //ready to start reading samples
            switch(d_bits_per_sample)
            {
            case 2:
                {
                    switch(d_channel_selector)
                    {
                    case 0:
                        // dual channel 2 bits per complex sample
                        break;
                    default:
                        //single channel 2 bits per complex sample (1 bit I + 1 bit Q, 8 samples per int16)
                        int n_int16_to_read = noutput_items / 8;
                        if (n_int16_to_read > 0)
                            {
                                int16_t memblock[n_int16_to_read];
                                binary_input_file->read(reinterpret_cast<char*>(memblock), n_int16_to_read * 2);
                                n_int16_to_read = binary_input_file->gcount() / 2; //from bytes to int16
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
                                else
                                    {
                                        //trigger the read of the next file in the sequence
                                        std::cout << "End of current file, reading the next Labsat file in sequence: " << generate_filename() << std::endl;

                                        d_current_file_number++;
                                        binary_input_file->close();
                                        binary_input_file->open(generate_filename().c_str(), std::ios::in|std::ios::binary);
                                        if (binary_input_file->is_open())
                                            {
                                                std::cout << "Labsat file source is reading samples from " << generate_filename() << std::endl;
                                            }
                                        else
                                            {
                                                std::cout << "Last file reached, LabSat source stop" << std::endl;
                                                return -1;
                                            }
                                    }
                            }
                        else
                            {
                                return 0;
                            }
                    };
                    break;
                }
            case 4:
                {
                    switch(d_channel_selector)
                    {
                    case 0:
                        // dual channel
                        break;
                    default:
                        //single channel 4 bits per complex sample (2 bit I + 2 bit Q, 4 samples per int16)
                        int n_int16_to_read = noutput_items / 4;
                        if (n_int16_to_read > 0)
                            {
                                int16_t memblock[n_int16_to_read];
                                binary_input_file->read(reinterpret_cast<char*>(memblock), n_int16_to_read * 2);
                                n_int16_to_read = binary_input_file->gcount() / 2; //from bytes to int16
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
                                else
                                    {
                                        //trigger the read of the next file in the sequence
                                        std::cout << "End of current file, reading the next Labsat file in sequence: " << generate_filename() << std::endl;

                                        d_current_file_number++;
                                        binary_input_file->close();
                                        binary_input_file->open(generate_filename().c_str(), std::ios::in|std::ios::binary);
                                        if (binary_input_file->is_open())
                                            {
                                                std::cout << "Labsat file source is reading samples from " << generate_filename() << std::endl;
                                            }
                                        else
                                            {
                                                std::cout << "Last file reached, LabSat source stop" << std::endl;
                                                return -1;
                                            }
                                    }
                            }
                        else
                            {
                                return 0;
                            }
                    }
                    break;
                }
            default:
                {
                    return -1;
                }
            }
        }
    std::cout << "Warning!!" << std::endl;
    return 0;
}
