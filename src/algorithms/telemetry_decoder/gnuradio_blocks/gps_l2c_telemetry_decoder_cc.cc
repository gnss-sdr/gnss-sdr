/*!
 * \file gps_l2c_telemetry_decoder_cc.cc
 * \brief Implementation of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
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

#include <iostream>
#include <sstream>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <boost/lexical_cast.hpp>
#include "gnss_synchro.h"
#include "gps_l2c_telemetry_decoder_cc.h"

using google::LogMessage;

// logging levels
#define EVENT 2         // logs important events which don't occur every block
#define FLOW 3          // logs the function calls of block processing functions
#define SAMP_SYNC 4     // about 1 log entry per sample -> high output
#define LMORE 5         //



gps_l2c_telemetry_decoder_cc_sptr
gps_l2c_make_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump)
{
    return gps_l2c_telemetry_decoder_cc_sptr(new gps_l2c_telemetry_decoder_cc(satellite, dump));
}



gps_l2c_telemetry_decoder_cc::gps_l2c_telemetry_decoder_cc(
        Gnss_Satellite satellite,
        bool dump) :
                gr::block("gps_l2c_telemetry_decoder_cc",
                gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // Telemetry Bit transition synchronization port out
    this->message_port_register_out(pmt::mp("preamble_timestamp_s"));
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "GPS L2C M TELEMETRY PROCESSING: satellite " << d_satellite;
    d_block_size = GPS_L2_SAMPLES_PER_SYMBOL * GPS_L2_SYMBOLS_PER_BIT * GPS_L2_CNAV_DATA_PAGE_BITS * 2; // two CNAV frames
    d_decimation_output_factor = 0;
    //set_output_multiple (1);
    d_average_count = 0;
    d_flag_invert_buffer_symbols = false;
    d_flag_invert_input_symbols = false;
    d_channel = 0;
    d_flag_valid_word = false;
    d_TOW_at_current_symbol = 0;
    d_TOW_at_Preamble = 0;
    //set_history(d_samples_per_bit*8); // At least a history of 8 bits are needed to correlate with the preamble
}



gps_l2c_telemetry_decoder_cc::~gps_l2c_telemetry_decoder_cc()
{
    d_dump_file.close();
}


void gps_l2c_telemetry_decoder_cc::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
    if (noutput_items != 0)
        {
            unsigned ninputs = ninput_items_required.size ();
            for (unsigned i = 0; i < ninputs; i++)
                ninput_items_required[i] = noutput_items;
        }
    //LOG(INFO) << "forecast(): " << "noutput_items=" << noutput_items << "\tninput_items_required ninput_items_required.size()=" << ninput_items_required.size();
}


void gps_l2c_telemetry_decoder_cc::set_decimation(int decimation)
{
    d_decimation_output_factor = decimation;
}


int gps_l2c_telemetry_decoder_cc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    // get pointers on in- and output gnss-synchro objects
    const Gnss_Synchro *in = (const Gnss_Synchro *)  input_items[0]; // input
    Gnss_Synchro *out = (Gnss_Synchro *) output_items[0];            // output

    bool flag_new_cnav_frame = false;
    int last_frame_preamble_start = 0;
    // copy correlation samples into samples vector
    d_sample_buf.push_back(in[0].Prompt_I);

    consume_each(1); //one by one

    // decode only if enough samples in buffer
    if(d_sample_buf.size() >= d_block_size)
        {
            if (in[0].Flag_valid_symbol_output == false) // check if the tracking is locked
                {
                    LOG(INFO) << "Discarting channel " << d_channel << " tracking not ready!" << std::endl;
                    d_flag_valid_word = false;
                }
            else
                {
                    d_flag_invert_buffer_symbols = d_flag_invert_input_symbols;
                    while (true)
                        {
                            if (d_flag_invert_buffer_symbols == true)
                                {
                                    for (std::vector<double>::iterator symbol_it = d_sample_buf.begin(); symbol_it != d_sample_buf.end(); symbol_it++)
                                        {
                                            *symbol_it = -(*symbol_it);
                                        }
                                    //LOG(INFO)<<"Inverting buffer symbols";
                                }
                            // align symbols in pairs
                            // and obtain the bits by decoding the symbols (viterbi decoder)
                            // they can be already aligned or shifted by one position
                            std::vector<int> bits;

                            d_symbol_aligner_and_decoder.get_bits(d_sample_buf, bits);

                            //std::stringstream ss;
                            //for (std::vector<int>::const_iterator bit_it = bits.begin(); bit_it < bits.end(); ++bit_it)
                            //    {
                            //        ss << *bit_it;
                            //    }
                            //LOG(INFO) << "get_bits=" << ss.str() << std::endl;

                            // search for preambles
                            // and extract the corresponding message candidates
                            std::vector<msg_candiate_int_t> msg_candidates;
                            d_frame_detector.get_frame_candidates(bits, msg_candidates);

                            // verify checksum
                            // and return the valid messages
                            std::vector<msg_candiate_int_t> valid_msgs;
                            d_crc_verifier.get_valid_frames(msg_candidates, valid_msgs);
                            if (valid_msgs.size() == 0)
                                {
                                    if (d_flag_invert_buffer_symbols == d_flag_invert_input_symbols)
                                        {
                                            d_flag_invert_buffer_symbols = not d_flag_invert_buffer_symbols;
                                        }
                                    else
                                        {//already tested the symbol inversion but CRC still fail
                                            LOG(INFO) << "Discarting this buffer, no CNAV frames detected CH " << this->d_channel;
                                            break;
                                        }
                                }
                            else
                                { //at least one frame has good CRC, keep the invert sign for the next frames
                                    d_flag_invert_input_symbols = d_flag_invert_buffer_symbols;
                                    std::vector<int> tmp_msg;
                                    std::string msg;
                                    //todo: now the symbol buffer size is two CNAV frames because the preamble is not detected.
                                    //      Use the first valid frame to realign the bufer symbols with the preamble start and not miss a frame
                                    LOG(INFO) << valid_msgs.size() << " GOOD L2C CNAV FRAME DETECTED! CH " <<this->d_channel;
                                    for (unsigned int i = 0;i < valid_msgs.size(); i++)
                                        {
                                            tmp_msg = valid_msgs.at(i).second;
                                            d_CNAV_Message.decode_page(tmp_msg);
                                            std::cout << "Valid CNAV frame with relative preamble start at " << valid_msgs.at(i).first << std::endl;
                                            flag_new_cnav_frame = true;
                                            d_flag_valid_word = true;
                                            last_frame_preamble_start = valid_msgs.at(i).first;
                                            // 4. Push the new navigation data to the queues
                                            if (d_CNAV_Message.have_new_ephemeris() == true)
                                                {
                                                    // get ephemeris object for this SV
                                                    std::shared_ptr<Gps_CNAV_Ephemeris> tmp_obj= std::make_shared<Gps_CNAV_Ephemeris>(d_CNAV_Message.get_ephemeris());
                                                    std::cout << "New GPS CNAV Ephemeris received for SV " << tmp_obj->i_satellite_PRN << std::endl;
                                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));

                                                }
                                            if (d_CNAV_Message.have_new_iono() == true)
                                                {
                                                    std::shared_ptr<Gps_CNAV_Iono> tmp_obj= std::make_shared<Gps_CNAV_Iono>(d_CNAV_Message.get_iono());
                                                    std::cout << "New GPS CNAV IONO model received for SV " << d_satellite.get_PRN() << std::endl;
                                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                                }
                                        }
                                    break;
                                }
                        }
                }
            // clear all processed samples in the input buffer
            d_sample_buf.clear();
        }

    // UPDATE GNSS SYNCHRO DATA
    Gnss_Synchro current_synchro_data; //structure to save the synchronization information and send the output object to the next block

    //1. Copy the current tracking output
    current_synchro_data = in[0];

    if (d_flag_valid_word == true)
        {
            double Prn_timestamp_at_preamble_ms = 0;
            //2. Add the telemetry decoder information
            if (flag_new_cnav_frame == true)
                {
                    //update TOW at the preamble instant
                    Prn_timestamp_at_preamble_ms = (in[0].Tracking_timestamp_secs * 1000.0) - (d_block_size - last_frame_preamble_start) * GPS_L2_M_PERIOD;
                    d_TOW_at_Preamble = d_CNAV_Message.d_TOW - GPS_L2_CNAV_DATA_PAGE_DURATION_S;
                    d_TOW_at_current_symbol = d_TOW_at_Preamble + (d_block_size - last_frame_preamble_start) * GPS_L2_M_PERIOD;
                    current_synchro_data.d_TOW = d_TOW_at_Preamble;
                    current_synchro_data.d_TOW_at_current_symbol = d_TOW_at_current_symbol;
                    current_synchro_data.d_TOW_hybrid_at_current_symbol = current_synchro_data.d_TOW_at_current_symbol;
                    current_synchro_data.Flag_preamble = false;
                    current_synchro_data.Prn_timestamp_ms = in[0].Tracking_timestamp_secs * 1000.0;
                    current_synchro_data.Prn_timestamp_at_preamble_ms = Prn_timestamp_at_preamble_ms;
                }
            else
                {
                    d_TOW_at_current_symbol = d_TOW_at_Preamble + (d_block_size - last_frame_preamble_start) * GPS_L2_M_PERIOD;
                    current_synchro_data.d_TOW = d_TOW_at_Preamble;
                    current_synchro_data.d_TOW_at_current_symbol = d_TOW_at_current_symbol;
                    current_synchro_data.d_TOW_hybrid_at_current_symbol = current_synchro_data.d_TOW_at_current_symbol;
                    current_synchro_data.Flag_preamble = false;
                    current_synchro_data.Prn_timestamp_ms = in[0].Tracking_timestamp_secs * 1000.0;
                    current_synchro_data.Prn_timestamp_at_preamble_ms = Prn_timestamp_at_preamble_ms;
                }
            current_synchro_data.Flag_valid_word = true;
        }
    else
        {
            current_synchro_data.Flag_valid_word = false;
        }

    d_average_count++;
    if (d_average_count == d_decimation_output_factor)
        {
            d_average_count = 0;
            //3. Make the output (copy the object contents to the GNURadio reserved memory)
            out[0] = current_synchro_data;
            //std::cout<<"GPS L2 TLM output on CH="<<this->d_channel << " SAMPLE STAMP="<<d_sample_counter/d_decimation_output_factor<<std::endl;
            return 1;
        }
    else
        {
            return 0;
        }
}



void gps_l2c_telemetry_decoder_cc::set_satellite(Gnss_Satellite satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "GPS L2C CNAV telemetry decoder in channel " << this->d_channel << " set to satellite " << d_satellite;
}



void gps_l2c_telemetry_decoder_cc::set_channel(int channel)
{
    d_channel = channel;
    LOG(INFO) << "GPS L2C CNAV channel set to " << channel;
}


// ### helper class for symbol alignment and viterbi decoding ###
gps_l2c_telemetry_decoder_cc::symbol_aligner_and_decoder::symbol_aligner_and_decoder()
{
    // convolutional code properties
    d_KK = 7;
    int nn = 2;
    int g_encoder[nn];
    g_encoder[0] = 121; //171o
    g_encoder[1] = 91; //133o

    d_vd1 = new Viterbi_Decoder(g_encoder, d_KK, nn);
    d_vd2 = new Viterbi_Decoder(g_encoder, d_KK, nn);
    d_past_symbol = 0;
}


gps_l2c_telemetry_decoder_cc::symbol_aligner_and_decoder::~symbol_aligner_and_decoder()
{
    delete d_vd1;
    delete d_vd2;
}


void gps_l2c_telemetry_decoder_cc::symbol_aligner_and_decoder::reset()
{
    d_past_symbol = 0;
    d_vd1->reset();
    d_vd2->reset();
}


bool gps_l2c_telemetry_decoder_cc::symbol_aligner_and_decoder::get_bits(const std::vector<double> & symbols, std::vector<int> & bits)
{
    const int traceback_depth = 5 * d_KK;
    int nbits_requested = symbols.size() / GPS_L2_SYMBOLS_PER_BIT;
    int nbits_decoded;
    // fill two vectors with the two possible symbol alignments
    std::vector<double> symbols_vd1(symbols); // aligned symbol vector -> copy input symbol vector
    std::vector<double> symbols_vd2;  // shifted symbol vector -> add past sample in front of input vector
    symbols_vd2.push_back(d_past_symbol);
    for (std::vector<double>::const_iterator symbol_it = symbols.begin(); symbol_it != symbols.end() - 1; ++symbol_it)
        {
            symbols_vd2.push_back(*symbol_it);
        }
    // arrays for decoded bits
    int * bits_vd1 = new int[nbits_requested];
    int * bits_vd2 = new int[nbits_requested];
    // decode
    float metric_vd1 = d_vd1->decode_continuous(symbols_vd1.data(), traceback_depth, bits_vd1, nbits_requested, nbits_decoded);
    float metric_vd2 = d_vd2->decode_continuous(symbols_vd2.data(), traceback_depth, bits_vd2, nbits_requested, nbits_decoded);
    //LOG(INFO)<<"metric_vd1="<<metric_vd1<<" metric_vd2="<<metric_vd2;
    // choose the bits with the better metric
    for (int i = 0; i < nbits_decoded; i++)
        {
            if (metric_vd1 > metric_vd2)
                {// symbols aligned
                    bits.push_back(bits_vd1[i]);
                }
            else
                {// symbols shifted
                    bits.push_back(bits_vd2[i]);
                }
        }
    d_past_symbol = symbols.back();
    delete[] bits_vd1;
    delete[] bits_vd2;
    return metric_vd1 > metric_vd2;
}


// ### helper class for detecting the preamble and collect the corresponding message candidates ###
void gps_l2c_telemetry_decoder_cc::frame_detector::reset()
{
    d_buffer.clear();
}


void gps_l2c_telemetry_decoder_cc::frame_detector::get_frame_candidates(const std::vector<int> & bits, std::vector<std::pair<int,std::vector<int>>> & msg_candidates)
{
    //std::stringstream ss;
    unsigned int cnav_msg_length = 300;
    std::vector<std::vector<int>> preambles = {{1, 0, 0, 0, 1, 0, 1 ,1}};
    //LOG(INFO) << "get_frame_candidates(): " << "d_buffer.size()=" << d_buffer.size() << "\tbits.size()=" << bits.size();
    //ss << "copy bits ";
    int count = 0;
    // copy new bits into the working buffer
    for (std::vector<int>::const_iterator bit_it = bits.begin(); bit_it < bits.end(); ++bit_it)
        {
            d_buffer.push_back(*bit_it);
            //ss << *bit_it;
            count++;
        }
    //LOG(INFO) << ss.str() << " into working buffer (" << count << " bits)";
    int relative_preamble_start = 0;
    while(d_buffer.size() >= cnav_msg_length)
        {
            // compare with all preambles
            for (std::vector<std::vector<int>>::iterator preample_it = preambles.begin(); preample_it < preambles.end(); ++preample_it)
                {
                    bool preamble_detected = true;
                    bool inv_preamble_detected = true;
                    // compare the buffer bits with the preamble bits
                    for (std::vector<int>::iterator preample_bit_it = preample_it->begin(); preample_bit_it < preample_it->end(); ++preample_bit_it)
                        {
                            preamble_detected = *preample_bit_it == d_buffer[preample_bit_it - preample_it->begin()] ? preamble_detected : false ;
                            inv_preamble_detected = *preample_bit_it != d_buffer[preample_bit_it - preample_it->begin()] ? inv_preamble_detected : false ;
                        }
                    if (preamble_detected || inv_preamble_detected)
                        {
                            // copy candidate
                            std::vector<int> candidate;
                            std::copy(d_buffer.begin(), d_buffer.begin() + cnav_msg_length, std::back_inserter(candidate));
                            if(inv_preamble_detected)
                                {
                                    // invert bits
                                    for (std::vector<int>::iterator candidate_bit_it = candidate.begin(); candidate_bit_it != candidate.end(); candidate_bit_it++)
                                        *candidate_bit_it = *candidate_bit_it == 0 ? 1 : 0;
                                }
                            msg_candidates.push_back(std::pair<int,std::vector<int>>(relative_preamble_start, candidate));
                            //ss.str("");
                            //ss << "preamble " << preample_it - preambles.begin() << (inv_preamble_detected?" inverted":" normal") << " detected! candidate=";
                            //for (std::vector<int>::iterator bit_it = candidate.begin(); bit_it < candidate.end(); ++bit_it)
                            //    ss << *bit_it;
                            //LOG(INFO) << ss.str();
                        }
                }
            relative_preamble_start++;
            // remove bit in front
            d_buffer.pop_front();
        }
}


// ### helper class for checking the CRC of the message candidates ###

void gps_l2c_telemetry_decoder_cc::crc_verifier::reset()
{

}

void gps_l2c_telemetry_decoder_cc::crc_verifier::get_valid_frames(const std::vector<msg_candiate_int_t> & msg_candidates, std::vector<msg_candiate_int_t> & valid_msgs)
{
    std::vector <unsigned char> tmp_msg;
    LOG(INFO) << "get_valid_frames(): " << "msg_candidates.size()=" << msg_candidates.size();
    // for each candidate
    for (std::vector<msg_candiate_int_t>::const_iterator candidate_it = msg_candidates.begin(); candidate_it < msg_candidates.end(); ++candidate_it)
        {
            // convert to bytes
            std::vector<unsigned char> candidate_bytes;
            zerropad_back_and_convert_to_bytes(candidate_it->second, candidate_bytes);
            // verify CRC
            d_checksum_agent.reset(0);
            d_checksum_agent.process_bytes(candidate_bytes.data(), candidate_bytes.size());
            unsigned int crc = d_checksum_agent.checksum();
            //LOG(INFO) << "candidate " << ": final crc remainder= " << std::hex << crc
            //                << std::setfill(' ') << std::resetiosflags(std::ios::hex);
            //  the final remainder must be zero for a valid message, because the CRC is done over the received CRC value
            if (crc == 0)
                {
                    valid_msgs.push_back(msg_candiate_int_t(candidate_it->first, candidate_it->second));
                    std::cout << "Valid CNAV message found!"<<std::endl;
                }
        }
}


void gps_l2c_telemetry_decoder_cc::crc_verifier::zerropad_back_and_convert_to_bytes(const std::vector<int> & msg_candidate, std::vector<unsigned char> & bytes)
{
    //std::stringstream ss;
    const size_t bits_per_byte = 8;
    unsigned char byte = 0;
    //LOG(INFO) << "zerropad_back_and_convert_to_bytes():" << byte;
    for (std::vector<int>::const_iterator candidate_bit_it = msg_candidate.begin(); candidate_bit_it < msg_candidate.end(); ++candidate_bit_it)
        {
            int idx_bit = candidate_bit_it - msg_candidate.begin();
            int bit_pos_in_current_byte = (bits_per_byte - 1) - (idx_bit % bits_per_byte);
            byte |= (unsigned char)(*candidate_bit_it) << bit_pos_in_current_byte;
            // ss << *candidate_bit_it;
            if (idx_bit % bits_per_byte == bits_per_byte - 1)
                {
                    bytes.push_back(byte);
                    //LOG(INFO) << ss.str() << " -> byte=" << std::setw(2) << std::setfill('0') << std::hex << (unsigned int)byte; ss.str("");
                    byte = 0;
                }
        }
    bytes.push_back(byte); // implies: insert 6 zeros at the end to fit the 250bits into a multiple of bytes
    //LOG(INFO) << " -> byte=" << std::setw(2)
    //            << std::setfill('0') << std::hex << (unsigned int)byte
    //            << std::setfill(' ') << std::resetiosflags(std::ios::hex);
}

void gps_l2c_telemetry_decoder_cc::crc_verifier::zerropad_front_and_convert_to_bytes(const std::vector<int> & msg_candidate, std::vector<unsigned char> & bytes)
{
    //std::stringstream ss;
    const size_t bits_per_byte = 8;
    unsigned char byte = 0;
    int idx_bit = 6; // insert 6 zeros at the front to fit the 250bits into a multiple of bytes
    //LOG(INFO) << "zerropad_front_and_convert_to_bytes():" << byte;
    for (std::vector<int>::const_iterator candidate_bit_it = msg_candidate.begin(); candidate_bit_it < msg_candidate.end(); ++candidate_bit_it)
        {
            int bit_pos_in_current_byte = (bits_per_byte - 1) - (idx_bit % bits_per_byte);
            byte |= (unsigned char)(*candidate_bit_it) << bit_pos_in_current_byte;
            // ss << *candidate_bit_it;
            if (idx_bit % bits_per_byte == bits_per_byte - 1)
                {
                    bytes.push_back(byte);
                    //LOG(INFO) << ss.str() << " -> byte=" << std::setw(2)
                    //            << std::setfill('0') << std::hex << (unsigned int)byte; ss.str("");
                    byte = 0;
                }
            idx_bit++;
        }
    //LOG(INFO) << " -> byte=" << std::setw(2)
    //            << std::setfill('0') << std::hex << (unsigned int)byte
    //            << std::setfill(' ') << std::resetiosflags(std::ios::hex);
}

