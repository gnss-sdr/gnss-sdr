/*!
 * \file sbas_l1_telemetry_decoder_gs.cc
 * \brief Implementation of a SBAS telemetry data decoder block
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "sbas_l1_telemetry_decoder_gs.h"
#include "gnss_synchro.h"
#include "viterbi_decoder.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt_sugar.h>  // for mp
#include <array>
#include <cmath>      // for abs
#include <exception>  // for exception
#include <iomanip>    // for operator<<, setw

// logging levels
#define EVENT 2      // logs important events which don't occur every block
#define FLOW 3       // logs the function calls of block processing functions
#define SAMP_SYNC 4  // about 1 log entry per sample -> high output
#define LMORE 5      //


sbas_l1_telemetry_decoder_gs_sptr sbas_l1_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    bool dump)
{
    return sbas_l1_telemetry_decoder_gs_sptr(new sbas_l1_telemetry_decoder_gs(satellite, dump));
}


sbas_l1_telemetry_decoder_gs::sbas_l1_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    bool dump) : gr::block("sbas_l1_telemetry_decoder_gs",
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    //prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "SBAS L1 TELEMETRY PROCESSING: satellite " << d_satellite;
    d_block_size = D_SAMPLES_PER_SYMBOL * D_SYMBOLS_PER_BIT * D_BLOCK_SIZE_IN_BITS;
    d_channel = 0;
    set_output_multiple(1);
}


sbas_l1_telemetry_decoder_gs::~sbas_l1_telemetry_decoder_gs()
{
    if (d_dump_file.is_open() == true)
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
                }
        }
}


void sbas_l1_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "SBAS telemetry decoder in channel " << this->d_channel << " set to satellite " << d_satellite;
}


void sbas_l1_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    LOG(INFO) << "SBAS channel set to " << channel;
}


// ### helper class for sample alignment ###
sbas_l1_telemetry_decoder_gs::Sample_Aligner::Sample_Aligner()
{
    d_n_smpls_in_history = 3;
    d_iir_par = 0.05;
    reset();
}


void sbas_l1_telemetry_decoder_gs::Sample_Aligner::reset()
{
    d_past_sample = 0;
    d_corr_paired = 0;
    d_corr_shifted = 0;
    d_aligned = true;
}


/*
 * samples length must be a multiple of two
 */
bool sbas_l1_telemetry_decoder_gs::Sample_Aligner::get_symbols(const std::vector<double> &samples, std::vector<double> &symbols)
{
    std::array<double, 3> smpls{};
    double corr_diff;
    bool stand_by = true;
    double sym;

    VLOG(FLOW) << "get_symbols(): "
               << "d_past_sample=" << d_past_sample << "\tsamples size=" << samples.size();

    for (uint32_t i_sym = 0; i_sym < samples.size() / sbas_l1_telemetry_decoder_gs::D_SAMPLES_PER_SYMBOL; i_sym++)
        {
            // get the next samples
            for (int32_t i = 0; i < d_n_smpls_in_history; i++)
                {
                    smpls[i] = static_cast<int32_t>(i_sym) * sbas_l1_telemetry_decoder_gs::D_SAMPLES_PER_SYMBOL + i - 1 == -1 ? d_past_sample : samples[i_sym * sbas_l1_telemetry_decoder_gs::D_SAMPLES_PER_SYMBOL + i - 1];
                }

            // update the pseudo correlations (IIR method) of the two possible alignments
            d_corr_paired = d_iir_par * smpls[1] * smpls[2] + (1 - d_iir_par) * d_corr_paired;
            d_corr_shifted = d_iir_par * smpls[0] * smpls[1] + (1 - d_iir_par) * d_corr_shifted;

            // decide which alignment is the correct one
            corr_diff = std::abs(d_corr_paired - d_corr_shifted);
            stand_by = d_aligned ? corr_diff < d_corr_paired / 2 : corr_diff < d_corr_shifted / 2;
            if (!stand_by)
                {
                    d_aligned = d_corr_paired >= d_corr_shifted;
                }

            // sum the correct pair of samples to a symbol, depending on the current alignment d_align
            sym = smpls[0 + int32_t(d_aligned) * 2] + smpls[1];
            symbols.push_back(sym);

            // sample alignment debug output
            VLOG(SAMP_SYNC) << std::setprecision(5)
                            << "smplp: " << std::setw(6) << smpls[0] << "   "
                            << "smpl0: " << std::setw(6)
                            << smpls[1] << "   "
                            << "smpl1: " << std::setw(6) << smpls[2] << "\t"
                            //<< "Flag_valid_tracking: " << std::setw(1) << in[0][0].Flag_valid_tracking << " " << std::setw(1) << in[0][0].Flag_valid_tracking << "\t"
                            << "d_corr_paired: " << std::setw(10) << d_corr_paired << "\t"
                            << "d_corr_shifted: " << std::setw(10) << d_corr_shifted << "\t"
                            << "corr_diff: " << std::setw(10) << corr_diff << "\t"
                            << "stand_by: " << std::setw(1) << stand_by << "\t"
                            << "d_aligned: " << std::setw(1) << d_aligned << "\t"
                            << "sym: " << std::setw(10) << sym << "\t";
        }

    // save last sample for next block
    double temp;
    temp = samples.back();
    d_past_sample = (temp);
    return d_aligned;
}


// ### helper class for symbol alignment and viterbi decoding ###
sbas_l1_telemetry_decoder_gs::Symbol_Aligner_And_Decoder::Symbol_Aligner_And_Decoder()
{
    // convolutional code properties
    d_KK = 7;
    const int32_t nn = 2;
    std::array<int32_t, nn> g_encoder{121, 91};

    d_vd1 = std::make_shared<Viterbi_Decoder>(g_encoder.data(), d_KK, nn);
    d_vd2 = std::make_shared<Viterbi_Decoder>(g_encoder.data(), d_KK, nn);
    d_past_symbol = 0;
}


void sbas_l1_telemetry_decoder_gs::Symbol_Aligner_And_Decoder::reset()
{
    d_past_symbol = 0;
    d_vd1->reset();
    d_vd2->reset();
}


bool sbas_l1_telemetry_decoder_gs::Symbol_Aligner_And_Decoder::get_bits(const std::vector<double> &symbols, std::vector<int32_t> &bits)
{
    const int32_t traceback_depth = 5 * d_KK;
    int32_t nbits_requested = symbols.size() / D_SYMBOLS_PER_BIT;
    int32_t nbits_decoded;
    // fill two vectors with the two possible symbol alignments
    std::vector<double> symbols_vd1(symbols);  // aligned symbol vector -> copy input symbol vector
    std::vector<double> symbols_vd2;           // shifted symbol vector -> add past sample in front of input vector
    symbols_vd2.push_back(d_past_symbol);
    for (auto symbol_it = symbols.cbegin(); symbol_it != symbols.cend() - 1; ++symbol_it)
        {
            symbols_vd2.push_back(*symbol_it);
        }
    // arrays for decoded bits
    std::vector<int32_t> bits_vd1(nbits_requested);
    std::vector<int32_t> bits_vd2(nbits_requested);
    // decode
    float metric_vd1 = d_vd1->decode_continuous(symbols_vd1.data(), traceback_depth, bits_vd1.data(), nbits_requested, nbits_decoded);
    float metric_vd2 = d_vd2->decode_continuous(symbols_vd2.data(), traceback_depth, bits_vd2.data(), nbits_requested, nbits_decoded);
    // choose the bits with the better metric
    for (int32_t i = 0; i < nbits_decoded; i++)
        {
            if (metric_vd1 > metric_vd2)
                {  // symbols aligned
                    bits.push_back(bits_vd1[i]);
                }
            else
                {  // symbols shifted
                    bits.push_back(bits_vd2[i]);
                }
        }
    d_past_symbol = symbols.back();
    return metric_vd1 > metric_vd2;
}


// ### helper class for detecting the preamble and collect the corresponding message candidates ###
void sbas_l1_telemetry_decoder_gs::Frame_Detector::reset()
{
    d_buffer.clear();
}


void sbas_l1_telemetry_decoder_gs::Frame_Detector::get_frame_candidates(const std::vector<int32_t> &bits, std::vector<std::pair<int32_t, std::vector<int32_t>>> &msg_candidates)
{
    std::stringstream ss;
    uint32_t sbas_msg_length = 250;
    std::vector<std::vector<int32_t>> preambles = {{0, 1, 0, 1, 0, 0, 1, 1},
        {1, 0, 0, 1, 1, 0, 1, 0},
        {1, 1, 0, 0, 0, 1, 1, 0}};
    VLOG(FLOW) << "get_frame_candidates(): "
               << "d_buffer.size()=" << d_buffer.size() << "\tbits.size()=" << bits.size();
    ss << "copy bits ";
    int32_t count = 0;
    // copy new bits into the working buffer
    for (auto bit_it = bits.cbegin(); bit_it < bits.cend(); ++bit_it)
        {
            d_buffer.push_back(*bit_it);
            ss << *bit_it;
            count++;
        }
    VLOG(SAMP_SYNC) << ss.str() << " into working buffer (" << count << " bits)";
    int32_t relative_preamble_start = 0;
    while (d_buffer.size() >= sbas_msg_length)
        {
            // compare with all preambles
            for (auto preample_it = preambles.begin(); preample_it < preambles.end(); ++preample_it)
                {
                    bool preamble_detected = true;
                    bool inv_preamble_detected = true;
                    // compare the buffer bits with the preamble bits
                    for (auto preample_bit_it = preample_it->begin(); preample_bit_it < preample_it->end(); ++preample_bit_it)
                        {
                            preamble_detected = *preample_bit_it == d_buffer[preample_bit_it - preample_it->begin()] ? preamble_detected : false;
                            inv_preamble_detected = *preample_bit_it != d_buffer[preample_bit_it - preample_it->begin()] ? inv_preamble_detected : false;
                        }
                    if (preamble_detected || inv_preamble_detected)
                        {
                            // copy candidate
                            std::vector<int32_t> candidate;
                            std::copy(d_buffer.begin(), d_buffer.begin() + sbas_msg_length, std::back_inserter(candidate));
                            if (inv_preamble_detected)
                                {
                                    // invert bits
                                    for (int &candidate_bit_it : candidate)
                                        {
                                            candidate_bit_it = candidate_bit_it == 0 ? 1 : 0;
                                        }
                                }
                            msg_candidates.emplace_back(relative_preamble_start, candidate);
                            ss.str("");
                            ss << "preamble " << preample_it - preambles.begin() << (inv_preamble_detected ? " inverted" : " normal") << " detected! candidate=";
                            for (auto bit_it = candidate.begin(); bit_it < candidate.end(); ++bit_it)
                                {
                                    ss << *bit_it;
                                }
                            VLOG(EVENT) << ss.str();
                        }
                }
            relative_preamble_start++;
            // remove bit in front
            d_buffer.pop_front();
        }
}


// ### helper class for checking the CRC of the message candidates ###
void sbas_l1_telemetry_decoder_gs::Crc_Verifier::reset()
{
}


void sbas_l1_telemetry_decoder_gs::Crc_Verifier::get_valid_frames(const std::vector<msg_candiate_int_t> &msg_candidates, std::vector<msg_candiate_char_t> &valid_msgs)
{
    std::stringstream ss;
    VLOG(FLOW) << "get_valid_frames(): "
               << "msg_candidates.size()=" << msg_candidates.size();
    // for each candidate
    for (auto candidate_it = msg_candidates.cbegin(); candidate_it < msg_candidates.cend(); ++candidate_it)
        {
            // convert to bytes
            std::vector<uint8_t> candidate_bytes;
            zerropad_back_and_convert_to_bytes(candidate_it->second, candidate_bytes);
            // verify CRC
            d_checksum_agent.reset(0);
            d_checksum_agent.process_bytes(candidate_bytes.data(), candidate_bytes.size());
            uint32_t crc = d_checksum_agent.checksum();
            VLOG(SAMP_SYNC) << "candidate " << candidate_it - msg_candidates.begin()
                            << ": final crc remainder= " << std::hex << crc
                            << std::setfill(' ') << std::resetiosflags(std::ios::hex);
            //  the final remainder must be zero for a valid message, because the CRC is done over the received CRC value
            if (crc == 0)
                {
                    valid_msgs.emplace_back(candidate_it->first, candidate_bytes);
                    ss << "Valid message found!";
                }
            else
                {
                    ss << "Not a valid message.";
                }
            ss << " Relbitoffset=" << candidate_it->first << " content=";
            for (auto byte_it = candidate_bytes.begin(); byte_it < candidate_bytes.end(); ++byte_it)
                {
                    ss << std::setw(2) << std::setfill('0') << std::hex << static_cast<uint32_t>((*byte_it));
                }
            VLOG(SAMP_SYNC) << ss.str() << std::setfill(' ') << std::resetiosflags(std::ios::hex) << std::endl;
        }
}


void sbas_l1_telemetry_decoder_gs::Crc_Verifier::zerropad_back_and_convert_to_bytes(const std::vector<int> &msg_candidate, std::vector<uint8_t> &bytes)
{
    std::stringstream ss;
    const size_t bits_per_byte = 8;
    uint8_t byte = 0;
    VLOG(LMORE) << "zerropad_back_and_convert_to_bytes():" << byte;
    for (auto candidate_bit_it = msg_candidate.cbegin(); candidate_bit_it < msg_candidate.cend(); ++candidate_bit_it)
        {
            int32_t idx_bit = candidate_bit_it - msg_candidate.begin();
            int32_t bit_pos_in_current_byte = (bits_per_byte - 1) - (idx_bit % bits_per_byte);
            byte |= static_cast<uint8_t>(*candidate_bit_it) << bit_pos_in_current_byte;
            ss << *candidate_bit_it;
            if (idx_bit % bits_per_byte == bits_per_byte - 1)
                {
                    bytes.push_back(byte);
                    VLOG(LMORE) << ss.str() << " -> byte=" << std::setw(2) << std::setfill('0') << std::hex << static_cast<uint32_t>(byte);
                    ss.str("");
                    byte = 0;
                }
        }
    bytes.push_back(byte);  // implies: insert 6 zeros at the end to fit the 250bits into a multiple of bytes
    VLOG(LMORE) << " -> byte=" << std::setw(2)
                << std::setfill('0') << std::hex << static_cast<uint32_t>(byte)
                << std::setfill(' ') << std::resetiosflags(std::ios::hex);
}


void sbas_l1_telemetry_decoder_gs::Crc_Verifier::zerropad_front_and_convert_to_bytes(const std::vector<int32_t> &msg_candidate, std::vector<uint8_t> &bytes)
{
    std::stringstream ss;
    const size_t bits_per_byte = 8;
    uint8_t byte = 0;
    int32_t idx_bit = 6;  // insert 6 zeros at the front to fit the 250bits into a multiple of bytes
    VLOG(LMORE) << "zerropad_front_and_convert_to_bytes():" << byte;
    for (auto candidate_bit_it = msg_candidate.cbegin(); candidate_bit_it < msg_candidate.cend(); ++candidate_bit_it)
        {
            int32_t bit_pos_in_current_byte = (bits_per_byte - 1) - (idx_bit % bits_per_byte);
            byte |= static_cast<uint8_t>(*candidate_bit_it) << bit_pos_in_current_byte;
            ss << *candidate_bit_it;
            if (idx_bit % bits_per_byte == bits_per_byte - 1)
                {
                    bytes.push_back(byte);
                    VLOG(LMORE) << ss.str() << " -> byte=" << std::setw(2)
                                << std::setfill('0') << std::hex << static_cast<uint32_t>(byte);
                    ss.str("");
                    byte = 0;
                }
            idx_bit++;
        }
    VLOG(LMORE) << " -> byte=" << std::setw(2)
                << std::setfill('0') << std::hex << static_cast<uint32_t>(byte)
                << std::setfill(' ') << std::resetiosflags(std::ios::hex);
}


int sbas_l1_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    VLOG(FLOW) << "general_work(): "
               << "noutput_items=" << noutput_items << "\toutput_items real size=" << output_items.size() << "\tninput_items size=" << ninput_items.size() << "\tinput_items real size=" << input_items.size() << "\tninput_items[0]=" << ninput_items[0];
    // get pointers on in- and output gnss-synchro objects
    auto *out = reinterpret_cast<Gnss_Synchro *>(output_items[0]);            // Get the output buffer pointer
    const auto *in = reinterpret_cast<const Gnss_Synchro *>(input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol{};  // structure to save the synchronization information and send the output object to the next block
    // 1. Copy the current tracking output
    current_symbol = in[0];
    // copy correlation samples into samples vector
    d_sample_buf.push_back(current_symbol.Prompt_I);  //add new symbol to the symbol queue

    // store the time stamp of the first sample in the processed sample block
    double sample_stamp = static_cast<double>(in[0].Tracking_sample_counter) / static_cast<double>(in[0].fs);

    // decode only if enough samples in buffer
    if (d_sample_buf.size() >= d_block_size)
        {
            // align correlation samples in pairs
            // and obtain the symbols by summing the paired correlation samples
            std::vector<double> symbols;
            bool sample_alignment = d_sample_aligner.get_symbols(d_sample_buf, symbols);

            // align symbols in pairs
            // and obtain the bits by decoding the symbol pairs
            std::vector<int32_t> bits;
            bool symbol_alignment = d_symbol_aligner_and_decoder.get_bits(symbols, bits);

            // search for preambles
            // and extract the corresponding message candidates
            std::vector<msg_candiate_int_t> msg_candidates;
            d_frame_detector.get_frame_candidates(bits, msg_candidates);

            // verify checksum
            // and return the valid messages
            std::vector<msg_candiate_char_t> valid_msgs;
            d_crc_verifier.get_valid_frames(msg_candidates, valid_msgs);

            // compute message sample stamp
            // and fill messages in SBAS raw message objects
            //std::vector<Sbas_Raw_Msg> sbas_raw_msgs;
            for (const auto &valid_msg : valid_msgs)
                {
                    int32_t message_sample_offset =
                        (sample_alignment ? 0 : -1) + D_SAMPLES_PER_SYMBOL * (symbol_alignment ? -1 : 0) + D_SAMPLES_PER_SYMBOL * D_SYMBOLS_PER_BIT * valid_msg.first;
                    double message_sample_stamp = sample_stamp + static_cast<double>(message_sample_offset) / 1000.0;
                    VLOG(EVENT) << "message_sample_stamp=" << message_sample_stamp
                                << " (sample_stamp=" << sample_stamp
                                << " sample_alignment=" << sample_alignment
                                << " symbol_alignment=" << symbol_alignment
                                << " relative_preamble_start=" << valid_msg.first
                                << " message_sample_offset=" << message_sample_offset
                                << ")";
                    //Sbas_Raw_Msg sbas_raw_msg(message_sample_stamp, this->d_satellite.get_PRN(), it->second);
                    //sbas_raw_msgs.push_back(sbas_raw_msg);
                }

            // parse messages
            // and send them to the SBAS raw message queue
            //for(std::vector<Sbas_Raw_Msg>::iterator it = sbas_raw_msgs.begin(); it != sbas_raw_msgs.end(); it++)
            //    {
            //std::cout << "SBAS message type " << it->get_msg_type() << " from PRN" << it->get_prn() << " received" << std::endl;
            //sbas_telemetry_data.update(*it);
            //    }

            // clear all processed samples in the input buffer
            d_sample_buf.clear();
        }

    // UPDATE GNSS SYNCHRO DATA
    // actually the SBAS telemetry decoder doesn't support ranging
    current_symbol.Flag_valid_word = false;  // indicate to observable block that this synchro object isn't valid for pseudorange computation
    out[0] = current_symbol;
    consume_each(1);  // tell scheduler input items consumed
    return 1;         // tell scheduler output items produced
}
