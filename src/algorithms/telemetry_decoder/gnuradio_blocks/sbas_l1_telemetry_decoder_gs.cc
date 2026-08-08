/*!
 * \file sbas_l1_telemetry_decoder_gs.cc
 * \brief Implementation of a SBAS telemetry data decoder block
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
 * \author Miguel Gómez López, 2026. mgomezl(at)ing.uc3m.es
 * \author Víctor Castillo Agüero, 2026. victorcastilloaguero(at)gmail.com
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

#include "sbas_l1_telemetry_decoder_gs.h"
#include "SBAS_L1.h"
#include "gnss_synchro.h"
#include "viterbi_decoder_sbas.h"
#include <gnuradio/thread/thread.h>  // for scoped_lock
#include <pmt/pmt_sugar.h>           // for mp
#include <algorithm>                 // for copy
#include <array>
#include <cmath>      // for abs
#include <exception>  // for exception
#include <iomanip>    // for operator<<, setw
#include <iostream>   // for std::cout
#include <map>        // for map
#include <mutex>      // for lock_guard, mutex
#include <set>        // for set
#include <string>     // for std::string
#include <utility>    // for std::move

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

// logging levels
#define EVENT 2      // logs important events which don't occur every block
#define FLOW 3       // logs the function calls of block processing functions
#define SAMP_SYNC 4  // about 1 log entry per sample -> high output
#define LMORE 5      //


class SbasL1DumpSession::Impl
{
public:
    bool open(std::ofstream &stream, const std::string &filename)
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto insertion = initialized_files.insert(filename);
        const auto open_mode = std::ios::out | (insertion.second ? std::ios::trunc : std::ios::app);
        stream.open(filename, open_mode);
        if (!stream.is_open() && insertion.second)
            {
                initialized_files.erase(insertion.first);
            }
        return insertion.second;
    }

    std::mutex mutex;
    std::set<std::string> initialized_files;
};


SbasL1DumpSession::SbasL1DumpSession() : d_impl(new Impl)
{
}


SbasL1DumpSession::~SbasL1DumpSession() = default;


bool SbasL1DumpSession::open(std::ofstream &stream, const std::string &filename)
{
    return d_impl->open(stream, filename);
}


std::shared_ptr<SbasL1DumpSession> sbas_l1_telemetry_decoder_gs::acquire_dump_session(const std::string &dump_filename)
{
    static std::mutex registry_mutex;
    static std::map<std::string, std::weak_ptr<SbasL1DumpSession>> sessions;

    std::lock_guard<std::mutex> lock(registry_mutex);
    auto session = sessions[dump_filename].lock();
    if (!session)
        {
            session = std::make_shared<SbasL1DumpSession>();
            sessions[dump_filename] = session;
        }
    return session;
}


sbas_l1_telemetry_decoder_gs_sptr sbas_l1_make_telemetry_decoder_gs(
    bool dump,
    std::string dump_filename)
{
    return sbas_l1_telemetry_decoder_gs_sptr(new sbas_l1_telemetry_decoder_gs(dump, std::move(dump_filename)));
}


sbas_l1_telemetry_decoder_gs::sbas_l1_telemetry_decoder_gs(
    bool dump,
    std::string dump_filename) : telemetry_impl_interface("sbas_l1_telemetry_decoder_gs",
                                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro))),
                                 d_dump(dump),
                                 d_channel(0),
                                 d_block_size(D_SAMPLES_PER_SYMBOL * D_SYMBOLS_PER_BIT * D_BLOCK_SIZE_IN_BITS)
{
    configure_basic_outputs();

    // initialize internal vars
    LOG(INFO) << "SBAS L1 TELEMETRY PROCESSING: satellite " << d_satellite;
    set_output_multiple(1);

    if (d_dump && !dump_filename.empty())
        {
            // Store the stem; actual EMS file is opened lazily in general_work()
            // after set_satellite() has been called and d_satellite PRN is valid.
            d_dump_filename = std::move(dump_filename);
            d_dump_session = acquire_dump_session(d_dump_filename);
        }
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
    if (d_ems_file.is_open())
        {
            d_ems_file.close();
        }
}


void sbas_l1_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    gr::thread::scoped_lock lock(d_setlock);
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "SBAS telemetry decoder in channel " << this->d_channel << " set to satellite " << d_satellite;
}


void sbas_l1_telemetry_decoder_gs::set_channel(int32_t channel)
{
    gr::thread::scoped_lock lock(d_setlock);
    d_channel = channel;
    LOG(INFO) << "SBAS channel set to " << channel;
}


void sbas_l1_telemetry_decoder_gs::reset()
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    d_sample_buf.clear();
    d_sample_stamps.clear();
    d_sample_aligner.reset();
    d_symbol_aligner_and_decoder.reset();
    d_frame_detector.reset();
    d_crc_verifier.reset();
    if (d_ems_file.is_open())
        {
            d_ems_file.close();
        }
}


std::vector<double> sbas_l1_telemetry_decoder_gs::consume_decoded_timestamps(
    std::deque<double> &pending_stamps,
    const std::vector<double> &input_stamps,
    int32_t decoded_bits)
{
    pending_stamps.insert(pending_stamps.end(), input_stamps.cbegin(), input_stamps.cend());

    std::vector<double> output_stamps;
    output_stamps.reserve(static_cast<size_t>(decoded_bits));
    for (int32_t i = 0; i < decoded_bits; ++i)
        {
            output_stamps.push_back(pending_stamps.front());
            pending_stamps.pop_front();
        }
    return output_stamps;
}


// ### helper class for sample alignment ###
sbas_l1_telemetry_decoder_gs::Sample_Aligner::Sample_Aligner()
{
    reset();
}


void sbas_l1_telemetry_decoder_gs::Sample_Aligner::reset()
{
    d_past_sample = 0;
    d_past_sample_stamp = 0;
    d_has_past_sample = false;
    d_corr_paired = 0;
    d_corr_shifted = 0;
    d_aligned = true;
}


/*
 * samples length must be a multiple of two
 */
bool sbas_l1_telemetry_decoder_gs::Sample_Aligner::get_symbols(
    const std::vector<double> &samples,
    const std::vector<double> &sample_stamps,
    std::vector<double> &symbols,
    std::vector<double> &symbol_stamps)
{
    std::array<double, 3> smpls{};
    std::array<double, 3> smpl_stamps{};
    double corr_diff;
    bool stand_by = true;
    double sym;

    VLOG(FLOW) << "get_symbols(): "
               << "d_past_sample=" << d_past_sample << "\tsamples size=" << samples.size();

    for (size_t i_sym = 0; i_sym < samples.size() / sbas_l1_telemetry_decoder_gs::D_SAMPLES_PER_SYMBOL; i_sym++)
        {
            // get the next samples
            for (int32_t i = 0; i < d_n_smpls_in_history; i++)
                {
                    const int32_t sample_index = static_cast<int32_t>(i_sym) * sbas_l1_telemetry_decoder_gs::D_SAMPLES_PER_SYMBOL + i - 1;
                    if (sample_index == -1)
                        {
                            smpls[i] = d_past_sample;
                            smpl_stamps[i] = d_has_past_sample
                                                 ? d_past_sample_stamp
                                                 : sample_stamps.front() - SBAS_L1_CODE_PERIOD_S;
                        }
                    else
                        {
                            smpls[i] = samples[static_cast<size_t>(sample_index)];
                            smpl_stamps[i] = sample_stamps[static_cast<size_t>(sample_index)];
                        }
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
            symbol_stamps.push_back(d_aligned ? smpl_stamps[1] : smpl_stamps[0]);

            // sample alignment debug output
            VLOG(SAMP_SYNC) << std::setprecision(5)
                            << "smplp: " << std::setw(6) << smpls[0] << "   "
                            << "smpl0: " << std::setw(6)
                            << smpls[1] << "   "
                            << "smpl1: " << std::setw(6) << smpls[2] << "\t"
                            // << "Flag_valid_tracking: " << std::setw(1) << in[0][0].Flag_valid_tracking << " " << std::setw(1) << in[0][0].Flag_valid_tracking << "\t"
                            << "d_corr_paired: " << std::setw(10) << d_corr_paired << "\t"
                            << "d_corr_shifted: " << std::setw(10) << d_corr_shifted << "\t"
                            << "corr_diff: " << std::setw(10) << corr_diff << "\t"
                            << "stand_by: " << std::setw(1) << stand_by << "\t"
                            << "d_aligned: " << std::setw(1) << d_aligned << "\t"
                            << "sym: " << std::setw(10) << sym << "\t";
        }

    // save last sample for next block
    d_past_sample = samples.back();
    d_past_sample_stamp = sample_stamps.back();
    d_has_past_sample = true;
    return d_aligned;
}


// ### helper class for symbol alignment and viterbi decoding ###
sbas_l1_telemetry_decoder_gs::Symbol_Aligner_And_Decoder::Symbol_Aligner_And_Decoder()
{
    // convolutional code properties
    const int32_t nn = 2;
    std::array<int32_t, nn> g_encoder{121, 91};

    d_vd1 = std::make_shared<Viterbi_Decoder_Sbas>(g_encoder.data(), d_KK, nn);
    d_vd2 = std::make_shared<Viterbi_Decoder_Sbas>(g_encoder.data(), d_KK, nn);
}


void sbas_l1_telemetry_decoder_gs::Symbol_Aligner_And_Decoder::reset()
{
    d_past_symbol = 0;
    d_past_symbol_stamp = 0;
    d_has_past_symbol = false;
    d_pending_bit_stamps_vd1.clear();
    d_pending_bit_stamps_vd2.clear();
    d_vd1->reset();
    d_vd2->reset();
}


bool sbas_l1_telemetry_decoder_gs::Symbol_Aligner_And_Decoder::get_bits(
    const std::vector<double> &symbols,
    const std::vector<double> &symbol_stamps,
    std::vector<int32_t> &bits,
    std::vector<double> &bit_stamps)
{
    const int32_t traceback_depth = 5 * d_KK;
    const int32_t nbits_requested = symbols.size() / D_SYMBOLS_PER_BIT;
    int32_t nbits_decoded_vd1;
    int32_t nbits_decoded_vd2;
    // fill two vectors with the two possible symbol alignments
    const std::vector<double> &symbols_vd1(symbols);  // aligned symbol vector -> copy input symbol vector
    std::vector<double> symbols_vd2;                  // shifted symbol vector -> add past sample in front of input vector
    std::vector<double> symbol_stamps_vd2;
    symbols_vd2.reserve(symbols.size());
    symbol_stamps_vd2.reserve(symbol_stamps.size());
    symbols_vd2.push_back(d_past_symbol);
    symbol_stamps_vd2.push_back(d_has_past_symbol
                                    ? d_past_symbol_stamp
                                    : symbol_stamps.front() - D_SAMPLES_PER_SYMBOL * SBAS_L1_CODE_PERIOD_S);
    for (size_t i = 0; i + 1 < symbols.size(); ++i)
        {
            symbols_vd2.push_back(symbols[i]);
            symbol_stamps_vd2.push_back(symbol_stamps[i]);
        }

    std::vector<double> input_bit_stamps_vd1(static_cast<size_t>(nbits_requested));
    std::vector<double> input_bit_stamps_vd2(static_cast<size_t>(nbits_requested));
    for (int32_t i = 0; i < nbits_requested; ++i)
        {
            const auto symbol_index = static_cast<size_t>(i * D_SYMBOLS_PER_BIT);
            input_bit_stamps_vd1[static_cast<size_t>(i)] = symbol_stamps[symbol_index];
            input_bit_stamps_vd2[static_cast<size_t>(i)] = symbol_stamps_vd2[symbol_index];
        }

    // arrays for decoded bits
    std::vector<int32_t> bits_vd1(nbits_requested);
    std::vector<int32_t> bits_vd2(nbits_requested);
    // decode
    const float metric_vd1 = d_vd1->decode_continuous(symbols_vd1.data(), traceback_depth, bits_vd1.data(), nbits_requested, nbits_decoded_vd1);
    const float metric_vd2 = d_vd2->decode_continuous(symbols_vd2.data(), traceback_depth, bits_vd2.data(), nbits_requested, nbits_decoded_vd2);
    const auto bit_stamps_vd1 = sbas_l1_telemetry_decoder_gs::consume_decoded_timestamps(d_pending_bit_stamps_vd1, input_bit_stamps_vd1, nbits_decoded_vd1);
    const auto bit_stamps_vd2 = sbas_l1_telemetry_decoder_gs::consume_decoded_timestamps(d_pending_bit_stamps_vd2, input_bit_stamps_vd2, nbits_decoded_vd2);

    // choose the bits with the better metric
    const bool use_vd1 = metric_vd1 > metric_vd2;
    const int32_t nbits_decoded = use_vd1 ? nbits_decoded_vd1 : nbits_decoded_vd2;
    const auto &selected_bits = use_vd1 ? bits_vd1 : bits_vd2;
    const auto &selected_bit_stamps = use_vd1 ? bit_stamps_vd1 : bit_stamps_vd2;
    for (int32_t i = 0; i < nbits_decoded; i++)
        {
            bits.push_back(selected_bits[static_cast<size_t>(i)]);
            bit_stamps.push_back(selected_bit_stamps[static_cast<size_t>(i)]);
        }
    d_past_symbol = symbols.back();
    d_past_symbol_stamp = symbol_stamps.back();
    d_has_past_symbol = true;
    return use_vd1;
}


// ### helper class for detecting the preamble and collect the corresponding message candidates ###
void sbas_l1_telemetry_decoder_gs::Frame_Detector::reset()
{
    d_buffer.clear();
    d_bit_stamps.clear();
}


void sbas_l1_telemetry_decoder_gs::Frame_Detector::get_frame_candidates(const std::vector<int32_t> &bits, const std::vector<double> &bit_stamps, std::vector<std::pair<double, std::vector<int32_t>>> &msg_candidates)
{
    std::stringstream ss;
    const uint32_t sbas_msg_length = 250;
    const std::vector<std::vector<int32_t>> preambles = {{0, 1, 0, 1, 0, 0, 1, 1},
        {1, 0, 0, 1, 1, 0, 1, 0},
        {1, 1, 0, 0, 0, 1, 1, 0}};
    VLOG(FLOW) << "get_frame_candidates(): "
               << "d_buffer.size()=" << d_buffer.size() << "\tbits.size()=" << bits.size();
    ss << "copy bits ";
    int32_t count = 0;
    // copy new bits (and their absolute timestamps) into the working buffer
    for (size_t i = 0; i < bits.size(); i++)
        {
            d_buffer.push_back(bits[i]);
            d_bit_stamps.push_back(bit_stamps[i]);
            ss << bits[i];
            count++;
        }
    VLOG(SAMP_SYNC) << ss.str() << " into working buffer (" << count << " bits)";
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
                            // the candidate's absolute timestamp is that of its first (preamble) bit
                            msg_candidates.emplace_back(d_bit_stamps.front(), candidate);
                            ss.str("");
                            ss << "preamble " << preample_it - preambles.begin() << (inv_preamble_detected ? " inverted" : " normal") << " detected! candidate=";
                            for (auto bit_it = candidate.begin(); bit_it < candidate.end(); ++bit_it)
                                {
                                    ss << *bit_it;
                                }
                            VLOG(EVENT) << ss.str();
                        }
                }
            // remove bit (and its timestamp) in front
            d_buffer.pop_front();
            d_bit_stamps.pop_front();
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
            const uint32_t crc = d_checksum_agent.checksum();
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
            VLOG(SAMP_SYNC) << ss.str() << std::setfill(' ') << std::resetiosflags(std::ios::hex) << '\n';
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
            const int32_t idx_bit = candidate_bit_it - msg_candidate.begin();
            const int32_t bit_pos_in_current_byte = (bits_per_byte - 1) - (idx_bit % bits_per_byte);
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
            const int32_t bit_pos_in_current_byte = (bits_per_byte - 1) - (idx_bit % bits_per_byte);
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
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with reset() called by the control thread
    VLOG(FLOW) << "general_work(): "
               << "noutput_items=" << noutput_items << "\toutput_items real size=" << output_items.size() << "\tninput_items size=" << ninput_items.size() << "\tinput_items real size=" << input_items.size() << "\tninput_items[0]=" << ninput_items[0];
    // get pointers on in- and output gnss-synchro objects
    auto *out = reinterpret_cast<Gnss_Synchro *>(output_items[0]);            // Get the output buffer pointer
    const auto *in = reinterpret_cast<const Gnss_Synchro *>(input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol{};  // structure to save the synchronization information and send the output object to the next block
    // 1. Copy the current tracking output
    current_symbol = in[0];
    // copy correlation samples into samples vector
    d_sample_buf.push_back(current_symbol.Prompt_I);  // add new symbol to the symbol queue

    // absolute reception timestamp of this sample; stored so that, once a message is
    // eventually found several blocks later, its true timestamp can be recovered instead
    // of approximated from whichever sample happens to be current at that later time.
    const double sample_stamp = static_cast<double>(in[0].Tracking_sample_counter) / static_cast<double>(in[0].fs);
    d_sample_stamps.push_back(sample_stamp);

    // decode only if enough samples in buffer
    if (d_sample_buf.size() >= d_block_size)
        {
            // align correlation samples in pairs
            // and obtain the symbols by summing the paired correlation samples
            std::vector<double> symbols;
            std::vector<double> symbol_stamps;
            d_sample_aligner.get_symbols(d_sample_buf, d_sample_stamps, symbols, symbol_stamps);

            // align symbols in pairs
            // and obtain the bits by decoding the symbol pairs
            std::vector<int32_t> bits;
            std::vector<double> bit_stamps;
            d_symbol_aligner_and_decoder.get_bits(symbols, symbol_stamps, bits, bit_stamps);

            // search for preambles
            // and extract the corresponding message candidates
            std::vector<msg_candiate_int_t> msg_candidates;
            d_frame_detector.get_frame_candidates(bits, bit_stamps, msg_candidates);

            // verify checksum
            // and return the valid messages
            std::vector<msg_candiate_char_t> valid_msgs;
            d_crc_verifier.get_valid_frames(msg_candidates, valid_msgs);

            // compute message sample stamp
            // and fill messages in SBAS raw message objects
            // std::vector<Sbas_Raw_Msg> sbas_raw_msgs;
            for (const auto &valid_msg : valid_msgs)
                {
                    // valid_msg.first is the absolute timestamp of the message's first bit
                    const double message_sample_stamp = valid_msg.first;
                    VLOG(EVENT) << "message_sample_stamp=" << message_sample_stamp
                                << " (first_bit_stamp=" << valid_msg.first << ")";
                    // extract message type: bits 8-13 of 250-bit message (top 6 bits of byte[1])
                    const int msg_type = (valid_msg.second.size() > 1)
                                             ? ((static_cast<int>(valid_msg.second[1]) >> 2) & 0x3F)
                                             : -1;
                    VLOG(EVENT) << "SBAS PRN " << d_satellite.get_PRN()
                                << " MT" << msg_type
                                << " at t=" << std::fixed << std::setprecision(3)
                                << message_sample_stamp << " s";
                    // Lazy-open message dump file on first decoded message so PRN is known.
                    // Dumps are overwritten (not appended) on every run.
                    if (d_dump && !d_dump_filename.empty() && !d_ems_file.is_open())
                        {
                            std::string ems_fn = d_dump_filename;
                            const std::string dat_ext(".dat");
                            if (ems_fn.size() > dat_ext.size() &&
                                ems_fn.compare(ems_fn.size() - dat_ext.size(), dat_ext.size(), dat_ext) == 0)
                                {
                                    ems_fn = ems_fn.substr(0, ems_fn.size() - dat_ext.size());
                                }
                            ems_fn += "_PRN" + std::to_string(d_satellite.get_PRN()) + ".ems";
                            // All channel decoders using this dump stem share a session.
                            // The first open of each PRN truncates stale data from a
                            // previous receiver run; later opens append after reset or
                            // channel reassignment, preserving messages from this run.
                            const bool first_open = d_dump_session->open(d_ems_file, ems_fn);
                            if (d_ems_file.is_open())
                                {
                                    if (first_open)
                                        {
                                            // NOTE on naming/format: this is a raw dump of decoded SBAS L1 messages
                                            // using the same field layout as ESA's EGNOS Message Service (EMS)
                                            // archive files (week, tow, prn, type : hex bytes). It is NOT a
                                            // conformant EMS file: this receiver has no absolute GPS time
                                            // reference, so "week" is always 0 and "tow" is the receiver's
                                            // elapsed time since recording start, not the real GPS time of week.
                                            d_ems_file << "; Raw SBAS L1 message dump (EMS-like layout, not a conformant EMS file)\n";
                                            d_ems_file << "; Satellite: SBAS PRN " << d_satellite.get_PRN() << "\n";
                                            d_ems_file << "; TOW field: receiver time in seconds from recording start\n";
                                            d_ems_file << ";            GPS week = 0  (no absolute GPS fix in this output)\n";
                                            d_ems_file << "; Msg bytes: 32 bytes (250-bit SBAS frame zero-padded to 256 bits)\n";
                                            d_ems_file << ";   Layout: preamble(8b) type(6b) data(212b) CRC-24Q(24b) pad(6b)\n";
                                            d_ems_file << "; Format:   <gps_week> <tow_s> <prn> <msg_type> : <64_hex_chars>\n";
                                            d_ems_file << ";\n";
                                        }
                                    LOG(INFO) << "SBAS raw message dump file "
                                              << (first_open ? "created: " : "reopened: ") << ems_fn;
                                }
                            else
                                {
                                    LOG(WARNING) << "Could not open SBAS raw message dump file: " << ems_fn;
                                }
                        }
                    if (d_ems_file.is_open())
                        {
                            d_ems_file << std::dec << std::setfill(' ')
                                       << std::setw(4) << 0 << " "
                                       << std::setw(10) << std::fixed << std::setprecision(3)
                                       << message_sample_stamp << " "
                                       << std::setw(3) << d_satellite.get_PRN() << " "
                                       << std::setw(2) << msg_type << " : ";
                            for (const auto byte : valid_msg.second)
                                {
                                    d_ems_file << std::setw(2) << std::setfill('0') << std::hex
                                               << static_cast<unsigned int>(byte);
                                }
                            d_ems_file << std::dec << std::setfill(' ') << '\n';
                            d_ems_file.flush();
                        }
                        // Sbas_Raw_Msg sbas_raw_msg(message_sample_stamp, this->d_satellite.get_PRN(), it->second);
                        // sbas_raw_msgs.push_back(sbas_raw_msg);
#if __cplusplus == 201103L
                    const int default_precision = std::cout.precision();
#else
                    const auto default_precision{std::cout.precision()};
#endif
                    std::cout << "New SBAS L1 NAV message received in channel " << d_channel
                              << ": MT" << msg_type << " from satellite " << d_satellite
                              << " with CN0=" << std::setprecision(2) << current_symbol.CN0_dB_hz
                              << std::setprecision(default_precision) << " dB-Hz" << std::endl;
                }

            // TODO: parse per-message-type content and apply SBAS corrections
            // -----------------------------------------------------------------------
            // SBAS L1 message parsing status (RTCA DO-229E / ICAO SARPs Appendix B)
            // -----------------------------------------------------------------------
            // SIGNAL / TRANSPORT LAYER — implemented:
            //   [x] Viterbi FEC decoding (rate-1/2, K=7 )
            //   [x] Three-preamble detection (0x53 / 0x9A / 0xC6)
            //   [x] CRC-24Q verification
            //   [x] 6-bit message type extraction
            //   [x] Raw EMS file output (32 bytes / message)
            //
            // NAVIGATION DATA LAYER — NOT yet implemented (all content is written to
            // the EMS file as raw bytes; no correction data is used by the receiver):
            //
            //   Satellite information messages (DO-229E §A.4.4):
            //   [ ] MT1  — PRN mask assignments (up to 51 of 210 GNSS/GEO PRNs)
            //   [ ] MT2–5 — Fast corrections + UDREI for satellites in mask
            //   [ ] MT6  — Integrity information (UDREI for all 51 mask slots)
            //   [ ] MT7  — Fast correction degradation factors (aij table)
            //   [ ] MT9  — GEO navigation message (ECEF pos/vel, URA, t0)
            //   [ ] MT17 — GEO satellite almanacs (health/status + rough position)
            //   [ ] MT24 — Mixed fast corrections / long-term corrections
            //   [ ] MT25 — Long-term satellite error corrections (orbit + clock)
            //   [ ] MT28 — Clock–Ephemeris covariance matrix (optional)
            //
            //   Ionospheric messages (DO-229E §A.4.4.9–10):
            //   [ ] MT18 — Ionospheric grid point (IGP) masks (per band 0–10)
            //   [ ] MT26 — Ionospheric grid delays (GIVD) + error bounds (GIVEI)
            //
            //   Ancillary messages:
            //   [ ] MT0  — Don't Use / test-mode flag (safety de-selection)
            //   [ ] MT10 — Degradation parameters (σ²_flt, σ²_iono for PA ops)
            //   [ ] MT12 — SBAS Network time / UTC offset (GPS-week + SOW)
            //   [ ] MT27 — Service message / δUDRE regional factors (optional)
            //   [ ] MT62 — Internal test message (optional, not used by EGNOS)
            //   [ ] MT63 — Null message (optional, not used by EGNOS)
            //
            //   SBAS L5 / DFMC messages (DO-229F, dual-freq multi-constellation):
            //   [ ] MT31 — Satellite Mask (up to 92 SV slots)
            //   [ ] MT32 — Clock-Ephemeris corrections + covariance matrix
            //   [ ] MT34–36 — Integrity messages (DFRECIs / DFREIs)
            //   [ ] MT37 — Degradation parameters + DFREI scale table
            //   [ ] MT39/40 — SBAS satellite ephemeris (Keplerian) + covariance
            //   [ ] MT47 — SBAS satellite almanacs (Keplerian, 2 GEOs per msg)
            //
            //   Receiver-level corrections (depends on navigation layer above):
            //   [ ] Apply fast + long-term satellite corrections to pseudoranges
            //   [ ] Apply ionospheric grid delay corrections (MOPS §A.4.4.10)
            //   [ ] Enable SBAS ranging (set Flag_valid_word = true when ranging ok)
            //   [ ] Propagate SBAS correction quality (UDRE, GIVE) into PVT
            //
            // Reference: https://gssc.esa.int/navipedia/index.php/
            //            The_EGNOS_SBAS_Message_Format_Explained
            // -----------------------------------------------------------------------
            // (old hook kept for reference)
            // for (auto it = sbas_raw_msgs.begin(); it != sbas_raw_msgs.end(); ++it)
            //     sbas_telemetry_data.update(*it);

            // clear all processed samples in the input buffer
            d_sample_buf.clear();
            d_sample_stamps.clear();
        }

    // UPDATE GNSS SYNCHRO DATA
    // actually the SBAS telemetry decoder doesn't support ranging
    current_symbol.Flag_valid_word = false;  // indicate to observable block that this synchro object isn't valid for pseudorange computation
    out[0] = std::move(current_symbol);
    consume_each(1);  // tell scheduler input items consumed
    return 1;         // tell scheduler output items produced
}
