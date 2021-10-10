/*!
 * \file sbas_l1_telemetry_decoder_gs.h
 * \brief Interface of a SBAS telemetry data decoder block
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
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

#ifndef GNSS_SDR_SBAS_L1_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_SBAS_L1_TELEMETRY_DECODER_GS_H

#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include <boost/crc.hpp>  // for crc_optimal
#include <gnuradio/block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#include <cstddef>           // for size_t
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>  // for std::shared_ptr
#include <string>
#include <utility>  // for pair
#include <vector>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks
 * \{ */


class Viterbi_Decoder_Sbas;

class sbas_l1_telemetry_decoder_gs;

using sbas_l1_telemetry_decoder_gs_sptr = gnss_shared_ptr<sbas_l1_telemetry_decoder_gs>;

sbas_l1_telemetry_decoder_gs_sptr sbas_l1_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    bool dump);

/*!
 * \brief This class implements a block that decodes the SBAS integrity and
 * corrections data defined in RTCA MOPS DO-229
 */
class sbas_l1_telemetry_decoder_gs : public gr::block
{
public:
    ~sbas_l1_telemetry_decoder_gs() override;
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int32_t channel);                    //!< Set receiver's channel
    inline void reset(){};

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) override;

private:
    friend sbas_l1_telemetry_decoder_gs_sptr sbas_l1_make_telemetry_decoder_gs(
        const Gnss_Satellite &satellite,
        bool dump);

    sbas_l1_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump);

    void viterbi_decoder(double *page_part_symbols, int32_t *page_part_bits);
    void align_samples();

    static const int32_t D_SAMPLES_PER_SYMBOL = 2;
    static const int32_t D_SYMBOLS_PER_BIT = 2;
    static const int32_t D_BLOCK_SIZE_IN_BITS = 30;

    bool d_dump;
    Gnss_Satellite d_satellite;
    int32_t d_channel;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    size_t d_block_size;               //!< number of samples which are processed during one invocation of the algorithms
    std::vector<double> d_sample_buf;  //!< input buffer holding the samples to be processed in one block

    typedef std::pair<int32_t, std::vector<int32_t>> msg_candiate_int_t;
    typedef std::pair<int32_t, std::vector<uint8_t>> msg_candiate_char_t;

    // helper class for sample alignment
    class Sample_Aligner
    {
    public:
        Sample_Aligner();
        void reset();
        /*
         * samples length must be a multiple of two
         * for block operation
         */
        bool get_symbols(const std::vector<double> &samples, std::vector<double> &symbols);

    private:
        int32_t d_n_smpls_in_history{3};
        double d_iir_par{0.05};
        double d_corr_paired{};
        double d_corr_shifted{};
        bool d_aligned{};
        double d_past_sample{};
    } d_sample_aligner;

    // helper class for symbol alignment and Viterbi decoding
    class Symbol_Aligner_And_Decoder
    {
    public:
        Symbol_Aligner_And_Decoder();
        void reset();
        bool get_bits(const std::vector<double> &symbols, std::vector<int32_t> &bits);

    private:
        int32_t d_KK{7};
        std::shared_ptr<Viterbi_Decoder_Sbas> d_vd1;
        std::shared_ptr<Viterbi_Decoder_Sbas> d_vd2;
        double d_past_symbol{0};
    } d_symbol_aligner_and_decoder;


    // helper class for detecting the preamble and collect the corresponding message candidates
    class Frame_Detector
    {
    public:
        void reset();
        void get_frame_candidates(const std::vector<int32_t> &bits, std::vector<std::pair<int32_t, std::vector<int32_t>>> &msg_candidates);

    private:
        std::deque<int32_t> d_buffer;
    } d_frame_detector;


    // helper class for checking the CRC of the message candidates
    class Crc_Verifier
    {
    public:
        void reset();
        void get_valid_frames(const std::vector<msg_candiate_int_t> &msg_candidates, std::vector<msg_candiate_char_t> &valid_msgs);

    private:
        typedef boost::crc_optimal<24, 0x1864CFBU, 0x0, 0x0, false, false> crc_24_q_type;
        crc_24_q_type d_checksum_agent;
        void zerropad_front_and_convert_to_bytes(const std::vector<int32_t> &msg_candidate, std::vector<uint8_t> &bytes);
        void zerropad_back_and_convert_to_bytes(const std::vector<int32_t> &msg_candidate, std::vector<uint8_t> &bytes);
    } d_crc_verifier;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SBAS_L1_TELEMETRY_DECODER_GS_H
