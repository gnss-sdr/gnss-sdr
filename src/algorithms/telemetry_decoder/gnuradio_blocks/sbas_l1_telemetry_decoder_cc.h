/*!
 * \file sbas_l1_telemetry_decoder_cc.h
 * \brief Interface of a SBAS telemetry data decoder block
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
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

#ifndef GNSS_SDR_SBAS_L1_TELEMETRY_DECODER_CC_H
#define GNSS_SDR_SBAS_L1_TELEMETRY_DECODER_CC_H

#include "gnss_satellite.h"
#include "viterbi_decoder.h"
#include <boost/crc.hpp>
#include <gnuradio/block.h>
#include <algorithm> // for copy
#include <deque>
#include <fstream>
#include <string>
#include <utility> // for pair
#include <vector>


class sbas_l1_telemetry_decoder_cc;

typedef boost::shared_ptr<sbas_l1_telemetry_decoder_cc> sbas_l1_telemetry_decoder_cc_sptr;

sbas_l1_telemetry_decoder_cc_sptr
sbas_l1_make_telemetry_decoder_cc(const Gnss_Satellite & satellite, bool dump);

/*!
 * \brief This class implements a block that decodes the SBAS integrity and corrections data defined in RTCA MOPS DO-229
 *
 */
class sbas_l1_telemetry_decoder_cc : public gr::block
{
public:
    ~sbas_l1_telemetry_decoder_cc();
    void set_satellite(const Gnss_Satellite & satellite);  //!< Set satellite PRN
    void set_channel(int channel);                         //!< Set receiver's channel

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend sbas_l1_telemetry_decoder_cc_sptr
    sbas_l1_make_telemetry_decoder_cc(const Gnss_Satellite & satellite, bool dump);
    sbas_l1_telemetry_decoder_cc(const Gnss_Satellite & satellite, bool dump);

    void viterbi_decoder(double *page_part_symbols, int *page_part_bits);
    void align_samples();

    static const int d_samples_per_symbol = 2;
    static const int d_symbols_per_bit = 2;
    static const int d_block_size_in_bits = 30;

    bool d_dump;
    Gnss_Satellite d_satellite;
    int d_channel;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    size_t d_block_size; //!< number of samples which are processed during one invocation of the algorithms
    std::vector<double> d_sample_buf; //!< input buffer holding the samples to be processed in one block

    typedef std::pair<int,std::vector<int>> msg_candiate_int_t;
    typedef std::pair<int,std::vector<unsigned char>> msg_candiate_char_t;

    // helper class for sample alignment
    class sample_aligner
    {
    public:
        sample_aligner();
        ~sample_aligner();
        void reset();
        /*
         * samples length must be a multiple of two
         * for block operation
         */
       bool get_symbols(const std::vector<double> samples, std::vector<double> &symbols);
    private:
        int d_n_smpls_in_history ;
        double d_iir_par;
        double d_corr_paired;
        double d_corr_shifted;
        bool d_aligned;
        double d_past_sample;
    } d_sample_aligner;

    // helper class for symbol alignment and Viterbi decoding
    class symbol_aligner_and_decoder
    {
    public:
        symbol_aligner_and_decoder();
        ~symbol_aligner_and_decoder();
        void reset();
        bool get_bits(const std::vector<double> symbols, std::vector<int> &bits);
    private:
        int d_KK;
        Viterbi_Decoder * d_vd1;
        Viterbi_Decoder * d_vd2;
        double d_past_symbol;
    } d_symbol_aligner_and_decoder;


    // helper class for detecting the preamble and collect the corresponding message candidates
    class frame_detector
    {
    public:
        void reset();
        void get_frame_candidates(const std::vector<int> bits, std::vector<std::pair<int, std::vector<int>>> &msg_candidates);
    private:
        std::deque<int> d_buffer;
    } d_frame_detector;


    // helper class for checking the CRC of the message candidates
    class crc_verifier
    {
    public:
        void reset();
        void get_valid_frames(const std::vector<msg_candiate_int_t> msg_candidates, std::vector<msg_candiate_char_t> &valid_msgs);
    private:
        typedef boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> crc_24_q_type;
        crc_24_q_type d_checksum_agent;
        void zerropad_front_and_convert_to_bytes(const std::vector<int> msg_candidate, std::vector<unsigned char> &bytes);
        void zerropad_back_and_convert_to_bytes(const std::vector<int> msg_candidate, std::vector<unsigned char> &bytes);
    } d_crc_verifier;

};

#endif
