/*!
 * \file galileo_telemetry_decoder_gs.h
 * \brief Implementation of a Galileo unified INAV and FNAV message demodulator block
 * \author Javier Arribas 2018. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GALILEO_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_GALILEO_TELEMETRY_DECODER_GS_H


#include "galileo_fnav_message.h"
#include "galileo_navigation_message.h"
#include "gnss_satellite.h"
#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>  // for boost::shared_ptr
#include <gnuradio/block.h>      // for block
#include <gnuradio/types.h>      // for gr_vector_const_void_star
#include <array>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class galileo_telemetry_decoder_gs;

using galileo_telemetry_decoder_gs_sptr = boost::shared_ptr<galileo_telemetry_decoder_gs>;

galileo_telemetry_decoder_gs_sptr galileo_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    int frame_type,
    bool dump);

/*!
 * \brief This class implements a block that decodes the INAV and FNAV data defined in Galileo ICD
 */
class galileo_telemetry_decoder_gs : public gr::block
{
public:
    ~galileo_telemetry_decoder_gs();
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int32_t channel);                    //!< Set receiver's channel
    void reset();
    int32_t flag_even_word_arrived;

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend galileo_telemetry_decoder_gs_sptr galileo_make_telemetry_decoder_gs(
        const Gnss_Satellite &satellite,
        int frame_type,
        bool dump);

    galileo_telemetry_decoder_gs(const Gnss_Satellite &satellite, int frame_type, bool dump);

    void viterbi_decoder(double *page_part_symbols, int32_t *page_part_bits);

    void deinterleaver(int32_t rows, int32_t cols, const double *in, double *out);

    void decode_INAV_word(double *page_part_symbols, int32_t frame_length);
    void decode_FNAV_word(double *page_symbols, int32_t frame_length);

    int d_frame_type;
    int32_t d_bits_per_preamble;
    int32_t d_samples_per_preamble;
    int32_t d_preamble_period_symbols;
    std::vector<int32_t> d_preamble_samples;
    uint32_t d_PRN_code_period_ms;
    uint32_t d_required_symbols;
    uint32_t d_frame_length_symbols;
    std::vector<double> d_page_part_symbols;

    boost::circular_buffer<float> d_symbol_history;

    uint64_t d_sample_counter;
    uint64_t d_preamble_index;
    uint64_t d_last_valid_preamble;
    uint32_t d_max_symbols_without_valid_frame;

    bool d_sent_tlm_failed_msg;
    uint32_t d_stat;
    bool d_flag_frame_sync;
    bool flag_PLL_180_deg_phase_locked;

    bool d_flag_parity;
    bool d_flag_preamble;
    int32_t d_CRC_error_counter;

    // navigation message vars
    Galileo_Navigation_Message d_inav_nav;
    Galileo_Fnav_Message d_fnav_nav;

    bool d_dump;
    Gnss_Satellite d_satellite;
    int32_t d_channel;

    uint32_t d_TOW_at_Preamble_ms;
    uint32_t d_TOW_at_current_symbol_ms;

    bool flag_TOW_set;
    double delta_t;  // GPS-GALILEO time offset

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    // vars for Viterbi decoder
    std::vector<int32_t> out0;
    std::vector<int32_t> out1;
    std::vector<int32_t> state0;
    std::vector<int32_t> state1;
    std::array<int32_t, 2> g_encoder{};
    const int32_t nn = 2;  // Coding rate 1/n
    const int32_t KK = 7;  // Constraint Length
    int32_t mm = KK - 1;
    int32_t CodeLength;
    int32_t DataLength;
};

#endif
