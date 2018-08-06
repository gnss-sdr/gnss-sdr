/*!
 * \file beidou_b1i_telemetry_decoder_cc.h
 * \brief Interface of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_BEIDOU_B1I_TELEMETRY_DECODER_CC_H
#define GNSS_SDR_BEIDOU_B1I_TELEMETRY_DECODER_CC_H

#include "beidou_b1I.h"
#include "beidou_b1i_subframe_fsm.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include <gnuradio/block.h>
#include <fstream>
#include <string>
#include <boost/circular_buffer.hpp>

class beidou_b1i_telemetry_decoder_cc;

typedef boost::shared_ptr<beidou_b1i_telemetry_decoder_cc> beidou_b1i_telemetry_decoder_cc_sptr;

beidou_b1i_telemetry_decoder_cc_sptr
beidou_b1i_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

/*!
 * \brief This class implements a block that decodes the NAV data defined in IS-GPS-200E
 *
 */
class beidou_b1i_telemetry_decoder_cc : public gr::block
{
public:
    ~beidou_b1i_telemetry_decoder_cc();
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int channel);                        
    void decode_word(int word_counter, boost::circular_buffer<signed int> *d_bit_buffer, unsigned int& d_BEIDOU_frame_4bytes);


    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend beidou_b1i_telemetry_decoder_cc_sptr
    beidou_b1i_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

    beidou_b1i_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

    bool beidou_word_parityCheck(unsigned int beidouword);

    // class private vars

    int *d_preambles_symbols;
    unsigned int d_stat;
    bool d_flag_frame_sync;

    // symbols
    boost::circular_buffer<signed int> d_symbol_history;
    boost::circular_buffer<signed int> d_symbol_nh_history;
    boost::circular_buffer<signed int> d_bit_buffer;

    double d_symbol_accumulator;
    short int d_symbol_accumulator_counter;

    // symbol counting
    bool d_make_correlation;
    unsigned int d_symbol_counter_corr;

    //bits and frame
    unsigned short int d_frame_bit_index;
    double bits_NH[BEIDOU_B1I_NH_CODE_LENGTH];
    unsigned int d_BEIDOU_frame_4bytes;
    unsigned int d_prev_BEIDOU_frame_4bytes;
    bool d_flag_parity;
    bool d_flag_preamble;
    bool d_flag_new_tow_available;
    int d_word_number;

    // navigation message vars
    Beidou_Navigation_Message_D1 d_nav;
    BeidouB1iSubframeFsm d_BEIDOU_FSM;

    bool d_dump;
    Gnss_Satellite d_satellite;
    int d_channel;

    unsigned long int d_preamble_time_samples;

    unsigned int d_TOW_at_Preamble_ms;
    unsigned int d_TOW_at_current_symbol_ms;
    unsigned int word_number;
    bool flag_TOW_set;
    bool flag_PLL_180_deg_phase_locked;

    std::string d_dump_filename;
    std::ofstream d_dump_file;
    bool sync_NH;
    bool new_sym;

};

#endif
