/*!
 * \file galileo_e1b_telemetry_decoder_cc.h
 * \brief Interface of a Galileo INAV message demodulator block
 * \author Javier Arribas 2013 jarribas(at)cttc.es,
 *         Mara Branzanti 2013 mara.branzanti(at)gmail.com
 *
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

#ifndef GNSS_SDR_GALILEO_E1B_TELEMETRY_DECODER_CC_H
#define GNSS_SDR_GALILEO_E1B_TELEMETRY_DECODER_CC_H


#include "Galileo_E1.h"
#include "galileo_navigation_message.h"
#include "galileo_ephemeris.h"
#include "galileo_almanac.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include <gnuradio/block.h>
#include <fstream>
#include <string>


class galileo_e1b_telemetry_decoder_cc;

typedef boost::shared_ptr<galileo_e1b_telemetry_decoder_cc> galileo_e1b_telemetry_decoder_cc_sptr;

galileo_e1b_telemetry_decoder_cc_sptr galileo_e1b_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

/*!
 * \brief This class implements a block that decodes the INAV data defined in Galileo ICD
 *
 */
class galileo_e1b_telemetry_decoder_cc : public gr::block
{
public:
    ~galileo_e1b_telemetry_decoder_cc();
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int channel);                        //!< Set receiver's channel
    int flag_even_word_arrived;

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend galileo_e1b_telemetry_decoder_cc_sptr
    galileo_e1b_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);
    galileo_e1b_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

    void viterbi_decoder(double *page_part_symbols, int *page_part_bits);

    void deinterleaver(int rows, int cols, double *in, double *out);

    void decode_word(double *symbols, int frame_length);

    unsigned short int d_preambles_bits[GALILEO_INAV_PREAMBLE_LENGTH_BITS];

    int *d_preambles_symbols;
    unsigned int d_samples_per_symbol;
    int d_symbols_per_preamble;

    std::deque<Gnss_Synchro> d_symbol_history;

    long unsigned int d_sample_counter;
    long unsigned int d_preamble_index;
    unsigned int d_stat;
    bool d_flag_frame_sync;

    bool d_flag_parity;
    bool d_flag_preamble;
    int d_CRC_error_counter;

    // navigation message vars
    Galileo_Navigation_Message d_nav;

    bool d_dump;
    Gnss_Satellite d_satellite;
    int d_channel;

    unsigned int d_TOW_at_Preamble_ms;
    unsigned int d_TOW_at_current_symbol_ms;

    bool flag_TOW_set;
    double delta_t;  //GPS-GALILEO time offset

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    // vars for Viterbi decoder
    int *out0, *out1, *state0, *state1;
    int g_encoder[2];
    const int nn = 2;  // Coding rate 1/n
    const int KK = 7;  // Constraint Length
    int mm = KK - 1;
    const int CodeLength = 240;
    int DataLength = (CodeLength / nn) - mm;
};

#endif
