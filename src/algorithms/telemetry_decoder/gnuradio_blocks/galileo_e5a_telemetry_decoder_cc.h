/*!
 * \file galileo_e5a_telemetry_decoder_cc.cc
 * \brief Implementation of a Galileo FNAV message demodulator block
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 *  		   Javier Arribas, 2017. jarribas(at)cttc.es
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          </ul>
 *
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

#ifndef GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_CC_H_
#define GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_CC_H_

#include "Galileo_E5a.h"
#include "gnss_satellite.h"
#include "galileo_fnav_message.h"
#include "galileo_ephemeris.h"
#include "galileo_almanac.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "gnss_synchro.h"
#include <gnuradio/block.h>
#include <deque>
#include <fstream>
#include <string>


class galileo_e5a_telemetry_decoder_cc;

typedef boost::shared_ptr<galileo_e5a_telemetry_decoder_cc> galileo_e5a_telemetry_decoder_cc_sptr;

galileo_e5a_telemetry_decoder_cc_sptr galileo_e5a_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);


/*!
 * \brief This class implements a block that decodes the FNAV data defined in Galileo ICD
 *
 */
class galileo_e5a_telemetry_decoder_cc : public gr::block
{
public:
    ~galileo_e5a_telemetry_decoder_cc();
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int channel);                        //!< Set receiver's channel
    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend galileo_e5a_telemetry_decoder_cc_sptr
    galileo_e5a_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);
    galileo_e5a_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

    void viterbi_decoder(double *page_part_symbols, int *page_part_bits);

    void deinterleaver(int rows, int cols, double *in, double *out);

    void decode_word(double *page_symbols, int frame_length);

    int d_preambles_bits[GALILEO_FNAV_PREAMBLE_LENGTH_BITS];
    int d_preamble_samples[GALILEO_FNAV_CODES_PER_PREAMBLE];
    std::deque<int> d_preamble_init;
    int d_stat;
    int d_CRC_error_counter;
    int d_channel;
    int d_symbol_counter;
    int corr_value;
    unsigned int required_symbols;
    long unsigned int d_sample_counter;
    long unsigned int d_preamble_index;
    bool d_flag_frame_sync;
    bool d_flag_preamble;
    bool d_dump;
    bool flag_TOW_set;
    bool flag_bit_start;
    bool new_symbol;
    double d_prompt_acum;
    double page_symbols[GALILEO_FNAV_SYMBOLS_PER_PAGE - GALILEO_FNAV_PREAMBLE_LENGTH_BITS];
    double d_TOW_at_current_symbol;
    double delta_t;  //GPS-GALILEO time offset
    std::string d_dump_filename;
    std::ofstream d_dump_file;
    std::deque<Gnss_Synchro> d_symbol_history;
    Gnss_Satellite d_satellite;
    // navigation message vars
    Galileo_Fnav_Message d_nav;

    // vars for Viterbi decoder
    int *out0, *out1, *state0, *state1;
    int g_encoder[2];
    const int nn = 2;  // Coding rate 1/n
    const int KK = 7;  // Constraint Length
    int mm = KK - 1;
    const int CodeLength = 488;
    int DataLength = (CodeLength / nn) - mm;
};

#endif /* GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_CC_H_ */
