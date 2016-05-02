/*!
 * \file galileo_e5a_telemetry_decoder_cc.cc
 * \brief Implementation of a Galileo FNAV message demodulator block
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          </ul>
 *
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

#ifndef GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_CC_H_
#define GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_CC_H_

#include <fstream>
#include <string>
#include <gnuradio/block.h>
#include "Galileo_E5a.h"
#include "concurrent_queue.h"
#include "gnss_satellite.h"
#include "galileo_fnav_message.h"
#include "galileo_ephemeris.h"
#include "galileo_almanac.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"

//#include "convolutional.h"

class galileo_e5a_telemetry_decoder_cc;

typedef boost::shared_ptr<galileo_e5a_telemetry_decoder_cc> galileo_e5a_telemetry_decoder_cc_sptr;

galileo_e5a_telemetry_decoder_cc_sptr galileo_e5a_make_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump);


/*!
 * \brief This class implements a block that decodes the FNAV data defined in Galileo ICD
 *
 */
class galileo_e5a_telemetry_decoder_cc : public gr::block
{
public:
    ~galileo_e5a_telemetry_decoder_cc();
    void set_satellite(Gnss_Satellite satellite);  //!< Set satellite PRN
    void set_channel(int channel);                 //!< Set receiver's channel
    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    /*!
     * \brief Function which tells the scheduler how many input items
     *        are required to produce noutput_items output items.
     */
    void forecast (int noutput_items, gr_vector_int &ninput_items_required);

private:
    friend galileo_e5a_telemetry_decoder_cc_sptr
    galileo_e5a_make_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump);
    galileo_e5a_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump);

    void viterbi_decoder(double *page_part_symbols, int *page_part_bits);

    void deinterleaver(int rows, int cols, double *in, double *out);

    void decode_word(double *page_symbols,int frame_length);

    int d_preamble_bits[GALILEO_FNAV_PREAMBLE_LENGTH_BITS];
    // signed int d_page_symbols[GALILEO_FNAV_SYMBOLS_PER_PAGE + GALILEO_FNAV_PREAMBLE_LENGTH_BITS];
    double d_page_symbols[GALILEO_FNAV_SYMBOLS_PER_PAGE + GALILEO_FNAV_PREAMBLE_LENGTH_BITS];
    // signed int *d_preamble_symbols;
    double d_current_symbol;
    long unsigned int d_symbol_counter;
    int d_prompt_counter;
    int d_sign_init;

    long unsigned int d_sample_counter;
    long unsigned int d_preamble_index;

    bool d_preamble_lock;
    bool d_flag_frame_sync;
    int d_state;

    bool d_flag_preamble;
    int d_CRC_error_counter;

    // navigation message vars
    Galileo_Fnav_Message d_nav;

    bool d_dump;
    Gnss_Satellite d_satellite;
    int d_channel;

    double d_preamble_time_seconds;

    double d_TOW_at_Preamble;
    double d_TOW_at_current_symbol;
    double Prn_timestamp_at_preamble_ms;
    bool flag_TOW_set;

    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif /* GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_CC_H_ */
