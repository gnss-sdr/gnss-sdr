/*!
 * \file galileo_e1b_telemetry_decoder_cc.h
 * \brief Interface of a Galileo INAV message demodulator block
 * \author Javier Arribas 2013 jarribas(at)cttc.es,
 *         Mara Branzanti 2013 mara.branzanti(at)gmail.com
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

#ifndef GNSS_SDR_GALILEO_E1B_TELEMETRY_DECODER_CC_H
#define GNSS_SDR_GALILEO_E1B_TELEMETRY_DECODER_CC_H

#include <fstream>
#include <string>
#include <gnuradio/block.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/trellis/interleaver.h>
#include <gnuradio/trellis/permutation.h>
#include <gnuradio/fec/viterbi.h>
#include "Galileo_E1.h"
#include "concurrent_queue.h"
#include "gnss_satellite.h"
#include "galileo_navigation_message.h"
#include "galileo_ephemeris.h"
#include "galileo_almanac.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"



class galileo_e1b_telemetry_decoder_cc;

typedef boost::shared_ptr<galileo_e1b_telemetry_decoder_cc> galileo_e1b_telemetry_decoder_cc_sptr;

galileo_e1b_telemetry_decoder_cc_sptr galileo_e1b_make_telemetry_decoder_cc(Gnss_Satellite satellite, long if_freq, long fs_in, unsigned
    int vector_length, boost::shared_ptr<gr::msg_queue> queue, bool dump);

/*!
 * \brief This class implements a block that decodes the INAV data defined in Galileo ICD
 *
 */
class galileo_e1b_telemetry_decoder_cc : public gr::block
{
public:
    ~galileo_e1b_telemetry_decoder_cc();
    void set_satellite(Gnss_Satellite satellite);  //!< Set satellite PRN
    void set_channel(int channel);                 //!< Set receiver's channel
    int flag_even_word_arrived;
    void set_ephemeris_queue(concurrent_queue<Galileo_Ephemeris> *ephemeris_queue); //!< Set the satellite data queue
    void set_iono_queue(concurrent_queue<Galileo_Iono> *iono_queue);                //!< Set the iono data queue
    void set_almanac_queue(concurrent_queue<Galileo_Almanac> *almanac_queue);       //!< Set the almanac data queue
    void set_utc_model_queue(concurrent_queue<Galileo_Utc_Model> *utc_model_queue); //!< Set the UTC model queue

    /*!
     * \brief Set decimation factor to average the GPS synchronization estimation output from the tracking module.
     */
    void set_decimation(int decimation);

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
    friend galileo_e1b_telemetry_decoder_cc_sptr
    galileo_e1b_make_telemetry_decoder_cc(Gnss_Satellite satellite, long if_freq, long fs_in,unsigned
            int vector_length, boost::shared_ptr<gr::msg_queue> queue, bool dump);
    galileo_e1b_telemetry_decoder_cc(Gnss_Satellite satellite, long if_freq, long fs_in, unsigned
            int vector_length, boost::shared_ptr<gr::msg_queue> queue, bool dump);

    void viterbi_decoder(double *page_part_symbols, int *page_part_bits);

    void deinterleaver(int rows, int cols, double *in, double *out);

    void decode_word(double *symbols,int frame_length);

    unsigned short int d_preambles_bits[GALILEO_INAV_PREAMBLE_LENGTH_BITS];

    signed int *d_preambles_symbols;
    unsigned int d_samples_per_symbol;
    int d_symbols_per_preamble;

    long unsigned int d_sample_counter;
    long unsigned int d_preamble_index;
    unsigned int d_stat;
    bool d_flag_frame_sync;

    bool d_flag_parity;
    bool d_flag_preamble;
    int d_CRC_error_counter;

    long d_fs_in;

    // navigation message vars
    Galileo_Navigation_Message d_nav;

    // Galileo ephemeris queue
    concurrent_queue<Galileo_Ephemeris> *d_ephemeris_queue;
    // ionospheric parameters queue
    concurrent_queue<Galileo_Iono> *d_iono_queue;
    // UTC model parameters queue
    concurrent_queue<Galileo_Utc_Model> *d_utc_model_queue;
    // Almanac queue
    concurrent_queue<Galileo_Almanac> *d_almanac_queue;

    boost::shared_ptr<gr::msg_queue> d_queue;
    unsigned int d_vector_length;
    bool d_dump;
    Gnss_Satellite d_satellite;
    int d_channel;

    // output averaging and decimation
    int d_average_count;
    int d_decimation_output_factor;

    double d_preamble_time_seconds;

    double d_TOW_at_Preamble;
    double d_TOW_at_current_symbol;

    double Prn_timestamp_at_preamble_ms;
    bool flag_TOW_set;
    double delta_t; //GPS-GALILEO time offset


    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
