/*!
 * \file beidou_b3i_telemetry_decoder_cc.h
 * \brief Implementation of an adapter of a BEIDOU BI1 DNAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Damian Miralles, 2019. dmiralles2009(at)gmail.com
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

#ifndef GNSS_SDR_BEIDOU_B3I_TELEMETRY_DECODER_CC_H
#define GNSS_SDR_BEIDOU_B3I_TELEMETRY_DECODER_CC_H

#include "beidou_dnav_navigation_message.h"
#include "beidou_dnav_ephemeris.h"
#include "beidou_dnav_almanac.h"
#include "beidou_dnav_utc_model.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "Beidou_B3I.h"
#include "Beidou_DNAV.h"
#include <gnuradio/block.h>
#include <fstream>
#include <string>


class beidou_b3i_telemetry_decoder_cc;

typedef boost::shared_ptr<beidou_b3i_telemetry_decoder_cc> beidou_b3i_telemetry_decoder_cc_sptr;

beidou_b3i_telemetry_decoder_cc_sptr beidou_b3i_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

//!!!! edit
/*!
 * \brief This class implements a block that decodes the GNAV data defined in BEIDOU ICD v5.1
 * \note Code added as part of GSoC 2018 program
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 *
 */
class beidou_b3i_telemetry_decoder_cc : public gr::block
{
public:
    ~beidou_b3i_telemetry_decoder_cc();                //!< Class destructor
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int channel);                        //!< Set receiver's channel

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend beidou_b3i_telemetry_decoder_cc_sptr
	beidou_b3i_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);
    beidou_b3i_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

    void decode_subframe(double *symbols, int32_t frame_length);
    void decode_word(int32_t word_counter, double* enc_word_symbols, int32_t* dec_word_symbols);
    void decode_bch15_11_01(int32_t *bits, int32_t *decbits);


    //!< Preamble decoding
    unsigned short int d_preambles_symbols[BEIDOU_DNAV_PREAMBLE_LENGTH_SYMBOLS];
    int32_t 	*d_preamble_samples;
    int32_t 	*d_secondary_code_symbols;
    uint32_t 	d_samples_per_symbol;
    int32_t 	d_symbols_per_preamble;
    int32_t     d_samples_per_preamble;
    int32_t    d_preamble_period_samples;
    double 		*d_subframe_symbols;
    uint32_t 	d_subframe_length_symbols;
    uint32_t    d_required_symbols;

    //!< Storage for incoming data
    std::deque<float> d_symbol_history;

    //!< Variables for internal functionality
    uint64_t d_sample_counter;  //!< Sample counter as an index (1,2,3,..etc) indicating number of samples processed
    uint64_t d_preamble_index;  //!< Index of sample number where preamble was found
    uint32_t d_stat;                 //!< Status of decoder
    bool d_flag_frame_sync;              //!< Indicate when a frame sync is achieved
    bool d_flag_preamble;                //!< Flag indicating when preamble was found
    int32_t d_CRC_error_counter;             //!< Number of failed CRC operations
    bool flag_SOW_set;                   //!< Indicates when time of week is set

    //!< Navigation Message variable
    Beidou_Dnav_Navigation_Message d_nav;

    //!< Values to populate gnss synchronization structure
    uint32_t d_TOW_at_Preamble_ms;
    uint32_t d_TOW_at_current_symbol_ms;
    bool Flag_valid_word;

    //!< Satellite Information and logging capacity
    Gnss_Satellite d_satellite;
    int32_t d_channel;
    bool d_dump;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
