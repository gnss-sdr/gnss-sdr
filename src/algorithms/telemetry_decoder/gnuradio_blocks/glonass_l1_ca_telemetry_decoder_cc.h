/*!
 * \file glonass_l1_ca_telemetry_decoder_cc.h
 * \brief Implementation of an adapter of a GLONASS L1 C/A NAV data decoder block
 * to a TelemetryDecoderInterface
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
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

#ifndef GNSS_SDR_GLONASS_L1_CA_TELEMETRY_DECODER_CC_H
#define GNSS_SDR_GLONASS_L1_CA_TELEMETRY_DECODER_CC_H


#include "glonass_gnav_navigation_message.h"
#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_almanac.h"
#include "glonass_gnav_utc_model.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "GLONASS_L1_L2_CA.h"
#include <gnuradio/block.h>
#include <fstream>
#include <string>


class glonass_l1_ca_telemetry_decoder_cc;

typedef boost::shared_ptr<glonass_l1_ca_telemetry_decoder_cc> glonass_l1_ca_telemetry_decoder_cc_sptr;

glonass_l1_ca_telemetry_decoder_cc_sptr glonass_l1_ca_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

/*!
 * \brief This class implements a block that decodes the GNAV data defined in GLONASS ICD v5.1
 * \note Code added as part of GSoC 2017 program
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 *
 */
class glonass_l1_ca_telemetry_decoder_cc : public gr::block
{
public:
    ~glonass_l1_ca_telemetry_decoder_cc();                //!< Class destructor
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int32_t channel);                    //!< Set receiver's channel

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend glonass_l1_ca_telemetry_decoder_cc_sptr
    glonass_l1_ca_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);
    glonass_l1_ca_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

    void decode_string(double *symbols, int32_t frame_length);

    //!< Help with coherent tracking
    double d_preamble_time_samples;

    //!< Preamble decoding
    uint16_t d_preambles_bits[GLONASS_GNAV_PREAMBLE_LENGTH_BITS];
    int32_t *d_preambles_symbols;
    uint32_t d_samples_per_symbol;
    int32_t d_symbols_per_preamble;

    //!< Storage for incoming data
    std::deque<Gnss_Synchro> d_symbol_history;

    //!< Variables for internal functionality
    uint64_t d_sample_counter;    //!< Sample counter as an index (1,2,3,..etc) indicating number of samples processed
    uint64_t d_preamble_index;    //!< Index of sample number where preamble was found
    uint32_t d_stat;              //!< Status of decoder
    bool d_flag_frame_sync;       //!< Indicate when a frame sync is achieved
    bool d_flag_parity;           //!< Flag indicating when parity check was achieved (crc check)
    bool d_flag_preamble;         //!< Flag indicating when preamble was found
    int32_t d_CRC_error_counter;  //!< Number of failed CRC operations
    bool flag_TOW_set;            //!< Indicates when time of week is set
    double delta_t;               //!< GPS-GLONASS time offset

    //!< Navigation Message variable
    Glonass_Gnav_Navigation_Message d_nav;

    //!< Values to populate gnss synchronization structure
    double d_TOW_at_current_symbol;
    bool Flag_valid_word;

    //!< Satellite Information and logging capacity
    Gnss_Satellite d_satellite;
    int32_t d_channel;
    bool d_dump;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
