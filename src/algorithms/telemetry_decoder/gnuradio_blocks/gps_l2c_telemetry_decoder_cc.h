/*!
 * \file gps_l2c_telemetry_decoder_cc.h
 * \brief Interface of a CNAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L2C_TELEMETRY_DECODER_CC_H
#define GNSS_SDR_GPS_L2C_TELEMETRY_DECODER_CC_H

#include <algorithm> // for copy
#include <deque>
#include <fstream>
#include <string>
#include <utility> // for pair
#include <vector>
#include <boost/crc.hpp>
#include <gnuradio/block.h>
#include "gnss_satellite.h"
#include "viterbi_decoder.h"
#include "gps_cnav_navigation_message.h"
#include "gps_cnav_ephemeris.h"
#include "gps_cnav_iono.h"
#include "concurrent_queue.h"
#include "GPS_L2C.h"

class gps_l2c_telemetry_decoder_cc;

typedef boost::shared_ptr<gps_l2c_telemetry_decoder_cc> gps_l2c_telemetry_decoder_cc_sptr;

gps_l2c_telemetry_decoder_cc_sptr
gps_l2c_make_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump);

/*!
 * \brief This class implements a block that decodes the SBAS integrity and corrections data defined in RTCA MOPS DO-229
 *
 */
class gps_l2c_telemetry_decoder_cc : public gr::block
{
public:
    ~gps_l2c_telemetry_decoder_cc();
    void set_satellite(Gnss_Satellite satellite);  //!< Set satellite PRN
    void set_channel(int channel);                 //!< Set receiver's channel
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
    friend gps_l2c_telemetry_decoder_cc_sptr
    gps_l2c_make_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump);
    gps_l2c_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump);

    void viterbi_decoder(double *page_part_symbols, int *page_part_bits);
    void align_samples();

    bool d_dump;
    Gnss_Satellite d_satellite;
    int d_channel;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    double d_TOW_at_current_symbol;
    double d_TOW_at_Preamble;
    bool d_flag_valid_word;

    bool d_flag_invert_input_symbols;
    bool d_flag_invert_buffer_symbols;
    int d_decimation_output_factor;
    int d_average_count;
    size_t d_block_size; //!< number of samples which are processed during one invocation of the algorithms
    std::vector<double> d_sample_buf; //!< input buffer holding the samples to be processed in one block

    typedef std::pair<int,std::vector<int>> msg_candiate_int_t;
    typedef std::pair<int,std::vector<unsigned char>> msg_candiate_char_t;

    // helper class for symbol alignment and Viterbi decoding
    class symbol_aligner_and_decoder
    {
    public:
        symbol_aligner_and_decoder();
        ~symbol_aligner_and_decoder();
        void reset();
        bool get_bits(const std::vector<double> & symbols, std::vector<int> & bits);
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
        void get_frame_candidates(const std::vector<int> & bits, std::vector<std::pair<int, std::vector<int>>> & msg_candidates);
    private:
        std::deque<int> d_buffer;
    } d_frame_detector;


    // helper class for checking the CRC of the message candidates
    class crc_verifier
    {
    public:
        void reset();
        void get_valid_frames(const std::vector<msg_candiate_int_t> & msg_candidates, std::vector<msg_candiate_int_t> & valid_msgs);
    private:
        typedef boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> crc_24_q_type;
        crc_24_q_type d_checksum_agent;
        void zerropad_front_and_convert_to_bytes(const std::vector<int> & msg_candidate, std::vector<unsigned char> & bytes);
        void zerropad_back_and_convert_to_bytes(const std::vector<int> & msg_candidate, std::vector<unsigned char> & bytes);
    } d_crc_verifier;

    Gps_CNAV_Navigation_Message d_CNAV_Message;
};


#endif
