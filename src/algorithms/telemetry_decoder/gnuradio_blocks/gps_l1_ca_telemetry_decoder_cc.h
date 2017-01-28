/*!
 * \file gps_l1_ca_telemetry_decoder_cc.h
 * \brief Interface of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L1_CA_TELEMETRY_DECODER_CC_H
#define GNSS_SDR_GPS_L1_CA_TELEMETRY_DECODER_CC_H

#include <fstream>
#include <string>
#include <gnuradio/block.h>
#include <deque>
#include "GPS_L1_CA.h"
#include "gps_l1_ca_subframe_fsm.h"
#include "concurrent_queue.h"
#include "gnss_satellite.h"



class gps_l1_ca_telemetry_decoder_cc;

typedef boost::shared_ptr<gps_l1_ca_telemetry_decoder_cc> gps_l1_ca_telemetry_decoder_cc_sptr;

gps_l1_ca_telemetry_decoder_cc_sptr
gps_l1_ca_make_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump);

/*!
 * \brief This class implements a block that decodes the NAV data defined in IS-GPS-200E
 *
 */
class gps_l1_ca_telemetry_decoder_cc : public gr::block
{
public:
    ~gps_l1_ca_telemetry_decoder_cc();
    void set_satellite(Gnss_Satellite satellite);  //!< Set satellite PRN
    void set_channel(int channel);                 //!< Set receiver's channel
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
    friend gps_l1_ca_telemetry_decoder_cc_sptr
    gps_l1_ca_make_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump);

    gps_l1_ca_telemetry_decoder_cc(Gnss_Satellite satellite, bool dump);

    bool gps_word_parityCheck(unsigned int gpsword);

    // constants
    //unsigned short int d_preambles_bits[GPS_CA_PREAMBLE_LENGTH_BITS];
    // class private vars

    int *d_preambles_symbols;
    unsigned int d_stat;
    bool d_flag_frame_sync;

    // symbols
    std::deque<double> d_symbol_history;
    std::deque<int> d_correlation_length_ms_history;
    double d_symbol_accumulator;
    short int d_symbol_accumulator_counter;

    //bits and frame
    unsigned short int d_frame_bit_index;
    unsigned int d_GPS_frame_4bytes;
    unsigned int d_prev_GPS_frame_4bytes;
    bool d_flag_parity;
    bool d_flag_preamble;
    int d_word_number;

    // output averaging and decimation
    int d_average_count;
    int d_decimation_output_factor;

    //double d_preamble_duration_seconds;
    // navigation message vars
    Gps_Navigation_Message d_nav;
    GpsL1CaSubframeFsm d_GPS_FSM;

    bool d_dump;
    Gnss_Satellite d_satellite;
    int d_channel;

    double d_preamble_time_seconds;

    long double d_TOW_at_Preamble;
    long double d_TOW_at_current_symbol;

    double Prn_timestamp_at_preamble_ms;
    bool flag_TOW_set;
    bool flag_PLL_180_deg_phase_locked;

    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
