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


#include "gnss_satellite.h"
#include "gps_cnav_navigation_message.h"
#include "gps_cnav_ephemeris.h"
#include "gps_cnav_iono.h"
#include <gnuradio/block.h>
#include <algorithm> // for copy
#include <deque>
#include <fstream>
#include <string>
#include <utility> // for pair
#include <vector>


extern "C" {
    #include "cnav_msg.h"
    #include "edc.h"
    #include "bits.h"
}

#include "GPS_L2C.h"

class gps_l2c_telemetry_decoder_cc;

typedef boost::shared_ptr<gps_l2c_telemetry_decoder_cc> gps_l2c_telemetry_decoder_cc_sptr;

gps_l2c_telemetry_decoder_cc_sptr
gps_l2c_make_telemetry_decoder_cc(const Gnss_Satellite & satellite, bool dump);

/*!
 * \brief This class implements a block that decodes the SBAS integrity and corrections data defined in RTCA MOPS DO-229
 *
 */
class gps_l2c_telemetry_decoder_cc : public gr::block
{
public:
    ~gps_l2c_telemetry_decoder_cc();
    void set_satellite(const Gnss_Satellite & satellite);  //!< Set satellite PRN
    void set_channel(int channel);                         //!< Set receiver's channel

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);


private:
    friend gps_l2c_telemetry_decoder_cc_sptr
    gps_l2c_make_telemetry_decoder_cc(const Gnss_Satellite & satellite, bool dump);
    gps_l2c_telemetry_decoder_cc(const Gnss_Satellite & satellite, bool dump);

    bool d_dump;
    Gnss_Satellite d_satellite;
    int d_channel;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    cnav_msg_decoder_t d_cnav_decoder;

    int d_state;
    int d_crc_error_count;

    double d_TOW_at_current_symbol;
    double d_TOW_at_Preamble;
    bool d_flag_valid_word;

    Gps_CNAV_Navigation_Message d_CNAV_Message;
};


#endif
