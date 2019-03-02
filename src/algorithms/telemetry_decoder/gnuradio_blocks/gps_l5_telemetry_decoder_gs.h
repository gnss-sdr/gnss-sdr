/*!
 * \file gps_l5_telemetry_decoder_gs.h
 * \brief Interface of a CNAV message demodulator block
 * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L5_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_GPS_L5_TELEMETRY_DECODER_GS_H


#include "GPS_L5.h"
#include "gnss_satellite.h"
#include "gps_cnav_navigation_message.h"
#include <boost/circular_buffer.hpp>
#include <gnuradio/block.h>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

extern "C"
{
#include "bits.h"
#include "cnav_msg.h"
#include "edc.h"
}


class gps_l5_telemetry_decoder_gs;

using gps_l5_telemetry_decoder_gs_sptr = boost::shared_ptr<gps_l5_telemetry_decoder_gs>;

gps_l5_telemetry_decoder_gs_sptr
gps_l5_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump);

/*!
 * \brief This class implements a GPS L5 Telemetry decoder
 *
 */
class gps_l5_telemetry_decoder_gs : public gr::block
{
public:
    ~gps_l5_telemetry_decoder_gs();
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int32_t channel);                    //!< Set receiver's channel
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend gps_l5_telemetry_decoder_gs_sptr
    gps_l5_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump);
    gps_l5_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump);

    bool d_dump;
    Gnss_Satellite d_satellite;
    int32_t d_channel;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    cnav_msg_decoder_t d_cnav_decoder{};

    uint32_t d_TOW_at_current_symbol_ms;
    uint32_t d_TOW_at_Preamble_ms;
    bool d_flag_valid_word;

    Gps_CNAV_Navigation_Message d_CNAV_Message;
    float bits_NH[GPS_L5I_NH_CODE_LENGTH]{};
    boost::circular_buffer<float> sym_hist;
    bool sync_NH;
    bool new_sym;
};


#endif
