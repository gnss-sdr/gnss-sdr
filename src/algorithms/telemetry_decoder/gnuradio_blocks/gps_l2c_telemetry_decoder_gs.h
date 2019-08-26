/*!
 * \file gps_l2c_telemetry_decoder_gs.h
 * \brief Interface of a CNAV message demodulator block
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GPS_L2C_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_GPS_L2C_TELEMETRY_DECODER_GS_H


#include "gnss_satellite.h"
#include "gps_cnav_navigation_message.h"
#include <boost/shared_ptr.hpp>  // for boost::shared_ptr
#include <gnuradio/block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#include <cstdint>
#include <fstream>
#include <string>

extern "C"
{
#include "cnav_msg.h"
}


class gps_l2c_telemetry_decoder_gs;

using gps_l2c_telemetry_decoder_gs_sptr = boost::shared_ptr<gps_l2c_telemetry_decoder_gs>;

gps_l2c_telemetry_decoder_gs_sptr gps_l2c_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    bool dump);

/*!
 * \brief This class implements a block that decodes CNAV data defined in IS-GPS-200K
 */
class gps_l2c_telemetry_decoder_gs : public gr::block
{
public:
    ~gps_l2c_telemetry_decoder_gs();
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int32_t channel);                    //!< Set receiver's channel
    void reset();

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend gps_l2c_telemetry_decoder_gs_sptr gps_l2c_make_telemetry_decoder_gs(
        const Gnss_Satellite &satellite,
        bool dump);

    gps_l2c_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump);

    bool d_dump;
    Gnss_Satellite d_satellite;
    int32_t d_channel;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    cnav_msg_decoder_t d_cnav_decoder{};

    int32_t d_state;
    int32_t d_crc_error_count;
    uint64_t d_sample_counter;
    bool d_sent_tlm_failed_msg;
    uint64_t d_last_valid_preamble;
    uint32_t d_max_symbols_without_valid_frame;

    double d_TOW_at_current_symbol;
    double d_TOW_at_Preamble;
    bool d_flag_valid_word;

    Gps_CNAV_Navigation_Message d_CNAV_Message;
};


#endif
