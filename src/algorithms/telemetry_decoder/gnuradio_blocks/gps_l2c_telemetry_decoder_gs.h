/*!
 * \file gps_l2c_telemetry_decoder_gs.h
 * \brief Interface of a CNAV message demodulator block
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_L2C_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_GPS_L2C_TELEMETRY_DECODER_GS_H


#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include "gps_cnav_navigation_message.h"
#include "tlm_conf.h"
#include <gnuradio/block.h>
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#include <cstdint>
#include <fstream>
#include <string>

extern "C"
{
#include "cnav_msg.h"
}

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks
 * \{ */


class gps_l2c_telemetry_decoder_gs;

using gps_l2c_telemetry_decoder_gs_sptr = gnss_shared_ptr<gps_l2c_telemetry_decoder_gs>;

gps_l2c_telemetry_decoder_gs_sptr gps_l2c_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf);

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
        const Tlm_Conf &conf);

    gps_l2c_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf);

    Gnss_Satellite d_satellite;

    cnav_msg_decoder_t d_cnav_decoder{};

    Gps_CNAV_Navigation_Message d_CNAV_Message;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    double d_TOW_at_current_symbol;
    double d_TOW_at_Preamble;

    uint64_t d_sample_counter;
    uint64_t d_last_valid_preamble;

    int32_t d_channel;
    int32_t d_state;
    int32_t d_crc_error_count;

    uint32_t d_max_symbols_without_valid_frame;

    bool d_dump;
    bool d_sent_tlm_failed_msg;
    bool d_flag_PLL_180_deg_phase_locked;
    bool d_flag_valid_word;
    bool d_dump_mat;
    bool d_remove_dat;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L2C_TELEMETRY_DECODER_GS_H
