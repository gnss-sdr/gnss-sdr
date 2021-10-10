/*!
 * \file glonass_l1_ca_telemetry_decoder_gs.h
 * \brief Implementation of a GLONASS L1 C/A NAV data decoder block
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.comK
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

#ifndef GNSS_SDR_GLONASS_L1_CA_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_GLONASS_L1_CA_TELEMETRY_DECODER_GS_H


#include "GLONASS_L1_L2_CA.h"
#include "glonass_gnav_navigation_message.h"
#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "nav_message_packet.h"
#include "tlm_conf.h"
#include "tlm_crc_stats.h"
#include <boost/circular_buffer.hpp>
#include <gnuradio/block.h>  // for block
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#include <array>
#include <cstdint>
#include <fstream>  // for ofstream
#include <memory>   // for std::unique_ptr
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks
 * \{ */


class glonass_l1_ca_telemetry_decoder_gs;

using glonass_l1_ca_telemetry_decoder_gs_sptr = gnss_shared_ptr<glonass_l1_ca_telemetry_decoder_gs>;

glonass_l1_ca_telemetry_decoder_gs_sptr glonass_l1_ca_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf);

/*!
 * \brief This class implements a block that decodes the GNAV data defined in GLONASS ICD v5.1
 * \note Code added as part of GSoC 2017 program
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 *
 */
class glonass_l1_ca_telemetry_decoder_gs : public gr::block
{
public:
    ~glonass_l1_ca_telemetry_decoder_gs() override;       //!< Class destructor
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int32_t channel);                    //!< Set receiver's channel
    inline void reset(){};

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) override;

private:
    friend glonass_l1_ca_telemetry_decoder_gs_sptr glonass_l1_ca_make_telemetry_decoder_gs(
        const Gnss_Satellite &satellite,
        const Tlm_Conf &conf);

    glonass_l1_ca_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf);

    const std::array<uint16_t, GLONASS_GNAV_PREAMBLE_LENGTH_BITS> d_preambles_bits{GLONASS_GNAV_PREAMBLE};

    const int32_t d_symbols_per_preamble = GLONASS_GNAV_PREAMBLE_LENGTH_SYMBOLS;

    void decode_string(const double *symbols, int32_t frame_length);

    // Help with coherent tracking

    // Preamble decoding
    std::array<int32_t, GLONASS_GNAV_PREAMBLE_LENGTH_SYMBOLS> d_preambles_symbols{};

    // Storage for incoming data
    boost::circular_buffer<Gnss_Synchro> d_symbol_history;

    // Navigation Message variable
    Glonass_Gnav_Navigation_Message d_nav;

    Gnss_Satellite d_satellite;

    Nav_Message_Packet d_nav_msg_packet;
    std::unique_ptr<Tlm_CRC_Stats> d_Tlm_CRC_Stats;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    double d_preamble_time_samples;
    double d_TOW_at_current_symbol;

    // Variables for internal functionality
    uint64_t d_sample_counter;  // Sample counter as an index (1,2,3,..etc) indicating number of samples processed
    uint64_t d_preamble_index;  // Index of sample number where preamble was found

    uint32_t d_stat;              // Status of decoder
    int32_t d_CRC_error_counter;  // Number of failed CRC operations
    int32_t d_channel;

    bool d_flag_frame_sync;  // Indicate when a frame sync is achieved
    bool d_flag_preamble;    // Flag indicating when preamble was found
    bool d_dump;
    bool d_dump_mat;
    bool d_remove_dat;
    bool d_enable_navdata_monitor;
    bool d_dump_crc_stats;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_L1_CA_TELEMETRY_DECODER_GS_H
