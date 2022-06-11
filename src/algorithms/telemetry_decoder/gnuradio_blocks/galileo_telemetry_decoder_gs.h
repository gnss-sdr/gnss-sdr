/*!
 * \file galileo_telemetry_decoder_gs.h
 * \brief Implementation of a Galileo unified INAV and FNAV message demodulator
 * block
 * \author Javier Arribas 2018. jarribas(at)cttc.es
 * \author Carles Fernandez, 2021-2022. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GALILEO_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_GALILEO_TELEMETRY_DECODER_GS_H

#include "galileo_cnav_message.h"     // for Galileo_Cnav_Message
#include "galileo_fnav_message.h"     // for Galileo_Fnav_Message
#include "galileo_inav_message.h"     // for Galileo_Inav_Message
#include "gnss_block_interface.h"     // for gnss_shared_ptr (adapts smart pointer type to GNU Radio version)
#include "gnss_satellite.h"           // for Gnss_Satellite
#include "gnss_time.h"                // for GnssTime
#include "nav_message_packet.h"       // for Nav_Message_Packet
#include "tlm_conf.h"                 // for Tlm_Conf
#include <boost/circular_buffer.hpp>  // for boost::circular_buffer
#include <gnuradio/block.h>           // for block
#include <gnuradio/types.h>           // for gr_vector_const_void_star
#include <pmt/pmt.h>                  // for pmt::pmt_t
#include <cstdint>                    // for int32_t, uint32_t
#include <fstream>                    // for std::ofstream
#include <memory>                     // for std::unique_ptr
#include <string>                     // for std::string
#include <vector>                     // for std::vector

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks
 * \{ */

class Viterbi_Decoder;               // forward declaration
class Tlm_CRC_Stats;                 // forward declaration
class galileo_telemetry_decoder_gs;  // forward declaration

using galileo_telemetry_decoder_gs_sptr = gnss_shared_ptr<galileo_telemetry_decoder_gs>;

galileo_telemetry_decoder_gs_sptr galileo_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf,
    int frame_type);

/*!
 * \brief This class implements a block that decodes the INAV and FNAV data defined in Galileo ICD
 */
class galileo_telemetry_decoder_gs : public gr::block
{
public:
    ~galileo_telemetry_decoder_gs() override;
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int32_t channel);                    //!< Set receiver's channel
    void reset();

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) override;

private:
    friend galileo_telemetry_decoder_gs_sptr galileo_make_telemetry_decoder_gs(
        const Gnss_Satellite &satellite,
        const Tlm_Conf &conf,
        int frame_type);

    galileo_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf, int frame_type);

    void msg_handler_read_galileo_tow_map(const pmt::pmt_t &msg);
    void deinterleaver(int32_t rows, int32_t cols, const float *in, float *out);
    void decode_INAV_word(float *page_part_symbols, int32_t frame_length);
    void decode_FNAV_word(float *page_symbols, int32_t frame_length);
    void decode_CNAV_word(uint64_t time_stamp, float *page_symbols, int32_t page_length);

    std::unique_ptr<Viterbi_Decoder> d_viterbi;
    std::vector<int32_t> d_preamble_samples;
    std::vector<float> d_page_part_symbols;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    boost::circular_buffer<float> d_symbol_history;

    Gnss_Satellite d_satellite;

    // navigation message vars
    Galileo_Cnav_Message d_cnav_nav;
    Galileo_Inav_Message d_inav_nav;
    Galileo_Fnav_Message d_fnav_nav;

    Nav_Message_Packet d_nav_msg_packet;
    GnssTime d_current_timetag{};

    std::unique_ptr<Tlm_CRC_Stats> d_Tlm_CRC_Stats;

    double d_delta_t;  // GPS-GALILEO time offset

    uint64_t d_symbol_counter;
    uint64_t d_preamble_index;
    uint64_t d_last_valid_preamble;
    uint64_t d_received_sample_counter;

    int32_t d_mm;
    int32_t d_codelength;
    int32_t d_datalength;
    int32_t d_frame_type;
    int32_t d_bits_per_preamble;
    int32_t d_samples_per_preamble;
    int32_t d_preamble_period_symbols;
    int32_t d_CRC_error_counter;
    int32_t d_channel;
    int32_t d_flag_even_word_arrived;

    uint32_t d_PRN_code_period_ms;
    uint32_t d_required_symbols;
    uint32_t d_frame_length_symbols;
    uint32_t d_stat;
    uint32_t d_TOW_at_Preamble_ms;
    uint32_t d_TOW_at_current_symbol_ms;
    uint32_t d_max_symbols_without_valid_frame;
    uint32_t d_received_tow_ms;

    char d_band;  // This variable will store which band we are dealing with (Galileo E1 or E5b)

    bool d_sent_tlm_failed_msg;
    bool d_flag_frame_sync;
    bool d_flag_PLL_180_deg_phase_locked;
    bool d_flag_preamble;
    bool d_dump;
    bool d_dump_mat;
    bool d_remove_dat;
    bool d_first_eph_sent;
    bool d_cnav_dummy_page;
    bool d_print_cnav_page;
    bool d_enable_navdata_monitor;
    bool d_dump_crc_stats;
    bool d_enable_reed_solomon_inav;
    bool d_valid_timetag;
    bool d_E6_TOW_set;
    bool d_there_are_e6_channels;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_TELEMETRY_DECODER_GS_H
