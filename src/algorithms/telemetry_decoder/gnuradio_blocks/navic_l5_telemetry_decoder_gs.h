/*!
 * \file navic_l5_telemetry_decoder_gs.h
 * \brief Implementation of a NavIC L5 LNAV data decoder block
 * \author Pradyumna Krishna, 2026. pradyumnakrishna(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NAVIC_L5_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_NAVIC_L5_TELEMETRY_DECODER_GS_H


#include "NAVIC_L5.h"
#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include "nav_message_packet.h"
#include "navic_lnav_navigation_message.h"
#include "tlm_conf.h"
#include "tlm_crc_stats.h"
#include <boost/circular_buffer.hpp>
#include <gnuradio/block.h>  // for block
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#include <array>
#include <cstdint>
#include <fstream>
#include <memory>  // for std::unique_ptr
#include <string>
#include <vector>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks
 * \{ */


class Viterbi_Decoder;  // forward declaration
class navic_l5_telemetry_decoder_gs;

using navic_l5_telemetry_decoder_gs_sptr = gnss_shared_ptr<navic_l5_telemetry_decoder_gs>;

navic_l5_telemetry_decoder_gs_sptr navic_l5_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf);


/*!
 * \brief This class implements a block that decodes the NavIC L5 LNAV data.
 */
class navic_l5_telemetry_decoder_gs : public gr::block
{
public:
    ~navic_l5_telemetry_decoder_gs() override;          //!< Class destructor
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int channel);                        //!< Set receiver's channel
    void reset();

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) override;

private:
    friend navic_l5_telemetry_decoder_gs_sptr navic_l5_make_telemetry_decoder_gs(
        const Gnss_Satellite &satellite,
        const Tlm_Conf &conf);

    navic_l5_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf);

    void decode_subframe(float *symbols, double cn0);
    void deinterleaver(int32_t rows, int32_t cols, const float *in, float *out);

    // Viterbi decoder
    std::unique_ptr<Viterbi_Decoder> d_viterbi;

    // Preamble decoding
    std::array<int32_t, NAVIC_L5_PREAMBLE_LENGTH_SYMBOLS> d_preamble_samples{};

    std::array<float, NAVIC_L5_PREAMBLE_PERIOD_SYMBOLS> d_subframe_symbols{};

    // Storage for incoming data
    boost::circular_buffer<float> d_symbol_history;

    // Navigation Message variable
    Navic_Lnav_Navigation_Message d_nav;

    Nav_Message_Packet d_nav_msg_packet;
    std::unique_ptr<Tlm_CRC_Stats> d_Tlm_CRC_Stats;

    // Satellite Information and logging capacity
    Gnss_Satellite d_satellite;
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    uint64_t d_sample_counter;  //!< Sample counter as an index indicating number of symbols processed
    uint64_t d_preamble_index;  //!< Index of symbol number where preamble was found

    // Symbol accumulator for 1ms→20ms integration
    double d_symbol_accumulator;       //!< Accumulates 1ms prompt values
    uint32_t d_symbol_accumulator_counter;  //!< Counts code periods accumulated
    uint64_t d_code_period_counter;    //!< Total code periods received

    // Raw prompt buffer for bit boundary search (stores ~13s of 1ms samples)
    std::vector<float> d_raw_prompt_buffer;
    uint64_t d_raw_prompt_write_idx;
    uint64_t d_raw_prompt_count;
    bool d_bit_phase_locked;

    int32_t d_channel;
    int32_t d_symbols_per_preamble;
    int32_t d_samples_per_preamble;
    int32_t d_preamble_period_samples;
    int32_t d_CRC_error_counter;  //!< Number of failed CRC operations
    uint32_t d_required_symbols;
    uint32_t d_stat;  //!< Status of decoder

    // Values to populate gnss synchronization structure
    uint64_t d_last_valid_preamble;
    uint32_t d_symbol_duration_ms;
    uint32_t d_TOW_at_Preamble_ms;
    uint32_t d_TOW_at_current_symbol_ms;

    bool d_flag_TOW_set;         //!< Indicates when time of week is set
    bool d_flag_frame_sync;      //!< Indicate when a frame sync is achieved
    bool d_flag_preamble;        //!< Flag indicating when preamble was found

    bool d_flag_valid_word;
    bool d_flag_pll_180_deg_locked;
    bool d_flag_bit_sync_detected;
    bool d_sent_tlm_failed_msg;
    bool d_dump;
    bool d_dump_mat;
    bool d_remove_dat;
    bool d_enable_navdata_monitor;
    bool d_dump_crc_stats;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NAVIC_L5_TELEMETRY_DECODER_GS_H
