/*!
 * \file gps_l1c_telemetry_decoder_gs.h
 * \brief GNU Radio block that decodes the GPS L1C CNAV-2 navigation message
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
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

#ifndef GNSS_SDR_GPS_L1C_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_GPS_L1C_TELEMETRY_DECODER_GS_H

#include "gnss_satellite.h"
#include "gps_cnav2_navigation_message.h"
#include "nav_message_packet.h"
#include "telemetry_impl_interface.h"
#include <boost/circular_buffer.hpp>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks
 * \{ */

class Gnss_Synchro;
class Tlm_Conf;
class Tlm_CRC_Stats;
class gps_l1c_telemetry_decoder_gs;

using gps_l1c_telemetry_decoder_gs_sptr = gnss_shared_ptr<gps_l1c_telemetry_decoder_gs>;

gps_l1c_telemetry_decoder_gs_sptr gps_l1c_make_telemetry_decoder_gs(const Tlm_Conf &conf);

/*!
 * \brief This class implements a block that decodes the CNAV-2 data
 * (IS-GPS-800J) carried by the L1CD symbols that the tracking block delivers
 * every 10 ms.
 *
 * The frame boundary and the data polarity are searched for by testing, for
 * every new symbol, whether the last 1800 symbols form a frame: the TOI
 * correlation and the parity checks of subframe 2 preselect the candidates,
 * which are confirmed by the LDPC decoder and the CRC. This works whether
 * the tracking block follows the pilot (the frame then starts at an overlay
 * code epoch) or the data component.
 */
class gps_l1c_telemetry_decoder_gs : public telemetry_impl_interface
{
public:
    ~gps_l1c_telemetry_decoder_gs() override;
    void set_satellite(const Gnss_Satellite &satellite) override;  //!< Set satellite PRN
    void set_channel(int32_t channel) override;                    //!< Set receiver's channel
    void reset() override;
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) override;

private:
    friend gps_l1c_telemetry_decoder_gs_sptr gps_l1c_make_telemetry_decoder_gs(const Tlm_Conf &conf);

    explicit gps_l1c_telemetry_decoder_gs(const Tlm_Conf &conf);

    bool decode_last_frame(bool inverted);
    bool search_frame(bool tracking_inverted);
    void publish_navigation(const Gnss_Synchro &symbol);
    void update_time_of_week(bool in_sync);
    void check_navigation_usability();  //!< Tells the user, once per satellite, when L1C cannot serve for PVT
    void clear_sync();

    Gps_CNAV2_Navigation_Message d_nav;
    Gnss_Satellite d_satellite;
    Nav_Message_Packet d_nav_msg_packet;
    std::unique_ptr<Tlm_CRC_Stats> d_Tlm_CRC_Stats;

    boost::circular_buffer<float> d_symbol_history;
    std::vector<float> d_frame;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    uint64_t d_sample_counter{};
    uint64_t d_last_valid_frame{};
    int32_t d_channel{};
    int32_t d_symbols_in_frame{};
    int32_t d_expected_toi{-1};
    int32_t d_frame_errors{};
    int32_t d_frames_without_tow{};
    uint32_t d_TOW_at_current_symbol_ms{};
    uint32_t d_TOW_at_frame_start_ms{};

    bool d_flag_frame_sync{};
    bool d_flag_valid_word{};
    bool d_inverted{};
    bool d_sent_tlm_failed_msg{};
    const bool d_dump;
    const bool d_dump_mat;
    const bool d_remove_dat;
    const bool d_enable_navdata_monitor;
    bool d_dump_crc_stats;
    const bool d_tow_to_trk;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1C_TELEMETRY_DECODER_GS_H
