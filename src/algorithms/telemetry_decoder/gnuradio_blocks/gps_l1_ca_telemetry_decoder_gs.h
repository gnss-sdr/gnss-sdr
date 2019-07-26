/*!
 * \file gps_l1_ca_telemetry_decoder_gs.h
 * \brief Interface of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L1_CA_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_GPS_L1_CA_TELEMETRY_DECODER_GS_H

#include "GPS_L1_CA.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "gps_navigation_message.h"
#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>  // for boost::shared_ptr
#include <gnuradio/block.h>      // for block
#include <gnuradio/types.h>      // for gr_vector_const_void_star
#include <array>                 // for array
#include <cstdint>               // for int32_t
#include <fstream>               // for ofstream
#include <string>                // for string


class gps_l1_ca_telemetry_decoder_gs;

using gps_l1_ca_telemetry_decoder_gs_sptr = boost::shared_ptr<gps_l1_ca_telemetry_decoder_gs>;

gps_l1_ca_telemetry_decoder_gs_sptr gps_l1_ca_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    bool dump);

/*!
 * \brief This class implements a block that decodes the NAV data defined in IS-GPS-200E
 */
class gps_l1_ca_telemetry_decoder_gs : public gr::block
{
public:
    ~gps_l1_ca_telemetry_decoder_gs();
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int channel);                        //!< Set receiver's channel
    void reset();

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend gps_l1_ca_telemetry_decoder_gs_sptr gps_l1_ca_make_telemetry_decoder_gs(
        const Gnss_Satellite &satellite,
        bool dump);

    gps_l1_ca_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump);
    bool gps_word_parityCheck(uint32_t gpsword);
    bool decode_subframe();

    int32_t d_bits_per_preamble;
    int32_t d_samples_per_preamble;
    int32_t d_preamble_period_symbols;
    std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_BITS> d_preamble_samples{};
    uint32_t d_required_symbols;
    uint32_t d_frame_length_symbols;
    bool flag_PLL_180_deg_phase_locked;

    // navigation message vars
    Gps_Navigation_Message d_nav;
    uint32_t d_prev_GPS_frame_4bytes;

    boost::circular_buffer<float> d_symbol_history;

    uint64_t d_sample_counter;
    uint64_t d_preamble_index;
    uint64_t d_last_valid_preamble;
    uint32_t d_max_symbols_without_valid_frame;

    bool d_sent_tlm_failed_msg;
    uint32_t d_stat;
    bool d_flag_frame_sync;
    bool d_flag_parity;
    bool d_flag_preamble;
    int32_t d_CRC_error_counter;

    Gnss_Satellite d_satellite;
    int32_t d_channel;

    uint32_t d_TOW_at_Preamble_ms;
    uint32_t d_TOW_at_current_symbol_ms;

    bool flag_TOW_set;
    bool d_dump;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
