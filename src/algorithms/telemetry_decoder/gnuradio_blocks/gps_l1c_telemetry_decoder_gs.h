/*!
 * \file galileo_telemetry_decoder_gs.h
 * \brief Implementation of a GPS L1C message demodulator
 * block
 * \author José Antonio Mayo 2026. contact(at)tatjam.eu
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

#ifndef GNSS_SDR_GPS_L1C_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_GPS_L1C_TELEMETRY_DECODER_GS_H

#include "GPS_L1C.h"
#include "gnss_synchro.h"
#include "ldpc_min_sum_decoder.h"
#include "telemetry_impl_interface.h"
#include <boost/circular_buffer.hpp>
#include <bitset>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks
 * \{ */

class gps_l1c_telemetry_decoder_gs;  // forward declaration

using gps_l1c_telemetry_decoder_gs_sptr = gnss_shared_ptr<gps_l1c_telemetry_decoder_gs>;

gps_l1c_telemetry_decoder_gs_sptr gps_l1c_make_telemetry_decoder_gs(const Tlm_Conf &conf);

struct GpsL1cFrame
{
    uint16_t toi;  //<! Decoded TOI value
    std::optional<std::bitset<600>> sf2;
    std::optional<std::bitset<274>> sf3;
};

class gps_l1c_telemetry_decoder_gs : public telemetry_impl_interface
{
public:
    void set_satellite(const Gnss_Satellite &satellite) override;  //!< Set satellite PRN
    void set_channel(int32_t channel) override;                    //!< Set receiver's channel
    void reset() override;

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) override;

private:
    boost::circular_buffer<float> d_symbol_history;
    uint32_t d_stat;

    LDPC_Min_Sum_Decoder d_sf2_decoder;
    LDPC_Min_Sum_Decoder d_sf3_decoder;

    friend gps_l1c_telemetry_decoder_gs_sptr gps_l1c_make_telemetry_decoder_gs(const Tlm_Conf &conf);

    explicit gps_l1c_telemetry_decoder_gs(const Tlm_Conf &conf);

    // start_index must point to the index in d_symbol_history that contains the first received bit
    // (MSB) of TOI, and 52 bits will be read after it, including the first received bit.
    // hypothesis is the full toi, but only the LSB will be tested.
    // Returns the toi evaluated (including MSB) and signed hypothesis value.
    std::pair<uint16_t, float> compute_toi_hypothesis(size_t start_index, uint16_t hypothesis, float &normalization);

    // Tests the oldest 52 bits and finds the TOI, returning -1 if no hypothesis holds.
    // Note that, due to cyclic nature of the TOI encoded codes, if this is called near the real TOI, nonsense values
    // may be returned.
    int16_t test_toi_hypotheses(const Gnss_Synchro &synchro);

    // Attempts parsing a L1C frame, reading 1800 bits starting at start_index.
    // The first 52 bits are ignored, and must have been parsed beforehand to decode the TOI.
    std::optional<GpsL1cFrame> try_parse_frame(size_t start_index, uint16_t toi);

    // Deinterleaves the 1748 bits of subframe 2 and 3, starting at start_index.
    // The first 52 bits are ignored
    std::array<float, GPS_L1C_SF_2_AND_3_ENCODED_BITS> deinterlave_frame(size_t start_index);

    // d_stat = 0
    void state_find_align(const Gnss_Synchro &synchro);

    // d_stat = 1
    void state_aligned(const Gnss_Synchro &synchro);
};

/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1C_TELEMETRY_DECODER_GS_H
