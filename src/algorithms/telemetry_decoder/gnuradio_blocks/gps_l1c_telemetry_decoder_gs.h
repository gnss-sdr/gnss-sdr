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

#include "telemetry_impl_interface.h"

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks
 * \{ */

class gps_l1c_telemetry_decoder_gs;  // forward declaration

using gps_l1c_telemetry_decoder_gs_sptr = gnss_shared_ptr<gps_l1c_telemetry_decoder_gs>;

gps_l1c_telemetry_decoder_gs_sptr gps_l1c_make_telemetry_decoder_gs(const Tlm_Conf &conf);

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
    friend gps_l1c_telemetry_decoder_gs_sptr gps_l1c_make_telemetry_decoder_gs(const Tlm_Conf &conf);

    explicit gps_l1c_telemetry_decoder_gs(const Tlm_Conf &conf);
};

/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1C_TELEMETRY_DECODER_GS_H
