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

#include "gnss_synchro.h"
#include "gps_l1c_telemetry_decoder_gs.h"

gps_l1c_telemetry_decoder_gs_sptr
gps_l1c_make_telemetry_decoder_gs(const Tlm_Conf &conf)
{
    return gps_l1c_telemetry_decoder_gs_sptr(new gps_l1c_telemetry_decoder_gs(conf));
}


gps_l1c_telemetry_decoder_gs::gps_l1c_telemetry_decoder_gs(const Tlm_Conf &conf)
    : telemetry_impl_interface("gps_l1c_telemetry_decoder_gs",
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    configure_basic_outputs();
}


void gps_l1c_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
}
void gps_l1c_telemetry_decoder_gs::set_channel(int32_t channel)
{
}

void gps_l1c_telemetry_decoder_gs::reset()
{
}

int gps_l1c_telemetry_decoder_gs::general_work(int noutput_items, gr_vector_int &ninput_items,
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    consume_each(1);
    return 0;
}
