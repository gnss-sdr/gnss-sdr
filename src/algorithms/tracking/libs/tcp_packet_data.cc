/*!
 * \file tcp_packet_data.cc
 * \brief Interface of the TCP packet data class
 * \author David Pubill, 2011. dpubill(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */
#include "tcp_packet_data.h"

Tcp_Packet_Data::Tcp_Packet_Data()
{
    proc_pack_code_error = 0;
    proc_pack_carr_error = 0;
    proc_pack_carrier_doppler_hz = 0;
}
