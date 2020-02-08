/*!
 * \file tcp_packet_data.h
 * \brief Interface of the TCP data packet class
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

#ifndef GNSS_SDR_TCP_PACKET_DATA_H_
#define GNSS_SDR_TCP_PACKET_DATA_H_

/*!
 * \brief Class that implements a TCP data packet
 */
class Tcp_Packet_Data
{
public:
    Tcp_Packet_Data();
    ~Tcp_Packet_Data() = default;
    float proc_pack_code_error;
    float proc_pack_carr_error;
    float proc_pack_carrier_doppler_hz;
};

#endif
