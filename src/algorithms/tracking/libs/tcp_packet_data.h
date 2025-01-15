/*!
 * \file tcp_packet_data.h
 * \brief Interface of the TCP data packet class
 * \author David Pubill, 2011. dpubill(at)cttc.es
 *
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

#ifndef GNSS_SDR_TCP_PACKET_DATA_H
#define GNSS_SDR_TCP_PACKET_DATA_H

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


/*!
 * \brief Class that implements a TCP data packet
 */
class Tcp_Packet_Data
{
public:
    Tcp_Packet_Data() = default;
    ~Tcp_Packet_Data() = default;
    float proc_pack_code_error = 0.0;
    float proc_pack_carr_error = 0.0;
    float proc_pack_carrier_doppler_hz = 0.0;
};


/** \} */
/** \} */
#endif
