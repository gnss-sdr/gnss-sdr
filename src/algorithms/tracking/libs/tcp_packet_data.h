/*!
 * \file tcp_packet_data.h
 * \brief Interface of the TCP data packet class
 * \author David Pubill, 2011. dpubill(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_TCP_PACKET_DATA_H_
#define GNSS_SDR_TCP_PACKET_DATA_H_

/*!
 * \brief Class that implements a TCP data packet
 */
class tcp_packet_data
{
public:
    tcp_packet_data();
    ~tcp_packet_data();
    float proc_pack_code_error;
    float proc_pack_carr_error;
    float proc_pack_carrier_doppler_hz;
};

#endif
