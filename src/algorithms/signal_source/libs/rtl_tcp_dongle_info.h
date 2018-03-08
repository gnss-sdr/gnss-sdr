/*!
 * \file rtl_tcp_dongle_info.h
 * \brief Interface for a structure sent by rtl_tcp defining the hardware.
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
 *
 * This file contains information taken from librtlsdr:
 *  http://git.osmocom.org/rtl-sdr/
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTL_TCP_DONGLE_INFO_H
#define GNSS_SDR_RTL_TCP_DONGLE_INFO_H

#include <boost/asio/ip/tcp.hpp>

/*!
 * \brief This class represents the dongle information
 * which is sent by rtl_tcp.
 */
class rtl_tcp_dongle_info
{
private:
    char magic_[4];
    uint32_t tuner_type_;
    uint32_t tuner_gain_count_;

public:
    enum
    {
        TUNER_UNKNOWN = 0,
        TUNER_E4000,
        TUNER_FC0012,
        TUNER_FC0013,
        TUNER_FC2580,
        TUNER_R820T,
        TUNER_R828D
    };

    rtl_tcp_dongle_info();

    boost::system::error_code read(boost::asio::ip::tcp::socket &socket);

    bool is_valid() const;

    const char *get_type_name() const;

    double clip_gain(int gain) const;

    inline uint32_t get_tuner_type() const
    {
        return tuner_type_;
    }

    inline uint32_t get_tuner_gain_count() const
    {
        return tuner_gain_count_;
    }
};


#endif  // GNSS_SDR_RTL_TCP_DONGLE_INFO_H
