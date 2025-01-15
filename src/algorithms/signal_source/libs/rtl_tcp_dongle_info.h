/*!
 * \file rtl_tcp_dongle_info.h
 * \brief Interface for a structure sent by rtl_tcp defining the hardware.
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
 *
 * This file contains information taken from librtlsdr:
 *  https://git.osmocom.org/rtl-sdr
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

#ifndef GNSS_SDR_RTL_TCP_DONGLE_INFO_H
#define GNSS_SDR_RTL_TCP_DONGLE_INFO_H

#include <boost/asio/ip/tcp.hpp>


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


/*!
 * \brief This class represents the dongle information
 * which is sent by rtl_tcp.
 */
class Rtl_Tcp_Dongle_Info
{
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

    Rtl_Tcp_Dongle_Info();

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

private:
    char magic_[4]{};
    uint32_t tuner_type_;
    uint32_t tuner_gain_count_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_RTL_TCP_DONGLE_INFO_H
