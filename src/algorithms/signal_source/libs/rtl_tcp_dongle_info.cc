/*!
 * \file rtl_tcp_dongle_info.cc
 * \brief Defines methods for retrieving and validating rtl_tcp donle
 *  info.
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

#include "rtl_tcp_dongle_info.h"
#include <string>
#include <vector>


Rtl_Tcp_Dongle_Info::Rtl_Tcp_Dongle_Info() : tuner_type_(0), tuner_gain_count_(0)
{
    std::memset(magic_, 0, sizeof(magic_));
}


boost::system::error_code Rtl_Tcp_Dongle_Info::read(boost::asio::ip::tcp::socket &socket)
{
    boost::system::error_code ec;

    std::vector<unsigned char> data(sizeof(char) * 4 + sizeof(uint32_t) * 2);
    size_t received_bits = socket.receive(boost::asio::buffer(data), 0, ec);
    if (!ec && (received_bits > 0))
        {
            std::memcpy(magic_, data.data(), 4);

            uint32_t type;
            std::memcpy(&type, &data[4], 4);

            tuner_type_ = boost::asio::detail::socket_ops::network_to_host_long(type);

            uint32_t count;
            std::memcpy(&count, &data[8], 4);

            tuner_gain_count_ = boost::asio::detail::socket_ops::network_to_host_long(count);
        }
    return ec;
}


const char *Rtl_Tcp_Dongle_Info::get_type_name() const
{
    switch (get_tuner_type())
        {
        default:
            return "UNKNOWN";
        case TUNER_E4000:
            return "E4000";
        case TUNER_FC0012:
            return "FC0012";
        case TUNER_FC0013:
            return "FC0013";
        case TUNER_FC2580:
            return "FC2580";
        case TUNER_R820T:
            return "R820T";
        case TUNER_R828D:
            return "R828D";
        }
}


double Rtl_Tcp_Dongle_Info::clip_gain(int gain) const
{
    // the following gain values have been copied from librtlsdr
    // all gain values are expressed in tenths of a dB

    std::vector<double> gains;
    switch (get_tuner_type())
        {
        case TUNER_E4000:
            gains = {-10, 15, 40, 65, 90, 115, 140, 165, 190, 215,
                240, 290, 340, 420};
            break;
        case TUNER_FC0012:
            gains = {-99, -40, 71, 179, 192};
            break;
        case TUNER_FC0013:
            gains = {-99, -73, -65, -63, -60, -58, -54, 58, 61,
                63, 65, 67, 68, 70, 71, 179, 181, 182,
                184, 186, 188, 191, 197};
            break;
        case TUNER_R820T:
            gains = {0, 9, 14, 27, 37, 77, 87, 125, 144, 157,
                166, 197, 207, 229, 254, 280, 297, 328,
                338, 364, 372, 386, 402, 421, 434, 439,
                445, 480, 496};
            break;
        default:
            // no gains
            break;
        }

    // clip
    if (gains.empty())
        {
            // no defined gains to clip to
            return gain;
        }

    double last_stop = gains.front();
    for (auto g : gains)
        {
            g /= 10.0;

            if (gain < g)
                {
                    if (std::abs(gain - g) < std::abs(gain - last_stop))
                        {
                            return g;
                        }
                    return last_stop;
                }
            last_stop = g;
        }
    return last_stop;
}


bool Rtl_Tcp_Dongle_Info::is_valid() const
{
    return std::memcmp(magic_, "RTL0", 4) == 0;
}
