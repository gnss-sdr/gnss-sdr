/*!
 * \file rtl_tcp_dongle_info.cc
 * \brief Defines methods for retrieving and validating rtl_tcp donle
 *  info.
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

#include "rtl_tcp_dongle_info.h"
#include <boost/foreach.hpp>
#include <string>

using boost::asio::ip::tcp;

rtl_tcp_dongle_info::rtl_tcp_dongle_info() : tuner_type_(0), tuner_gain_count_(0)
{
    std::memset(magic_, 0, sizeof(magic_));
}


boost::system::error_code rtl_tcp_dongle_info::read(boost::asio::ip::tcp::socket &socket)
{
    boost::system::error_code ec;

    unsigned char data[sizeof(char) * 4 + sizeof(uint32_t) * 2];
    socket.receive(boost::asio::buffer(data), 0, ec);
    if (!ec)
        {
            std::memcpy(magic_, data, 4);

            uint32_t type;
            std::memcpy(&type, &data[4], 4);

            tuner_type_ = boost::asio::detail::socket_ops::network_to_host_long(type);

            uint32_t count;
            std ::memcpy(&count, &data[8], 4);

            tuner_gain_count_ = boost::asio::detail::socket_ops::network_to_host_long(count);
        }
    return ec;
}


const char *rtl_tcp_dongle_info::get_type_name() const
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


double rtl_tcp_dongle_info::clip_gain(int gain) const
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
    if (gains.size() == 0)
        {
            // no defined gains to clip to
            return gain;
        }
    else
        {
            double last_stop = gains.front();
            BOOST_FOREACH (double g, gains)
                {
                    g /= 10.0;

                    if (gain < g)
                        {
                            if (std::abs(gain - g) < std::abs(gain - last_stop))
                                {
                                    return g;
                                }
                            else
                                {
                                    return last_stop;
                                }
                        }
                    last_stop = g;
                }
            return last_stop;
        }
}


bool rtl_tcp_dongle_info::is_valid() const
{
    return std::memcmp(magic_, "RTL0", 4) == 0;
}
