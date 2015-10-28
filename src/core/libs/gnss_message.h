/*!
 * \file gnss_message.h
 * \brief Definitions for a generic GNSS message PMT for use with the GnuRadio
 * message passing API.
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(@)gmail.com
 *
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

#ifndef GNSS_MESSAGE_H
#define GNSS_MESSAGE_H

#include <pmt/pmt.h>
#include <string>

#define GNSS_MESSAGE_PORT_ID pmt::mp( "gnss_messages" )

namespace gnss_message {
    std::string get_message( const pmt::pmt_t &msg );
    double get_timestamp( const pmt::pmt_t &msg );
    pmt::pmt_t make( const std::string &msg, double timestamp );

    bool is_valid( const pmt::pmt_t &msg );

}

#endif
