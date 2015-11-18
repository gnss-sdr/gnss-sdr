/*!
 * \file gnss_message.cc
 * \brief Implementation for a generic GNSS message PMT for use with the GnuRadio
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

#include "gnss_message.h"
#include <glog/logging.h>

bool gnss_message::is_valid( const pmt::pmt_t &msg )
{

    if( !pmt::is_dict( msg ) ){
        DLOG(INFO) << "PMT message not a dict " << pmt::write_string( msg );
        return false;
    }

    if( !pmt::dict_has_key( msg, pmt::mp("message") ) )
    {
        DLOG(INFO) << "PMT message does not contain a message key " << pmt::write_string( msg );
        return false;
    }

    if( !pmt::dict_has_key( msg, pmt::mp("timestamp") ) )
    {
        DLOG(INFO) << "PMT message does not contain a timestamp key " << pmt::write_string( msg );
        return false;
    }

    return true;
}


std::string gnss_message::get_message( const pmt::pmt_t &msg )
{
    if( !gnss_message::is_valid( msg ) )
    {
        std::string errMsg("Error: gnss_message ");
        errMsg += pmt::write_string( msg ) + " is not a valid gnss_message";

        LOG(ERROR) << errMsg;
    }

    pmt::pmt_t the_string;
    pmt::pmt_t not_found;
    the_string = pmt::dict_ref( msg, pmt::mp( "message" ), not_found );

    return pmt::symbol_to_string( the_string );
}

double gnss_message::get_timestamp( const pmt::pmt_t &msg )
{
    if( !gnss_message::is_valid( msg ) )
    {
        std::string errMsg("Error: gnss_message ");
        errMsg += pmt::write_string( msg ) + " is not a valid gnss_message";

        LOG(ERROR) << errMsg;
    }

    pmt::pmt_t the_timestamp;
    pmt::pmt_t not_found;
    the_timestamp = pmt::dict_ref( msg, pmt::mp( "timestamp" ), not_found );

    return pmt::to_double( the_timestamp );
}

pmt::pmt_t gnss_message::make( const std::string &msg, double timestamp )
{
    pmt::pmt_t ret = pmt::make_dict();

    ret = pmt::dict_add( ret, pmt::mp( "message" ), pmt::mp( msg ) );

    ret = pmt::dict_add( ret, pmt::mp( "timestamp" ), pmt::from_double( timestamp ) );

    return ret;
}

