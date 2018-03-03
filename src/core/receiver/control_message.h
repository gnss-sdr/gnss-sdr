/*!
 * \file control_message.h
 * \brief Interface for the different control messages.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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


#ifndef GNSS_SDR_CONTROL_MESSAGE_H_
#define GNSS_SDR_CONTROL_MESSAGE_H_

/*!
 * \brief This class defines the different Control Messages
 */
class ControlMessage
{
public:
    static unsigned int const ack_success = 0;
    static unsigned int const ack_failed = 1;
    static unsigned int const trk_failed = 2;
    static unsigned int const channel_init = 3;
};

#endif /*GNSS_SDR_CONTROL_MESSAGE_H_*/
