/*!
 * \file GPS_P_CODE.h
 * \brief  Defines system parameters for GPS P Code
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
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


#ifndef GNSS_SDR_GPS_P_CODE_H
#define GNSS_SDR_GPS_P_CODE_H



// carrier and code frequencies
const double GPS_P_CODE_RATE_HZ      = 10.23e6;   //!< GPS P code rate [chips/s]
const double GPS_P_CODE_PERIOD       = 3600.0*24.0*7.0;     //!< GPS P code period [seconds]
const double GPS_P_CODE_LENGTH_CHIPS = 1023.0;    //!< GPS P code length [chips]
const double GPS_X1_EPOCH_SECONDS    = 1.5;  //! GPS X1 epoch in seconds
const double GPS_X2_EPOCH_SECONDS    = GPS_X1_EPOCH_SECONDS + 37/GPS_P_CODE_RATE_HZ;  //! GPS X2 epoch in seconds
const unsigned int GPS_X1A_EPOCHS_PER_X1_EPOCH = 3750; //! Number of X1A epochs per X1 epoch

const unsigned int GPS_X1A_CODE_LENGTH = 4092; //! Length of the X1A code in chips
const unsigned int GPS_X1B_CODE_LENGTH = 4093; //! Length of the X1A code in chips
const unsigned int GPS_X2A_CODE_LENGTH = 4092; //! Length of the X1A code in chips
const unsigned int GPS_X2B_CODE_LENGTH = 4093; //! Length of the X1A code in chips

#endif /* GNSS_SDR_GPS_P_CODE_H */

