/*
 * \file gps_utc_model.h
 * \brief  Interface of a GPS UTC MODEL storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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

#include "gps_utc_model.h"
#include <cmath>


Gps_Utc_Model::Gps_Utc_Model()
{
    valid = false;
    d_A1 = 0;
    d_A0 = 0;
    d_t_OT = 0;
    i_WN_T = 0;
    d_DeltaT_LS = 0;
    i_WN_LSF = 0;
    i_DN = 0;
    d_DeltaT_LSF = 0;
}

double Gps_Utc_Model::utc_time(double gpstime_corrected, int i_GPS_week)
{
    double t_utc;
    double t_utc_daytime;
    double Delta_t_UTC =  d_DeltaT_LS + d_A0 + d_A1 * (gpstime_corrected - d_t_OT + 604800 * static_cast<double>(i_GPS_week - i_WN_T));

    // Determine if the effectivity time of the leap second event is in the past
    int  weeksToLeapSecondEvent = i_WN_LSF - i_GPS_week;

    if (weeksToLeapSecondEvent >= 0) // is not in the past
        {
            //Detect if the effectivity time and user's time is within six hours  = 6 * 60 *60 = 21600 s
            int secondOfLeapSecondEvent = i_DN * 24 * 60 * 60;
            if (weeksToLeapSecondEvent > 0)
                {
                    t_utc_daytime = fmod(gpstime_corrected - Delta_t_UTC, 86400);
                }
            else //we are in the same week than the leap second event
                {
                    if  (std::abs(gpstime_corrected - secondOfLeapSecondEvent) > 21600)
                        {
                            /* 20.3.3.5.2.4a
                             * Whenever the effectivity time indicated by the WN_LSF and the DN values
                             * is not in the past (relative to the user's present time), and the user's
                             * present time does not fall in the time span which starts at six hours prior
                             * to the effectivity time and ends at six hours after the effectivity time,
                             * the UTC/GPS-time relationship is given by
                             */
                            t_utc_daytime = fmod(gpstime_corrected - Delta_t_UTC, 86400);
                        }
                    else
                        {
                            /* 20.3.3.5.2.4b
                             * Whenever the user's current time falls within the time span of six hours
                             * prior to the effectivity time to six hours after the effectivity time,
                             * proper accommodation of the leap second event with a possible week number
                             * transition is provided by the following expression for UTC:
                             */
                            int W = fmod(gpstime_corrected - Delta_t_UTC - 43200, 86400) + 43200;
                            t_utc_daytime = fmod(W, 86400 + d_DeltaT_LSF - d_DeltaT_LS);
                            //implement something to handle a leap second event!
                        }
                    if ( (gpstime_corrected - secondOfLeapSecondEvent) > 21600)
                        {
                            Delta_t_UTC = d_DeltaT_LSF + d_A0 + d_A1 * (gpstime_corrected - d_t_OT + 604800 * static_cast<double>(i_GPS_week - i_WN_T));
                            t_utc_daytime = fmod(gpstime_corrected - Delta_t_UTC, 86400);
                        }
                }
        }
    else // the effectivity time is in the past
        {
            /* 20.3.3.5.2.4c
             * Whenever the effectivity time of the leap second event, as indicated by the
             * WNLSF and DN values, is in the "past" (relative to the user's current time),
             * and the user's current time does not fall in the time span as given above
             * in 20.3.3.5.2.4b,*/
            Delta_t_UTC = d_DeltaT_LSF + d_A0 + d_A1 * (gpstime_corrected - d_t_OT + 604800 * static_cast<double>(i_GPS_week - i_WN_T));
            t_utc_daytime = fmod(gpstime_corrected - Delta_t_UTC, 86400);
        }

    double secondsOfWeekBeforeToday = 86400 * floor(gpstime_corrected / 86400);
    t_utc = secondsOfWeekBeforeToday + t_utc_daytime;
    return t_utc;
}

