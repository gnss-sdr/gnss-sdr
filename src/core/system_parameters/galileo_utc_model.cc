/*
 * \file galileo_utc_model.h
 * \brief  Interface of a Galileo UTC MODEL storage
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

#include "galileo_utc_model.h"
#include <cmath>

Galileo_Utc_Model::Galileo_Utc_Model()
{
    //valid = false;
    /*Word type 6: GST-UTC conversion parameters*/
    A0_6 = 0;
    A1_6 = 0;
    Delta_tLS_6 = 0;
    t0t_6 = 0;
    WNot_6 = 0;
    WN_LSF_6 = 0;
    DN_6 = 0;
    Delta_tLSF_6 = 0;
    flag_utc_model = false;
}

double Galileo_Utc_Model::GST_to_UTC_time(double t_e, int WN)
{
    double t_Utc;
    double t_Utc_daytime;
    double Delta_t_Utc = 0;
    // Determine if the effectivity time of the leap second event is in the past
    int weeksToLeapSecondEvent = WN_LSF_6 - (WN % 256);

    if ((weeksToLeapSecondEvent) >= 0)  // is not in the past
        {
            //Detect if the effectivity time and user's time is within six hours  = 6 * 60 *60 = 21600 s
            int secondOfLeapSecondEvent = DN_6 * 24 * 60 * 60;
            if (std::abs(t_e - secondOfLeapSecondEvent) > 21600)
                {
                    /* 5.1.7a GST->UTC case a
                     * Whenever the leap second adjusted time indicated by the WN_LSF and the DN values
                     * is not in the past (relative to the user's present time), and the user's
                     * present time does not fall in the time span which starts at six hours prior
                     * to the effective time and ends at six hours after the effective time,
                     * the GST/Utc relationship is given by
                     */
                    Delta_t_Utc = Delta_tLS_6 + A0_6 + A1_6 * (t_e - t0t_6 + 604800 * static_cast<double>((WN % 256) - WNot_6));
                    t_Utc_daytime = fmod(t_e - Delta_t_Utc, 86400);
                }
            else
                {
                    /* 5.1.7b GST->UTC case b
                     * Whenever the user's current time falls within the time span of six hours
                     * prior to the leap second adjustment to six hours after the adjustment time, ,
                     * the effective time is computed according to the following equations:
                     */
                    Delta_t_Utc = Delta_tLS_6 + A0_6 + A1_6 * (t_e - t0t_6 + 604800 * static_cast<double>((WN % 256) - WNot_6));
                    double W = fmod(t_e - Delta_t_Utc - 43200, 86400) + 43200;
                    t_Utc_daytime = fmod(W, 86400 + Delta_tLSF_6 - Delta_tLS_6);
                    //implement something to handle a leap second event!
                }
        }
    else  // the effectivity time is in the past
        {
            /* 5.1.7c GST->UTC case c
             * Whenever the leap second adjustment time, as indicated by the WN_LSF and DN values,
             * is in the past (relative to the user's current time) and the user's present time does not
             * fall in the time span which starts six hours prior to the leap second adjustment time and
             * ends six hours after the adjustment time, the effective time is computed according to
             * the following equation:
             */
            Delta_t_Utc = Delta_tLSF_6 + A0_6 + A1_6 * (t_e - t0t_6 + 604800 * static_cast<double>((WN % 256) - WNot_6));
            t_Utc_daytime = fmod(t_e - Delta_t_Utc, 86400);
        }

    double secondsOfWeekBeforeToday = 86400 * floor(t_e / 86400);
    t_Utc = secondsOfWeekBeforeToday + t_Utc_daytime;
    return t_Utc;
}
