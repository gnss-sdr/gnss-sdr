/*
 * \file galileo_utc_model.h
 * \brief  Interface of a Galileo UTC MODEL storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
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

#include "galileo_utc_model.h"
#include <cmath>


double Galileo_Utc_Model::GST_to_UTC_time(double t_e, int32_t WN) const
{
    double t_Utc;
    double t_Utc_daytime;
    double Delta_t_Utc = 0;
    // Determine if the effectivity time of the leap second event is in the past
    const int32_t weeksToLeapSecondEvent = WN_LSF - (WN % 256);

    if ((weeksToLeapSecondEvent) >= 0)  // is not in the past
        {
            // Detect if the effectivity time and user's time is within six hours  = 6 * 60 *60 = 21600 s
            const int secondOfLeapSecondEvent = DN * 24 * 60 * 60;
            if (std::abs(t_e - secondOfLeapSecondEvent) > 21600)
                {
                    /* 5.1.7a GST->UTC case a
                     * Whenever the leap second adjusted time indicated by the WN_LSF and the DN values
                     * is not in the past (relative to the user's present time), and the user's
                     * present time does not fall in the time span which starts at six hours prior
                     * to the effective time and ends at six hours after the effective time,
                     * the GST/Utc relationship is given by
                     */
                    Delta_t_Utc = Delta_tLS + A0 + A1 * (t_e - tot + 604800 * static_cast<double>((WN % 256) - WNot));
                    t_Utc_daytime = fmod(t_e - Delta_t_Utc, 86400);
                }
            else
                {
                    /* 5.1.7b GST->UTC case b
                     * Whenever the user's current time falls within the time span of six hours
                     * prior to the leap second adjustment to six hours after the adjustment time,
                     * the effective time is computed according to the following equations:
                     */
                    Delta_t_Utc = Delta_tLS + A0 + A1 * (t_e - tot + 604800 * static_cast<double>((WN % 256) - WNot));
                    const double W = fmod(t_e - Delta_t_Utc - 43200, 86400) + 43200;
                    t_Utc_daytime = fmod(W, 86400 + Delta_tLSF - Delta_tLS);
                    // implement something to handle a leap second event!
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
            Delta_t_Utc = Delta_tLSF + A0 + A1 * (t_e - tot + 604800 * static_cast<double>((WN % 256) - WNot));
            t_Utc_daytime = fmod(t_e - Delta_t_Utc, 86400);
        }

    const double secondsOfWeekBeforeToday = 86400 * floor(t_e / 86400);
    t_Utc = secondsOfWeekBeforeToday + t_Utc_daytime;
    return t_Utc;
}
