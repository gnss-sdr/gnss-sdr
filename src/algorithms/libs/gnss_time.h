/* -------------------------------------------------------------------------
 *
 * Copyright (C) 2021 (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 * Satellite Systems Simulator
 *
 * This file is part of GNSS-SDR.
 *
 */

#ifndef GNSS_SDR_GNSS_TIME_H
#define GNSS_SDR_GNSS_TIME_H

#include <cstdint>

class GnssTime
{
public:
    double rx_time;
    int week;               /*!< GPS week number (since January 1980) */
    int tow_ms;             /* time of week [ms]*/
    double tow_ms_fraction; /* tow ms fractional part [ms]*/
};

#endif
