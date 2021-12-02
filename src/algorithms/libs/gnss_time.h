/*!
 * \file gnss_time.h
 * \brief class that stores both the receiver time, relative to the receiver start and the GNSS time (absolute)
 * \author Javier Arribas 2022. jarribas(at)cttc.es
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
