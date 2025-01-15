/*!
 * \file galileo_ephemeris.cc
 * \brief  Interface of a Galileo EPHEMERIS storage and orbital model functions
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 *
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

#include "galileo_ephemeris.h"


double Galileo_Ephemeris::Galileo_System_Time(double week_number, double TOW)
{
    /* GALILEO SYSTEM TIME, ICD 5.1.2
     *
     * input parameter:
     *
     * week_number: The Week Number is an integer counter that gives the
     * sequential week number from the origin of the Galileo time. It covers
     * 4096 weeks (about 78 years). Then the counter is reset to zero to cover
     * additional period modulo 4096
     *
     * TOW: The Time of Week is defined as the number of seconds that have
     * occurred since the transition from the previous week. The TOW covers an
     * entire week from 0 to 604799 seconds and is reset to zero at the end of
     * each week
     *
     *  week_number and TOW are received in page 5
     *
     * output:
     *
     * t: it is the transmitted time in Galileo System Time (expressed
     * in seconds)
     *
     * The GST start epoch shall be 00:00 UT on Sunday 22nd August 1999
     * (midnight between 21st and 22nd August). At the start epoch, GST shall be
     * ahead of UTC by thirteen (13) leap seconds. Since the next leap second
     * was inserted at 01.01.2006, this implies that as of 01.01.2006 GST is
     * ahead of UTC by fourteen (14) leap seconds.
     *
     * The epoch denoted in the navigation messages by TOW and week_number will
     * be measured relative to the leading edge of the first chip of the first
     * code sequence of the first page symbol. The transmission timing of the
     * navigation message provided through the TOW is synchronised to each
     * satellite’s version of Galileo System Time (GST).
     *
     */
    const double sec_in_day = 86400;
    const double day_in_week = 7;
    double t = week_number * sec_in_day * day_in_week + TOW;  // second from the origin of the Galileo time
    return t;
}
