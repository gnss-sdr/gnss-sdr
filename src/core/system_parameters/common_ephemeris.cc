/*!
 * \file common_ephemeris.cc
 * \brief Base class for GNSS Ephemeris
 * \author Vladislav P, 2022. vladisslav2011(at)gmail.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "common_ephemeris.h"
#include <cmath>
#include <numeric>

bool Common_Ephemeris::validate(history_set &hist, const std::shared_ptr<Common_Ephemeris> &eph, const int thr, const bool first_pass)
{
    double dev_last = -1.0;
    double dev_val = -1.0;
    const int prn = eph->PRN - 1;
    bool ret = false;
    if (thr == 0)
        {
            return true;
        }
    if (hist[prn].last_eph.get())
        {
            dev_last = hist[prn].last_eph->max_deviation(*eph.get());
#ifdef EPHEMERIS_VALIDATOR_DEBUG
            if (hist[prn].last_eph.get() == eph.get())
                {
                    std::cout << "\nhist[prn].last_eph.get() == eph.get()\n\n";
                }
#endif
        }
    if (hist[prn].valid_eph.get())
        {
            dev_val = hist[prn].valid_eph->max_deviation(*eph.get());
#ifdef EPHEMERIS_VALIDATOR_DEBUG
            if (hist[prn].valid_eph.get() == eph.get())
                {
                    std::cout << "\nhist[prn].last_eph.get() == eph.get()\n\n";
                }
#endif
            if (dev_last < dev_val)
                {
                    if (dev_last < DEVIATION_THRESHOLD)
                        {
                            hist[prn].valid_eph = eph;
                            hist[prn].valid_eph_count = 2;
                            ret = hist[prn].valid_eph_count >= hist[prn].valid_eph_thr;
                            hist[prn].valid_eph_thr = (thr > 2) ? thr : 2;
                        }
                }
            else
                {
                    if (dev_val < DEVIATION_THRESHOLD)
                        {
                            hist[prn].valid_eph_count++;
                            hist[prn].valid_eph_thr = (thr > 2) ? thr : 2;
                            ret = hist[prn].valid_eph_count >= hist[prn].valid_eph_thr;
                        }
                }
        }
    else
        {
            hist[prn].valid_eph = eph;
            hist[prn].valid_eph_count = 1;
            hist[prn].valid_eph_thr = thr;
            ret = first_pass || hist[prn].valid_eph_count >= hist[prn].valid_eph_thr;
        }
    hist[prn].last_eph = eph;
#ifdef EPHEMERIS_VALIDATOR_DEBUG
    std::cout << "PRN " << eph->PRN << " dev_last = " << dev_last << " dev_val = " << dev_val << " count = " << hist[prn].valid_eph_count << "\n";
#endif
    return ret;
}
