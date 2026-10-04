/*!
 * \file gps_week_rollover.cc
 * \brief Resolution of the GPS mod-1024 week-number rollover from the
 * receiver configuration
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "gps_week_rollover.h"
#include <boost/date_time/gregorian/gregorian.hpp>
#include <algorithm>
#include <exception>
#include <iostream>

namespace
{
// Parses "YYYY-MM-DD" or "YYYY" ("YYYY" maps to July 1 of that year).
// Returns a not_a_date_time value if the string cannot be parsed.
boost::gregorian::date parse_observation_date(const std::string& date_str)
{
    try
        {
            if (date_str.size() == 4 && std::all_of(date_str.begin(), date_str.end(), [](char c) { return c >= '0' && c <= '9'; }))
                {
                    return {static_cast<uint16_t>(std::stoi(date_str)), 7, 1};
                }
            return boost::gregorian::from_simple_string(date_str);
        }
    catch (const std::exception& e)
        {
            return boost::gregorian::date(boost::gregorian::not_a_date_time);
        }
}
}  // namespace


int32_t gps_ref_week_from_config(const std::string& observation_date, bool pre_2009_file)
{
    const boost::gregorian::date gps_epoch(1980, 1, 6);
    if (!observation_date.empty())
        {
            const boost::gregorian::date date = parse_observation_date(observation_date);
            if (date.is_not_a_date())
                {
                    std::cout << "Warning: could not parse GNSS-SDR.observation_date=" << observation_date
                              << " (expected YYYY-MM-DD or YYYY format), ignoring it.\n";
                }
            else if (date < gps_epoch)
                {
                    std::cout << "Warning: GNSS-SDR.observation_date=" << observation_date
                              << " is earlier than the GPS epoch (1980-01-06), ignoring it.\n";
                }
            else
                {
                    if (pre_2009_file)
                        {
                            std::cout << "Warning: the deprecated GNSS-SDR.pre_2009_file flag is ignored because GNSS-SDR.observation_date is set.\n";
                        }
                    return static_cast<int32_t>((date - gps_epoch).days() / 7);
                }
        }
    if (pre_2009_file)
        {
            static bool notice_printed = false;
            if (!notice_printed)
                {
                    std::cout << "Warning: GNSS-SDR.pre_2009_file is deprecated. Please set GNSS-SDR.observation_date=YYYY-MM-DD\n"
                              << "(the approximate date of the signal capture) instead, which also works for files\n"
                              << "recorded after the April 2019 week rollover.\n";
                    notice_printed = true;
                }
            // Mid-point of the Aug 1999 - Apr 2019 era: reproduces the legacy
            // fixed +1024 week adjustment for every mod-1024 week number
            return 1535;
        }
    return 0;
}
