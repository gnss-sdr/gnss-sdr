/*!
 * \file gps_week_rollover.h
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


#ifndef GNSS_SDR_GPS_WEEK_ROLLOVER_H
#define GNSS_SDR_GPS_WEEK_ROLLOVER_H

#include <cstdint>
#include <string>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief Derives the reference GPS week number used to resolve the GPS
 * mod-1024 week-number rollover when post-processing recorded signal files.
 *
 * \param observation_date Value of the GNSS-SDR.observation_date
 * configuration option: approximate date of the signal capture, in
 * "YYYY-MM-DD" or "YYYY" format. An empty string means "not set".
 * \param pre_2009_file Value of the deprecated GNSS-SDR.pre_2009_file flag.
 *
 * \returns The full GPS week number of the observation date. Each mod-1024
 * broadcast week number is then resolved to the 1024-week era that places it
 * closest to this reference week (see adjgpsweek()). Returns 0 if neither
 * option is set, which keeps the default behavior of deriving the era from
 * the system clock. If only the deprecated flag is set, it returns week 1535
 * (mid-point of the 1999-2019 era), which reproduces the legacy fixed +1024
 * week adjustment, and a one-time deprecation notice is printed.
 */
int32_t gps_ref_week_from_config(const std::string& observation_date, bool pre_2009_file);


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_WEEK_ROLLOVER_H
