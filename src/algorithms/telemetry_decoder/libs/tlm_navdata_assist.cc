/*!
 * \file tlm_navdata_assist.cc
 * \brief Class that provides telemetry data assistance
 * \author Marc Majoral, 2021. mmajoral(at)cttc.es
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

#include "tlm_navdata_assist.h"
#include <cassert>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iostream>

Tlm_navdata_assist::Tlm_navdata_assist(const Tlm_Conf &conf)
{
    navdata_assist_real_time = conf.navdata_assist_real_time;
    navdata_assist_Tow_ms = conf.navdata_assist_Tow_ms;
    navdata_assist_samplestamp = conf.navdata_assist_samplestamp;
    navdata_assist_GNSS_UTC_leap_s = conf.navdata_assist_GNSS_UTC_leap_s;
}

uint32_t Tlm_navdata_assist::get_TOW_at_current_symbol_ms(uint64_t Tracking_sample_counter, uint64_t fs)
{
    if (navdata_assist_real_time)
        {
            // 1 get current time date
            std::chrono::high_resolution_clock::time_point pt = std::chrono::high_resolution_clock::now();
            std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(pt.time_since_epoch());
            std::time_t tim = sec.count();

            // 2 get UTC time
            std::tm tm_utc;
            memcpy(&tm_utc, std::gmtime(&tim), sizeof(std::tm));

            // 3 compute GNSS system time with an accuracy of 1 s
            uint32_t GNSS_system_wday = tm_utc.tm_wday;                                    // week day (Sunday => 0)
            uint32_t GNSS_system_time_h = tm_utc.tm_hour;                                  // hour
            uint32_t GNSS_system_time_m = tm_utc.tm_min;                                   // min
            uint32_t GNSS_system_time_s = tm_utc.tm_sec + navdata_assist_GNSS_UTC_leap_s;  // second
            if (GNSS_system_time_s > 59)
                {
                    GNSS_system_time_m = GNSS_system_time_m + GNSS_system_time_s / 60;
                    GNSS_system_time_s = GNSS_system_time_s % 60;

                    if (GNSS_system_time_m > 60)
                        {
                            GNSS_system_time_h = GNSS_system_time_h + GNSS_system_time_m / 60;
                            GNSS_system_time_m = GNSS_system_time_m % 60;
                            if (GNSS_system_time_h > 24)
                                {
                                    GNSS_system_wday = GNSS_system_wday + GNSS_system_time_h / 24;
                                    GNSS_system_time_h = GNSS_system_time_h % 24;
                                    if (GNSS_system_wday > 6)
                                        {
                                            GNSS_system_wday = GNSS_system_wday % 6;
                                        }
                                }
                        }
                }

            // compute TOW
            return (GNSS_system_wday * 24 * 60 * 60 + GNSS_system_time_h * 60 * 60 + GNSS_system_time_m * 60 + GNSS_system_time_s) * 1000;
        }
    else
        {
            // compute TOW
            return navdata_assist_Tow_ms + (Tracking_sample_counter - navdata_assist_samplestamp) * 1000 / fs;
        }
}

Tlm_navdata_assist::~Tlm_navdata_assist()
{
}
