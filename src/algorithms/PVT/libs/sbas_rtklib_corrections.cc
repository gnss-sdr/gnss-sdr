/*!
 * \file sbas_rtklib_corrections.cc
 * \brief Bridge between raw SBAS messages and RTKLIB correction state
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

#include "sbas_rtklib_corrections.h"
#include "SBAS_L1.h"
#include "rtklib_sbas.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

constexpr size_t Sbas_Rtklib_Corrections::MAX_PENDING_MESSAGES;
constexpr double Sbas_Rtklib_Corrections::FUTURE_MESSAGE_TOLERANCE_S;

Sbas_Rtklib_Corrections::Sbas_Rtklib_Corrections(int32_t selected_prn)
    : d_selected_prn(selected_prn)
{
    // A zero-initialized IOD would incorrectly admit correction messages that
    // arrive before their corresponding mask. Force the first MT1/MT18 to set it.
    d_navigation_data.sbssat.iodp = -1;
    for (auto& ionosphere_band : d_navigation_data.sbsion)
        {
            ionosphere_band.iodi = -1;
        }
    d_navigation_data.seph = d_sbas_ephemerides.data();
}


void Sbas_Rtklib_Corrections::push(const Sbas_Raw_Message& message)
{
    if (message.prn() < SBAS_L1_PRN_MIN || message.prn() > SBAS_L1_PRN_MAX)
        {
            return;
        }
    if (d_selected_prn != 0 && message.prn() != static_cast<uint32_t>(d_selected_prn))
        {
            return;
        }

    d_pending_messages.push_back(message);
    if (d_pending_messages.size() > MAX_PENDING_MESSAGES)
        {
            d_pending_messages.pop_front();
        }
}


void Sbas_Rtklib_Corrections::update(int32_t gps_week, double gps_tow_s,
    double receiver_sample_stamp_s)
{
    if (gps_week <= 0 || d_pending_messages.empty())
        {
            return;
        }

    std::vector<Sbas_Raw_Message> messages(d_pending_messages.cbegin(), d_pending_messages.cend());
    std::stable_sort(messages.begin(), messages.end(),
        [](const Sbas_Raw_Message& lhs, const Sbas_Raw_Message& rhs) {
            return lhs.sample_stamp_s() < rhs.sample_stamp_s();
        });

    d_pending_messages.clear();
    for (const auto& message : messages)
        {
            if (message.sample_stamp_s() > receiver_sample_stamp_s + FUTURE_MESSAGE_TOLERANCE_S)
                {
                    d_pending_messages.push_back(message);
                    continue;
                }

            if (d_active_prn == 0)
                {
                    d_active_prn = d_selected_prn != 0 ? d_selected_prn : static_cast<int32_t>(message.prn());
                }
            if (message.prn() != static_cast<uint32_t>(d_active_prn))
                {
                    continue;
                }

            int32_t message_week = gps_week;
            double message_tow_s = gps_tow_s + message.sample_stamp_s() - receiver_sample_stamp_s;
            while (message_tow_s < 0.0)
                {
                    message_tow_s += 604800.0;
                    --message_week;
                }
            while (message_tow_s >= 604800.0)
                {
                    message_tow_s -= 604800.0;
                    ++message_week;
                }

            auto integer_tow_s = static_cast<int32_t>(std::lround(message_tow_s));
            if (integer_tow_s >= 604800)
                {
                    integer_tow_s -= 604800;
                    ++message_week;
                }

            sbsmsg_t rtklib_message{};
            rtklib_message.week = message_week;
            rtklib_message.tow = integer_tow_s;
            rtklib_message.prn = static_cast<int32_t>(message.prn());
            std::copy_n(message.frame().cbegin(), SBAS_L1_DATA_FIELD_BYTES, rtklib_message.msg);
            rtklib_message.msg[SBAS_L1_DATA_FIELD_BYTES - 1] &= 0xC0U;

            sbsupdatecorr(&rtklib_message, &d_navigation_data);
        }
}


void Sbas_Rtklib_Corrections::copy_to(nav_t& navigation_data)
{
    navigation_data.sbssat = d_navigation_data.sbssat;
    for (size_t band = 0; band <= MAXBAND; ++band)
        {
            navigation_data.sbsion[band] = d_navigation_data.sbsion[band];
        }
    navigation_data.seph = d_sbas_ephemerides.data();
    navigation_data.ns = NSATSBS * 2;
}
