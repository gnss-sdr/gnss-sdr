/*!
 * \file sbas_rtklib_corrections.h
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

#ifndef GNSS_SDR_SBAS_RTKLIB_CORRECTIONS_H
#define GNSS_SDR_SBAS_RTKLIB_CORRECTIONS_H

#include "rtklib.h"
#include "sbas_raw_message.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */

/*!
 * \brief Maintains RTKLIB SBAS state from timestamped raw L1 messages.
 *
 * Raw messages use the receiver sample-clock time base. update() relates that
 * clock to GPST at an observation epoch before passing messages to RTKLIB.
 */
class Sbas_Rtklib_Corrections
{
public:
    explicit Sbas_Rtklib_Corrections(int32_t selected_prn = 0);

    Sbas_Rtklib_Corrections(const Sbas_Rtklib_Corrections&) = delete;
    Sbas_Rtklib_Corrections& operator=(const Sbas_Rtklib_Corrections&) = delete;

    void push(const Sbas_Raw_Message& message);
    void update(int32_t gps_week, double gps_tow_s, double receiver_sample_stamp_s);
    void copy_to(nav_t& navigation_data);

    int32_t active_prn() const { return d_active_prn; }
    size_t pending_messages() const { return d_pending_messages.size(); }
    const sbssat_t& satellite_corrections() const { return d_navigation_data.sbssat; }
    const sbsion_t* ionosphere_corrections() const { return d_navigation_data.sbsion; }

private:
    static constexpr size_t MAX_PENDING_MESSAGES = 1200U;
    static constexpr double FUTURE_MESSAGE_TOLERANCE_S = 0.5;

    std::deque<Sbas_Raw_Message> d_pending_messages;
    std::array<seph_t, NSATSBS * 2> d_sbas_ephemerides{};
    nav_t d_navigation_data{};
    int32_t d_selected_prn{};
    int32_t d_active_prn{};
};

/** \} */
/** \} */

#endif  // GNSS_SDR_SBAS_RTKLIB_CORRECTIONS_H
