/*!
 * \file ntrip_rtk_signals.h
 * \brief Signal table of the NTRIP RTK path
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2026 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NTRIP_RTK_SIGNALS_H
#define GNSS_SDR_NTRIP_RTK_SIGNALS_H

#include "rtklib.h"
#include "signal_enabled_flags.h"
#include "signal_flag.h"
#include <array>
#include <cstdint>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */

/*!
 * Single home for the signal knowledge of the NTRIP RTK path: which tracked
 * signals it supports and, for each one, the constellation and the RTKLIB
 * frequency slot it occupies. The adapter's channel-set whitelist, its
 * derived slot count and navigation-system mask, and the solver's per-system
 * band mapping all derive from this table, so admitting a new signal (e.g.
 * BeiDou B3I or QZSS) starts with a one-row change here.
 */
struct Ntrip_Rtk_Signal
{
    uint32_t flag;    //!< signal_flag bit of the tracked channel signal
    int system;       //!< RTKLIB constellation bit (SYS_XXX)
    int band_slot;    //!< RTKLIB frequency slot the signal occupies (0-based)
    bool first_band;  //!< entry band of its constellation (defines the navsys mask)
};


/*!
 * Keep each constellation's second bands ordered by preference: with more
 * than one enabled (a configuration the adapter rejects today),
 * ntrip_rtk_second_band_slot() picks the first match.
 */
inline constexpr std::array<Ntrip_Rtk_Signal, 6> NTRIP_RTK_SIGNALS = {{
    {GPS_1C, SYS_GPS, 0, true},    // GPS L1 C/A
    {GPS_2S, SYS_GPS, 1, false},   // GPS L2C
    {GPS_L5, SYS_GPS, 2, false},   // GPS L5 (shares the third slot with E5a)
    {GAL_1B, SYS_GAL, 0, true},    // Galileo E1
    {GAL_E5a, SYS_GAL, 2, false},  // Galileo E5a
    {BDS_B1C, SYS_BDS, 0, true},   // BeiDou B1C
}};


//! Bitmask of every signal_flag the NTRIP RTK path supports
inline constexpr uint32_t ntrip_rtk_supported_signal_mask()
{
    uint32_t mask = 0;
    for (const auto& signal : NTRIP_RTK_SIGNALS)
        {
            mask |= signal.flag;
        }
    return mask;
}


//! RTKLIB navigation-system mask spanned by the enabled NTRIP RTK channels
inline int ntrip_rtk_navigation_systems(const Signal_Enabled_Flags& enabled)
{
    int navsys = 0;
    for (const auto& signal : NTRIP_RTK_SIGNALS)
        {
            if (signal.first_band && enabled.check_any_enabled(signal.flag))
                {
                    navsys |= signal.system;
                }
        }
    return navsys;
}


//! Number of RTKLIB frequency slots the enabled NTRIP RTK channels require
inline int ntrip_rtk_required_bands(const Signal_Enabled_Flags& enabled)
{
    int bands = 1;
    for (const auto& signal : NTRIP_RTK_SIGNALS)
        {
            if (enabled.check_any_enabled(signal.flag) && signal.band_slot + 1 > bands)
                {
                    bands = signal.band_slot + 1;
                }
        }
    return bands;
}


/*!
 * RTKLIB slot of a satellite system's second band given the enabled channels.
 * QZSS rides the GPS bands. When no second band of the system is enabled, the
 * constellation's canonical second slot is returned (Galileo/BeiDou: 2, both
 * anticipate slot-2 signals; GPS/QZSS: 1) so unsupported slots can be cleared.
 */
inline int ntrip_rtk_second_band_slot(int system, const Signal_Enabled_Flags& enabled)
{
    const int lookup_system = (system == SYS_QZS) ? SYS_GPS : system;
    for (const auto& signal : NTRIP_RTK_SIGNALS)
        {
            if (signal.system == lookup_system && !signal.first_band &&
                enabled.check_any_enabled(signal.flag))
                {
                    return signal.band_slot;
                }
        }
    return (lookup_system == SYS_GAL || lookup_system == SYS_BDS) ? 2 : 1;
}


/** \} */
/** \} */
#endif  // GNSS_SDR_NTRIP_RTK_SIGNALS_H
