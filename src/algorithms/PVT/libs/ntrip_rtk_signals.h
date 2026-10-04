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
#include <string>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */

/*!
 * Single home for the signal knowledge of the NTRIP RTK path: which tracked
 * signals it supports and, for each one, the constellation, the RTKLIB
 * frequency slot it occupies, and the channel signal code. The adapter's
 * channel-set validity predicate, its error message, its derived slot count
 * and navigation-system mask, and the client's per-slot observation filter
 * all derive from this table, and the solver cross-checks its own band
 * mapping against it at construction — so admitting a new SIGNAL of a
 * supported constellation is a one-row change (plus documentation and
 * tests; a slot-sharing preference rule like BeiDou's prefer-B1C stays an
 * explicit exception in the client). A new SYSTEM additionally needs
 * base-side admission in the client and the solver, and navigation-system
 * compatibility handling in the adapter.
 */
struct Ntrip_Rtk_Signal
{
    uint32_t flag;       //!< signal_flag bit of the tracked channel signal
    int system;          //!< RTKLIB constellation bit (SYS_XXX)
    int band_slot;       //!< RTKLIB frequency slot the signal occupies (0-based)
    const char* signal;  //!< two-character channel signal code (Gnss_Synchro convention)
};


//! The entry band of every constellation occupies RTKLIB's first slot, so
//! "first band" is band_slot == 0 by definition — a fact, not a column that
//! could drift from the slot assignment
inline bool ntrip_rtk_is_entry_band(const Ntrip_Rtk_Signal& signal)
{
    return signal.band_slot == 0;
}


/*!
 * The signal table. A function-local static keeps the header buildable as
 * C++11 (an inline variable would need C++17) while still providing a single
 * instance. Keep each constellation's second bands ordered by preference:
 * with more than one enabled (a configuration the adapter rejects today),
 * ntrip_rtk_second_band_slot() picks the first match.
 */
inline const std::array<Ntrip_Rtk_Signal, 6>& ntrip_rtk_signals()
{
    static const std::array<Ntrip_Rtk_Signal, 6> table = {{
        {GPS_1C, SYS_GPS, 0, "1C"},   // GPS L1 C/A
        {GPS_2S, SYS_GPS, 1, "2S"},   // GPS L2C
        {GPS_L5, SYS_GPS, 2, "L5"},   // GPS L5 (shares the third slot with E5a)
        {GAL_1B, SYS_GAL, 0, "1B"},   // Galileo E1
        {GAL_E5a, SYS_GAL, 2, "5X"},  // Galileo E5a
        {BDS_B1C, SYS_BDS, 0, "1D"},  // BeiDou B1C
    }};
    return table;
}


//! Bitmask of every signal_flag the NTRIP RTK path supports
inline uint32_t ntrip_rtk_supported_signal_mask()
{
    uint32_t mask = 0;
    for (const auto& signal : ntrip_rtk_signals())
        {
            mask |= signal.flag;
        }
    return mask;
}


//! RTKLIB navigation-system mask spanned by the enabled NTRIP RTK channels
inline int ntrip_rtk_navigation_systems(const Signal_Enabled_Flags& enabled)
{
    int navsys = 0;
    for (const auto& signal : ntrip_rtk_signals())
        {
            if (ntrip_rtk_is_entry_band(signal) && enabled.check_any_enabled(signal.flag))
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
    for (const auto& signal : ntrip_rtk_signals())
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
    for (const auto& signal : ntrip_rtk_signals())
        {
            if (signal.system == lookup_system && !ntrip_rtk_is_entry_band(signal) &&
                enabled.check_any_enabled(signal.flag))
                {
                    return signal.band_slot;
                }
        }
    // no enabled second band: the system's first listed second band from the
    // table, or the anticipated third slot for a constellation with no
    // second-band row yet (BeiDou B3I would live there)
    for (const auto& signal : ntrip_rtk_signals())
        {
            if (signal.system == lookup_system && !ntrip_rtk_is_entry_band(signal))
                {
                    return signal.band_slot;
                }
        }
    return 2;
}


//! Constellation label of an RTKLIB system bit, for messages
inline const char* ntrip_rtk_system_label(int system)
{
    switch (system)
        {
        case SYS_GPS:
            return "GPS";
        case SYS_GAL:
            return "Galileo";
        case SYS_BDS:
            return "BeiDou";
        case SYS_QZS:
            return "QZSS";
        default:
            return "unknown system";
        }
}


/*!
 * True when the enabled channel set follows the NTRIP RTK rules, all of
 * them derived from the table's columns: only signals the table lists, at
 * least one constellation entry band, at most one second band per system,
 * and every second band accompanied by its system's entry band.
 */
inline bool ntrip_rtk_valid_channel_set(const Signal_Enabled_Flags& enabled)
{
    if ((enabled.flags & ~ntrip_rtk_supported_signal_mask()) != 0)
        {
            return false;
        }
    bool any_first_band = false;
    for (const auto& signal : ntrip_rtk_signals())
        {
            if (!enabled.check_any_enabled(signal.flag))
                {
                    continue;
                }
            if (ntrip_rtk_is_entry_band(signal))
                {
                    any_first_band = true;
                    continue;
                }
            bool first_band_enabled = false;
            int enabled_second_bands = 0;
            for (const auto& sibling : ntrip_rtk_signals())
                {
                    if (sibling.system != signal.system)
                        {
                            continue;
                        }
                    if (ntrip_rtk_is_entry_band(sibling))
                        {
                            first_band_enabled = first_band_enabled ||
                                                 enabled.check_any_enabled(sibling.flag);
                        }
                    else if (enabled.check_any_enabled(sibling.flag))
                        {
                            ++enabled_second_bands;
                        }
                }
            if (!first_band_enabled || enabled_second_bands > 1)
                {
                    return false;
                }
        }
    return any_first_band;
}


//! Human-readable list of the supported per-system channel sets, derived
//! from the table for the adapter's rejection message
inline std::string ntrip_rtk_supported_sets_description()
{
    std::string description;
    for (const auto& signal : ntrip_rtk_signals())
        {
            if (!ntrip_rtk_is_entry_band(signal))
                {
                    continue;
                }
            if (!description.empty())
                {
                    description += "; ";
                }
            description += ntrip_rtk_system_label(signal.system);
            description += ' ';
            description += signal.signal;
            std::string second_bands;
            for (const auto& sibling : ntrip_rtk_signals())
                {
                    if (sibling.system != signal.system || ntrip_rtk_is_entry_band(sibling))
                        {
                            continue;
                        }
                    if (!second_bands.empty())
                        {
                            second_bands += " or ";
                        }
                    second_bands += sibling.signal;
                }
            description += second_bands.empty() ? " alone" : (" alone or with one of " + second_bands);
        }
    return description;
}


/** \} */
/** \} */
#endif  // GNSS_SDR_NTRIP_RTK_SIGNALS_H
