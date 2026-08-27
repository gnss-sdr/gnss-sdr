/*!
 * \file galileo_ephemeris_store.h
 * \brief Source-aware storage for Galileo I/NAV and F/NAV ephemerides
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2026 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_EPHEMERIS_STORE_H
#define GNSS_SDR_GALILEO_EPHEMERIS_STORE_H

#include "galileo_ephemeris.h"
#include <map>


class Galileo_Ephemeris_Store
{
public:
    using Ephemeris_Map = std::map<int, Galileo_Ephemeris>;

    bool insert(const Galileo_Ephemeris& ephemeris)
    {
        Ephemeris_Map* destination = mutable_map(ephemeris.nav_message_type);
        if (destination == nullptr)
            {
                return false;
            }
        (*destination)[static_cast<int>(ephemeris.PRN)] = ephemeris;
        return true;
    }

    const Galileo_Ephemeris* find(int prn, Galileo_Nav_Message_Type source) const
    {
        const Ephemeris_Map* source_map = map(source);
        if (source_map == nullptr)
            {
                return nullptr;
            }
        const auto ephemeris = source_map->find(prn);
        return ephemeris == source_map->cend() ? nullptr : &ephemeris->second;
    }

    const Ephemeris_Map& inav() const
    {
        return d_inav_ephemeris_map;
    }

    Ephemeris_Map& inav()
    {
        return d_inav_ephemeris_map;
    }

    const Ephemeris_Map& fnav() const
    {
        return d_fnav_ephemeris_map;
    }

    Ephemeris_Map& fnav()
    {
        return d_fnav_ephemeris_map;
    }

    const Ephemeris_Map& by_source(Galileo_Nav_Message_Type source) const
    {
        static const Ephemeris_Map empty_ephemeris_map;
        const Ephemeris_Map* source_map = map(source);
        return source_map == nullptr ? empty_ephemeris_map : *source_map;
    }

    Ephemeris_Map combined_view(Galileo_Nav_Message_Type preferred_source) const
    {
        Ephemeris_Map result;
        const Ephemeris_Map& secondary = preferred_source == Galileo_Nav_Message_Type::FNAV ? d_inav_ephemeris_map : d_fnav_ephemeris_map;
        const Ephemeris_Map& preferred = preferred_source == Galileo_Nav_Message_Type::FNAV ? d_fnav_ephemeris_map : d_inav_ephemeris_map;
        result.insert(secondary.cbegin(), secondary.cend());
        for (const auto& ephemeris : preferred)
            {
                result[ephemeris.first] = ephemeris.second;
            }
        return result;
    }

    bool empty() const
    {
        return d_inav_ephemeris_map.empty() && d_fnav_ephemeris_map.empty();
    }

    void clear()
    {
        d_inav_ephemeris_map.clear();
        d_fnav_ephemeris_map.clear();
    }

private:
    const Ephemeris_Map* map(Galileo_Nav_Message_Type source) const
    {
        if (source == Galileo_Nav_Message_Type::INAV)
            {
                return &d_inav_ephemeris_map;
            }
        if (source == Galileo_Nav_Message_Type::FNAV)
            {
                return &d_fnav_ephemeris_map;
            }
        return nullptr;
    }

    Ephemeris_Map* mutable_map(Galileo_Nav_Message_Type source)
    {
        if (source == Galileo_Nav_Message_Type::INAV)
            {
                return &d_inav_ephemeris_map;
            }
        if (source == Galileo_Nav_Message_Type::FNAV)
            {
                return &d_fnav_ephemeris_map;
            }
        return nullptr;
    }

    Ephemeris_Map d_inav_ephemeris_map;
    Ephemeris_Map d_fnav_ephemeris_map;
};

#endif  // GNSS_SDR_GALILEO_EPHEMERIS_STORE_H
