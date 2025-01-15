/*!
 * \file concurrent_map.h
 * \brief Interface of a thread-safe std::map
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_CONCURRENT_MAP_H
#define GNSS_SDR_CONCURRENT_MAP_H

#include <map>
#include <mutex>
#include <utility>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver core_receiver
 * \{ */


template <typename Data>


/*!
 * \brief This class implements a thread-safe std::map
 *
 */
class Concurrent_Map
{
public:
    void write(int key, Data const& data)
    {
        std::lock_guard<std::mutex> lock(the_mutex);
        auto data_iter = the_map.find(key);
        if (data_iter != the_map.end())
            {
                data_iter->second = data;  // update
            }
        else
            {
                the_map.insert(std::pair<int, Data>(key, data));  // insert does not overwrite if the item already exists in the map!
            }
    }

    std::map<int, Data> get_map_copy() const&
    {
        std::lock_guard<std::mutex> lock(the_mutex);
        return the_map;  // This implicitly creates a copy
    }

    std::map<int, Data> get_map_copy() &&
    {
        std::lock_guard<std::mutex> lock(the_mutex);
        return std::move(the_map);
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(the_mutex);
        return the_map.size();
    }

    bool read(int key, Data& p_data) const
    {
        std::lock_guard<std::mutex> lock(the_mutex);
        auto data_iter = the_map.find(key);
        if (data_iter != the_map.end())
            {
                p_data = data_iter->second;
                return true;
            }
        return false;
    }

private:
    std::map<int, Data> the_map;
    mutable std::mutex the_mutex;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CONCURRENT_MAP_H
