/*!
 * \file concurrent_map.h
 * \brief Interface of a thread-safe std::map
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CONCURRENT_MAP_H
#define GNSS_SDR_CONCURRENT_MAP_H

#include <map>
#include <mutex>
#include <utility>


template <typename Data>


/*!
 * \brief This class implements a thread-safe std::map
 *
 */
class Concurrent_Map
{
    typedef typename std::map<int, Data>::iterator Data_iterator;  // iterator is scope dependent
public:
    void write(int key, Data const& data)
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        Data_iterator data_iter;
        data_iter = the_map.find(key);
        if (data_iter != the_map.end())
            {
                data_iter->second = data;  // update
            }
        else
            {
                the_map.insert(std::pair<int, Data>(key, data));  // insert SILENTLY fails if the item already exists in the map!
            }
        lock.unlock();
    }

    std::map<int, Data> get_map_copy()
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        std::map<int, Data> map_aux = the_map;
        lock.unlock();
        return map_aux;
    }

    size_t size()
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        size_t size_ = the_map.size();
        lock.unlock();
        return size_;
    }

    bool read(int key, Data& p_data)
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        Data_iterator data_iter;
        data_iter = the_map.find(key);
        if (data_iter != the_map.end())
            {
                p_data = data_iter->second;
                lock.unlock();
                return true;
            }
        lock.unlock();
        return false;
    }

private:
    std::map<int, Data> the_map;
    mutable std::mutex the_mutex;
};

#endif
