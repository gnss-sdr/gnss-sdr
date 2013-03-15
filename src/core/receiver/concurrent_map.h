/*!
 * \file concurrent_queue.h
 * \brief Interface of a thread-safe std::map
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CONCURRENT_MAP_H
#define GNSS_SDR_CONCURRENT_MAP_H

#include <map>

template<typename Data>


/*!
 * \brief This class implements a thread-safe std::map
 *
 */
class concurrent_map
{
	typedef typename std::map<int,Data>::iterator Data_iterator; //iterator is ambit dependant
private:
    std::map<int,Data> the_map;
    boost::mutex the_mutex;
public:
    void write(int key, Data const& data)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        the_map.insert(std::pair<int,Data>(key,data));
        lock.unlock();
    }

    std::map<int,Data> get_map_copy()
       {
           boost::mutex::scoped_lock lock(the_mutex);
           return the_map;
           lock.unlock();
       }

    int size()
    {
        boost::mutex::scoped_lock lock(the_mutex);
        return the_map.size();
        lock.unlock();
    }

    bool read(int key, Data& p_data)
    {
      boost::mutex::scoped_lock lock(the_mutex);
      Data_iterator data_iter;
 	  data_iter = the_map.find(key);
      if (data_iter != the_map.end())
        {
    	  p_data= data_iter->second;
          lock.unlock();
    	  return true;
        }else{
            lock.unlock();
            return false;
        }
    }
};
#endif
