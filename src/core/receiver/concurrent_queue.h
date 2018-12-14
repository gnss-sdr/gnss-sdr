/*!
 * \file concurrent_queue.h
 * \brief Interface of a thread-safe std::queue
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_CONCURRENT_QUEUE_H
#define GNSS_SDR_CONCURRENT_QUEUE_H

#include <boost/thread.hpp>
#include <queue>

template <typename Data>

/*!
 * \brief This class implements a thread-safe std::queue
 *
 * Thread-safe object queue which uses the library
 * boost_thread to perform MUTEX based on the code available at
 * http://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html
 */
class concurrent_queue
{
private:
    std::queue<Data> the_queue;
    mutable boost::mutex the_mutex;
    boost::condition_variable the_condition_variable;

public:
    void push(Data const& data)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        the_queue.push(data);
        lock.unlock();
        the_condition_variable.notify_one();
    }

    bool empty() const
    {
        boost::mutex::scoped_lock lock(the_mutex);
        return the_queue.empty();
    }

    bool try_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if (the_queue.empty())
            {
                return false;
            }
        popped_value = the_queue.front();
        the_queue.pop();
        return true;
    }

    void wait_and_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        while (the_queue.empty())
            {
                the_condition_variable.wait(lock);
            }
        popped_value = the_queue.front();
        the_queue.pop();
    }
};
#endif
