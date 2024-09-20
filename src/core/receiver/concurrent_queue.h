/*!
 * \file concurrent_queue.h
 * \brief Interface of a thread-safe std::queue
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

#ifndef GNSS_SDR_CONCURRENT_QUEUE_H
#define GNSS_SDR_CONCURRENT_QUEUE_H

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <queue>
#include <utility>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver
 * \{ */


template <typename Data>

/*!
 * \brief This class implements a thread-safe std::queue
 */
class Concurrent_Queue
{
public:
    void push(const Data& data)
    {
        {
            std::lock_guard<std::mutex> lock(the_mutex);
            the_queue.push(data);
        }
        the_condition_variable.notify_one();
    }

    void push(Data&& data)
    {
        {
            std::lock_guard<std::mutex> lock(the_mutex);
            the_queue.push(std::move(data));
        }
        the_condition_variable.notify_one();
    }

    bool empty() const noexcept
    {
        return size() == 0;
    }

    size_t size() const noexcept
    {
        std::lock_guard<std::mutex> lock(the_mutex);
        return the_queue.size();
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(the_mutex);
        std::queue<Data>().swap(the_queue);
    }

    bool try_pop(Data& popped_value)
    {
        std::lock_guard<std::mutex> lock(the_mutex);
        if (the_queue.empty())
            {
                return false;
            }
        popped_value = std::move(the_queue.front());
        the_queue.pop();
        return true;
    }

    void wait_and_pop(Data& popped_value)
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        the_condition_variable.wait(lock, [this] { return !the_queue.empty(); });
        popped_value = std::move(the_queue.front());
        the_queue.pop();
    }

    bool timed_wait_and_pop(Data& popped_value, int wait_ms)
    {
        std::unique_lock<std::mutex> lock(the_mutex);
        if (!the_condition_variable.wait_for(lock,
                std::chrono::milliseconds(wait_ms),
                [this] { return !the_queue.empty(); }))
            {
                return false;
            }
        popped_value = std::move(the_queue.front());
        the_queue.pop();
        return true;
    }

private:
    std::queue<Data> the_queue;
    mutable std::mutex the_mutex;
    std::condition_variable the_condition_variable;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CONCURRENT_QUEUE_H
