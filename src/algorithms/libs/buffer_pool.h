/*!
 * \file buffer_pool.h
 * \brief Generic thread-safe volk_gnsssdr::vector-based buffer pool interface
 * \author Vladislav P, 2026. vladisslav2011(at)gmail.com
 *
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

#ifndef GNSS_SDR_BUFFER_POOL_H_
#define GNSS_SDR_BUFFER_POOL_H_

#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <iostream>
#include <mutex>
#include <vector>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */

template <class T>
class BufferPool
{
public:
    BufferPool(BufferPool&) = delete;
    BufferPool(BufferPool&&) = delete;
    BufferPool(const BufferPool&) = delete;
    BufferPool& operator=(const BufferPool&) = delete;
    BufferPool& operator=(BufferPool&) = delete;
    BufferPool& operator=(BufferPool&&) = delete;
    static BufferPool& instance()
    {
        static BufferPool inst{};
        return inst;
    }
    volk_gnsssdr::vector<T> take()
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (d_store.empty())
            {
                throw std::runtime_error("BufferPool<" + std::string(typeid(T).name()) + ">::take(): the pool is empty");
            }
        volk_gnsssdr::vector<T> rv = std::move(d_store.back());
        d_store.pop_back();
// debug
#if 0
        std::cout<<(void*)this<<" take["<<d_store.size()<<"]: "<<(&rv)<<" s="<<rv.size()<<" d="<<rv.data()<<"\n";
#endif
        return rv;
    }
    void release(volk_gnsssdr::vector<T>&& item)
    {
        std::lock_guard<std::mutex> lock(d_mutex);
// debug
#if 0
        std::cout<<(void*)this<<" release["<<d_store.size()<<"]: "<<(&item)<<" s="<<item.size()<<" d="<<item.data()<<"\n";
#endif
        d_store.emplace_back(std::move(item));
    }
    void resize(size_t n)
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        d_store.resize(n);
        if (d_buffer_capacity > d_buffer_size)
            {
                foreach_unsafe([&](volk_gnsssdr::vector<T>& buffer) {
                    buffer.reserve(d_buffer_capacity);
                });
            }
        if (d_buffer_size)
            {
                foreach_unsafe([&](volk_gnsssdr::vector<T>& buffer) {
                    buffer.resize(d_buffer_size);
                });
            }
    }
    void reserve(size_t n)
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        d_store.reserve(n);
    }
    size_t size()
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        return d_store.size();
    }
    void foreach (std::function<void(volk_gnsssdr::vector<T>&)> fn)
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        foreach_unsafe(fn);
    }
    void resize_buffers(size_t n)
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (d_buffer_size == n)
            {
                return;
            }
        d_buffer_size = n;
        d_buffer_capacity = std::max(d_buffer_capacity, d_buffer_size);
        if (d_store.empty())
            {
                return;
            }
        foreach_unsafe([&](volk_gnsssdr::vector<T>& buffer) {
            buffer.resize(n);
        });
    }
    void reserve_buffers(size_t n)
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (d_buffer_capacity >= n)
            {
                return;
            }
        d_buffer_capacity = n;
        if (d_store.empty())
            {
                return;
            }
        foreach_unsafe([&](volk_gnsssdr::vector<T>& buffer) {
            buffer.reserve(n);
        });
    }
    size_t buffer_size()
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (d_store.empty())
            {
                return 0;
            }
        return d_store[0].size();
    }
    size_t buffer_capacity()
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (d_store.empty())
            {
                return 0;
            }
        return d_store[0].capacity();
    }

private:
    BufferPool() = default;
    void foreach_unsafe(std::function<void(volk_gnsssdr::vector<T>&)> fn)
    {
        for (auto& it : d_store)
            {
                fn(it);
            }
    }
    std::vector<volk_gnsssdr::vector<T>> d_store{};
    size_t d_buffer_size{};
    size_t d_buffer_capacity{};
    std::mutex d_mutex{};
};

/** \} */
/** \} */
#endif  // GNSS_SDR_BUFFER_POOL_H_
