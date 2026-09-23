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
    static volk_gnsssdr::vector<T> take()
    {
        BufferPool& self = get();
        std::lock_guard<std::mutex> lock(self.d_mutex);
        if (self.d_store.empty())
            {
                throw std::runtime_error("BufferPool<" + std::string(typeid(T).name()) + ">::take(): the pool is empty");
            }
        volk_gnsssdr::vector<T> rv = std::move(self.d_store.back());
        self.d_store.pop_back();
// debug
#if 0
        std::cout<<&self<<" take["<<self.d_store.size()<<"]: "<<(&rv)<<" s="<<rv.size()<<" d="<<rv.data()<<"\n";
#endif
        return rv;
    }
    static void release(volk_gnsssdr::vector<T>&& item)
    {
        BufferPool& self = get();
        std::lock_guard<std::mutex> lock(self.d_mutex);
// debug
#if 0
        std::cout<<&self<<" release["<<self.d_store.size()<<"]: "<<(&item)<<" s="<<item.size()<<" d="<<item.data()<<"\n";
#endif
        self.d_store.emplace_back(std::move(item));
    }
    static void resize(size_t n)
    {
        BufferPool& self = get();
        std::lock_guard<std::mutex> lock(self.d_mutex);
        self.d_store.resize(n);
        if (self.d_buffer_capacity > self.d_buffer_size)
            {
                self.foreach_unsafe([&](volk_gnsssdr::vector<T>& buffer) {
                    buffer.reserve(self.d_buffer_capacity);
                });
            }
        if (self.d_buffer_size)
            {
                self.foreach_unsafe([&](volk_gnsssdr::vector<T>& buffer) {
                    buffer.resize(self.d_buffer_size);
                });
            }
    }
    static void reserve(size_t n)
    {
        BufferPool& self = get();
        std::lock_guard<std::mutex> lock(self.d_mutex);
        self.d_store.reserve(n);
    }
    static size_t size()
    {
        BufferPool& self = get();
        std::lock_guard<std::mutex> lock(self.d_mutex);
        return self.d_store.size();
    }
    static void foreach (std::function<void(volk_gnsssdr::vector<T>&)> fn)
    {
        BufferPool& self = get();
        std::lock_guard<std::mutex> lock(self.d_mutex);
        self.foreach_unsafe(fn);
    }
    static void resize_buffers(size_t n)
    {
        BufferPool& self = get();
        std::lock_guard<std::mutex> lock(self.d_mutex);
        if (self.d_buffer_size == n)
            {
                return;
            }
        self.d_buffer_size = n;
        self.d_buffer_capacity = std::max(self.d_buffer_capacity, self.d_buffer_size);
        if (self.d_store.empty())
            {
                return;
            }
        self.foreach_unsafe([&](volk_gnsssdr::vector<T>& buffer) {
            buffer.resize(n);
        });
    }
    static void reserve_buffers(size_t n)
    {
        BufferPool& self = get();
        std::lock_guard<std::mutex> lock(self.d_mutex);
        if (self.d_buffer_capacity >= n)
            {
                return;
            }
        self.d_buffer_capacity = n;
        if (self.d_store.empty())
            {
                return;
            }
        self.foreach_unsafe([&](volk_gnsssdr::vector<T>& buffer) {
            buffer.reserve(n);
        });
    }
    static size_t buffer_size()
    {
        BufferPool& self = get();
        std::lock_guard<std::mutex> lock(self.d_mutex);
        if (self.d_store.empty())
            {
                return 0;
            }
        return self.d_store[0].size();
    }
    static size_t buffer_capacity()
    {
        BufferPool& self = get();
        std::lock_guard<std::mutex> lock(self.d_mutex);
        if (self.d_store.empty())
            {
                return 0;
            }
        return self.d_store[0].capacity();
    }

private:
    BufferPool() = default;
    static BufferPool& get()
    {
        static BufferPool inst{};
        return inst;
    }
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
