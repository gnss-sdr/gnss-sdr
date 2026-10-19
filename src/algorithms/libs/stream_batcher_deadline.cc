/*!
 * \file stream_batcher_deadline.cc
 * \brief Shared cancellable deadline service for stream publication
 * \author Carles Fernandez Prades, 2026 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "stream_batcher_deadline.h"
#include <condition_variable>
#include <map>
#include <mutex>
#include <thread>
#include <utility>

class StreamBatcherDeadline::Impl
{
public:
    struct Entry
    {
        std::function<void(uint64_t)> notify;
        std::chrono::steady_clock::time_point deadline;
        uint64_t generation = 0;
        bool enabled = false;
        bool armed = false;
        bool pending = false;
    };

    Impl() : worker_([this]() { run(); }) {}
    ~Impl()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        changed_.notify_one();
        worker_.join();
    }

    std::mutex mutex_;
    std::condition_variable changed_;
    std::map<uint64_t, Entry> entries_;
    uint64_t next_id_ = 0;

private:
    void run()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!stopping_)
            {
                auto next = std::chrono::steady_clock::time_point::max();
                const auto now = std::chrono::steady_clock::now();
                for (auto& item : entries_)
                    {
                        auto& entry = item.second;
                        if (!entry.enabled || !entry.armed || entry.pending)
                            {
                                continue;
                            }
                        if (entry.deadline <= now)
                            {
                                entry.armed = false;
                                entry.pending = true;
                                // Serialize posting with remove(): after removal returns,
                                // no callback can access the owning block. The callback
                                // only queues a message; it must not call this service.
                                entry.notify(entry.generation);
                            }
                        else if (entry.deadline < next)
                            {
                                next = entry.deadline;
                            }
                    }
                if (next == std::chrono::steady_clock::time_point::max())
                    {
                        changed_.wait(lock);
                    }
                else
                    {
                        changed_.wait_until(lock, next);
                    }
            }
    }

    bool stopping_ = false;
    std::thread worker_;
};


StreamBatcherDeadline::StreamBatcherDeadline() : impl_(new Impl()) {}
StreamBatcherDeadline::~StreamBatcherDeadline() = default;


uint64_t StreamBatcherDeadline::add(std::function<void(uint64_t)> notify)
{
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    const uint64_t id = ++impl_->next_id_;
    impl_->entries_[id].notify = std::move(notify);
    return id;
}


void StreamBatcherDeadline::remove(uint64_t id)
{
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    impl_->entries_.erase(id);
    impl_->changed_.notify_one();
}


void StreamBatcherDeadline::enable(uint64_t id, bool enabled)
{
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    auto& entry = impl_->entries_.at(id);
    entry.enabled = enabled;
    entry.armed = false;
    if (enabled)
        {
            // A new scheduler run may discard old queued messages. Do not let
            // an unacknowledged notification from that run suppress its timer.
            entry.pending = false;
        }
    impl_->changed_.notify_one();
}


void StreamBatcherDeadline::arm(uint64_t id, uint64_t generation, std::chrono::steady_clock::time_point deadline)
{
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    auto& entry = impl_->entries_.at(id);
    if (entry.enabled)
        {
            entry.generation = generation;
            entry.deadline = deadline;
            entry.armed = true;
            impl_->changed_.notify_one();
        }
}


void StreamBatcherDeadline::cancel(uint64_t id)
{
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    impl_->entries_.at(id).armed = false;
    impl_->changed_.notify_one();
}


void StreamBatcherDeadline::acknowledge(uint64_t id)
{
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    impl_->entries_.at(id).pending = false;
    impl_->changed_.notify_one();
}
