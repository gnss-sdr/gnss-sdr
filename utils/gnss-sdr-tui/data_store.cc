/*!
 * \file data_store.cc
 * \brief Thread-safe shared data model for the GNSS-SDR TUI.
 * \author Generated for GNSS-SDR contributors
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2025 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * -----------------------------------------------------------------------------
 */

#include "data_store.h"
#include <algorithm>

void DataStore::update_satellite(int channel, const SatelliteData& data)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    const bool is_new = satellites_.find(channel) == satellites_.end();
    const bool had_lock = !is_new && satellites_[channel].flag_pll_locked;
    satellites_[channel] = data;

    cn0_history_[channel].push_back(data.cn0_db_hz);
    if (cn0_history_[channel].size() > kMaxHistory)
        {
            cn0_history_[channel].pop_front();
        }

    if (is_new)
        {
            log_.push_back({std::chrono::system_clock::now(),
                data.system + std::to_string(data.prn) + " acquired on ch " + std::to_string(channel)});
        }
    else if (!had_lock && data.flag_pll_locked)
        {
            log_.push_back({std::chrono::system_clock::now(),
                data.system + std::to_string(data.prn) + " PLL locked on ch " + std::to_string(channel)});
        }
    else if (had_lock && !data.flag_pll_locked)
        {
            log_.push_back({std::chrono::system_clock::now(),
                data.system + std::to_string(data.prn) + " PLL unlock on ch " + std::to_string(channel)});
        }
    if (data.flag_cycle_slip)
        {
            log_.push_back({std::chrono::system_clock::now(),
                data.system + std::to_string(data.prn) + " cycle slip on ch " + std::to_string(channel)});
        }
    while (log_.size() > kMaxLogEntries) log_.pop_front();
}

SatelliteData DataStore::get_satellite(int channel) const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    auto it = satellites_.find(channel);
    if (it != satellites_.end())
        {
            return it->second;
        }
    return SatelliteData{};
}

std::map<int, SatelliteData> DataStore::get_all_satellites() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return satellites_;
}

std::vector<std::pair<int, SatelliteData>> DataStore::get_sorted_satellites() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::pair<int, SatelliteData>> vec(satellites_.begin(), satellites_.end());
    std::sort(vec.begin(), vec.end(),
        [](const auto& a, const auto& b) {
            if (a.second.system != b.second.system)
                return a.second.system < b.second.system;
            return a.second.prn < b.second.prn;
        });
    return vec;
}

size_t DataStore::satellite_count() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return satellites_.size();
}

void DataStore::record_cn0(int channel, double cn0)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    cn0_history_[channel].push_back(cn0);
    if (cn0_history_[channel].size() > kMaxHistory)
        {
            cn0_history_[channel].pop_front();
        }
}

std::deque<double> DataStore::get_cn0_history(int channel) const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    auto it = cn0_history_.find(channel);
    if (it != cn0_history_.end())
        {
            return it->second;
        }
    return {};
}

void DataStore::update_pvt(const PvtData& data)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    const bool first_fix = !pvt_.has_value();
    pvt_ = data;
    if (first_fix)
        {
            log_.push_back({std::chrono::system_clock::now(),
                "First PVT fix: " + std::to_string(data.latitude) + ", " + std::to_string(data.longitude)});
            while (log_.size() > kMaxLogEntries) log_.pop_front();
        }
}

std::optional<PvtData> DataStore::get_pvt() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return pvt_;
}

void DataStore::add_log(const std::string& msg)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    log_.push_back({std::chrono::system_clock::now(), msg});
    while (log_.size() > kMaxLogEntries) log_.pop_front();
}

std::deque<LogEntry> DataStore::get_logs(size_t count) const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (count == 0 || count >= log_.size())
        {
            return log_;
        }
    return std::deque<LogEntry>(log_.end() - static_cast<ptrdiff_t>(count), log_.end());
}

std::string constellation_color(const std::string& system)
{
    if (system == "G") return "GPS";
    if (system == "E") return "Galileo";
    if (system == "R") return "GLONASS";
    if (system == "C") return "BeiDou";
    if (system == "S") return "SBAS";
    return "Unknown";
}
