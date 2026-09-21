/*!
 * \file data_store.h
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

#ifndef GNSS_SDR_TUI_DATA_STORE_H
#define GNSS_SDR_TUI_DATA_STORE_H

#include <chrono>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

struct SatelliteData
{
    std::string system;
    std::string signal;
    uint32_t prn{};
    int32_t channel_id{};
    double cn0_db_hz{};
    double carrier_doppler_hz{};
    double carrier_phase_rads{};
    double pseudorange_m{};
    double azimuth_deg{};
    double elevation_deg{};
    bool flag_valid_pseudorange{};
    bool flag_pll_locked{};
    bool flag_cycle_slip{};
};

struct PvtData
{
    double latitude{};
    double longitude{};
    double height{};
    double hdop{};
    double vdop{};
    double pdop{};
    double gdop{};
    uint32_t valid_sats{};
    std::string utc_time;
};

struct LogEntry
{
    std::chrono::system_clock::time_point timestamp;
    std::string message;
};

class DataStore
{
public:
    void update_satellite(int channel, const SatelliteData& data);
    SatelliteData get_satellite(int channel) const;
    std::map<int, SatelliteData> get_all_satellites() const;
    std::vector<std::pair<int, SatelliteData>> get_sorted_satellites() const;
    size_t satellite_count() const;

    void record_cn0(int channel, double cn0);
    std::deque<double> get_cn0_history(int channel) const;

    void update_pvt(const PvtData& data);
    std::optional<PvtData> get_pvt() const;

    void add_log(const std::string& msg);
    std::deque<LogEntry> get_logs(size_t count = 0) const;

private:
    mutable std::mutex mutex_;
    std::map<int, SatelliteData> satellites_;
    std::map<int, std::deque<double>> cn0_history_;
    std::optional<PvtData> pvt_;
    std::deque<LogEntry> log_;
    static constexpr size_t kMaxLogEntries = 200;
    static constexpr size_t kMaxHistory = 30;
};

std::string constellation_color(const std::string& system);

#endif  // GNSS_SDR_TUI_DATA_STORE_H
