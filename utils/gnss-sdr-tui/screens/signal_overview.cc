/*!
 * \file signal_overview.cc
 * \brief Signal overview panel for the GNSS-SDR TUI.
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

#include "signal_overview.h"
#include <algorithm>
#include <deque>
#include <map>
#include <sstream>
#include <vector>

namespace
{

std::string sparkline(const std::deque<double>& vals, double max_val)
{
    static const char* bar[] = {" ", "\xe2\x96\x81", "\xe2\x96\x82", "\xe2\x96\x83",
        "\xe2\x96\x84", "\xe2\x96\x85", "\xe2\x96\x86", "\xe2\x96\x87", "\xe2\x96\x88"};
    std::string out;
    for (auto v : vals)
        {
            int idx = std::min(8, std::max(0, static_cast<int>(v / max_val * 8.0)));
            out += bar[idx];
        }
    return out;
}

ftxui::Color color_for_sys(const std::string& sys)
{
    if (sys == "G") return ftxui::Color::GreenLight;
    if (sys == "E") return ftxui::Color::CornflowerBlue;
    if (sys == "R") return ftxui::Color::RedLight;
    if (sys == "C") return ftxui::Color::YellowLight;
    if (sys == "S") return ftxui::Color::Cyan;
    return ftxui::Color::Default;
}

}  // namespace

ftxui::Element render_signal_overview(const DataStore& store)
{
    using namespace ftxui;

    const auto sorted = store.get_sorted_satellites();
    const auto pvt = store.get_pvt();

    Elements left_lines;

    left_lines.push_back(text("Satellites: ") | bold);
    left_lines.push_back(text(std::to_string(sorted.size()) + " tracked") | dim);

    std::map<std::string, int> const_counts;
    for (const auto& [ch, sat] : sorted)
        {
            const_counts[constellation_color(sat.system)]++;
        }

    for (const auto& [name, count] : const_counts)
        {
            Color c = Color::Default;
            if (name == "GPS") c = Color::GreenLight;
            else if (name == "Galileo") c = Color::CornflowerBlue;
            else if (name == "GLONASS") c = Color::RedLight;
            else if (name == "BeiDou") c = Color::YellowLight;
            else if (name == "SBAS") c = Color::Cyan;
            left_lines.push_back(
                hbox({text(name) | color(c) | bold, text(": "), text(std::to_string(count))}));
        }

    if (!sorted.empty())
        {
            left_lines.push_back(separator());
            left_lines.push_back(text("Top signals:") | bold);

            std::vector<std::pair<int, SatelliteData>> top(sorted.begin(), sorted.end());
            std::sort(top.begin(), top.end(),
                [](const auto& a, const auto& b) {
                    return a.second.cn0_db_hz > b.second.cn0_db_hz;
                });

            for (size_t i = 0; i < std::min(top.size(), size_t(3)); ++i)
                {
                    const auto& sat = top[i].second;
                    std::ostringstream ss;
                    ss.precision(1);
                    ss << std::fixed << sat.cn0_db_hz;
                    int bars = static_cast<int>(sat.cn0_db_hz / 5.0);
                    std::string bar_str(bars, '#');
                    left_lines.push_back(
                        hbox({text(sat.system + std::to_string(sat.prn)) | color(color_for_sys(sat.system)),
                            text(" " + ss.str()) | bold,
                            text(" " + bar_str) | color(Color::GreenLight)}));
                }

            const auto& best = top[0].second;
            const auto hist = store.get_cn0_history(best.channel_id);
            if (!hist.empty())
                {
                    left_lines.push_back(separator());
                    left_lines.push_back(
                        text(best.system + std::to_string(best.prn) + " CN0 trend:") | dim);
                    left_lines.push_back(text(sparkline(hist, 55.0)));
                }
        }

    if (pvt)
        {
            left_lines.push_back(separator());
            left_lines.push_back(text("Position: ") | bold);
            left_lines.push_back(text("Lat: " + std::to_string(pvt->latitude).substr(0, 8)));
            left_lines.push_back(text("Lon: " + std::to_string(pvt->longitude).substr(0, 8)));
            left_lines.push_back(text("HDOP: " + std::to_string(pvt->hdop).substr(0, 4)));
        }

    return vbox(std::move(left_lines)) | border | size(WIDTH, GREATER_THAN, 24);
}
