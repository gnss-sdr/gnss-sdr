/*!
 * \file satellite_detail.cc
 * \brief Detailed satellite info overlay for the GNSS-SDR TUI.
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

#include "satellite_detail.h"
#include <algorithm>
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

}  // namespace

ftxui::Element render_satellite_detail(const DataStore& store, int channel)
{
    using namespace ftxui;

    const auto sat = store.get_satellite(channel);
    if (sat.system.empty())
        {
            Elements out;
            out.push_back(text("Satellite Detail") | bold | center);
            out.push_back(separator());
            out.push_back(text("Satellite not found on channel " + std::to_string(channel)) | dim | center);
            out.push_back(separator());
            out.push_back(text("Press Enter or Esc to close") | dim | center);
            return vbox(std::move(out)) | border | center;
        }

    const auto history = store.get_cn0_history(channel);

    Elements lines;
    lines.push_back(text("Satellite Detail: " + sat.system + std::to_string(sat.prn)) | bold | center);
    lines.push_back(separator());

    auto field = [](const std::string& label, const std::string& value) {
        return hbox({text("  " + label + ": ") | dim, text(value) | bold});
    };

    lines.push_back(field("Channel", std::to_string(sat.channel_id)));
    lines.push_back(field("System", constellation_color(sat.system) + " (" + sat.system + ")"));
    lines.push_back(field("Signal", sat.signal));
    lines.push_back(field("PRN", std::to_string(sat.prn)));
    lines.push_back(separator());

    std::ostringstream cn0_s, dop_s, phase_s, pr_s;
    cn0_s.precision(1);
    cn0_s << std::fixed << sat.cn0_db_hz << " dB-Hz";
    dop_s.precision(1);
    dop_s << std::fixed << sat.carrier_doppler_hz << " Hz";
    phase_s.precision(2);
    phase_s << std::fixed << sat.carrier_phase_rads << " rad";
    pr_s.precision(3);
    pr_s << std::fixed << sat.pseudorange_m << " m";

    lines.push_back(field("CN0", cn0_s.str()));
    lines.push_back(field("Doppler", dop_s.str()));
    lines.push_back(field("Carrier Phase", phase_s.str()));
    lines.push_back(field("Pseudorange", pr_s.str()));
    lines.push_back(separator());

    lines.push_back(field("PLL Locked", sat.flag_pll_locked ? "YES" : "NO"));
    lines.push_back(field("Valid Pseudorange", sat.flag_valid_pseudorange ? "YES" : "NO"));
    lines.push_back(field("Cycle Slip", sat.flag_cycle_slip ? "YES" : "NO"));

    if (!history.empty())
        {
            lines.push_back(separator());
            lines.push_back(text("CN0 History (last " + std::to_string(history.size()) + " samples):") | dim);
            lines.push_back(text(sparkline(history, 55.0)));
        }

    lines.push_back(separator());
    lines.push_back(text("Press Enter or Esc to close") | dim | center);

    return vbox(std::move(lines)) | border | center;
}
