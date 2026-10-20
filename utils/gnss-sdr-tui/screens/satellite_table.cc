/*!
 * \file satellite_table.cc
 * \brief FTXUI satellite status table for the GNSS-SDR TUI.
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

#include "satellite_table.h"
#include <ftxui/dom/table.hpp>
#include <algorithm>
#include <sstream>
#include <vector>

namespace
{

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

ftxui::Element render_satellite_table(const DataStore& store, int selected_row)
{
    using namespace ftxui;

    const auto sorted = store.get_sorted_satellites();

    using Row = std::vector<std::string>;
    std::vector<Row> rows;

    rows.push_back({"PRN", "Sys", "Sig", "CN0", "Doppler", "Lock"});

    for (const auto& [ch, sat] : sorted)
        {
            std::ostringstream prn_str;
            prn_str << sat.system << std::to_string(sat.prn);
            std::ostringstream cn0_str;
            cn0_str.precision(1);
            cn0_str << std::fixed << sat.cn0_db_hz;
            std::ostringstream dop_str;
            dop_str.precision(0);
            dop_str << std::fixed << sat.carrier_doppler_hz;
            rows.push_back({prn_str.str(), sat.system, sat.signal,
                cn0_str.str(), dop_str.str(),
                sat.flag_pll_locked ? "LOCK" : "----"});
        }

    auto table = ftxui::Table(rows);
    table.SelectAll().Border(ftxui::ROUNDED);

    if (rows.size() > 1)
        {
            table.SelectRow(0).Decorate(ftxui::bold);
            table.SelectRow(0).Separator(ftxui::HEAVY);

            table.SelectAll().DecorateCellsAlternateRow(bgcolor(Color::Black), 2, 0);
            table.SelectAll().DecorateCellsAlternateRow(bgcolor(Color::GrayDark), 2, 1);

            for (size_t i = 1; i < rows.size(); ++i)
                {
                    const auto& sat = sorted[i - 1].second;
                    const auto fc = color_for_sys(sat.system);
                    table.SelectCell(0, static_cast<int>(i)).Decorate(color(fc) | bold);
                    table.SelectCell(1, static_cast<int>(i)).Decorate(color(fc));
                    if (sat.flag_pll_locked)
                        {
                            table.SelectCell(5, static_cast<int>(i)).Decorate(color(Color::GreenLight) | bold);
                        }
                    else
                        {
                            table.SelectCell(5, static_cast<int>(i)).Decorate(color(Color::RedLight));
                        }

                    if (static_cast<int>(i) == selected_row)
                        {
                            table.SelectRow(static_cast<int>(i)).Decorate(bgcolor(Color::DarkBlue));
                        }
                }
        }

    auto element = table.Render() | ftxui::flex;
    return element;
}
