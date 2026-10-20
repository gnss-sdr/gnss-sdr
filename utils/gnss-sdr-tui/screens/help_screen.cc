/*!
 * \file help_screen.cc
 * \brief Help overlay for the GNSS-SDR TUI.
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

#include "help_screen.h"
#include <vector>

ftxui::Element render_help_screen()
{
    ftxui::Elements out;
    out.push_back(ftxui::text("GNSS-SDR TUI Help") | ftxui::bold | ftxui::center);
    out.push_back(ftxui::separator());

    auto make_row = [](const std::string& key, const std::string& desc) {
        ftxui::Elements row;
        row.push_back(ftxui::text("  " + key));
        row.push_back(ftxui::text(desc) | ftxui::dim);
        return ftxui::hbox(std::move(row));
    };

    out.push_back(make_row("q          ", "Quit"));
    out.push_back(make_row("h          ", "Toggle this help"));
    out.push_back(make_row("+ / -      ", "Increase / decrease refresh rate"));
    out.push_back(ftxui::separator());
    out.push_back(ftxui::text("Color legend:"));
    out.push_back(ftxui::hbox({ftxui::text("  G ")
                | ftxui::bgcolor(ftxui::Color::Green),
            ftxui::text(" GPS   "),
            ftxui::text("  E  ")
                | ftxui::bgcolor(ftxui::Color::Blue),
            ftxui::text(" Galileo   "),
            ftxui::text("  R  ")
                | ftxui::bgcolor(ftxui::Color::RedLight),
            ftxui::text(" GLONASS")}));
    out.push_back(ftxui::hbox({ftxui::text("  C  ")
                | ftxui::bgcolor(ftxui::Color::Yellow),
            ftxui::text(" BeiDou   "),
            ftxui::text("  S  ")
                | ftxui::bgcolor(ftxui::Color::Cyan),
            ftxui::text(" SBAS")}));
    out.push_back(ftxui::separator());
    out.push_back(ftxui::text("Press h to close") | ftxui::dim | ftxui::center);

    return ftxui::vbox(std::move(out)) | ftxui::border | ftxui::center;
}
