/*!
 * \file event_log.cc
 * \brief FTXUI event log panel for the GNSS-SDR TUI.
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

#include "event_log.h"
#include <ctime>
#include <iomanip>
#include <sstream>
#include <vector>

ftxui::Element render_event_log(const DataStore& store)
{
    using namespace ftxui;

    const auto logs = store.get_logs(20);
    if (logs.empty())
        {
            Elements out;
            out.push_back(text("Events") | bold | center);
            out.push_back(separator());
            out.push_back(text("No events yet...") | dim | center);
            return vbox(std::move(out)) | border | size(HEIGHT, LESS_THAN, 6);
        }

    Elements inner;
    for (const auto& entry : logs)
        {
            const auto tt = std::chrono::system_clock::to_time_t(entry.timestamp);
            std::tm tm{};
            localtime_r(&tt, &tm);
            std::ostringstream ts;
            ts << std::put_time(&tm, "%H:%M:%S");
            inner.push_back(text(ts.str() + " " + entry.message) | dim);
        }

    Elements out;
    out.push_back(text("Events") | bold | center);
    out.push_back(separator());
    out.push_back(vbox(std::move(inner)));
    return vbox(std::move(out)) | border | size(HEIGHT, LESS_THAN, 6);
}
