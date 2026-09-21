/*!
 * \file pvt_panel.cc
 * \brief FTXUI PVT information panel for the GNSS-SDR TUI.
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

#include "pvt_panel.h"
#include <ftxui/dom/elements.hpp>
#include <sstream>

ftxui::Element render_pvt_panel(const DataStore& store)
{
    using namespace ftxui;

    auto pvt = store.get_pvt();
    if (!pvt)
        {
            return vbox({
                text("PVT") | bold | center,
                separator(),
                text("Waiting for fix...") | dim | center,
            }) | border;
        }

    std::ostringstream lat_str, lon_str, h_str;
    lat_str.precision(5);
    lat_str << std::fixed << pvt->latitude;
    lon_str.precision(5);
    lon_str << std::fixed << pvt->longitude;
    h_str.precision(1);
    h_str << std::fixed << pvt->height;

    std::ostringstream hdop_str, vdop_str, pdop_str, gdop_str;
    hdop_str.precision(1);
    hdop_str << std::fixed << pvt->hdop;
    vdop_str.precision(1);
    vdop_str << std::fixed << pvt->vdop;
    pdop_str.precision(1);
    pdop_str << std::fixed << pvt->pdop;
    gdop_str.precision(1);
    gdop_str << std::fixed << pvt->gdop;

    return vbox({
        text("PVT Solution") | bold | center,
        separator(),
        hbox({text(" Lat: "), text(lat_str.str()) | bold}),
        hbox({text(" Lon: "), text(lon_str.str()) | bold}),
        hbox({text(" Hgt: "), text(h_str.str() + " m") | bold}),
        separator(),
        hbox({text(" HDOP: "), text(hdop_str.str())}),
        hbox({text(" VDOP: "), text(vdop_str.str())}),
        hbox({text(" PDOP: "), text(pdop_str.str())}),
        hbox({text(" GDOP: "), text(gdop_str.str())}),
        separator(),
        hbox({text(" Sats: "), text(std::to_string(pvt->valid_sats)) | bold}),
        hbox({text(" Time: "), text(pvt->utc_time) | dim}),
    }) | border;
}
