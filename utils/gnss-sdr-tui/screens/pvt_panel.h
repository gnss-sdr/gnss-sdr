/*!
 * \file pvt_panel.h
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

#ifndef GNSS_SDR_TUI_PVT_PANEL_H
#define GNSS_SDR_TUI_PVT_PANEL_H

#include "data_store.h"
#include <ftxui/dom/elements.hpp>

ftxui::Element render_pvt_panel(const DataStore& store);

#endif  // GNSS_SDR_TUI_PVT_PANEL_H
