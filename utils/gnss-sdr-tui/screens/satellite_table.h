/*!
 * \file satellite_table.h
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

#ifndef GNSS_SDR_TUI_SATELLITE_TABLE_H
#define GNSS_SDR_TUI_SATELLITE_TABLE_H

#include "data_store.h"
#include <ftxui/dom/elements.hpp>

ftxui::Element render_satellite_table(const DataStore& store, int selected_row = -1);

#endif  // GNSS_SDR_TUI_SATELLITE_TABLE_H
