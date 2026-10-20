/*!
 * \file signal_overview.h
 * \brief FTXUI Canvas-based signal overview for the GNSS-SDR TUI.
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

#ifndef GNSS_SDR_TUI_SIGNAL_OVERVIEW_H
#define GNSS_SDR_TUI_SIGNAL_OVERVIEW_H

#include "data_store.h"
#include <ftxui/dom/elements.hpp>

ftxui::Element render_signal_overview(const DataStore& store);

#endif  // GNSS_SDR_TUI_SIGNAL_OVERVIEW_H
