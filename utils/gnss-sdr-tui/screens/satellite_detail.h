/*!
 * \file satellite_detail.h
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

#ifndef GNSS_SDR_TUI_SATELLITE_DETAIL_H
#define GNSS_SDR_TUI_SATELLITE_DETAIL_H

#include "data_store.h"
#include <ftxui/dom/elements.hpp>

ftxui::Element render_satellite_detail(const DataStore& store, int channel);

#endif  // GNSS_SDR_TUI_SATELLITE_DETAIL_H
