/*!
 * \file dump_logger_helper.h
 * \brief Utility functions for logging to a file
 * \authors <ul>
 *          <li> Mathieu Favreau, 2026. favreau.mathieu(at)hotmail.com
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_DUMP_LOGGER_HELPER_H
#define GNSS_SDR_DUMP_LOGGER_HELPER_H

#include <fstream>

template <typename T>
void write_value(std::ofstream& file, T value)
{
    file.write(reinterpret_cast<char*>(&value), sizeof(T));
}

#endif  // GNSS_SDR_DUMP_LOGGER_HELPER_H
