/*!
 * \file gnss_sdr_create_directory.h
 * \brief Create a directory
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GNSS_SDR_CREATE_DIRECTORY_H
#define GNSS_SDR_GNSS_SDR_CREATE_DIRECTORY_H

#include <string>

bool gnss_sdr_create_directory(const std::string& foldername);

#endif
