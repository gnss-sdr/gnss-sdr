/*!
 * \file gnss_sdr_create_directory.h
 * \brief Create a directory
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GNSS_SDR_CREATE_DIRECTORY_H
#define GNSS_SDR_GNSS_SDR_CREATE_DIRECTORY_H

#include <string>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


bool gnss_sdr_create_directory(const std::string& foldername);


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SDR_CREATE_DIRECTORY_H
