/*!
 * \file gnss_sdr_flags.h
 * \brief Helper file for gnss-sdr commandline flags
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FLAGS_H_
#define GNSS_SDR_FLAGS_H_


#include <gflags/gflags.h>


DECLARE_string(c);                  //<! path to the configuration file
DECLARE_string(config_file);        //<! path to the configuration file

DECLARE_string(log_dir);            //<! path to the folder in which logging will be stored

// Declare flags for signal sources
DECLARE_string(s);                  //<! path to the file containing the signal samples
DECLARE_string(signal_source);      //<! path to the file containing the signal samples

//PVT
DECLARE_string(RINEX_version);      //<! RINEX version


#endif
