/*!
 * \file gnss_sdr_flags.cc
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


#include <gnss_sdr_flags.h>


DEFINE_string(c, "-", "Path to the configuration file (if set, overrides --config_file)");

DEFINE_string(s, "-",
        "If defined, path to the file containing the signal samples (overrides the configuration file and --signal_source)");

DEFINE_string(signal_source, "-",
        "If defined, path to the file containing the signal samples (overrides the configuration file)");

DEFINE_string(RINEX_version, "3.02", "Specifies the RINEX version (2.11 or 3.02)");
