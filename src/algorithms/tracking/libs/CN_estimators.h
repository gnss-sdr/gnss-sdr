/*!
 * \file CN_estimators.h
 * \brief Library with a set of Carrier to Noise estimators and lock detectors
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Library with a set of Carrier to Noise estimators and lock detectors
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef GNSS_SDR_CN_ESTIMATORS_H_
#define GNSS_SDR_CN_ESTIMATORS_H_

#include <gnuradio/gr_complex.h>

float gps_l1_ca_CN0_SNV(gr_complex* Prompt_buffer, int length, long fs_in);

float carrier_lock_detector(gr_complex* Prompt_buffer, int length);

#endif
