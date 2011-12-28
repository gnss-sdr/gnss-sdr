/*!
 * \file tracking_discriminators.h
 * \brief Library with a set of code tracking and carrier tracking disctiminators
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Library with a set of code tracking and carrier tracking disctiminators that is used by the tracking algorithms
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

#ifndef GNSS_SDR_TRACKING_DISCRIMINATORS_H_
#define GNSS_SDR_TRACKING_DISCRIMINATORS_H_

#include <gnuradio/gr_complex.h>

float fll_four_quadrant_atan(gr_complex prompt_s1, gr_complex prompt_s2,float t1, float t2);

float pll_four_quadrant_atan(gr_complex prompt_s1);

float pll_cloop_two_quadrant_atan(gr_complex prompt_s1);

float dll_nc_e_minus_l_normalized(gr_complex early_s1, gr_complex late_s1);


#endif
