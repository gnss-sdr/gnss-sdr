/*!
 * \file gn3s_api.h
 * \brief GNU Radio source block to acces to SiGe GN3S USB sampler.
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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

#ifndef INCLUDED_GN3S_API_H
#define INCLUDED_GN3S_API_H

#include <gnuradio/attributes.h>

#ifdef gr_gn3s_EXPORTS
#  define GN3S_API __GR_ATTR_EXPORT
#else
#  define GN3S_API __GR_ATTR_IMPORT
#endif

#endif /* INCLUDED_GN3S_API_H */
