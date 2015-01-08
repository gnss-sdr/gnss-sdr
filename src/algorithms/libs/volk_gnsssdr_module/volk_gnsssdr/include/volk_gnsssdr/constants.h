/*!
 * \file constants.h
 * \brief volk_gnsssdr constants
 * \author Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *
 * Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
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
 */

#ifndef GNSS_SDR_VOLK_GNSSSDR_CONSTANTS_H
#define GNSS_SDR_VOLK_GNSSSDR_CONSTANTS_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>

__VOLK_DECL_BEGIN

VOLK_API char* volk_gnsssdr_prefix();
VOLK_API char* volk_gnsssdr_build_date();
VOLK_API char* volk_gnsssdr_version();
VOLK_API char* volk_gnsssdr_c_compiler();
VOLK_API char* volk_gnsssdr_compiler_flags();
VOLK_API char* volk_gnsssdr_available_machines();

__VOLK_DECL_END

#endif /* GNSS_SDR_VOLK_GNSSSDR_CONSTANTS_H */
