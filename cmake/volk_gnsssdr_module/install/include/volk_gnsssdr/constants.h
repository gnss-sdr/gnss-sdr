/*!
 * \file constants.h
 * \brief Definition of VOLK_GNSSSDR-related constants
 * \author Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_VOLK_GNSSSDR_CONSTANTS_H
#define INCLUDED_VOLK_GNSSSDR_CONSTANTS_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>

__VOLK_DECL_BEGIN

VOLK_API const char* volk_gnsssdr_prefix();
VOLK_API const char* volk_gnsssdr_version();
VOLK_API const char* volk_gnsssdr_c_compiler();
VOLK_API const char* volk_gnsssdr_compiler_flags();
VOLK_API const char* volk_gnsssdr_available_machines();

__VOLK_DECL_END

#endif /* INCLUDED_VOLK_GNSSSDR_CONSTANTS_H */
