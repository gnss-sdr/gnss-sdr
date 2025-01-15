/*!
 * \file gnss_sdr_string_literals.h
 * \brief This file implements the ""s operator for std::string in C++11, and
 * puts it into the std::string_literals namespace. This is already implemented
 * in C++14, so this is only compiled when using C++11. The .cc file is required
 * for avoiding the duplication of symbols.
 *
 * \author Carles Fernandez-Prades, 2021. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_STRING_LITERALS_H
#define GNSS_SDR_STRING_LITERALS_H

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */

#if __cplusplus == 201103L

#include <cstddef>
#include <string>

namespace std
{
namespace string_literals
{
std::string operator"" s(const char* str, std::size_t len);
}  // namespace string_literals
}  // namespace std

#endif  // __cplusplus == 201103L

/** \} */
/** \} */

#endif  // GNSS_SDR_STRING_LITERALS_H
