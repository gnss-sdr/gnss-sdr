/*!
 * \file item_type_helpers.h
 * \brief Utility functions for converting between item types
 * \authors <ul>
 *          <li> Cillian O'Driscoll, 2019. cillian.odriscoll(at)gmail.com
 *          </ul>
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

#ifndef GNSS_SDR_ITEM_TYPE_HELPERS_H
#define GNSS_SDR_ITEM_TYPE_HELPERS_H


#include <cstdint>
#include <functional>
#include <string>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


using item_type_converter_t = std::function<void(void *, const void *, uint32_t)>;

/*!
 * \brief Check if a string is a valid item type
 *
 * \description Valid item types include:
 *     "byte", "short", "float", "ibyte", "ishort", "cbyte", "cshort", "gr_complex"
 *
 */
bool item_type_valid(const std::string &item_type);

/*!
 * \brief Return the size of the given item type, or zero if unknown
 */
size_t item_type_size(const std::string &item_type);

/*!
 * \brief Determine if an item_type is complex
 */
bool item_type_is_complex(const std::string &item_type);

/*!
 * \brief Create a function to convert an array of input_type to an array of output_type
 *
 * \description Provides a generic interface to generate conversion functions for mapping
 * arrays of items.
 *
 * \param input_type - String representation of the input item type
 * \param output_type - String representation of the output item type
 *
 * The item types accepted are:
 *
 *  1. "byte" for 8 bit integers
 *  2. "cbyte" for complex (interleaved) 8 bit integers
 *  4. "ibyte" for complex (interleaved) 8 bit integers
 *  4. "short" for 16 bit integers
 *  5. "cshort" for complex (interleaved) 16 bit integers
 *  6. "ishort" for complex (interleaved) 16 bit integers
 *  7. "float" for 32 bit floating point values
 *  8. "gr_complex" for complex (interleaved) 32 bit floating point values
 *
 * \returns A function object with the following prototype:
 *  void convert_fun( void *dest, void *src, int num_items );
 *
 */
item_type_converter_t make_vector_converter(const std::string &input_type,
    const std::string &output_type);


/** \} */
/** \} */
#endif  // GNSS_SDR_ITEM_TYPE_HELPERS_H
