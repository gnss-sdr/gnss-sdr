/*!
 * \file item_type_helpers.h
 * \brief Utility functions for converting between item types
 * \authors <ul>
 *          <li> Cillian O'Driscoll, 2017. cillian.odriscoll(at)gmail.com
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#ifndef ITEM_TYPE_HELPERS_H_
#define ITEM_TYPE_HELPERS_H_


#include <functional>
#include <string>

using item_type_converter_t = std::function<void(void *, const void *, unsigned)>;
/*!
 * \brief Convert a gnss-sdr item type string to internal 
 */
std::string external_item_type_to_internal(const std::string &external_item_type);


/*!
 * \brief Check if a string is a valid item type
 *
 * \description Valid item types include:
 *     "i8", "ic8", "i16", "ic16", "i32", "ic32", "f32", "fc32"
 *
 *  where "i" denotes integer, "f" denotes float and "c" is for complex and
 *  the number indicates the number of bits in the representation
 */
bool item_type_valid(const std::string &item_type);

/*!
 * \brief Return the size of the given item type, or zero if unknown
 */
size_t item_type_size(const std::string &item_type);

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
 *  1. "i8" for 8 bit integers
 *  2. "ic8" for complex (interleaved) 8 bit integers
 *  3. "i16" for 16 bit integers
 *  4. "ic16" for complex (interleaved) 16 bit integers
 *  5. "f32" for 32 bit floating point values
 *  6. "fc32" for complex (interleaved) 32 bit floating point values
 *
 * \returns A function object with the following prototype:
 *  void convert_fun( void *dest, void *src, int num_items );
 *
 *  
 */
item_type_converter_t make_vector_converter(std::string input_type,
    std::string output_type);

#endif
