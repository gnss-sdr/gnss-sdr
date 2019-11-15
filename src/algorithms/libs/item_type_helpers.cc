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

#include "item_type_helpers.h"
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cstring>  // memcpy

bool item_type_valid(const std::string &item_type)
{
    if (item_type != "byte" and item_type != "cbyte" and item_type != "ibyte" and
        item_type != "short" and item_type != "cshort" and item_type != "ishort" and
        item_type != "float" and item_type != "gr_complex")
        {
            return false;
        }

    return true;
}

size_t item_type_size(const std::string &item_type)
{
    if (item_type == "byte" or item_type == "ibyte")
        {
            return sizeof(int8_t);
        }
    else if (item_type == "cbyte")
        {
            return 2 * sizeof(int8_t);
        }
    else if (item_type == "short" or item_type == "ishort" )
        {
            return sizeof(int16_t);
        }
    else if (item_type == "cshort")
        {
            return 2 * sizeof(int16_t);
        }
    else if (item_type == "float")
        {
            return sizeof(float);
        }
    else if (item_type == "gr_complex")
        {
            return 2 * sizeof(float);
        }
    else
        {
            return 0;
        }
}

bool item_type_is_complex(const std::string &item_type)
{
    return (item_type == "ibyte") or (item_type == "cbyte") or (item_type == "ishort") or (item_type == "cshort") or (item_type == "gr_complex");
}

void copy_converter(void *dest, const void *src, unsigned int num_items, size_t item_size)
{
    std::memcpy(dest, src, num_items * item_size);
}

void convert_8i_16i(void *dest, const void *src, unsigned int num_items)
{
    volk_8i_convert_16i(reinterpret_cast<int16_t *>(dest),
        reinterpret_cast<const int8_t *>(src), num_items);
}

void convert_8i_32f(void *dest, const void *src, unsigned int num_items)
{
    volk_8i_s32f_convert_32f(reinterpret_cast<float *>(dest),
        reinterpret_cast<const int8_t *>(src), 1.0f, num_items);
}

void convert_8ic_16ic(void *dest, const void *src, unsigned int num_items)
{
    volk_8i_convert_16i(reinterpret_cast<int16_t *>(dest),
        reinterpret_cast<const int8_t *>(src), 2 * num_items);
}

void convert_8ic_32fc(void *dest, const void *src, unsigned int num_items)
{
    volk_8i_s32f_convert_32f(reinterpret_cast<float *>(dest),
        reinterpret_cast<const int8_t *>(src), 1.0f, 2 * num_items);
}

void convert_16i_8i(void *dest, const void *src, unsigned int num_items)
{
    volk_16i_convert_8i(reinterpret_cast<int8_t *>(dest),
        reinterpret_cast<const int16_t *>(src), num_items);
}

void convert_16i_32f(void *dest, const void *src, unsigned int num_items)
{
    volk_16i_s32f_convert_32f(reinterpret_cast<float *>(dest),
        reinterpret_cast<const int16_t *>(src), 1.0f, num_items);
}

void convert_16ic_8ic(void *dest, const void *src, unsigned int num_items)
{
    volk_16i_convert_8i(reinterpret_cast<int8_t *>(dest),
        reinterpret_cast<const int16_t *>(src), 2 * num_items);
}

void convert_16ic_32fc(void *dest, const void *src, unsigned int num_items)
{
    volk_16i_s32f_convert_32f(reinterpret_cast<float *>(dest),
        reinterpret_cast<const int16_t *>(src), 1.0f, 2 * num_items);
}

void convert_32f_8i(void *dest, const void *src, unsigned int num_items)
{
    volk_32f_s32f_convert_8i(reinterpret_cast<int8_t *>(dest),
        reinterpret_cast<const float *>(src), 1.0f, num_items);
}

void convert_32f_16i(void *dest, const void *src, unsigned int num_items)
{
    volk_32f_s32f_convert_16i(reinterpret_cast<int16_t *>(dest),
        reinterpret_cast<const float *>(src), 1.0f, num_items);
}

void convert_32fc_8ic(void *dest, const void *src, unsigned int num_items)
{
    volk_32f_s32f_convert_8i(reinterpret_cast<int8_t *>(dest),
        reinterpret_cast<const float *>(src), 1.0f, 2 * num_items);
}

void convert_32fc_16ic(void *dest, const void *src, unsigned int num_items)
{
    volk_32f_s32f_convert_16i(reinterpret_cast<int16_t *>(dest),
        reinterpret_cast<const float *>(src), 1.0f, 2 * num_items);
}

item_type_converter_t make_vector_converter(std::string input_type,
    std::string output_type)
{
    if (not item_type_valid(input_type) or not item_type_valid(output_type))
        {
            throw std::runtime_error("make_vector_converter: invalid item types : " + input_type + " " + output_type);
        }

    if (input_type == output_type)
        {
            size_t input_size = item_type_size(input_type);
            return std::bind(copy_converter, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, input_size);
        }

    if (input_type == "byte")
        {
            if (output_type == "short")
                {
                    return std::bind(convert_8i_16i, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
            else if (output_type == "float")
                {
                    return std::bind(convert_8i_32f, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
        }
    else if (input_type == "cbyte")
        {
            if (output_type == "ibyte" )
                {
                    size_t input_size = item_type_size(input_type);
                    return std::bind(copy_converter, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3, input_size);
                }
            if (output_type == "cshort" or output_type == "ishort" )
                {
                    return std::bind(convert_8ic_16ic, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
            else if (output_type == "gr_complex")
                {
                    return std::bind(convert_8ic_32fc, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
        }
    else if (input_type == "ibyte")
        {
            if (output_type == "cbyte" )
                {
                    size_t input_size = item_type_size(input_type);
                    return std::bind(copy_converter, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3, input_size);
                }
            else if (output_type == "cshort" or output_type == "ishort")
                {
                    return std::bind(convert_8i_16i, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
            else if (output_type == "gr_complex")
                {
                    return std::bind(convert_8i_32f, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
        }
    else if (input_type == "short")
        {
            if (output_type == "byte")
                {
                    return std::bind(convert_16i_8i, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
            else if (output_type == "float")
                {
                    return std::bind(convert_16i_32f, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
        }
    else if (input_type == "cshort")
        {
            if (output_type == "cbyte" or output_type == "ibyte" )
                {
                    return std::bind(convert_16ic_8ic, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
            if (output_type == "ishort")
                {
                    size_t input_size = item_type_size(input_type);
                    return std::bind(copy_converter, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3, input_size);
                }
            else if (output_type == "gr_complex")
                {
                    return std::bind(convert_16ic_32fc, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
        }
    else if (input_type == "ishort")
        {
            if (output_type == "cbyte" or output_type == "ibyte" )
                {
                    return std::bind(convert_16i_8i, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
            if (output_type == "cshort")
                {
                    size_t input_size = item_type_size(input_type);
                    return std::bind(copy_converter, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3, input_size);
                }
            else if (output_type == "gr_complex")
                {
                    return std::bind(convert_16i_32f, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
        }
    else if (input_type == "float")
        {
            if (output_type == "byte")
                {
                    return std::bind(convert_32f_8i, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
            else if (output_type == "short")
                {
                    return std::bind(convert_32f_16i, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
        }
    else if (input_type == "gr_complex")
        {
            if (output_type == "cbyte" or output_type == "ibyte")
                {
                    return std::bind(convert_32fc_8ic, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
            else if (output_type == "cshort" or output_type == "ishort" )
                {
                    return std::bind(convert_32fc_16ic, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3);
                }
        }

    throw std::runtime_error("make_vector_converter: invalid conversion : " + input_type + " to " + output_type);
}
