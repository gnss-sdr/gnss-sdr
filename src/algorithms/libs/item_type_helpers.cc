/*!
 * \file item_type_helpers.cc
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
    else if (item_type == "short" or item_type == "ishort")
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


void copy_converter(void *dest, const void *src, uint32_t num_items, size_t item_size)
{
    std::memcpy(dest, src, num_items * item_size);
}


void convert_8i_16i(void *dest, const void *src, uint32_t num_items)
{
    volk_8i_convert_16i(reinterpret_cast<int16_t *>(dest),
        reinterpret_cast<const int8_t *>(src), num_items);
}


void convert_8i_32f(void *dest, const void *src, uint32_t num_items)
{
    volk_8i_s32f_convert_32f(reinterpret_cast<float *>(dest),
        reinterpret_cast<const int8_t *>(src), 1.0F, num_items);
}


void convert_8ic_16ic(void *dest, const void *src, uint32_t num_items)
{
    volk_8i_convert_16i(reinterpret_cast<int16_t *>(dest),
        reinterpret_cast<const int8_t *>(src), 2 * num_items);
}


void convert_8ic_32fc(void *dest, const void *src, uint32_t num_items)
{
    volk_8i_s32f_convert_32f(reinterpret_cast<float *>(dest),
        reinterpret_cast<const int8_t *>(src), 1.0F, 2 * num_items);
}


void convert_16i_8i(void *dest, const void *src, uint32_t num_items)
{
    volk_16i_convert_8i(reinterpret_cast<int8_t *>(dest),
        reinterpret_cast<const int16_t *>(src), num_items);
}


void convert_16i_32f(void *dest, const void *src, uint32_t num_items)
{
    volk_16i_s32f_convert_32f(reinterpret_cast<float *>(dest),
        reinterpret_cast<const int16_t *>(src), 1.0F, num_items);
}


void convert_16ic_8ic(void *dest, const void *src, uint32_t num_items)
{
    volk_16i_convert_8i(reinterpret_cast<int8_t *>(dest),
        reinterpret_cast<const int16_t *>(src), 2 * num_items);
}


void convert_16ic_32fc(void *dest, const void *src, uint32_t num_items)
{
    volk_16i_s32f_convert_32f(reinterpret_cast<float *>(dest),
        reinterpret_cast<const int16_t *>(src), 1.0F, 2 * num_items);
}


void convert_32f_8i(void *dest, const void *src, uint32_t num_items)
{
    volk_32f_s32f_convert_8i(reinterpret_cast<int8_t *>(dest),
        reinterpret_cast<const float *>(src), 1.0F, num_items);
}


void convert_32f_16i(void *dest, const void *src, uint32_t num_items)
{
    volk_32f_s32f_convert_16i(reinterpret_cast<int16_t *>(dest),
        reinterpret_cast<const float *>(src), 1.0F, num_items);
}


void convert_32fc_8ic(void *dest, const void *src, uint32_t num_items)
{
    volk_32f_s32f_convert_8i(reinterpret_cast<int8_t *>(dest),
        reinterpret_cast<const float *>(src), 1.0F, 2 * num_items);
}


void convert_32fc_16ic(void *dest, const void *src, uint32_t num_items)
{
    volk_32f_s32f_convert_16i(reinterpret_cast<int16_t *>(dest),
        reinterpret_cast<const float *>(src), 1.0F, 2 * num_items);
}


item_type_converter_t make_vector_converter(const std::string &input_type,
    const std::string &output_type)
{
    if (not item_type_valid(input_type) or not item_type_valid(output_type))
        {
            throw std::runtime_error("make_vector_converter: invalid item types : " + input_type + " " + output_type);
        }

    if (input_type == output_type)
        {
            size_t input_size = item_type_size(input_type);
#ifdef DO_NOT_USE_LAMBDAS
            return std::bind(copy_converter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, input_size);  // NOLINT(modernize-avoid-bind)
#else
            return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return copy_converter(arg1, arg2, arg3, input_size); };
#endif
        }

    if (input_type == "byte")
        {
            if (output_type == "short")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_8i_16i, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_8i_16i(arg1, arg2, arg3); };
#endif
                }
            else if (output_type == "float")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_8i_32f, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_8i_32f(arg1, arg2, arg3); };
#endif
                }
        }
    else if (input_type == "cbyte")
        {
            if (output_type == "ibyte")
                {
                    size_t input_size = item_type_size(input_type);
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(copy_converter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, input_size);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return copy_converter(arg1, arg2, arg3, input_size); };
#endif
                }
            if (output_type == "cshort" or output_type == "ishort")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_8ic_16ic, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_8ic_16ic(arg1, arg2, arg3); };
#endif
                }
            else if (output_type == "gr_complex")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_8ic_32fc, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_8ic_32fc(arg1, arg2, arg3); };
#endif
                }
        }
    else if (input_type == "ibyte")
        {
            if (output_type == "cbyte")
                {
                    size_t input_size = item_type_size(input_type);
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(copy_converter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, input_size);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return copy_converter(arg1, arg2, arg3, input_size); };
#endif
                }
            else if (output_type == "cshort" or output_type == "ishort")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_8i_16i, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_8i_16i(arg1, arg2, arg3); };
#endif
                }
            else if (output_type == "gr_complex")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_8i_32f, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_8i_32f(arg1, arg2, arg3); };
#endif
                }
        }
    else if (input_type == "short")
        {
            if (output_type == "byte")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_16i_8i, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_16i_8i(arg1, arg2, arg3); };
#endif
                }
            else if (output_type == "float")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_16i_32f, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_16i_32f(arg1, arg2, arg3); };
#endif
                }
        }
    else if (input_type == "cshort")
        {
            if (output_type == "cbyte" or output_type == "ibyte")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_16ic_8ic, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_16ic_8ic(arg1, arg2, arg3); };
#endif
                }
            if (output_type == "ishort")
                {
                    size_t input_size = item_type_size(input_type);
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(copy_converter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, input_size);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return copy_converter(arg1, arg2, arg3, input_size); };
#endif
                }
            else if (output_type == "gr_complex")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_16ic_32fc, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_16ic_32fc(arg1, arg2, arg3); };
#endif
                }
        }
    else if (input_type == "ishort")
        {
            if (output_type == "cbyte" or output_type == "ibyte")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_16i_8i, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_16i_8i(arg1, arg2, arg3); };
#endif
                }
            if (output_type == "cshort")
                {
                    size_t input_size = item_type_size(input_type);
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(copy_converter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, input_size);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return copy_converter(arg1, arg2, arg3, input_size); };
#endif
                }
            else if (output_type == "gr_complex")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_16i_32f, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_16i_32f(arg1, arg2, arg3); };
#endif
                }
        }
    else if (input_type == "float")
        {
            if (output_type == "byte")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_32f_8i, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_32f_8i(arg1, arg2, arg3); };
#endif
                }
            else if (output_type == "short")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_32f_16i, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_32f_16i(arg1, arg2, arg3); };
#endif
                }
        }
    else if (input_type == "gr_complex")
        {
            if (output_type == "cbyte" or output_type == "ibyte")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_32fc_8ic, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_32fc_8ic(arg1, arg2, arg3); };
#endif
                }
            else if (output_type == "cshort" or output_type == "ishort")
                {
#ifdef DO_NOT_USE_LAMBDAS
                    return std::bind(convert_32fc_16ic, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);  // NOLINT(modernize-avoid-bind)
#else
                    return [=](auto &&arg1, auto &&arg2, auto &&arg3) { return convert_32fc_16ic(arg1, arg2, arg3); };
#endif
                }
        }

    throw std::runtime_error("make_vector_converter: invalid conversion : " + input_type + " to " + output_type);
}
