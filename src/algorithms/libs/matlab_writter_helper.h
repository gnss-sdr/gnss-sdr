/*!
 * \file matlab_writter_helper.h
 * \brief Utility functions for logging to a matlab file
 * \authors <ul>
 *          <li> Mathieu Favreau, 2025. favreau.mathieu(at)hotmail.com
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_MATLAB_WRITTER_HELPER_H
#define GNSS_SDR_MATLAB_WRITTER_HELPER_H

#include <matio.h>
#include <array>

template <typename T>
struct matlab_type_traits;

template <>
struct matlab_type_traits<int32_t>
{
    static constexpr matio_classes class_type = MAT_C_INT32;
    static constexpr matio_types data_type = MAT_T_INT32;
};

template <>
struct matlab_type_traits<uint32_t>
{
    static constexpr matio_classes class_type = MAT_C_UINT32;
    static constexpr matio_types data_type = MAT_T_UINT32;
};

template <>
struct matlab_type_traits<int64_t>
{
    static constexpr matio_classes class_type = MAT_C_INT64;
    static constexpr matio_types data_type = MAT_T_INT64;
};

template <>
struct matlab_type_traits<uint64_t>
{
    static constexpr matio_classes class_type = MAT_C_UINT64;
    static constexpr matio_types data_type = MAT_T_UINT64;
};

template <>
struct matlab_type_traits<float>
{
    static constexpr matio_classes class_type = MAT_C_SINGLE;
    static constexpr matio_types data_type = MAT_T_SINGLE;
};

template <>
struct matlab_type_traits<double>
{
    static constexpr matio_classes class_type = MAT_C_DOUBLE;
    static constexpr matio_types data_type = MAT_T_DOUBLE;
};


template <size_t Rank, typename T>
void write_matlab_var(const char* name, T data, mat_t* matfp, std::array<size_t, 2>& dims)
{
    using traits = matlab_type_traits<T>;
    matvar_t* matvar = Mat_VarCreate(name, traits::class_type, traits::data_type, Rank, dims.data(), &data, 0);
    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);
    Mat_VarFree(matvar);
}


template <size_t Rank, typename T>
void write_matlab_var(const char* name, T* data, mat_t* matfp, std::array<size_t, 2>& dims)
{
    using traits = matlab_type_traits<T>;
    matvar_t* matvar = Mat_VarCreate(name, traits::class_type, traits::data_type, Rank, dims.data(), data, 0);
    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);
    Mat_VarFree(matvar);
}

#endif  // GNSS_SDR_NAV_MESSAGE_MONITOR_H
