/*!
 * \file matio_test.cc
 * \brief  This file implements tests for the matio library
 *  in long arrays.
 * \author Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
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

#include "gnss_sdr_filesystem.h"
#include <gnuradio/gr_complex.h>
#include <gtest/gtest.h>
#include <matio.h>
#include <array>

TEST(MatioTest, WriteAndReadDoubles)
{
    // Write a .mat file
    mat_t *matfp;
    matvar_t *matvar;
    std::string filename = "./test.mat";
    matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT73);
    ASSERT_FALSE(reinterpret_cast<long *>(matfp) == nullptr) << "Error creating .mat file";

    std::array<double, 10> x{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::array<size_t, 2> dims{10, 1};
    matvar = Mat_VarCreate("x", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims.data(), x.data(), 0);
    ASSERT_FALSE(reinterpret_cast<long *>(matvar) == nullptr) << "Error creating variable for ’x’";

    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
    Mat_VarFree(matvar);

    Mat_Close(matfp);

    // Read a .mat file
    mat_t *matfp_read;
    matvar_t *matvar_read;

    matfp_read = Mat_Open(filename.c_str(), MAT_ACC_RDONLY);
    ASSERT_FALSE(reinterpret_cast<long *>(matfp_read) == nullptr) << "Error reading .mat file";

    matvar_read = Mat_VarReadInfo(matfp_read, "x");
    ASSERT_FALSE(reinterpret_cast<long *>(matvar_read) == nullptr) << "Error reading variable in .mat file";

    matvar_read = Mat_VarRead(matfp_read, "x");
    auto *x_read = reinterpret_cast<double *>(matvar_read->data);
    Mat_Close(matfp_read);

    for (int i = 0; i < 10; i++)
        {
            EXPECT_DOUBLE_EQ(x[i], x_read[i]);
        }
    Mat_VarFree(matvar_read);
    errorlib::error_code ec;
    ASSERT_EQ(fs::remove(fs::path(filename), ec), true);
}


TEST(MatioTest, WriteAndReadGrComplex)
{
    // Write a .mat file
    mat_t *matfp;
    matvar_t *matvar1;
    std::string filename = "./test3.mat";
    matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT73);
    ASSERT_FALSE(reinterpret_cast<long *>(matfp) == nullptr) << "Error creating .mat file";

    const std::vector<gr_complex> x_v = {{1, 10}, {2, 9}, {3, 8}, {4, 7}, {5, 6}, {6, -5}, {7, -4}, {8, 3}, {9, 2}, {10, 1}};
    const unsigned int size = x_v.size();
    std::array<float, 10> x_real{};
    std::array<float, 10> x_imag{};
    unsigned int i = 0;
    for (auto it : x_v)
        {
            x_real[i] = it.real();
            x_imag[i] = it.imag();
            i++;
        }

    struct mat_complex_split_t x = {x_real.data(), x_imag.data()};
    std::array<size_t, 2> dims{static_cast<size_t>(size), 1};
    matvar1 = Mat_VarCreate("x", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims.data(), &x, MAT_F_COMPLEX);
    ASSERT_FALSE(reinterpret_cast<long *>(matvar1) == nullptr) << "Error creating variable for ’x’";

    std::vector<gr_complex> x2 = {{1.1, -10}, {2, -9}, {3, -8}, {4, -7}, {5, 6}, {6, -5}, {7, -4}, {8, 3}, {9, 2}, {10, 1}};
    const unsigned int size_y = x2.size();
    std::array<float, 10> y_real{};
    std::array<float, 10> y_imag{};
    i = 0;
    for (auto it : x2)
        {
            y_real[i] = it.real();
            y_imag[i] = it.imag();
            i++;
        }

    struct mat_complex_split_t y = {y_real.data(), y_imag.data()};
    std::array<size_t, 2> dims_y{static_cast<size_t>(size_y), 1};
    matvar_t *matvar2;
    matvar2 = Mat_VarCreate("y", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims_y.data(), &y, MAT_F_COMPLEX);
    ASSERT_FALSE(reinterpret_cast<long *>(matvar2) == nullptr) << "Error creating variable for ’y’";

    Mat_VarWrite(matfp, matvar1, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
    Mat_VarWrite(matfp, matvar2, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
    Mat_VarFree(matvar1);
    Mat_VarFree(matvar2);

    Mat_Close(matfp);

    // Read a .mat file
    mat_t *matfp_read;
    matvar_t *matvar_read;

    matfp_read = Mat_Open(filename.c_str(), MAT_ACC_RDONLY);
    ASSERT_FALSE(reinterpret_cast<long *>(matfp_read) == nullptr) << "Error reading .mat file";

    matvar_read = Mat_VarReadInfo(matfp_read, "x");
    ASSERT_FALSE(reinterpret_cast<long *>(matvar_read) == nullptr) << "Error reading variable in .mat file";

    matvar_read = Mat_VarRead(matfp_read, "x");
    auto *x_read_st = reinterpret_cast<mat_complex_split_t *>(matvar_read->data);
    auto *x_read_real = reinterpret_cast<float *>(x_read_st->Re);
    auto *x_read_imag = reinterpret_cast<float *>(x_read_st->Im);
    std::vector<gr_complex> x_v_read;
    for (unsigned int k = 0; k < size; k++)
        {
            x_v_read.emplace_back(x_read_real[k], x_read_imag[k]);
        }

    Mat_Close(matfp_read);
    Mat_VarFree(matvar_read);

    for (unsigned int k = 0; k < size; k++)
        {
            EXPECT_FLOAT_EQ(x_v[k].real(), x_v_read[k].real());
            EXPECT_FLOAT_EQ(x_v[k].imag(), x_v_read[k].imag());
        }
    errorlib::error_code ec;
    ASSERT_EQ(fs::remove(fs::path(filename), ec), true);
}
