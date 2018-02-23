/*!
 * \file matio_test.cc
 * \brief  This file implements tests for the matio library
 *  in long arrays.
 * \author Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
 *
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

#include <matio.h>
#include <cstdio>
#include <gnuradio/gr_complex.h>
#include <gtest/gtest.h>

TEST(MatioTest, WriteAndReadDoubles)
{
    // Write a .mat file
    mat_t *matfp;
    matvar_t *matvar;
    std::string filename = "./test.mat";
    matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT73);
    ASSERT_FALSE(reinterpret_cast<long*>(matfp) == NULL) << "Error creating .mat file";

    double x[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    size_t dims[2] = {10, 1};
    matvar = Mat_VarCreate("x", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, x, 0);
    ASSERT_FALSE(reinterpret_cast<long*>(matvar) == NULL) << "Error creating variable for ’x’";

    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
    Mat_VarFree(matvar);

    Mat_Close(matfp);

    // Read a .mat file
    mat_t *matfp_read;
    matvar_t *matvar_read;

    matfp_read = Mat_Open(filename.c_str(), MAT_ACC_RDONLY);
    ASSERT_FALSE(reinterpret_cast<long*>(matfp_read) == NULL) << "Error reading .mat file";

    matvar_read = Mat_VarReadInfo(matfp_read, "x");
    ASSERT_FALSE(reinterpret_cast<long*>(matvar_read) == NULL) << "Error reading variable in .mat file";

    matvar_read = Mat_VarRead(matfp_read, "x");
    double *x_read = reinterpret_cast<double*>(matvar_read->data);
    Mat_Close(matfp_read);

    for(int i = 0; i < 10; i++)
        {
            EXPECT_DOUBLE_EQ(x[i], x_read[i]);
        }
    Mat_VarFree(matvar_read);
    ASSERT_EQ(remove(filename.c_str()), 0);
}


TEST(MatioTest, WriteAndReadGrComplex)
{
    // Write a .mat file
    mat_t *matfp;
    matvar_t *matvar1;
    std::string filename = "./test3.mat";
    matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT73);
    ASSERT_FALSE(reinterpret_cast<long*>(matfp) == NULL) << "Error creating .mat file";

    std::vector<gr_complex> x_v = { {1, 10}, {2, 9}, {3, 8}, {4, 7}, {5, 6}, {6, -5}, {7, -4}, {8, 3}, {9, 2}, {10, 1}};
    const unsigned int size = x_v.size();
    float x_real[size];
    float x_imag[size];
    unsigned int i = 0;
    for (std::vector<gr_complex>::const_iterator it = x_v.cbegin(); it != x_v.cend(); it++)
        {
            x_real[i] = it->real();
            x_imag[i] = it->imag();
            i++;
        }

    struct mat_complex_split_t x = {x_real, x_imag};
    size_t dims[2] = {static_cast<size_t>(size), 1};
    matvar1 = Mat_VarCreate("x", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, &x, MAT_F_COMPLEX);
    ASSERT_FALSE(reinterpret_cast<long*>(matvar1) == NULL) << "Error creating variable for ’x’";

    std::vector<gr_complex> x2 = { {1.1, -10}, {2, -9}, {3, -8}, {4, -7}, {5, 6}, {6, -5}, {7, -4}, {8, 3}, {9, 2}, {10, 1}};
    const unsigned int size_y = x2.size();
    float y_real[size_y];
    float y_imag[size_y];
    i = 0;
    for (std::vector<gr_complex>::const_iterator it = x2.cbegin(); it != x2.cend(); it++)
        {
            y_real[i] = it->real();
            y_imag[i] = it->imag();
            i++;
        }

    struct mat_complex_split_t y = {y_real, y_imag};
    size_t dims_y[2] = {static_cast<size_t>(size_y), 1};
    matvar_t *matvar2;
    matvar2 = Mat_VarCreate("y", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims_y, &y, MAT_F_COMPLEX);
    ASSERT_FALSE(reinterpret_cast<long*>(matvar2) == NULL) << "Error creating variable for ’y’";

    Mat_VarWrite(matfp, matvar1, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
    Mat_VarWrite(matfp, matvar2, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
    Mat_VarFree(matvar1);
    Mat_VarFree(matvar2);

    Mat_Close(matfp);

    // Read a .mat file
    mat_t *matfp_read;
    matvar_t *matvar_read;

    matfp_read = Mat_Open(filename.c_str(), MAT_ACC_RDONLY);
    ASSERT_FALSE(reinterpret_cast<long*>(matfp_read) == NULL) << "Error reading .mat file";

    matvar_read = Mat_VarReadInfo(matfp_read, "x");
    ASSERT_FALSE(reinterpret_cast<long*>(matvar_read) == NULL) << "Error reading variable in .mat file";

    matvar_read = Mat_VarRead(matfp_read, "x");
    mat_complex_split_t *x_read_st = reinterpret_cast<mat_complex_split_t*>(matvar_read->data);
    float * x_read_real = reinterpret_cast<float*>(x_read_st->Re);
    float * x_read_imag = reinterpret_cast<float*>(x_read_st->Im);
    std::vector<gr_complex> x_v_read;
    for(unsigned int i = 0; i < size; i++)
        {
            x_v_read.push_back(gr_complex(x_read_real[i], x_read_imag[i]));
        }

    Mat_Close(matfp_read);
    Mat_VarFree(matvar_read);

    for(unsigned int i = 0; i < size; i++)
        {
            EXPECT_FLOAT_EQ(x_v[i].real(), x_v_read[i].real());
            EXPECT_FLOAT_EQ(x_v[i].imag(), x_v_read[i].imag());
        }
    ASSERT_EQ(remove(filename.c_str()), 0);
}
