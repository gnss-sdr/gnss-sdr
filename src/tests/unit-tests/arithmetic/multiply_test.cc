/*!
 * \file multiply_test.cc
 * \brief  This file implements tests for the multiplication of long arrays.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#include <algorithm>
#include <chrono>
#include <complex>
#include <numeric>
#include <armadillo>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>

DEFINE_int32(size_multiply_test, 100000, "Size of the arrays used for multiply testing");


TEST(MultiplyTest, StandardCDoubleImplementation)
{
    double* input = new double[FLAGS_size_multiply_test];
    double* output = new double[FLAGS_size_multiply_test];
    std::fill_n(input, FLAGS_size_multiply_test, 0.0);
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for(int i = 0; i < FLAGS_size_multiply_test; i++)
        {
            output[i] = input[i] * input[i];
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
              << " doubles in standard C finished in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;

    double acc = 0;
    double expected = 0;
    for(int i = 0; i < FLAGS_size_multiply_test; i++)
        {
            acc += output[i];
        }
    delete[] input;
    delete[] output;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
    ASSERT_EQ(expected, acc);
}


TEST(MultiplyTest, ArmadilloImplementation)
{
    arma::vec input(FLAGS_size_multiply_test, arma::fill::zeros);
    arma::vec output(FLAGS_size_multiply_test);

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    output = input % input;

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
              << "-length double Armadillo vectors finished in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
    ASSERT_EQ(0, arma::norm(output,2));
}



TEST(MultiplyTest, StandardCComplexImplementation)
{
    std::complex<float>* input = new std::complex<float>[FLAGS_size_multiply_test];
    std::complex<float>* output = new std::complex<float>[FLAGS_size_multiply_test];
    std::fill_n(input, FLAGS_size_multiply_test, std::complex<float>(0.0, 0.0));
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for(int i = 0; i < FLAGS_size_multiply_test; i++)
        {
            output[i] = input[i] * input[i];
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
              << " complex<float> in standard C finished in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;

    std::complex<float> expected(0,0);
    std::complex<float> result(0,0);
    for(int i = 0; i < FLAGS_size_multiply_test; i++)
         {
             result += output[i];
         }
    delete[] input;
    delete[] output;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
    ASSERT_EQ(expected, result);
}



TEST(MultiplyTest, C11ComplexImplementation)
{
    const std::vector<std::complex<float>> input(FLAGS_size_multiply_test);
    std::vector<std::complex<float>> output(FLAGS_size_multiply_test);
    int pos = 0;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    // Trying a range-based for
    for (const auto &item : input)
        {
            output[pos++] = item * item;
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
              << " complex<float> vector (C++11-style) finished in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);

    std::complex<float> expected(0,0);
    auto result = std::inner_product(output.begin(), output.end(), output.begin(), expected);
    ASSERT_EQ(expected, result);
}


TEST(MultiplyTest, ArmadilloComplexImplementation)
{
    arma::cx_fvec input(FLAGS_size_multiply_test, arma::fill::zeros);
    arma::cx_fvec output(FLAGS_size_multiply_test);

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    output = input % input;

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
              << "-length complex float Armadillo vectors finished in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
    ASSERT_EQ(0, arma::norm(output,2));
}




TEST(MultiplyTest, VolkComplexImplementation)
{
    std::complex<float>* input = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_multiply_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::complex<float>* output = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_multiply_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::fill_n(input, FLAGS_size_multiply_test, std::complex<float>(0.0, 0.0));

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    volk_32fc_x2_multiply_32fc(output, input, input, FLAGS_size_multiply_test);

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
              << "-length complex float vector using VOLK finished in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);

    float* mag = static_cast<float*>(volk_gnsssdr_malloc(FLAGS_size_multiply_test * sizeof(float), volk_gnsssdr_get_alignment()));
    volk_32fc_magnitude_32f(mag, output, FLAGS_size_multiply_test);

    float* result = new float(0);
    volk_32f_accumulator_s32f(result, mag, FLAGS_size_multiply_test);
    // Comparing floating-point numbers is tricky.
    // Due to round-off errors, it is very unlikely that two floating-points will match exactly.
    // See http://code.google.com/p/googletest/wiki/AdvancedGuide#Floating-Point_Comparison
    float expected = 0;
    ASSERT_FLOAT_EQ(expected, result[0]);
    volk_gnsssdr_free(input);
    volk_gnsssdr_free(output);
    volk_gnsssdr_free(mag);
}

