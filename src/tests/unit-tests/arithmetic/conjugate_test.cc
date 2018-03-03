/*!
 * \file conjugate_test.cc
 * \brief  This file implements tests for conjugation of long arrays.
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
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
#include <armadillo>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>

DEFINE_int32(size_conjugate_test, 100000, "Size of the arrays used for conjugate testing");


TEST(ConjugateTest, StandardCComplexImplementation)
{
    std::complex<float>* input = new std::complex<float>[FLAGS_size_conjugate_test];
    std::complex<float>* output = new std::complex<float>[FLAGS_size_conjugate_test];
    std::fill_n(input, FLAGS_size_conjugate_test, std::complex<float>(0.0, 0.0));

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for (int i = 0; i < FLAGS_size_conjugate_test; i++)
        {
            output[i] = std::conj(input[i]);
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
              << "-length complex float vector in standard C finished in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;

    delete[] input;
    delete[] output;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}


TEST(ConjugateTest, C11ComplexImplementation)
{
    const std::vector<std::complex<float>> input(FLAGS_size_conjugate_test);
    std::vector<std::complex<float>> output(FLAGS_size_conjugate_test);
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    int pos = 0;
    for (const auto& item : input)
        {
            output[pos++] = std::conj(item);
        }
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
              << " complex<float> vector (C++11-style) finished in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);

    std::complex<float> expected(0, 0);
    std::complex<float> result(0, 0);
    for (const auto& item : output)
        {
            result += item;
        }
    ASSERT_EQ(expected, result);
}


TEST(ConjugateTest, ArmadilloComplexImplementation)
{
    arma::cx_fvec input(FLAGS_size_conjugate_test, arma::fill::zeros);
    arma::cx_fvec output(FLAGS_size_conjugate_test);

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    output = arma::conj(input);

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
              << "-length complex float Armadillo vector finished in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}


TEST(ConjugateTest, VolkComplexImplementation)
{
    std::complex<float>* input = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_conjugate_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::complex<float>* output = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_conjugate_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::fill_n(input, FLAGS_size_conjugate_test, std::complex<float>(0.0, 0.0));

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    volk_32fc_conjugate_32fc(output, input, FLAGS_size_conjugate_test);

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
              << "-length complex float vector using VOLK finished in " << elapsed_seconds.count() * 1e6
              << " microseconds" << std::endl;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
    volk_gnsssdr_free(input);
    volk_gnsssdr_free(output);
}
