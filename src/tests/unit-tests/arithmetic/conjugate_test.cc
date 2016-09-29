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


#include <complex>
#include <ctime>
#include <armadillo>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>

DEFINE_int32(size_conjugate_test, 100000, "Size of the arrays used for conjugate testing");



TEST(Conjugate_Test, StandardCComplexImplementation)
{
    std::complex<float>* input = new std::complex<float>[FLAGS_size_conjugate_test];
    std::complex<float>* output = new std::complex<float>[FLAGS_size_conjugate_test];
    memset(input, 0, sizeof(std::complex<float>) * FLAGS_size_conjugate_test);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(int i = 0; i < FLAGS_size_conjugate_test; i++)
        {
            output[i] = std::conj(input[i]);
        }

    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
              << "-length complex float vector in standard C finished in " << (end - begin)
              << " microseconds" << std::endl;

    delete[] input;
    delete[] output;
    ASSERT_LE(0, end - begin);

}


TEST(Conjugate_Test, C11ComplexImplementation)
{
    const std::vector<std::complex<float>> input(FLAGS_size_conjugate_test);
    std::vector<std::complex<float>> output(FLAGS_size_conjugate_test);
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;
    int pos = 0;
    for (const auto &item : input)
        {
            output[pos++] = std::conj(item);
        }
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
              << " complex<float> vector (C++11-style) finished in " << (end - begin)
              << " microseconds" << std::endl;
    ASSERT_LE(0, end - begin);

    std::complex<float> expected(0,0);
    std::complex<float> result(0,0);
    for (const auto &item : output)
        {
            result += item;
        }
    ASSERT_EQ(expected, result);
}


TEST(Conjugate_Test, ArmadilloComplexImplementation)
{
    arma::cx_fvec input(FLAGS_size_conjugate_test, arma::fill::zeros);
    arma::cx_fvec output(FLAGS_size_conjugate_test);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    output = arma::conj(input);

    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
              << "-length complex float Armadillo vector finished in " << (end - begin)
              << " microseconds" << std::endl;
    ASSERT_LE(0, end - begin);
}


TEST(Conjugate_Test, VolkComplexImplementation)
{
    std::complex<float>* input = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_conjugate_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::complex<float>* output = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_conjugate_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    memset(input, 0, sizeof(std::complex<float>) * FLAGS_size_conjugate_test);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    volk_32fc_conjugate_32fc(output, input, FLAGS_size_conjugate_test);

    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout << "Conjugate of a "<< FLAGS_size_conjugate_test
              << "-length complex float vector using VOLK finished in " << (end - begin)
              << " microseconds" << std::endl;
    ASSERT_LE(0, end - begin);
    volk_gnsssdr_free(input);
    volk_gnsssdr_free(output);
}
