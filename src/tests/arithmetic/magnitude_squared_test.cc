/*!
 * \file magnitude_squared_test.cc
 * \brief  This file implements tests for the computation of magnitude squared
 *  in long arrays.
 * \author Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

DEFINE_int32(size_magnitude_test, 100000, "Size of the arrays used for magnitude testing");


TEST(MagnitudeSquared_Test, StandardCComplexImplementation)
{
    std::complex<float>* input = new std::complex<float>[FLAGS_size_magnitude_test];
    memset(input, 0, sizeof(std::complex<float>) * FLAGS_size_magnitude_test);
    const float* inputPtr = (float*)input;
    float* output = new float[FLAGS_size_magnitude_test];
    unsigned int number = 0;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    for(number = 0; number < (unsigned int)FLAGS_size_magnitude_test; number++)
        {
            const float real = *inputPtr++;
            const float imag = *inputPtr++;
            *output++ = (real*real) + (imag*imag);
        }

    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout <<  "The squared magnitude of a " << FLAGS_size_magnitude_test
              << "-length vector computed in " << (end - begin)
              << " microseconds" << std::endl;
    ASSERT_LE(0, end - begin);
    delete input;
    delete output;
}



TEST(MagnitudeSquared_Test, ArmadilloComplexImplementation)
{
    arma::cx_fvec input(FLAGS_size_magnitude_test, arma::fill::zeros);
    arma::fvec output(FLAGS_size_magnitude_test);
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    output = arma::abs(arma::square(input));

    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout <<  "The squared magnitude of a " << FLAGS_size_magnitude_test
              << "-length vector using Armadillo computed in " << (end - begin)
              << " microseconds" << std::endl;
    ASSERT_LE(0, end - begin);
}



TEST(MagnitudeSquared_Test, VolkComplexImplementation)
{
    std::complex<float>* input = new std::complex<float>[FLAGS_size_magnitude_test];
    memset(input, 0, sizeof(std::complex<float>) * FLAGS_size_magnitude_test);
    float* output = new float[FLAGS_size_magnitude_test];
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    volk_32fc_magnitude_squared_32f(output, input, (unsigned int)FLAGS_size_magnitude_test);

    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    std::cout <<  "The squared magnitude of a " << FLAGS_size_magnitude_test
              << "-length vector using VOLK computed in " << (end - begin)
              << " microseconds" << std::endl;
    ASSERT_LE(0, end - begin);
    delete input;
    delete output;
}

//            volk_32f_accumulator_s32f_a(&d_input_power, d_magnitude, d_fft_size);

