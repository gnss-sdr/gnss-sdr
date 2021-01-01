/*!
 * \file magnitude_squared_test.cc
 * \brief  This file implements tests for the computation of magnitude squared
 *  in long arrays.
 * \author Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
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

#include <armadillo>
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>
#include <chrono>
#include <complex>

DEFINE_int32(size_magnitude_test, 100000, "Size of the arrays used for magnitude testing");


TEST(MagnitudeSquaredTest, StandardCComplexImplementation)
{
    auto* input = new std::complex<float>[FLAGS_size_magnitude_test];
    auto* output = new float[FLAGS_size_magnitude_test];
    unsigned int number = 0;
    for (number = 0; number < static_cast<unsigned int>(FLAGS_size_magnitude_test); number++)
        {
            input[number] = std::complex<float>(0.0, 0.0);
        }
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for (number = 0; number < static_cast<unsigned int>(FLAGS_size_magnitude_test); number++)
        {
            output[number] = (input[number].real() * input[number].real()) + (input[number].imag() * input[number].imag());
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "The squared magnitude of a " << FLAGS_size_magnitude_test
              << "-length complex vector in standard C computed in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    delete[] input;
    delete[] output;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}


TEST(MagnitudeSquaredTest, C11ComplexImplementation)
{
    const std::vector<std::complex<float>> input(FLAGS_size_magnitude_test);
    std::vector<float> output(FLAGS_size_magnitude_test);
    int pos = 0;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for (const auto& item : input)
        {
            output[pos++] = std::norm(item);
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "The squared magnitude of a " << FLAGS_size_magnitude_test
              << " complex<float> vector (C++11-style) finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);

    std::complex<float> expected(0, 0);
    std::complex<float> result(0, 0);
    for (const auto& item : output)
        {
            result += item;
        }
    ASSERT_EQ(expected, result);
}


TEST(MagnitudeSquaredTest, ArmadilloComplexImplementation)
{
    arma::cx_fvec input(FLAGS_size_magnitude_test, arma::fill::zeros);
    arma::fvec output(FLAGS_size_magnitude_test);
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    output = arma::abs(arma::square(input));

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "The squared magnitude of a " << FLAGS_size_magnitude_test
              << "-length vector using Armadillo computed in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}


TEST(MagnitudeSquaredTest, VolkComplexImplementation)
{
    auto* input = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_magnitude_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::fill_n(input, FLAGS_size_magnitude_test, std::complex<float>(0.0, 0.0));
    auto* output = static_cast<float*>(volk_gnsssdr_malloc(FLAGS_size_magnitude_test * sizeof(float), volk_gnsssdr_get_alignment()));
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    volk_32fc_magnitude_squared_32f(output, input, static_cast<unsigned int>(FLAGS_size_magnitude_test));

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "The squared magnitude of a " << FLAGS_size_magnitude_test
              << "-length vector using VOLK computed in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    volk_gnsssdr_free(input);
    volk_gnsssdr_free(output);
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}


TEST(MagnitudeSquaredTest, VolkComplexImplementationAlloc)
{
    volk_gnsssdr::vector<std::complex<float>> input(FLAGS_size_magnitude_test);  // or: input(FLAGS_size_magnitude_test, std::complex<float>(0.0, 0.0));
    std::fill_n(input.begin(), FLAGS_size_magnitude_test, std::complex<float>(0.0, 0.0));
    volk_gnsssdr::vector<float> output(FLAGS_size_magnitude_test);

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    volk_32fc_magnitude_squared_32f(output.data(), input.data(), static_cast<unsigned int>(FLAGS_size_magnitude_test));

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "The squared magnitude of a " << FLAGS_size_magnitude_test
              << "-length vector using VOLK ALLOC computed in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}

//  volk_32f_accumulator_s32f(&d_input_power, d_magnitude, d_fft_size);
