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

#if USE_GLOG_AND_GFLAGS
DEFINE_int32(size_magnitude_test, 100000, "Size of the arrays used for magnitude testing");
#else
ABSL_FLAG(int32_t, size_magnitude_test, 100000, "Size of the arrays used for magnitude testing");
#endif

TEST(MagnitudeSquaredTest, StandardCComplexImplementation)
{
#if USE_GLOG_AND_GFLAGS
    auto* input = new std::complex<float>[FLAGS_size_magnitude_test];
    auto* output = new float[FLAGS_size_magnitude_test];
#else
    auto* input = new std::complex<float>[absl::GetFlag(FLAGS_size_magnitude_test)];
    auto* output = new float[absl::GetFlag(FLAGS_size_magnitude_test)];
#endif
    unsigned int number = 0;
#if USE_GLOG_AND_GFLAGS
    for (number = 0; number < static_cast<unsigned int>(FLAGS_size_magnitude_test); number++)
#else
    for (number = 0; number < static_cast<unsigned int>(absl::GetFlag(FLAGS_size_magnitude_test)); number++)
#endif
        {
            input[number] = std::complex<float>(0.0, 0.0);
        }
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
#if USE_GLOG_AND_GFLAGS
    for (number = 0; number < static_cast<unsigned int>(FLAGS_size_magnitude_test); number++)
#else
    for (number = 0; number < static_cast<unsigned int>(absl::GetFlag(FLAGS_size_magnitude_test)); number++)
#endif
        {
            output[number] = (input[number].real() * input[number].real()) + (input[number].imag() * input[number].imag());
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

#if USE_GLOG_AND_GFLAGS
    std::cout << "The squared magnitude of a " << FLAGS_size_magnitude_test
#else
    std::cout << "The squared magnitude of a " << absl::GetFlag(FLAGS_size_magnitude_test)
#endif
              << "-length complex vector in standard C computed in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    delete[] input;
    delete[] output;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}


TEST(MagnitudeSquaredTest, C11ComplexImplementation)
{
#if USE_GLOG_AND_GFLAGS
    const std::vector<std::complex<float>> input(FLAGS_size_magnitude_test);
    std::vector<float> output(FLAGS_size_magnitude_test);
#else
    const std::vector<std::complex<float>> input(absl::GetFlag(FLAGS_size_magnitude_test));
    std::vector<float> output(absl::GetFlag(FLAGS_size_magnitude_test));
#endif
    int pos = 0;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    for (const auto& item : input)
        {
            output[pos++] = std::norm(item);
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

#if USE_GLOG_AND_GFLAGS
    std::cout << "The squared magnitude of a " << FLAGS_size_magnitude_test
#else
    std::cout << "The squared magnitude of a " << absl::GetFlag(FLAGS_size_magnitude_test)
#endif
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
#if USE_GLOG_AND_GFLAGS
    arma::cx_fvec input(FLAGS_size_magnitude_test, arma::fill::zeros);
    arma::fvec output(FLAGS_size_magnitude_test);
#else
    arma::cx_fvec input(absl::GetFlag(FLAGS_size_magnitude_test), arma::fill::zeros);
    arma::fvec output(absl::GetFlag(FLAGS_size_magnitude_test));
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    output = arma::abs(arma::square(input));

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

#if USE_GLOG_AND_GFLAGS
    std::cout << "The squared magnitude of a " << FLAGS_size_magnitude_test
#else
    std::cout << "The squared magnitude of a " << absl::GetFlag(FLAGS_size_magnitude_test)
#endif
              << "-length vector using Armadillo computed in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}


TEST(MagnitudeSquaredTest, VolkComplexImplementation)
{
#if USE_GLOG_AND_GFLAGS
    auto* input = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_magnitude_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::fill_n(input, FLAGS_size_magnitude_test, std::complex<float>(0.0, 0.0));
    auto* output = static_cast<float*>(volk_gnsssdr_malloc(FLAGS_size_magnitude_test * sizeof(float), volk_gnsssdr_get_alignment()));
#else
    auto* input = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(absl::GetFlag(FLAGS_size_magnitude_test) * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::fill_n(input, absl::GetFlag(FLAGS_size_magnitude_test), std::complex<float>(0.0, 0.0));
    auto* output = static_cast<float*>(volk_gnsssdr_malloc(absl::GetFlag(FLAGS_size_magnitude_test) * sizeof(float), volk_gnsssdr_get_alignment()));
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

#if USE_GLOG_AND_GFLAGS
    volk_32fc_magnitude_squared_32f(output, input, static_cast<unsigned int>(FLAGS_size_magnitude_test));
#else
    volk_32fc_magnitude_squared_32f(output, input, static_cast<unsigned int>(absl::GetFlag(FLAGS_size_magnitude_test)));
#endif
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

#if USE_GLOG_AND_GFLAGS
    std::cout << "The squared magnitude of a " << FLAGS_size_magnitude_test
#else
    std::cout << "The squared magnitude of a " << absl::GetFlag(FLAGS_size_magnitude_test)
#endif
              << "-length vector using VOLK computed in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    volk_gnsssdr_free(input);
    volk_gnsssdr_free(output);
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}


TEST(MagnitudeSquaredTest, VolkComplexImplementationAlloc)
{
#if USE_GLOG_AND_GFLAGS
    volk_gnsssdr::vector<std::complex<float>> input(FLAGS_size_magnitude_test);  // or: input(FLAGS_size_magnitude_test, std::complex<float>(0.0, 0.0));
    std::fill_n(input.begin(), FLAGS_size_magnitude_test, std::complex<float>(0.0, 0.0));
    volk_gnsssdr::vector<float> output(FLAGS_size_magnitude_test);
#else
    volk_gnsssdr::vector<std::complex<float>> input(absl::GetFlag(FLAGS_size_magnitude_test));
    std::fill_n(input.begin(), absl::GetFlag(FLAGS_size_magnitude_test), std::complex<float>(0.0, 0.0));
    volk_gnsssdr::vector<float> output(absl::GetFlag(FLAGS_size_magnitude_test));
#endif

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
#if USE_GLOG_AND_GFLAGS
    volk_32fc_magnitude_squared_32f(output.data(), input.data(), static_cast<unsigned int>(FLAGS_size_magnitude_test));
#else
    volk_32fc_magnitude_squared_32f(output.data(), input.data(), static_cast<unsigned int>(absl::GetFlag(FLAGS_size_magnitude_test)));
#endif
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
#if USE_GLOG_AND_GFLAGS
    std::cout << "The squared magnitude of a " << FLAGS_size_magnitude_test
#else
    std::cout << "The squared magnitude of a " << absl::GetFlag(FLAGS_size_magnitude_test)
#endif
              << "-length vector using VOLK ALLOC computed in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}

//  volk_32f_accumulator_s32f(&d_input_power, d_magnitude, d_fft_size);
