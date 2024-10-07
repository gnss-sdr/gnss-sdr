/*!
 * \file conjugate_test.cc
 * \brief  This file implements tests for conjugation of long arrays.
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
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
DEFINE_int32(size_conjugate_test, 100000, "Size of the arrays used for conjugate testing");
#else
ABSL_FLAG(int32_t, size_conjugate_test, 100000, "Size of the arrays used for conjugate testing");
#endif


TEST(ConjugateTest, StandardCComplexImplementation)
{
#if USE_GLOG_AND_GFLAGS
    auto* input = new std::complex<float>[FLAGS_size_conjugate_test];
    auto* output = new std::complex<float>[FLAGS_size_conjugate_test];
    std::fill_n(input, FLAGS_size_conjugate_test, std::complex<float>(0.0, 0.0));
#else
    auto* input = new std::complex<float>[absl::GetFlag(FLAGS_size_conjugate_test)];
    auto* output = new std::complex<float>[absl::GetFlag(FLAGS_size_conjugate_test)];
    std::fill_n(input, absl::GetFlag(FLAGS_size_conjugate_test), std::complex<float>(0.0, 0.0));
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
#if USE_GLOG_AND_GFLAGS
    for (int i = 0; i < FLAGS_size_conjugate_test; i++)
#else
    for (int i = 0; i < absl::GetFlag(FLAGS_size_conjugate_test); i++)
#endif
        {
            output[i] = std::conj(input[i]);
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
#if USE_GLOG_AND_GFLAGS
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
#else
    std::cout << "Conjugate of a " << absl::GetFlag(FLAGS_size_conjugate_test)
#endif
              << "-length complex float vector in standard C finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";

    delete[] input;
    delete[] output;
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}


TEST(ConjugateTest, C11ComplexImplementation)
{
#if USE_GLOG_AND_GFLAGS
    const std::vector<std::complex<float>> input(FLAGS_size_conjugate_test);
    std::vector<std::complex<float>> output(FLAGS_size_conjugate_test);
#else
    const std::vector<std::complex<float>> input(absl::GetFlag(FLAGS_size_conjugate_test));
    std::vector<std::complex<float>> output(absl::GetFlag(FLAGS_size_conjugate_test));
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    int pos = 0;
    for (const auto& item : input)
        {
            output[pos++] = std::conj(item);
        }
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
#if USE_GLOG_AND_GFLAGS
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
#else
    std::cout << "Conjugate of a " << absl::GetFlag(FLAGS_size_conjugate_test)
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


TEST(ConjugateTest, ArmadilloComplexImplementation)
{
#if USE_GLOG_AND_GFLAGS
    arma::cx_fvec input(FLAGS_size_conjugate_test, arma::fill::zeros);
    arma::cx_fvec output(FLAGS_size_conjugate_test);
#else
    arma::cx_fvec input(absl::GetFlag(FLAGS_size_conjugate_test), arma::fill::zeros);
    arma::cx_fvec output(absl::GetFlag(FLAGS_size_conjugate_test));
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    output = arma::conj(input);

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

#if USE_GLOG_AND_GFLAGS
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
#else
    std::cout << "Conjugate of a " << absl::GetFlag(FLAGS_size_conjugate_test)
#endif
              << "-length complex float Armadillo vector finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}


TEST(ConjugateTest, VolkComplexImplementation)
{
#if USE_GLOG_AND_GFLAGS
    auto* input = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_conjugate_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    auto* output = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_conjugate_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::fill_n(input, FLAGS_size_conjugate_test, std::complex<float>(0.0, 0.0));
#else
    auto* input = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(absl::GetFlag(FLAGS_size_conjugate_test) * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    auto* output = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(absl::GetFlag(FLAGS_size_conjugate_test) * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::fill_n(input, absl::GetFlag(FLAGS_size_conjugate_test), std::complex<float>(0.0, 0.0));
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
#if USE_GLOG_AND_GFLAGS
    volk_32fc_conjugate_32fc(output, input, FLAGS_size_conjugate_test);
#else
    volk_32fc_conjugate_32fc(output, input, absl::GetFlag(FLAGS_size_conjugate_test));
#endif
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
#if USE_GLOG_AND_GFLAGS
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
#else
    std::cout << "Conjugate of a " << absl::GetFlag(FLAGS_size_conjugate_test)
#endif
              << "-length complex float vector using VOLK finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
    volk_gnsssdr_free(input);
    volk_gnsssdr_free(output);
}


TEST(ConjugateTest, VolkComplexImplementationAlloc)
{
#if USE_GLOG_AND_GFLAGS
    volk_gnsssdr::vector<std::complex<float>> input(FLAGS_size_conjugate_test, std::complex<float>(0.0, 0.0));
    volk_gnsssdr::vector<std::complex<float>> output(FLAGS_size_conjugate_test);
#else
    volk_gnsssdr::vector<std::complex<float>> input(absl::GetFlag(FLAGS_size_conjugate_test), std::complex<float>(0.0, 0.0));
    volk_gnsssdr::vector<std::complex<float>> output(absl::GetFlag(FLAGS_size_conjugate_test));
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
#if USE_GLOG_AND_GFLAGS
    volk_32fc_conjugate_32fc(output.data(), input.data(), FLAGS_size_conjugate_test);
#else
    volk_32fc_conjugate_32fc(output.data(), input.data(), absl::GetFlag(FLAGS_size_conjugate_test));
#endif
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
#if USE_GLOG_AND_GFLAGS
    std::cout << "Conjugate of a " << FLAGS_size_conjugate_test
#else
    std::cout << "Conjugate of a " << absl::GetFlag(FLAGS_size_conjugate_test)
#endif
              << "-length complex float vector using VOLK ALLOC finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
}
