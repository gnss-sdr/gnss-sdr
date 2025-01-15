/*!
 * \file multiply_test.cc
 * \brief  This file implements tests for the multiplication of long arrays.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
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
#include <algorithm>
#include <chrono>
#include <complex>
#include <numeric>

#if USE_GLOG_AND_GFLAGS
DEFINE_int32(size_multiply_test, 100000, "Size of the arrays used for multiply testing");
#else
ABSL_FLAG(int32_t, size_multiply_test, 100000, "Size of the arrays used for multiply testing");
#endif

TEST(MultiplyTest, StandardCDoubleImplementation)
{
#if USE_GLOG_AND_GFLAGS
    auto* input = new double[FLAGS_size_multiply_test];
    auto* output = new double[FLAGS_size_multiply_test];
    std::fill_n(input, FLAGS_size_multiply_test, 0.0);
#else
    auto* input = new double[absl::GetFlag(FLAGS_size_multiply_test)];
    auto* output = new double[absl::GetFlag(FLAGS_size_multiply_test)];
    std::fill_n(input, absl::GetFlag(FLAGS_size_multiply_test), 0.0);
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
#if USE_GLOG_AND_GFLAGS
    for (int i = 0; i < FLAGS_size_multiply_test; i++)
#else
    for (int i = 0; i < absl::GetFlag(FLAGS_size_multiply_test); i++)
#endif
        {
            output[i] = input[i] * input[i];
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
#if USE_GLOG_AND_GFLAGS
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
#else
    std::cout << "Element-wise multiplication of " << absl::GetFlag(FLAGS_size_multiply_test)
#endif
              << " doubles in standard C finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";

    double acc = 0.0;
    double expected = 0.0;
#if USE_GLOG_AND_GFLAGS
    for (int i = 0; i < FLAGS_size_multiply_test; i++)
#else
    for (int i = 0; i < absl::GetFlag(FLAGS_size_multiply_test); i++)
#endif
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
#if USE_GLOG_AND_GFLAGS
    arma::vec input(FLAGS_size_multiply_test, arma::fill::zeros);
    arma::vec output(FLAGS_size_multiply_test);
#else
    arma::vec input(absl::GetFlag(FLAGS_size_multiply_test), arma::fill::zeros);
    arma::vec output(absl::GetFlag(FLAGS_size_multiply_test));
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    output = input % input;

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
#if USE_GLOG_AND_GFLAGS
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
#else
    std::cout << "Element-wise multiplication of " << absl::GetFlag(FLAGS_size_multiply_test)
#endif
              << "-length double Armadillo vectors finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
    ASSERT_EQ(0, arma::norm(output, 2));
}


TEST(MultiplyTest, StandardCComplexImplementation)
{
#if USE_GLOG_AND_GFLAGS
    auto* input = new std::complex<float>[FLAGS_size_multiply_test];
    auto* output = new std::complex<float>[FLAGS_size_multiply_test];
    std::fill_n(input, FLAGS_size_multiply_test, std::complex<float>(0.0, 0.0));
#else
    auto* input = new std::complex<float>[absl::GetFlag(FLAGS_size_multiply_test)];
    auto* output = new std::complex<float>[absl::GetFlag(FLAGS_size_multiply_test)];
    std::fill_n(input, absl::GetFlag(FLAGS_size_multiply_test), std::complex<float>(0.0, 0.0));
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

#if USE_GLOG_AND_GFLAGS
    for (int i = 0; i < FLAGS_size_multiply_test; i++)
#else
    for (int i = 0; i < absl::GetFlag(FLAGS_size_multiply_test); i++)
#endif
        {
            output[i] = input[i] * input[i];
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
#if USE_GLOG_AND_GFLAGS
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
#else
    std::cout << "Element-wise multiplication of " << absl::GetFlag(FLAGS_size_multiply_test)
#endif
              << " complex<float> in standard C finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";

    std::complex<float> expected(0.0, 0.0);
    std::complex<float> result(0.0, 0.0);
#if USE_GLOG_AND_GFLAGS
    for (int i = 0; i < FLAGS_size_multiply_test; i++)
#else
    for (int i = 0; i < absl::GetFlag(FLAGS_size_multiply_test); i++)
#endif
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
#if USE_GLOG_AND_GFLAGS
    const std::vector<std::complex<float>> input(FLAGS_size_multiply_test);
    std::vector<std::complex<float>> output(FLAGS_size_multiply_test);
#else
    const std::vector<std::complex<float>> input(absl::GetFlag(FLAGS_size_multiply_test));
    std::vector<std::complex<float>> output(absl::GetFlag(FLAGS_size_multiply_test));
#endif
    int pos = 0;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    // Trying a range-based for
    for (const auto& item : input)
        {
            output[pos++] = item * item;
        }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
#if USE_GLOG_AND_GFLAGS
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
#else
    std::cout << "Element-wise multiplication of " << absl::GetFlag(FLAGS_size_multiply_test)
#endif
              << " complex<float> vector (C++11-style) finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);

    std::complex<float> expected(0.0, 0.0);
    auto result = std::inner_product(output.begin(), output.end(), output.begin(), expected);
    ASSERT_EQ(expected, result);
}


TEST(MultiplyTest, ArmadilloComplexImplementation)
{
#if USE_GLOG_AND_GFLAGS
    arma::cx_fvec input(FLAGS_size_multiply_test, arma::fill::zeros);
    arma::cx_fvec output(FLAGS_size_multiply_test);
#else
    arma::cx_fvec input(absl::GetFlag(FLAGS_size_multiply_test), arma::fill::zeros);
    arma::cx_fvec output(absl::GetFlag(FLAGS_size_multiply_test));
#endif

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    output = input % input;

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

#if USE_GLOG_AND_GFLAGS
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
#else
    std::cout << "Element-wise multiplication of " << absl::GetFlag(FLAGS_size_multiply_test)
#endif
              << "-length complex float Armadillo vectors finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);
    ASSERT_EQ(0, arma::norm(output, 2));
}


TEST(MultiplyTest, VolkComplexImplementation)
{
#if USE_GLOG_AND_GFLAGS
    auto* input = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_multiply_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    auto* output = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(FLAGS_size_multiply_test * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::fill_n(input, FLAGS_size_multiply_test, std::complex<float>(0.0, 0.0));
#else
    auto* input = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(absl::GetFlag(FLAGS_size_multiply_test) * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    auto* output = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(absl::GetFlag(FLAGS_size_multiply_test) * sizeof(std::complex<float>), volk_gnsssdr_get_alignment()));
    std::fill_n(input, absl::GetFlag(FLAGS_size_multiply_test), std::complex<float>(0.0, 0.0));
#endif

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

#if USE_GLOG_AND_GFLAGS
    volk_32fc_x2_multiply_32fc(output, input, input, FLAGS_size_multiply_test);
#else
    volk_32fc_x2_multiply_32fc(output, input, input, absl::GetFlag(FLAGS_size_multiply_test));
#endif

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
#if USE_GLOG_AND_GFLAGS
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
#else
    std::cout << "Element-wise multiplication of " << absl::GetFlag(FLAGS_size_multiply_test)
#endif
              << "-length complex float vector using VOLK finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);

#if USE_GLOG_AND_GFLAGS
    auto* mag = static_cast<float*>(volk_gnsssdr_malloc(FLAGS_size_multiply_test * sizeof(float), volk_gnsssdr_get_alignment()));
    volk_32fc_magnitude_32f(mag, output, FLAGS_size_multiply_test);
#else
    auto* mag = static_cast<float*>(volk_gnsssdr_malloc(absl::GetFlag(FLAGS_size_multiply_test) * sizeof(float), volk_gnsssdr_get_alignment()));
    volk_32fc_magnitude_32f(mag, output, absl::GetFlag(FLAGS_size_multiply_test));
#endif

    auto* result = new float(0.0);

#if USE_GLOG_AND_GFLAGS
    volk_32f_accumulator_s32f(result, mag, FLAGS_size_multiply_test);
#else
    volk_32f_accumulator_s32f(result, mag, absl::GetFlag(FLAGS_size_multiply_test));
#endif

    // Comparing floating-point numbers is tricky.
    // Due to round-off errors, it is very unlikely that two floating-points will match exactly.
    // See https://google.github.io/googletest/reference/assertions.html#floating-point
    float expected = 0.0;
    ASSERT_FLOAT_EQ(expected, result[0]);
    volk_gnsssdr_free(input);
    volk_gnsssdr_free(output);
    volk_gnsssdr_free(mag);
}


TEST(MultiplyTest, VolkComplexImplementationAlloc)
{
#if USE_GLOG_AND_GFLAGS
    volk_gnsssdr::vector<std::complex<float>> input(FLAGS_size_multiply_test, std::complex<float>(0.0, 0.0));
    volk_gnsssdr::vector<std::complex<float>> output(FLAGS_size_multiply_test);
#else
    volk_gnsssdr::vector<std::complex<float>> input(absl::GetFlag(FLAGS_size_multiply_test), std::complex<float>(0.0, 0.0));
    volk_gnsssdr::vector<std::complex<float>> output(absl::GetFlag(FLAGS_size_multiply_test));
#endif
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

#if USE_GLOG_AND_GFLAGS
    volk_32fc_x2_multiply_32fc(output.data(), input.data(), input.data(), FLAGS_size_multiply_test);
#else
    volk_32fc_x2_multiply_32fc(output.data(), input.data(), input.data(), absl::GetFlag(FLAGS_size_multiply_test));
#endif

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

#if USE_GLOG_AND_GFLAGS
    std::cout << "Element-wise multiplication of " << FLAGS_size_multiply_test
#else
    std::cout << "Element-wise multiplication of " << absl::GetFlag(FLAGS_size_multiply_test)
#endif
              << "-length complex float vector using VOLK ALLOC finished in " << elapsed_seconds.count() * 1e6
              << " microseconds\n";
    ASSERT_LE(0, elapsed_seconds.count() * 1e6);

#if USE_GLOG_AND_GFLAGS
    volk_gnsssdr::vector<float> mag(FLAGS_size_multiply_test);
    volk_32fc_magnitude_32f(mag.data(), output.data(), FLAGS_size_multiply_test);
#else
    volk_gnsssdr::vector<float> mag(absl::GetFlag(FLAGS_size_multiply_test));
    volk_32fc_magnitude_32f(mag.data(), output.data(), absl::GetFlag(FLAGS_size_multiply_test));
#endif

    auto* result = new float(0.0);

#if USE_GLOG_AND_GFLAGS
    volk_32f_accumulator_s32f(result, mag.data(), FLAGS_size_multiply_test);
#else
    volk_32f_accumulator_s32f(result, mag.data(), absl::GetFlag(FLAGS_size_multiply_test));
#endif
    // Comparing floating-point numbers is tricky.
    // Due to round-off errors, it is very unlikely that two floating-points will match exactly.
    // See https://google.github.io/googletest/reference/assertions.html#floating-point
    float expected = 0.0;
    ASSERT_FLOAT_EQ(expected, result[0]);
}
