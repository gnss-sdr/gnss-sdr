/*!
 * \file benchmark_redd_solomon.cc
 * \brief Benchmark for Reed Solomon decoder
 * \author Carles Fernandez-Prades, 2021. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "gnss_sdr_make_unique.h"  // for std::unique_ptr in C++11
#include "reed_solomon.h"
#include <benchmark/benchmark.h>
#include <memory>
#include <vector>

void bm_e1b_erasurecorrection_shortened(benchmark::State& state)
{
    std::vector<uint8_t> code_vector = {147, 109, 66, 23, 234, 140, 74, 234, 49, 89, 241, 253, 169, 161, 89, 93, 75, 142, 83, 102, 98, 218, 14, 197, 155, 151, 43, 181, 9, 163, 142, 111, 8, 118, 21, 47, 135, 139, 108, 215, 51, 147, 185, 52, 17, 151, 97, 102, 238, 71, 83, 114, 47, 80, 67, 199, 215, 162, 238, 77, 12, 72, 235, 21, 148, 213, 230, 54, 183, 82, 49, 104, 12, 228, 150, 157, 220, 112, 236, 187, 63, 31, 175, 47, 210, 164, 17, 104, 98, 46, 252, 165, 194, 57, 26, 213, 14, 133, 176, 148, 34, 9, 167, 43, 204, 198, 25, 164, 233, 55, 153, 31, 237, 84, 212, 76, 137, 242};

    auto rs = std::make_unique<ReedSolomon>(60, 29, 1, 195, 0, 137);

    // Use case: We have received Word 1, Word 3, Word 18, Word 20
    // So we have: c_0, c_2, g_1, g_3

    // Delete c_1
    for (int i = 16; i < 30; i++)
        {
            code_vector[i] = 0;
        }

    // Delete c_3
    for (int i = 44; i < 58; i++)
        {
            code_vector[i] = 0;
        }

    // Delete g_0
    for (int i = 58; i < 73; i++)
        {
            code_vector[i] = 0;
        }

    // Delete g_2
    for (int i = 88; i < 103; i++)
        {
            code_vector[i] = 0;
        }

    std::vector<uint8_t> code_vector_missing = code_vector;

    while (state.KeepRunning())
        {
            std::vector<int> erasure_positions;
            erasure_positions.reserve(60);
            for (int i = 16; i < 30; i++)
                {
                    erasure_positions.push_back(i);
                }

            // Delete c_3
            for (int i = 44; i < 58; i++)
                {
                    erasure_positions.push_back(i);
                }

            // Delete g_0
            for (int i = 58; i < 73; i++)
                {
                    erasure_positions.push_back(i + 137);  // erasure position refers to the unshortened code, so we add 137
                }

            // Delete g_2
            for (int i = 88; i < 103; i++)
                {
                    erasure_positions.push_back(i + 137);
                }

            int result = rs->decode(code_vector, erasure_positions);

            if (result < 0)
                {
                    state.SkipWithError("Failed to decode data!");
                    break;
                }
            state.PauseTiming();
            code_vector = code_vector_missing;
            state.ResumeTiming();
        }
}


void bm_e1b_erasurecorrection_unshortened(benchmark::State& state)
{
    std::vector<uint8_t> code_vector = {147, 109, 66, 23, 234, 140, 74, 234, 49, 89, 241, 253, 169, 161, 89, 93, 75, 142, 83, 102, 98, 218, 14, 197, 155, 151, 43, 181, 9, 163, 142, 111, 8, 118, 21, 47, 135, 139, 108, 215, 51, 147, 185, 52, 17, 151, 97, 102, 238, 71, 83, 114, 47, 80, 67, 199, 215, 162, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 238, 77, 12, 72, 235, 21, 148, 213, 230, 54, 183, 82, 49, 104, 12, 228, 150, 157, 220, 112, 236, 187, 63, 31, 175, 47, 210, 164, 17, 104, 98, 46, 252, 165, 194, 57, 26, 213, 14, 133, 176, 148, 34, 9, 167, 43, 204, 198, 25, 164, 233, 55, 153, 31, 237, 84, 212, 76, 137, 242};

    auto rs = std::make_unique<ReedSolomon>(60, 29, 1, 195, 0, 137);

    // Use case: We have received Word 1, Word 3, Word 18, Word 20
    // So we have: c_0, c_2, g_1, g_3

    // Delete c_1
    for (int i = 16; i < 30; i++)
        {
            code_vector[i] = 0;
        }

    // Delete c_3
    for (int i = 44; i < 58; i++)
        {
            code_vector[i] = 0;
        }

    // Delete g_0
    for (int i = 137 + 58; i < 137 + 73; i++)
        {
            code_vector[i] = 0;
        }

    // Delete g_2
    for (int i = 137 + 88; i < 137 + 103; i++)
        {
            code_vector[i] = 0;
        }

    std::vector<uint8_t> code_vector_missing = code_vector;

    while (state.KeepRunning())
        {
            std::vector<int> erasure_positions;
            erasure_positions.reserve(60);
            for (int i = 16; i < 30; i++)
                {
                    erasure_positions.push_back(i);
                }

            // Delete c_3
            for (int i = 44; i < 58; i++)
                {
                    erasure_positions.push_back(i);
                }

            // Delete g_0
            for (int i = 58; i < 73; i++)
                {
                    erasure_positions.push_back(i + 137);  // erasure position refers to the unshortened code, so we add 137
                }

            // Delete g_2
            for (int i = 88; i < 103; i++)
                {
                    erasure_positions.push_back(i + 137);
                }

            int result = rs->decode(code_vector, erasure_positions);

            if (result < 0)
                {
                    state.SkipWithError("Failed to decode data!");
                    break;
                }
            state.PauseTiming();
            code_vector = code_vector_missing;
            state.ResumeTiming();
        }
}


void bm_e6b_correction(benchmark::State& state)
{
    const std::vector<uint8_t> expected_output = {71, 12, 25, 210, 178, 81, 243, 9, 112, 98, 196, 203, 48, 125, 114, 165, 181, 193, 71, 174, 168, 42, 31, 128, 245, 87, 150, 58, 192, 66, 130, 179};

    std::vector<uint8_t> encoded_input = {
        71, 12, 25, 210, 178, 81, 243, 9, 112, 98, 196, 203, 48, 125, 114, 165, 181, 193, 71, 174, 168, 42, 31, 128, 245, 87, 150, 58, 192, 66, 130, 179, 133, 210, 122, 224, 75, 138, 20, 205, 14, 245, 209, 187, 246, 228, 12, 39, 244, 238, 223, 217, 84, 233, 137, 168, 153, 8, 94, 26, 99, 169, 149, 203, 115, 69, 211, 43, 70, 96, 70, 38, 160, 1, 232, 153, 223, 165, 93, 205, 101, 170, 60, 188, 198, 82, 168, 79, 95, 23, 118, 215, 187, 136, 24, 99, 252, 3, 144, 166, 117, 45, 168, 239, 77, 42, 246, 33, 122, 97, 242, 236, 13, 217, 96, 186, 71, 250, 242, 177, 125, 87, 27, 13, 118, 181, 178, 12, 27, 66, 31, 74, 127, 46, 112, 127, 116, 122, 190, 71, 240, 95, 78, 194, 113, 80, 46, 126, 74, 136, 118, 133, 105, 176, 47, 230, 162, 195, 93, 157, 72, 119, 13, 232, 151, 200, 191, 143, 75, 161, 111, 29, 158, 16, 181, 165, 92, 39, 17, 218, 228, 58, 176, 233, 55, 211, 195, 73, 37, 137, 232, 241, 150, 236, 152, 153, 53, 74, 81, 91, 160, 244, 21, 95, 176, 179, 141, 39, 61, 136, 16, 58, 160, 51, 210, 31, 134, 63, 203, 96, 219, 44, 231, 61, 220, 0, 241, 220, 207, 17, 52, 150, 117, 54, 222, 128, 101, 213, 164, 234, 74, 224, 57, 246, 70, 27, 202, 229, 4, 243, 128, 211, 158, 199, 4};

    // Introduce t = (n-k)/2 = 111 errors:
    for (int i = 0; i < 222; i += 2)
        {
            encoded_input[i] = 0;
        }
    std::vector<uint8_t> code_vector_missing = encoded_input;

    auto rs = std::make_unique<ReedSolomon>();

    while (state.KeepRunning())
        {
            int result = rs->decode(encoded_input);
            if (result < 0)
                {
                    state.SkipWithError("Failed to decode data!");
                    break;
                }
            state.PauseTiming();
            encoded_input = code_vector_missing;
            state.ResumeTiming();
        }
}

void bm_e6b_erasure(benchmark::State& state)
{
    const std::vector<uint8_t> expected_output = {71, 12, 25, 210, 178, 81, 243, 9, 112, 98, 196, 203, 48, 125, 114, 165, 181, 193, 71, 174, 168, 42, 31, 128, 245, 87, 150, 58, 192, 66, 130, 179};

    std::vector<uint8_t> encoded_input = {
        71, 12, 25, 210, 178, 81, 243, 9, 112, 98, 196, 203, 48, 125, 114, 165, 181, 193, 71, 174, 168, 42, 31, 128, 245, 87, 150, 58, 192, 66, 130, 179, 133, 210, 122, 224, 75, 138, 20, 205, 14, 245, 209, 187, 246, 228, 12, 39, 244, 238, 223, 217, 84, 233, 137, 168, 153, 8, 94, 26, 99, 169, 149, 203, 115, 69, 211, 43, 70, 96, 70, 38, 160, 1, 232, 153, 223, 165, 93, 205, 101, 170, 60, 188, 198, 82, 168, 79, 95, 23, 118, 215, 187, 136, 24, 99, 252, 3, 144, 166, 117, 45, 168, 239, 77, 42, 246, 33, 122, 97, 242, 236, 13, 217, 96, 186, 71, 250, 242, 177, 125, 87, 27, 13, 118, 181, 178, 12, 27, 66, 31, 74, 127, 46, 112, 127, 116, 122, 190, 71, 240, 95, 78, 194, 113, 80, 46, 126, 74, 136, 118, 133, 105, 176, 47, 230, 162, 195, 93, 157, 72, 119, 13, 232, 151, 200, 191, 143, 75, 161, 111, 29, 158, 16, 181, 165, 92, 39, 17, 218, 228, 58, 176, 233, 55, 211, 195, 73, 37, 137, 232, 241, 150, 236, 152, 153, 53, 74, 81, 91, 160, 244, 21, 95, 176, 179, 141, 39, 61, 136, 16, 58, 160, 51, 210, 31, 134, 63, 203, 96, 219, 44, 231, 61, 220, 0, 241, 220, 207, 17, 52, 150, 117, 54, 222, 128, 101, 213, 164, 234, 74, 224, 57, 246, 70, 27, 202, 229, 4, 243, 128, 211, 158, 199, 4};

    // Introduce 223 erasures:
    std::vector<int> erasure_positions;
    erasure_positions.reserve(223);
    for (int i = 0; i < 223; i++)
        {
            encoded_input[i] = 0;
            erasure_positions.push_back(i);
        }
    std::vector<uint8_t> code_vector_missing = encoded_input;

    auto rs = std::make_unique<ReedSolomon>();

    while (state.KeepRunning())
        {
            int result = rs->decode(encoded_input, erasure_positions);
            if (result < 0)
                {
                    state.SkipWithError("Failed to decode data!");
                    break;
                }
            state.PauseTiming();
            encoded_input = code_vector_missing;
            state.ResumeTiming();
        }
}


BENCHMARK(bm_e1b_erasurecorrection_shortened);
BENCHMARK(bm_e1b_erasurecorrection_unshortened);
BENCHMARK(bm_e6b_correction);
BENCHMARK(bm_e6b_erasure);
BENCHMARK_MAIN();
