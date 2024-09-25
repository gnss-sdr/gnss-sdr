/*!
 * \file item_type_helpers_test.cc
 * \brief  This file implements unit tests for the item_type_helpers
 *      custom block
 * \author Cillian O'Driscoll, 2019. cillian.odriscoll (at) gmail.com
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

#include "item_type_helpers.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <random>

class ItemTypeHelpersTest : public ::testing::Test
{
protected:
    static constexpr size_t N = 1000;

public:
    ItemTypeHelpersTest()
    {
        std::random_device r;
        std::default_random_engine e(r());

        std::uniform_int_distribution<int8_t> udist_int8(-100, 100);
        std::uniform_int_distribution<int16_t> udist_int16(-100, 100);
        std::uniform_real_distribution<float> udist_float(-100, 100);

        std::generate(byte_array_in.begin(), byte_array_in.end(), [&udist_int8, &e]() { return udist_int8(e); });

        std::generate(short_array_in.begin(), short_array_in.end(), [&udist_int16, &e]() { return udist_int16(e); });

        std::generate(float_array_in.begin(), float_array_in.end(), [&udist_float, &e]() { return udist_float(e); });
    }

    std::vector<std::string> valid_item_types = {"byte", "ibyte", "cbyte",
        "short", "ishort", "cshort", "float", "gr_complex"};

    std::vector<std::string> invalid_item_types = {"i8", "tfgs", "cbite",
        "shirt", "qshort", "csort", "flat", "igr_complex"};

    std::array<int8_t, 2 * N> byte_array_in;
    std::array<int8_t, 2 * N> byte_array_out;

    std::array<int16_t, 2 * N> short_array_in;
    std::array<int16_t, 2 * N> short_array_out;

    std::array<float, 2 * N> float_array_in;
    std::array<float, 2 * N> float_array_out;
};


TEST_F(ItemTypeHelpersTest, CheckValidTypes)
{
    for (auto &valid_type : valid_item_types)
        {
            EXPECT_TRUE(item_type_valid(valid_type));
        }

    for (auto &invalid_type : invalid_item_types)
        {
            EXPECT_FALSE(item_type_valid(invalid_type));
        }
}


TEST_F(ItemTypeHelpersTest, CheckSizes)
{
    EXPECT_EQ(item_type_size("byte"), 1);
    EXPECT_EQ(item_type_size("ibyte"), 1);
    EXPECT_EQ(item_type_size("cbyte"), 2);

    EXPECT_EQ(item_type_size("short"), 2);
    EXPECT_EQ(item_type_size("ishort"), 2);
    EXPECT_EQ(item_type_size("cshort"), 4);

    EXPECT_EQ(item_type_size("float"), 4);
    EXPECT_EQ(item_type_size("gr_complex"), 8);

    for (auto &invalid_type : invalid_item_types)
        {
            EXPECT_EQ(item_type_size(invalid_type), 0);
        }
}


TEST_F(ItemTypeHelpersTest, CheckMakeConverters)
{
    for (auto &input_type : valid_item_types)
        {
            for (auto &output_type : valid_item_types)
                {
                    item_type_converter_t converter = nullptr;

                    if (item_type_is_complex(input_type) == item_type_is_complex(output_type))
                        {
                            converter = make_vector_converter(input_type, output_type);
                            EXPECT_NE(converter, nullptr);
                        }
                    else
                        {
                            EXPECT_THROW(converter = make_vector_converter(input_type, output_type), std::runtime_error);
                        }
                }
        }
}


TEST_F(ItemTypeHelpersTest, CheckConversionsReal)
{
    std::string input_type = "byte";
    std::string output_type = "byte";
    item_type_converter_t converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(byte_array_out.data(), byte_array_in.data(), N);
    EXPECT_TRUE(std::equal(byte_array_in.begin(), byte_array_in.begin() + N, byte_array_out.begin()));

    input_type = "byte";
    output_type = "short";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(short_array_out.data(), byte_array_in.data(), N);
    converter = make_vector_converter(output_type, input_type);
    EXPECT_NE(converter, nullptr);
    converter(byte_array_out.data(), short_array_out.data(), N);
    EXPECT_TRUE(std::equal(byte_array_out.begin(), byte_array_out.begin() + N, byte_array_in.begin()));

    input_type = "byte";
    output_type = "float";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(float_array_out.data(), byte_array_in.data(), N);
    converter = make_vector_converter(output_type, input_type);
    EXPECT_NE(converter, nullptr);
    converter(byte_array_out.data(), float_array_out.data(), N);
    EXPECT_TRUE(std::equal(byte_array_out.begin(), byte_array_out.begin() + N, byte_array_in.begin()));

    input_type = "short";
    output_type = "short";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(short_array_out.data(), short_array_in.data(), N);
    EXPECT_TRUE(std::equal(short_array_in.begin(), short_array_in.begin() + N, short_array_out.begin()));

    input_type = "short";
    output_type = "float";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(float_array_out.data(), short_array_in.data(), N);
    converter = make_vector_converter(output_type, input_type);
    EXPECT_NE(converter, nullptr);
    converter(short_array_out.data(), float_array_out.data(), N);
    EXPECT_TRUE(std::equal(short_array_out.begin(), short_array_out.begin() + N, short_array_in.begin()));

    input_type = "float";
    output_type = "float";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(float_array_out.data(), float_array_in.data(), N);
    EXPECT_TRUE(std::equal(float_array_in.begin(), float_array_in.begin() + N, float_array_out.begin()));
}


TEST_F(ItemTypeHelpersTest, CheckConversionsComplex)
{
    std::string input_type = "cbyte";
    std::string output_type = "cbyte";
    item_type_converter_t converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(byte_array_out.data(), byte_array_in.data(), N);
    EXPECT_TRUE(std::equal(byte_array_in.begin(), byte_array_in.begin() + N, byte_array_out.begin()));

    input_type = "cbyte";
    output_type = "ibyte";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(byte_array_out.data(), byte_array_in.data(), N);
    EXPECT_TRUE(std::equal(byte_array_in.begin(), byte_array_in.begin() + N, byte_array_out.begin()));

    input_type = "cbyte";
    output_type = "cshort";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(short_array_out.data(), byte_array_in.data(), N);
    converter = make_vector_converter(output_type, input_type);
    EXPECT_NE(converter, nullptr);
    converter(byte_array_out.data(), short_array_out.data(), N);
    EXPECT_TRUE(std::equal(byte_array_out.begin(), byte_array_out.begin() + N, byte_array_in.begin()));

    input_type = "cbyte";
    output_type = "ishort";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(short_array_out.data(), byte_array_in.data(), N);
    converter = make_vector_converter(output_type, input_type);
    EXPECT_NE(converter, nullptr);
    converter(byte_array_out.data(), short_array_out.data(), N);
    EXPECT_TRUE(std::equal(byte_array_out.begin(), byte_array_out.begin() + N, byte_array_in.begin()));

    input_type = "cbyte";
    output_type = "gr_complex";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(float_array_out.data(), byte_array_in.data(), N);
    converter = make_vector_converter(output_type, input_type);
    EXPECT_NE(converter, nullptr);
    converter(byte_array_out.data(), float_array_out.data(), N);
    EXPECT_TRUE(std::equal(byte_array_out.begin(), byte_array_out.begin() + N, byte_array_in.begin()));

    input_type = "cshort";
    output_type = "cshort";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(short_array_out.data(), short_array_in.data(), N);
    EXPECT_TRUE(std::equal(short_array_in.begin(), short_array_in.begin() + N, short_array_out.begin()));

    input_type = "cshort";
    output_type = "ishort";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(short_array_out.data(), short_array_in.data(), N);
    EXPECT_TRUE(std::equal(short_array_in.begin(), short_array_in.begin() + N, short_array_out.begin()));

    input_type = "cshort";
    output_type = "gr_complex";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(float_array_out.data(), short_array_in.data(), N);
    converter = make_vector_converter(output_type, input_type);
    EXPECT_NE(converter, nullptr);
    converter(short_array_out.data(), float_array_out.data(), N);
    EXPECT_TRUE(std::equal(short_array_out.begin(), short_array_out.begin() + N, short_array_in.begin()));

    input_type = "gr_complex";
    output_type = "gr_complex";
    converter = make_vector_converter(input_type, output_type);
    EXPECT_NE(converter, nullptr);
    converter(float_array_out.data(), float_array_in.data(), N);
    EXPECT_TRUE(std::equal(float_array_in.begin(), float_array_in.begin() + N, float_array_out.begin()));
}
