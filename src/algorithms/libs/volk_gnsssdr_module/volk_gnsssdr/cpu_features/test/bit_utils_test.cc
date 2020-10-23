// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include "gtest/gtest.h"
#include "internal/bit_utils.h"

namespace cpu_features
{
namespace
{
TEST(UtilsTest, IsBitSet)
{
    for (size_t bit_set = 0; bit_set < 32; ++bit_set)
        {
            const uint32_t value = 1UL << bit_set;
            for (uint32_t i = 0; i < 32; ++i)
                {
                    EXPECT_EQ(IsBitSet(value, i), i == bit_set);
                }
        }

    // testing 0, all bits should be 0.
    for (uint32_t i = 0; i < 32; ++i)
        {
            EXPECT_FALSE(IsBitSet(0, i));
        }

    // testing ~0, all bits should be 1.
    for (uint32_t i = 0; i < 32; ++i)
        {
            EXPECT_TRUE(IsBitSet(-1, i));
        }
}

TEST(UtilsTest, ExtractBitRange)
{
    // Extracting all bits gives the same number.
    EXPECT_EQ(ExtractBitRange(123, 31, 0), 123);
    // Extracting 1 bit gives parity.
    EXPECT_EQ(ExtractBitRange(123, 0, 0), 1);
    EXPECT_EQ(ExtractBitRange(122, 0, 0), 0);

    EXPECT_EQ(ExtractBitRange(0xF0, 7, 4), 0xF);
    EXPECT_EQ(ExtractBitRange(0x42 << 2, 10, 2), 0x42);
}

}  // namespace
}  // namespace cpu_features
