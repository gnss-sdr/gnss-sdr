// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0


#ifndef CPU_FEATURES_INCLUDE_INTERNAL_BIT_UTILS_H_
#define CPU_FEATURES_INCLUDE_INTERNAL_BIT_UTILS_H_

#include "cpu_features_macros.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

CPU_FEATURES_START_CPP_NAMESPACE

inline static bool IsBitSet(uint32_t reg, uint32_t bit)
{
    return (reg >> bit) & 0x1;
}

inline static uint32_t ExtractBitRange(uint32_t reg, uint32_t msb,
    uint32_t lsb)
{
    const uint64_t bits = msb - lsb + 1ULL;
    const uint64_t mask = (1ULL << bits) - 1ULL;
    assert(msb >= lsb);
    return (reg >> lsb) & mask;
}

CPU_FEATURES_END_CPP_NAMESPACE

#endif  // CPU_FEATURES_INCLUDE_INTERNAL_BIT_UTILS_H_
