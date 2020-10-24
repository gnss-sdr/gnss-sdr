// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0


#ifndef CPU_FEATURES_INCLUDE_INTERNAL_CPUID_X86_H_
#define CPU_FEATURES_INCLUDE_INTERNAL_CPUID_X86_H_

#include "cpu_features_macros.h"
#include <stdint.h>

CPU_FEATURES_START_CPP_NAMESPACE

// A struct to hold the result of a call to cpuid.
typedef struct
{
    uint32_t eax, ebx, ecx, edx;
} Leaf;

// Returns the result of a call to the cpuid instruction.
Leaf GetCpuidLeaf(uint32_t leaf_id, int ecx);

// Returns the eax value of the XCR0 register.
uint32_t GetXCR0Eax(void);

CPU_FEATURES_END_CPP_NAMESPACE

#endif  // CPU_FEATURES_INCLUDE_INTERNAL_CPUID_X86_H_
