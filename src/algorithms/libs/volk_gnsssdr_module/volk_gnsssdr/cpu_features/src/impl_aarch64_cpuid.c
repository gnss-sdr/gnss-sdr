// SPDX-FileCopyrightText: 2023 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include "cpu_features_macros.h"

#ifdef CPU_FEATURES_ARCH_AARCH64
#if (defined(CPU_FEATURES_OS_FREEBSD) || defined(CPU_FEATURES_OS_LINUX) || \
     defined(CPU_FEATURES_OS_ANDROID))
#if (defined(CPU_FEATURES_COMPILER_GCC) || defined(CPU_FEATURES_COMPILER_CLANG))

#include "internal/cpuid_aarch64.h"

#ifdef CPU_FEATURES_MOCK_CPUID_AARCH64
// Implementation will be provided by test/cpuinfo_aarch64_test.cc.
#else
uint64_t GetMidrEl1(void)
{
    uint64_t midr_el1;
    // clang-format off
    __asm("mrs %0, MIDR_EL1" : "=r"(midr_el1));
    // clang-format on
    return midr_el1;
}
#endif  // CPU_FEATURES_MOCK_CPUID_AARCH64

#else
#error "Unsupported compiler, aarch64 cpuid requires either GCC or Clang."
#endif  // (defined(CPU_FEATURES_COMPILER_GCC) ||
        // defined(CPU_FEATURES_COMPILER_CLANG))
#endif  // (defined(CPU_FEATURES_OS_FREEBSD) || defined(CPU_FEATURES_OS_LINUX)
        // || defined(CPU_FEATURES_OS_ANDROID))
#endif  // CPU_FEATURES_ARCH_AARCH64