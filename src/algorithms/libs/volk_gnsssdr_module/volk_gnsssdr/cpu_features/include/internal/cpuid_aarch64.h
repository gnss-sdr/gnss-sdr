// SPDX-FileCopyrightText: 2023 Google LLC
// SPDX-License-Identifier: Apache-2.0

#ifndef CPU_FEATURES_INCLUDE_CPUID_AARCH64_H_
#define CPU_FEATURES_INCLUDE_CPUID_AARCH64_H_

#include "cpu_features_macros.h"
#include <stdint.h>

CPU_FEATURES_START_CPP_NAMESPACE

uint64_t GetMidrEl1(void);

CPU_FEATURES_END_CPP_NAMESPACE

#endif  // CPU_FEATURES_INCLUDE_CPUID_AARCH64_H_
