// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0

#include "internal/hwcaps.h"
#include "define_introspection.inl"

#define LINE(ENUM, NAME, CPUINFO_FLAG, HWCAP, HWCAP2) \
    [ENUM] = (HardwareCapabilities){HWCAP, HWCAP2},
static const HardwareCapabilities kHardwareCapabilities[] = {
    INTROSPECTION_TABLE};
#undef LINE

#define LINE(ENUM, NAME, CPUINFO_FLAG, HWCAP, HWCAP2) [ENUM] = CPUINFO_FLAG,
static const char* kCpuInfoFlags[] = {INTROSPECTION_TABLE};
#undef LINE
