// SPDX-FileCopyrightText: 2022 Google LLC
// SPDX-License-Identifier: Apache-2.0
#ifndef CPU_FEATURES_INCLUDE_INTERNAL_WINDOWS_UTILS_H_
#define CPU_FEATURES_INCLUDE_INTERNAL_WINDOWS_UTILS_H_

#include "cpu_features_macros.h"

#ifdef CPU_FEATURES_OS_WINDOWS

#include <windows.h>  // IsProcessorFeaturePresent

// modern WinSDK winnt.h contains newer features detection definitions
#if !defined(PF_SSSE3_INSTRUCTIONS_AVAILABLE)
#define PF_SSSE3_INSTRUCTIONS_AVAILABLE 36
#endif

#if !defined(PF_SSE4_1_INSTRUCTIONS_AVAILABLE)
#define PF_SSE4_1_INSTRUCTIONS_AVAILABLE 37
#endif

#if !defined(PF_SSE4_2_INSTRUCTIONS_AVAILABLE)
#define PF_SSE4_2_INSTRUCTIONS_AVAILABLE 38
#endif

#endif  // CPU_FEATURES_OS_WINDOWS
#endif  // CPU_FEATURES_INCLUDE_INTERNAL_WINDOWS_UTILS_H_
