// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0

// The following preprocessor constants must be defined before including this
// file:
//  - DEFINE_TABLE_FEATURE_TYPE, the underlying type (e.g. X86Features)
//  - DEFINE_TABLE_FEATURES, the list of FEATURE macros to be inserted.

// This file is to be included once per `cpuinfo_XXX.c` in order to construct
// feature getters and setters functions as well as several enum indexed tables
// from the db file.
// - `kGetters` a table of getters function pointers from feature enum to
// retrieve a feature,
// - `kSetters` a table of setters function pointers from feature enum to set a
// feature,
// - `kCpuInfoFlags` a table of strings from feature enum to /proc/cpuinfo
// flags,
// - `kHardwareCapabilities` a table of HardwareCapabilities structs indexed by
// their feature enum.

#ifndef SRC_DEFINE_TABLES_H_
#define SRC_DEFINE_TABLES_H_

#define FEATURE(ENUM, NAME, CPUINFO_FLAG, HWCAP, HWCAP2) [ENUM] = CPUINFO_FLAG,
static const char* kCpuInfoFlags[] = {DEFINE_TABLE_FEATURES};
#undef FEATURE

#ifndef DEFINE_TABLE_DONT_GENERATE_HWCAPS
#define FEATURE(ENUM, NAME, CPUINFO_FLAG, HWCAP, HWCAP2) \
    [ENUM] = (HardwareCapabilities){HWCAP, HWCAP2},
static const HardwareCapabilities kHardwareCapabilities[] = {
    DEFINE_TABLE_FEATURES};
#undef FEATURE
#endif  // DEFINE_TABLE_DONT_GENERATE_HWCAPS

#define FEATURE(ENUM, NAME, CPUINFO_FLAG, HWCAP, HWCAP2)                    \
    static void set_##ENUM(DEFINE_TABLE_FEATURE_TYPE* features, bool value) \
    {                                                                       \
        features->NAME = value;                                             \
    }                                                                       \
    static int get_##ENUM(const DEFINE_TABLE_FEATURE_TYPE* features)        \
    {                                                                       \
        return features->NAME;                                              \
    }
DEFINE_TABLE_FEATURES
#undef FEATURE

#define FEATURE(ENUM, NAME, CPUINFO_FLAG, HWCAP, HWCAP2) [ENUM] = set_##ENUM,
static void (*const kSetters[])(DEFINE_TABLE_FEATURE_TYPE*,
    bool) = {DEFINE_TABLE_FEATURES};
#undef FEATURE

#define FEATURE(ENUM, NAME, CPUINFO_FLAG, HWCAP, HWCAP2) [ENUM] = get_##ENUM,
static int (*const kGetters[])(const DEFINE_TABLE_FEATURE_TYPE*) = {
    DEFINE_TABLE_FEATURES};
#undef FEATURE

#endif  // SRC_DEFINE_TABLES_H_
