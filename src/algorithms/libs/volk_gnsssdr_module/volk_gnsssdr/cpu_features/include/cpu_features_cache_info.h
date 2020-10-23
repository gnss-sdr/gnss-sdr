// SPDX-FileCopyrightText: 2017 Google LLC
// SPDX-License-Identifier: Apache-2.0

#ifndef CPU_FEATURES_INCLUDE_CPUINFO_COMMON_H_
#define CPU_FEATURES_INCLUDE_CPUINFO_COMMON_H_

#include "cpu_features_macros.h"

CPU_FEATURES_START_CPP_NAMESPACE

typedef enum
{
    CPU_FEATURE_CACHE_NULL = 0,
    CPU_FEATURE_CACHE_DATA = 1,
    CPU_FEATURE_CACHE_INSTRUCTION = 2,
    CPU_FEATURE_CACHE_UNIFIED = 3,
    CPU_FEATURE_CACHE_TLB = 4,
    CPU_FEATURE_CACHE_DTLB = 5,
    CPU_FEATURE_CACHE_STLB = 6,
    CPU_FEATURE_CACHE_PREFETCH = 7
} CacheType;

typedef struct
{
    int level;
    CacheType cache_type;
    int cache_size;    // Cache size in bytes
    int ways;          // Associativity, 0 undefined, 0xFF fully associative
    int line_size;     // Cache line size in bytes
    int tlb_entries;   // number of entries for TLB
    int partitioning;  // number of lines per sector
} CacheLevelInfo;

// Increase this value if more cache levels are needed.
#ifndef CPU_FEATURES_MAX_CACHE_LEVEL
#define CPU_FEATURES_MAX_CACHE_LEVEL 10
#endif
typedef struct
{
    int size;
    CacheLevelInfo levels[CPU_FEATURES_MAX_CACHE_LEVEL];
} CacheInfo;

CPU_FEATURES_END_CPP_NAMESPACE

#endif  // CPU_FEATURES_INCLUDE_CPUINFO_COMMON_H_
