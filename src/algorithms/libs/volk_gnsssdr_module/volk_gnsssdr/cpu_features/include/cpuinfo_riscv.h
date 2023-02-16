// SPDX-FileCopyrightText: 2022 Google LLC
// SPDX-License-Identifier: Apache-2.0

#ifndef CPU_FEATURES_INCLUDE_CPUINFO_RISCV_H_
#define CPU_FEATURES_INCLUDE_CPUINFO_RISCV_H_

#include "cpu_features_cache_info.h"
#include "cpu_features_macros.h"

#if !defined(CPU_FEATURES_ARCH_RISCV)
#error "Including cpuinfo_riscv.h from a non-riscv target."
#endif

CPU_FEATURES_START_CPP_NAMESPACE

typedef struct
{
    // Base
    int RV32I : 1;  // Base Integer Instruction Set, 32-bit
    int RV64I : 1;  // Base Integer Instruction Set, 64-bit

    // Extension
    int M : 1;         // Standard Extension for Integer Multiplication/Division
    int A : 1;         // Standard Extension for Atomic Instructions
    int F : 1;         // Standard Extension for Single-Precision Floating-Point
    int D : 1;         // Standard Extension for Double-Precision Floating-Point
    int Q : 1;         // Standard Extension for Quad-Precision Floating-Point
    int C : 1;         // Standard Extension for Compressed Instructions
    int Zicsr : 1;     // Control and Status Register (CSR)
    int Zifencei : 1;  // Instruction-Fetch Fence
} RiscvFeatures;

typedef struct
{
    RiscvFeatures features;
    char uarch[64];   // 0 terminated string
    char vendor[64];  // 0 terminated string
} RiscvInfo;

typedef enum
{
    RISCV_RV32I,
    RISCV_RV64I,
    RISCV_M,
    RISCV_A,
    RISCV_F,
    RISCV_D,
    RISCV_Q,
    RISCV_C,
    RISCV_Zicsr,
    RISCV_Zifencei,
    RISCV_LAST_,
} RiscvFeaturesEnum;

RiscvInfo GetRiscvInfo(void);
int GetRiscvFeaturesEnumValue(const RiscvFeatures* features,
    RiscvFeaturesEnum value);
const char* GetRiscvFeaturesEnumName(RiscvFeaturesEnum);

CPU_FEATURES_END_CPP_NAMESPACE

#endif  // CPU_FEATURES_INCLUDE_CPUINFO_RISCV_H_