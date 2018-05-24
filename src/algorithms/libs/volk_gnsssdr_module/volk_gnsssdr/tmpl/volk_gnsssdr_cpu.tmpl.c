/* Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 */

#include <volk_gnsssdr/volk_gnsssdr_cpu.h>
#include <volk_gnsssdr/volk_gnsssdr_config_fixed.h>
#include <stdlib.h>
#include <string.h>

struct VOLK_CPU volk_gnsssdr_cpu;

#if defined(__i386__) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64)
#define VOLK_CPU_x86
#endif

#if defined(VOLK_CPU_x86)

//implement get cpuid for gcc compilers using a system or local copy of cpuid.h
#if defined(__GNUC__)
#include <cpuid.h>
#define cpuid_x86(op, r) __get_cpuid(op, (unsigned int *)r + 0, (unsigned int *)r + 1, (unsigned int *)r + 2, (unsigned int *)r + 3)
#define cpuid_x86_count(op, count, regs) __cpuid_count(op, count, *((unsigned int *)regs), *((unsigned int *)regs + 1), *((unsigned int *)regs + 2), *((unsigned int *)regs + 3))

/* Return Intel AVX extended CPU capabilities register.
     * This function will bomb on non-AVX-capable machines, so
     * check for AVX capability before executing.
     */
#if ((__GNUC__ > 4 || __GNUC__ == 4 && __GNUC_MINOR__ >= 2) || (__clang_major__ >= 3)) && defined(HAVE_XGETBV)
static inline unsigned long long _xgetbv(unsigned int index)
{
    unsigned int eax, edx;
    __VOLK_ASM __VOLK_VOLATILE("xgetbv"
                               : "=a"(eax), "=d"(edx)
                               : "c"(index));
    return ((unsigned long long)edx << 32) | eax;
}
#define __xgetbv() _xgetbv(0)
#else
#define __xgetbv() 0
#endif

//implement get cpuid for MSVC compilers using __cpuid intrinsic
#elif defined(_MSC_VER) && defined(HAVE_INTRIN_H)
#include <intrin.h>
#define cpuid_x86(op, r) __cpuid(((int *)r), op)

#if defined(_XCR_XFEATURE_ENABLED_MASK)
#define __xgetbv() _xgetbv(_XCR_XFEATURE_ENABLED_MASK)
#else
#define __xgetbv() 0
#endif

#else
#error "A get cpuid for volk_gnsssdr is not available on this compiler..."
#endif  //defined(__GNUC__)

#endif  //defined(VOLK_CPU_x86)

static inline unsigned int cpuid_count_x86_bit(unsigned int level, unsigned int count, unsigned int reg, unsigned int bit)
{
#if defined(VOLK_CPU_x86)
    unsigned int regs[4] = {0};
    cpuid_x86_count(level, count, regs);
    return regs[reg] >> bit & 0x01;
#else
    return 0;
#endif
}

static inline unsigned int cpuid_x86_bit(unsigned int reg, unsigned int op, unsigned int bit)
{
#if defined(VOLK_CPU_x86)
    unsigned int regs[4];
    memset(regs, 0, sizeof(unsigned int) * 4);
    cpuid_x86(op, regs);
    return regs[reg] >> bit & 0x01;
#else
    return 0;
#endif
}

static inline unsigned int check_extended_cpuid(unsigned int val)
{
#if defined(VOLK_CPU_x86)
    unsigned int regs[4];
    memset(regs, 0, sizeof(unsigned int) * 4);
    cpuid_x86(0x80000000, regs);
    return regs[0] >= val;
#else
    return 0;
#endif
}

static inline unsigned int get_avx_enabled(void)
{
#if defined(VOLK_CPU_x86)
    return __xgetbv() & 0x6;
#else
    return 0;
#endif
}

static inline unsigned int get_avx2_enabled(void)
{
#if defined(VOLK_CPU_x86)
    return __xgetbv() & 0x6;
#else
    return 0;
#endif
}

//neon detection is linux specific
#if defined(__arm__) && defined(__linux__)
#include <asm/hwcap.h>
#include <linux/auxvec.h>
#include <stdio.h>
#define VOLK_CPU_ARMV7
#endif

static int has_neonv7(void)
{
#if defined(VOLK_CPU_ARMV7)
    FILE *auxvec_f;
    unsigned long auxvec[2];
    unsigned int found_neon = 0;
    auxvec_f = fopen("/proc/self/auxv", "rb");
    if (!auxvec_f) return 0;

    size_t r = 1;
    //so auxv is basically 32b of ID and 32b of value
    //so it goes like this
    while (!found_neon && r)
        {
            r = fread(auxvec, sizeof(unsigned long), 2, auxvec_f);
            if ((auxvec[0] == AT_HWCAP) && (auxvec[1] & HWCAP_NEON))
                found_neon = 1;
        }

    fclose(auxvec_f);
    return found_neon;
#else
    return 0;
#endif
}

//\todo: Fix this to really check for neon on aarch64
//neon detection is linux specific
#if defined(__aarch64__) && defined(__linux__)
#include <asm/hwcap.h>
#include <linux/auxvec.h>
#include <stdio.h>
#define VOLK_CPU_ARMV8
#endif

static int has_neonv8(void)
{
#if defined(VOLK_CPU_ARMV8)
    FILE *auxvec_f;
    unsigned long auxvec[2];
    unsigned int found_neon = 0;
    auxvec_f = fopen("/proc/self/auxv", "rb");
    if (!auxvec_f) return 0;

    size_t r = 1;
    //so auxv is basically 32b of ID and 32b of value
    //so it goes like this
    while (!found_neon && r)
        {
            r = fread(auxvec, sizeof(unsigned long), 2, auxvec_f);
            if ((auxvec[0] == AT_HWCAP) && (auxvec[1] & HWCAP_ASIMD))
                found_neon = 1;
        }

    fclose(auxvec_f);
    return found_neon;
#else
    return 0;
#endif
}

static int has_neon(void)
{
#if defined(VOLK_CPU_ARMV8) || defined(VOLK_CPU_ARMV7)
    if (has_neonv7() || has_neonv8())
        return 1;
    else
        return 0;
#else
    return 0;
#endif
}
// clang-format off

%for arch in archs:
static int i_can_has_${arch.name} (void) {
    %for check, params in arch.checks:
    if (${check}(<% joined_params = ', '.join(params)%>${joined_params}) == 0) return 0;
    %endfor
    return 1;
}
%endfor

#if defined(HAVE_FENV_H)
    #if defined(FE_TONEAREST)
        #include <fenv.h>
        static inline void set_float_rounding(void){
            fesetround(FE_TONEAREST);
        }
    #else
        static inline void set_float_rounding(void){
            //do nothing
        }
    #endif
#elif defined(_MSC_VER)
    #include <float.h>
    static inline void set_float_rounding(void){
        unsigned int cwrd;
        _controlfp_s(&cwrd, 0, 0);
        _controlfp_s(&cwrd, _RC_NEAR, _MCW_RC);
    }
#else
    static inline void set_float_rounding(void){
        //do nothing
    }
#endif

void volk_gnsssdr_cpu_init() {
    %for arch in archs:
    volk_gnsssdr_cpu.has_${arch.name} = &i_can_has_${arch.name};
    %endfor
    set_float_rounding();
}

unsigned int volk_gnsssdr_get_lvarch() {
    unsigned int retval = 0;
    volk_gnsssdr_cpu_init();
    %for arch in archs:
    retval += volk_gnsssdr_cpu.has_${arch.name}() << LV_${arch.name.upper()};
    %endfor
    return retval;
}
// clang-format on
