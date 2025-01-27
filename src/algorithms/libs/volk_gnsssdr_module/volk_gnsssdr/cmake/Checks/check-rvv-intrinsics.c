/*
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2025 C. Fernandez-Prades cfernandez(at)cttc.es
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if (__riscv_v_intrinsic >= 1000000 || __clang_major__ >= 18 || __GNUC__ >= 14)
int main() { return 0; }
#else
#error "rvv intrinsics aren't supported"
#endif
