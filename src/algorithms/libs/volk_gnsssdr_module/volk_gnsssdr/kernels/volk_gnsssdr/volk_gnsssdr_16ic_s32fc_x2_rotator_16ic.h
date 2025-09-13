/*!
 * \file volk_gnsssdr_16ic_s32fc_x2_rotator_16ic.h
 * \brief VOLK_GNSSSDR kernel: rotates a 16 bits complex vector.
 * \authors <ul>
 *          <li> Carles Fernandez-Prades, 2015  cfernandez at cttc.es
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that rotates a 16-bit complex vector
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

/*!
 * \page volk_gnsssdr_16ic_s32fc_x2_rotator_16ic
 *
 * \b Overview
 *
 * Rotates a complex vector (16-bit integer samples each component).
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t* phase_inc, lv_32fc_t* phase, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li inVector:   Vector to be rotated.
 * \li phase_inc:  Phase increment in each sample = lv_cmake(cos(phase_step_rad), sin(phase_step_rad))
 * \li phase:      Initial phase = lv_cmake(cos(initial_phase_rad), sin(initial_phase_rad))
 * \li num_points: Number of complex values to be rotated and stored into \p outVector
 *
 * \b Outputs
 * \li phase:      Final phase.
 * \li outVector:  The resampled vector.
 *
 */

#ifndef INCLUDED_volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_H

#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <math.h>


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_generic(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t* phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    unsigned int i = 0;
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;
    for (i = 0; i < (unsigned int)(num_points); ++i)
        {
            tmp16 = *inVector++;
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            *outVector++ = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= *phase_inc;
            // Regenerate phase
            if (i % 512 == 0)
                {
                    // printf("Phase before regeneration %i: %f,%f  Modulus: %f\n", n,lv_creal(*phase),lv_cimag(*phase), cabsf(*phase));
#ifdef __cplusplus
                    (*phase) /= std::abs((*phase));
#else
                    (*phase) /= hypotf(lv_creal(*phase), lv_cimag(*phase));
#endif
                    // printf("Phase after regeneration %i: %f,%f  Modulus: %f\n", n,lv_creal(*phase),lv_cimag(*phase), cabsf(*phase));
                }
        }
}

#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_generic_reload(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t* phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    unsigned int ROTATOR_RELOAD = 512;
    unsigned int n = 0;
    unsigned int j = 0;
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;
    for (; n < num_points / ROTATOR_RELOAD; n++)
        {
            for (j = 0; j < ROTATOR_RELOAD; j++)
                {
                    tmp16 = *inVector++;
                    tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
                    *outVector++ = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
                    (*phase) *= *phase_inc;
                }
                // Regenerate phase
                // printf("Phase before regeneration %i: %f,%f  Modulus: %f\n", n,lv_creal(*phase),lv_cimag(*phase), cabsf(*phase));
#ifdef __cplusplus
            (*phase) /= std::abs((*phase));
#else
            (*phase) /= hypotf(lv_creal(*phase), lv_cimag(*phase));
#endif
            // printf("Phase after regeneration %i: %f,%f  Modulus: %f\n", n,lv_creal(*phase),lv_cimag(*phase), cabsf(*phase));
        }
    for (j = 0; j < num_points % ROTATOR_RELOAD; j++)
        {
            tmp16 = *inVector++;
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            *outVector++ = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= *phase_inc;
        }
}

#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_a_sse3(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t* phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    unsigned int number;
    __m128 a, b, two_phase_acc_reg, two_phase_inc_reg;
    __m128i c1, c2, result;
    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_inc[2];
    two_phase_inc[0] = *phase_inc * *phase_inc;
    two_phase_inc[1] = *phase_inc * *phase_inc;
    two_phase_inc_reg = _mm_load_ps((float*)two_phase_inc);
    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_acc[2];
    two_phase_acc[0] = (*phase);
    two_phase_acc[1] = (*phase) * *phase_inc;
    two_phase_acc_reg = _mm_load_ps((float*)two_phase_acc);

    const lv_16sc_t* _in = inVector;
    lv_16sc_t* _out = outVector;

    __m128 yl, yh, tmp1, tmp2, tmp3;
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;

    for (number = 0; number < sse_iters; number++)
        {
            a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
            b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            c1 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // next two samples
            _in += 2;
            a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            __VOLK_GNSSSDR_PREFETCH(_in + 8);
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
            b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            c2 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // store four output samples
            result = _mm_packs_epi32(c1, c2);  // convert from 32ic to 16ic
            _mm_store_si128((__m128i*)_out, result);

            // Regenerate phase
            if ((number % 512) == 0)
                {
                    tmp1 = _mm_mul_ps(two_phase_acc_reg, two_phase_acc_reg);
                    tmp2 = _mm_hadd_ps(tmp1, tmp1);
                    tmp1 = _mm_shuffle_ps(tmp2, tmp2, 0xD8);
                    tmp2 = _mm_sqrt_ps(tmp1);
                    two_phase_acc_reg = _mm_div_ps(two_phase_acc_reg, tmp2);
                }

            // next two samples
            _in += 2;
            _out += 4;
        }

    _mm_store_ps((float*)two_phase_acc, two_phase_acc_reg);
    (*phase) = two_phase_acc[0];

    for (number = sse_iters * 4; number < num_points; ++number)
        {
            tmp16 = *_in++;
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            *_out++ = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= *phase_inc;
        }
}

#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_a_sse3_reload(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t* phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    const unsigned int ROTATOR_RELOAD = 512;
    unsigned int n;
    unsigned int j;
    __m128 a, b, two_phase_acc_reg, two_phase_inc_reg;
    __m128i c1, c2, result;
    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_inc[2];
    two_phase_inc[0] = *phase_inc * *phase_inc;
    two_phase_inc[1] = *phase_inc * *phase_inc;
    two_phase_inc_reg = _mm_load_ps((float*)two_phase_inc);
    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_acc[2];
    two_phase_acc[0] = (*phase);
    two_phase_acc[1] = (*phase) * *phase_inc;
    two_phase_acc_reg = _mm_load_ps((float*)two_phase_acc);

    const lv_16sc_t* _in = inVector;

    lv_16sc_t* _out = outVector;

    __m128 yl, yh, tmp1, tmp2, tmp3;
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;

    for (n = 0; n < sse_iters / ROTATOR_RELOAD; n++)
        {
            for (j = 0; j < ROTATOR_RELOAD; j++)
                {
                    a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
                    // complex 32fc multiplication b=a*two_phase_acc_reg
                    yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
                    yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
                    tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
                    a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
                    tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
                    b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
                    c1 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

                    // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
                    yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
                    yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
                    tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
                    tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
                    tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
                    two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

                    // next two samples
                    _in += 2;
                    a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
                    __VOLK_GNSSSDR_PREFETCH(_in + 8);
                    // complex 32fc multiplication b=a*two_phase_acc_reg
                    yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
                    yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
                    tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
                    a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
                    tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
                    b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
                    c2 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

                    // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
                    yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
                    yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
                    tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
                    tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
                    tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
                    two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

                    // store four output samples
                    result = _mm_packs_epi32(c1, c2);  // convert from 32ic to 16ic
                    _mm_store_si128((__m128i*)_out, result);

                    // next two samples
                    _in += 2;
                    _out += 4;
                }
            // Regenerate phase
            tmp1 = _mm_mul_ps(two_phase_acc_reg, two_phase_acc_reg);
            tmp2 = _mm_hadd_ps(tmp1, tmp1);
            tmp1 = _mm_shuffle_ps(tmp2, tmp2, 0xD8);
            tmp2 = _mm_sqrt_ps(tmp1);
            two_phase_acc_reg = _mm_div_ps(two_phase_acc_reg, tmp2);
        }

    for (j = 0; j < sse_iters % ROTATOR_RELOAD; j++)
        {
            a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
            b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            c1 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // next two samples
            _in += 2;
            a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            __VOLK_GNSSSDR_PREFETCH(_in + 8);
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
            b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            c2 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // store four output samples
            result = _mm_packs_epi32(c1, c2);  // convert from 32ic to 16ic
            _mm_store_si128((__m128i*)_out, result);

            // next two samples
            _in += 2;
            _out += 4;
        }

    _mm_store_ps((float*)two_phase_acc, two_phase_acc_reg);
    (*phase) = two_phase_acc[0];

    for (n = sse_iters * 4; n < num_points; ++n)
        {
            tmp16 = *_in++;
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            *_out++ = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= *phase_inc;
        }
}

#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_u_sse3(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t* phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    unsigned int number;
    __m128 a, b, two_phase_acc_reg, two_phase_inc_reg;
    __m128i c1, c2, result;
    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_inc[2];
    two_phase_inc[0] = *phase_inc * *phase_inc;
    two_phase_inc[1] = *phase_inc * *phase_inc;
    two_phase_inc_reg = _mm_load_ps((float*)two_phase_inc);
    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_acc[2];
    two_phase_acc[0] = (*phase);
    two_phase_acc[1] = (*phase) * *phase_inc;
    two_phase_acc_reg = _mm_load_ps((float*)two_phase_acc);

    const lv_16sc_t* _in = inVector;

    lv_16sc_t* _out = outVector;

    __m128 yl, yh, tmp1, tmp2, tmp3;
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;

    for (number = 0; number < sse_iters; number++)
        {
            a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
            b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            c1 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // next two samples
            _in += 2;
            a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            __VOLK_GNSSSDR_PREFETCH(_in + 8);
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
            b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            c2 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // store four output samples
            result = _mm_packs_epi32(c1, c2);  // convert from 32ic to 16ic
            _mm_storeu_si128((__m128i*)_out, result);

            // Regenerate phase
            if ((number % 512) == 0)
                {
                    tmp1 = _mm_mul_ps(two_phase_acc_reg, two_phase_acc_reg);
                    tmp2 = _mm_hadd_ps(tmp1, tmp1);
                    tmp1 = _mm_shuffle_ps(tmp2, tmp2, 0xD8);
                    tmp2 = _mm_sqrt_ps(tmp1);
                    two_phase_acc_reg = _mm_div_ps(two_phase_acc_reg, tmp2);
                }

            // next two samples
            _in += 2;
            _out += 4;
        }

    _mm_store_ps((float*)two_phase_acc, two_phase_acc_reg);
    (*phase) = two_phase_acc[0];

    for (number = sse_iters * 4; number < num_points; ++number)
        {
            tmp16 = *_in++;
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            *_out++ = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= *phase_inc;
        }
}

#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_u_sse3_reload(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t* phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    unsigned int ROTATOR_RELOAD = 512;
    unsigned int n;
    unsigned int j;
    __m128 a, b, two_phase_acc_reg, two_phase_inc_reg;
    __m128i c1, c2, result;
    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_inc[2];
    two_phase_inc[0] = *phase_inc * *phase_inc;
    two_phase_inc[1] = *phase_inc * *phase_inc;
    two_phase_inc_reg = _mm_load_ps((float*)two_phase_inc);
    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_acc[2];
    two_phase_acc[0] = (*phase);
    two_phase_acc[1] = (*phase) * *phase_inc;
    two_phase_acc_reg = _mm_load_ps((float*)two_phase_acc);

    const lv_16sc_t* _in = inVector;

    lv_16sc_t* _out = outVector;

    __m128 yl, yh, tmp1, tmp2, tmp3;
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;

    for (n = 0; n < sse_iters / ROTATOR_RELOAD; n++)
        {
            for (j = 0; j < ROTATOR_RELOAD; j++)
                {
                    a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
                    // complex 32fc multiplication b=a*two_phase_acc_reg
                    yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
                    yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
                    tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
                    a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
                    tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
                    b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
                    c1 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

                    // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
                    yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
                    yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
                    tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
                    tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
                    tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
                    two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

                    // next two samples
                    _in += 2;
                    a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
                    __VOLK_GNSSSDR_PREFETCH(_in + 8);
                    // complex 32fc multiplication b=a*two_phase_acc_reg
                    yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
                    yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
                    tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
                    a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
                    tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
                    b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
                    c2 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

                    // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
                    yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
                    yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
                    tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
                    tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
                    tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
                    two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

                    // store four output samples
                    result = _mm_packs_epi32(c1, c2);  // convert from 32ic to 16ic
                    _mm_storeu_si128((__m128i*)_out, result);

                    // next two samples
                    _in += 2;
                    _out += 4;
                }
            // Regenerate phase
            tmp1 = _mm_mul_ps(two_phase_acc_reg, two_phase_acc_reg);
            tmp2 = _mm_hadd_ps(tmp1, tmp1);
            tmp1 = _mm_shuffle_ps(tmp2, tmp2, 0xD8);
            tmp2 = _mm_sqrt_ps(tmp1);
            two_phase_acc_reg = _mm_div_ps(two_phase_acc_reg, tmp2);
        }

    for (j = 0; j < sse_iters % ROTATOR_RELOAD; j++)
        {
            a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
            b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            c1 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // next two samples
            _in += 2;
            a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            __VOLK_GNSSSDR_PREFETCH(_in + 8);
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(a, yl);                 // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            a = _mm_shuffle_ps(a, a, 0xB1);           // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(a, yh);                 // tmp2 = ai*ci,ar*ci,bi*di,br*di
            b = _mm_addsub_ps(tmp1, tmp2);            // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            c2 = _mm_cvtps_epi32(b);                  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // store four output samples
            result = _mm_packs_epi32(c1, c2);  // convert from 32ic to 16ic
            _mm_storeu_si128((__m128i*)_out, result);

            // next two samples
            _in += 2;
            _out += 4;
        }

    _mm_store_ps((float*)two_phase_acc, two_phase_acc_reg);
    (*phase) = two_phase_acc[0];

    for (n = sse_iters * 4; n < num_points; ++n)
        {
            tmp16 = *_in++;
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            *_out++ = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= *phase_inc;
        }
}

#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_NEON
#include <arm_neon.h>

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_neon(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t* phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    unsigned int i = 0;
    const unsigned int neon_iters = num_points / 4;
    lv_16sc_t tmp16_;
    lv_32fc_t tmp32_;

    float arg_phase0 = cargf(*phase);
    float arg_phase_inc = cargf(*phase_inc);
    float phase_est = 0.0;

    const lv_16sc_t* _in = inVector;
    lv_16sc_t* _out = outVector;

    lv_32fc_t ___phase4 = *phase_inc * *phase_inc * *phase_inc * *phase_inc;
    __VOLK_ATTR_ALIGNED(16)
    float32_t __phase4_real[4] = {lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4)};
    __VOLK_ATTR_ALIGNED(16)
    float32_t __phase4_imag[4] = {lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4)};

    float32x4_t _phase4_real = vld1q_f32(__phase4_real);
    float32x4_t _phase4_imag = vld1q_f32(__phase4_imag);

    lv_32fc_t phase2 = (lv_32fc_t)(*phase) * *phase_inc;
    lv_32fc_t phase3 = phase2 * *phase_inc;
    lv_32fc_t phase4 = phase3 * *phase_inc;

    __VOLK_ATTR_ALIGNED(16)
    float32_t __phase_real[4] = {lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4)};
    __VOLK_ATTR_ALIGNED(16)
    float32_t __phase_imag[4] = {lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4)};

    float32x4_t _phase_real = vld1q_f32(__phase_real);
    float32x4_t _phase_imag = vld1q_f32(__phase_imag);

    float32x4_t half = vdupq_n_f32(0.5f);
    int16x4x2_t tmp16;
    int32x4x2_t tmp32i;
    float32x4x2_t tmp32f, tmp_real, tmp_imag;
    float32x4_t sign, PlusHalf, Round;

    if (neon_iters > 0)
        {
            for (; i < neon_iters; ++i)
                {
                    /* load 4 complex numbers (int 16 bits each component) */
                    tmp16 = vld2_s16((int16_t*)_in);
                    __VOLK_GNSSSDR_PREFETCH(_in + 8);
                    _in += 4;

                    /* promote them to int 32 bits */
                    tmp32i.val[0] = vmovl_s16(tmp16.val[0]);
                    tmp32i.val[1] = vmovl_s16(tmp16.val[1]);

                    /* promote them to float 32 bits */
                    tmp32f.val[0] = vcvtq_f32_s32(tmp32i.val[0]);
                    tmp32f.val[1] = vcvtq_f32_s32(tmp32i.val[1]);

                    /* complex multiplication of four complex samples (float 32 bits each component) */
                    tmp_real.val[0] = vmulq_f32(tmp32f.val[0], _phase_real);
                    tmp_real.val[1] = vmulq_f32(tmp32f.val[1], _phase_imag);
                    tmp_imag.val[0] = vmulq_f32(tmp32f.val[0], _phase_imag);
                    tmp_imag.val[1] = vmulq_f32(tmp32f.val[1], _phase_real);

                    tmp32f.val[0] = vsubq_f32(tmp_real.val[0], tmp_real.val[1]);
                    tmp32f.val[1] = vaddq_f32(tmp_imag.val[0], tmp_imag.val[1]);

                    /* downcast results to int32 */
                    /* in __aarch64__ we can do that with vcvtaq_s32_f32(ret1); vcvtaq_s32_f32(ret2); */
                    sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[0]), 31)));
                    PlusHalf = vaddq_f32(tmp32f.val[0], half);
                    Round = vsubq_f32(PlusHalf, sign);
                    tmp32i.val[0] = vcvtq_s32_f32(Round);

                    sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[1]), 31)));
                    PlusHalf = vaddq_f32(tmp32f.val[1], half);
                    Round = vsubq_f32(PlusHalf, sign);
                    tmp32i.val[1] = vcvtq_s32_f32(Round);

                    /* downcast results to int16 */
                    tmp16.val[0] = vqmovn_s32(tmp32i.val[0]);
                    tmp16.val[1] = vqmovn_s32(tmp32i.val[1]);

                    /* compute next four phases */
                    tmp_real.val[0] = vmulq_f32(_phase_real, _phase4_real);
                    tmp_real.val[1] = vmulq_f32(_phase_imag, _phase4_imag);
                    tmp_imag.val[0] = vmulq_f32(_phase_real, _phase4_imag);
                    tmp_imag.val[1] = vmulq_f32(_phase_imag, _phase4_real);

                    _phase_real = vsubq_f32(tmp_real.val[0], tmp_real.val[1]);
                    _phase_imag = vaddq_f32(tmp_imag.val[0], tmp_imag.val[1]);

                    /* store the four complex results */
                    vst2_s16((int16_t*)_out, tmp16);
                    _out += 4;
                    // Regenerate phase
                    if ((i % 512) == 0)
                        {
                            // printf("Computed phase:  %f\n", cos(cargf(lv_cmake(_phase_real[0],_phase_imag[0]))));
                            phase_est = arg_phase0 + (i + 1) * 4 * arg_phase_inc;
                            // printf("Estimated phase: %f\n\n", cos(phase_est));

                            *phase = lv_cmake(cos(phase_est), sin(phase_est));
                            phase2 = (lv_32fc_t)(*phase) * *phase_inc;
                            phase3 = phase2 * *phase_inc;
                            phase4 = phase3 * *phase_inc;

                            __VOLK_ATTR_ALIGNED(16)
                            float32_t ____phase_real[4] = {lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4)};
                            __VOLK_ATTR_ALIGNED(16)
                            float32_t ____phase_imag[4] = {lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4)};

                            _phase_real = vld1q_f32(____phase_real);
                            _phase_imag = vld1q_f32(____phase_imag);
                        }
                }
            vst1q_f32((float32_t*)__phase_real, _phase_real);
            vst1q_f32((float32_t*)__phase_imag, _phase_imag);

            (*phase) = lv_cmake((float32_t)__phase_real[0], (float32_t)__phase_imag[0]);
        }
    for (i = 0; i < neon_iters % 4; ++i)
        {
            tmp16_ = *_in++;
            tmp32_ = lv_cmake((float32_t)lv_creal(tmp16_), (float32_t)lv_cimag(tmp16_)) * (*phase);
            *_out++ = lv_cmake((int16_t)rintf(lv_creal(tmp32_)), (int16_t)rintf(lv_cimag(tmp32_)));
            (*phase) *= *phase_inc;
        }
}

#endif /* LV_HAVE_NEON */


#ifdef LV_HAVE_NEON
#include <arm_neon.h>

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_neon_reload(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t* phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    unsigned int i = 0;
    const unsigned int neon_iters = num_points / 4;
    const unsigned int ROTATOR_RELOAD = 512;
    unsigned int n;
    unsigned int j;

    lv_16sc_t tmp16_;
    lv_32fc_t tmp32_;

    float arg_phase0 = cargf(*phase);
    float arg_phase_inc = cargf(*phase_inc);
    float phase_est = 0.0;

    const lv_16sc_t* _in = inVector;
    lv_16sc_t* _out = outVector;

    lv_32fc_t ___phase4 = *phase_inc * *phase_inc * *phase_inc * *phase_inc;
    __VOLK_ATTR_ALIGNED(16)
    float32_t __phase4_real[4] = {lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4)};
    __VOLK_ATTR_ALIGNED(16)
    float32_t __phase4_imag[4] = {lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4)};

    float32x4_t _phase4_real = vld1q_f32(__phase4_real);
    float32x4_t _phase4_imag = vld1q_f32(__phase4_imag);

    lv_32fc_t phase2 = (lv_32fc_t)(*phase) * *phase_inc;
    lv_32fc_t phase3 = phase2 * *phase_inc;
    lv_32fc_t phase4 = phase3 * *phase_inc;

    __VOLK_ATTR_ALIGNED(16)
    float32_t __phase_real[4] = {lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4)};
    __VOLK_ATTR_ALIGNED(16)
    float32_t __phase_imag[4] = {lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4)};

    float32x4_t _phase_real = vld1q_f32(__phase_real);
    float32x4_t _phase_imag = vld1q_f32(__phase_imag);

    float32x4_t half = vdupq_n_f32(0.5f);
    int16x4x2_t tmp16;
    int32x4x2_t tmp32i;
    float32x4x2_t tmp32f, tmp_real, tmp_imag;
    float32x4_t sign, PlusHalf, Round;

    if (neon_iters > 0)
        {
            for (n = 0; n < neon_iters / ROTATOR_RELOAD; n++)
                {
                    for (j = 0; j < ROTATOR_RELOAD; j++)
                        {
                            /* load 4 complex numbers (int 16 bits each component) */
                            tmp16 = vld2_s16((int16_t*)_in);
                            __VOLK_GNSSSDR_PREFETCH(_in + 8);
                            _in += 4;

                            /* promote them to int 32 bits */
                            tmp32i.val[0] = vmovl_s16(tmp16.val[0]);
                            tmp32i.val[1] = vmovl_s16(tmp16.val[1]);

                            /* promote them to float 32 bits */
                            tmp32f.val[0] = vcvtq_f32_s32(tmp32i.val[0]);
                            tmp32f.val[1] = vcvtq_f32_s32(tmp32i.val[1]);

                            /* complex multiplication of four complex samples (float 32 bits each component) */
                            tmp_real.val[0] = vmulq_f32(tmp32f.val[0], _phase_real);
                            tmp_real.val[1] = vmulq_f32(tmp32f.val[1], _phase_imag);
                            tmp_imag.val[0] = vmulq_f32(tmp32f.val[0], _phase_imag);
                            tmp_imag.val[1] = vmulq_f32(tmp32f.val[1], _phase_real);

                            tmp32f.val[0] = vsubq_f32(tmp_real.val[0], tmp_real.val[1]);
                            tmp32f.val[1] = vaddq_f32(tmp_imag.val[0], tmp_imag.val[1]);

                            /* downcast results to int32 */
                            /* in __aarch64__ we can do that with vcvtaq_s32_f32(ret1); vcvtaq_s32_f32(ret2); */
                            sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[0]), 31)));
                            PlusHalf = vaddq_f32(tmp32f.val[0], half);
                            Round = vsubq_f32(PlusHalf, sign);
                            tmp32i.val[0] = vcvtq_s32_f32(Round);

                            sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[1]), 31)));
                            PlusHalf = vaddq_f32(tmp32f.val[1], half);
                            Round = vsubq_f32(PlusHalf, sign);
                            tmp32i.val[1] = vcvtq_s32_f32(Round);

                            /* downcast results to int16 */
                            tmp16.val[0] = vqmovn_s32(tmp32i.val[0]);
                            tmp16.val[1] = vqmovn_s32(tmp32i.val[1]);

                            /* compute next four phases */
                            tmp_real.val[0] = vmulq_f32(_phase_real, _phase4_real);
                            tmp_real.val[1] = vmulq_f32(_phase_imag, _phase4_imag);
                            tmp_imag.val[0] = vmulq_f32(_phase_real, _phase4_imag);
                            tmp_imag.val[1] = vmulq_f32(_phase_imag, _phase4_real);

                            _phase_real = vsubq_f32(tmp_real.val[0], tmp_real.val[1]);
                            _phase_imag = vaddq_f32(tmp_imag.val[0], tmp_imag.val[1]);

                            /* store the four complex results */
                            vst2_s16((int16_t*)_out, tmp16);
                            _out += 4;
                        }
                    // Regenerate phase
                    // printf("Computed phase:  %f\n", cos(cargf(lv_cmake(_phase_real[0],_phase_imag[0]))));
                    phase_est = arg_phase0 + (n + 1) * ROTATOR_RELOAD * 4 * arg_phase_inc;
                    // printf("Estimated phase: %f\n\n", cos(phase_est));
                    *phase = lv_cmake(cos(phase_est), sin(phase_est));
                    phase2 = (lv_32fc_t)(*phase) * *phase_inc;
                    phase3 = phase2 * *phase_inc;
                    phase4 = phase3 * *phase_inc;

                    __VOLK_ATTR_ALIGNED(16)
                    float32_t ____phase_real[4] = {lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4)};
                    __VOLK_ATTR_ALIGNED(16)
                    float32_t ____phase_imag[4] = {lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4)};

                    _phase_real = vld1q_f32(____phase_real);
                    _phase_imag = vld1q_f32(____phase_imag);
                }

            for (j = 0; j < neon_iters % ROTATOR_RELOAD; j++)
                {
                    /* load 4 complex numbers (int 16 bits each component) */
                    tmp16 = vld2_s16((int16_t*)_in);
                    __VOLK_GNSSSDR_PREFETCH(_in + 8);
                    _in += 4;

                    /* promote them to int 32 bits */
                    tmp32i.val[0] = vmovl_s16(tmp16.val[0]);
                    tmp32i.val[1] = vmovl_s16(tmp16.val[1]);

                    /* promote them to float 32 bits */
                    tmp32f.val[0] = vcvtq_f32_s32(tmp32i.val[0]);
                    tmp32f.val[1] = vcvtq_f32_s32(tmp32i.val[1]);

                    /* complex multiplication of four complex samples (float 32 bits each component) */
                    tmp_real.val[0] = vmulq_f32(tmp32f.val[0], _phase_real);
                    tmp_real.val[1] = vmulq_f32(tmp32f.val[1], _phase_imag);
                    tmp_imag.val[0] = vmulq_f32(tmp32f.val[0], _phase_imag);
                    tmp_imag.val[1] = vmulq_f32(tmp32f.val[1], _phase_real);

                    tmp32f.val[0] = vsubq_f32(tmp_real.val[0], tmp_real.val[1]);
                    tmp32f.val[1] = vaddq_f32(tmp_imag.val[0], tmp_imag.val[1]);

                    /* downcast results to int32 */
                    /* in __aarch64__ we can do that with vcvtaq_s32_f32(ret1); vcvtaq_s32_f32(ret2); */
                    sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[0]), 31)));
                    PlusHalf = vaddq_f32(tmp32f.val[0], half);
                    Round = vsubq_f32(PlusHalf, sign);
                    tmp32i.val[0] = vcvtq_s32_f32(Round);

                    sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[1]), 31)));
                    PlusHalf = vaddq_f32(tmp32f.val[1], half);
                    Round = vsubq_f32(PlusHalf, sign);
                    tmp32i.val[1] = vcvtq_s32_f32(Round);

                    /* downcast results to int16 */
                    tmp16.val[0] = vqmovn_s32(tmp32i.val[0]);
                    tmp16.val[1] = vqmovn_s32(tmp32i.val[1]);

                    /* compute next four phases */
                    tmp_real.val[0] = vmulq_f32(_phase_real, _phase4_real);
                    tmp_real.val[1] = vmulq_f32(_phase_imag, _phase4_imag);
                    tmp_imag.val[0] = vmulq_f32(_phase_real, _phase4_imag);
                    tmp_imag.val[1] = vmulq_f32(_phase_imag, _phase4_real);

                    _phase_real = vsubq_f32(tmp_real.val[0], tmp_real.val[1]);
                    _phase_imag = vaddq_f32(tmp_imag.val[0], tmp_imag.val[1]);

                    /* store the four complex results */
                    vst2_s16((int16_t*)_out, tmp16);
                    _out += 4;
                }

            vst1q_f32((float32_t*)__phase_real, _phase_real);
            vst1q_f32((float32_t*)__phase_imag, _phase_imag);

            (*phase) = lv_cmake((float32_t)__phase_real[0], (float32_t)__phase_imag[0]);
        }
    for (i = 0; i < neon_iters % 4; ++i)
        {
            tmp16_ = *_in++;
            tmp32_ = lv_cmake((float32_t)lv_creal(tmp16_), (float32_t)lv_cimag(tmp16_)) * (*phase);
            *_out++ = lv_cmake((int16_t)rintf(lv_creal(tmp32_)), (int16_t)rintf(lv_cimag(tmp32_)));
            (*phase) *= *phase_inc;
        }
}

#endif /* LV_HAVE_NEON */


#ifdef LV_HAVE_RVV
#include <riscv_vector.h>

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_rvv(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t* phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    size_t ROTATOR_RELOAD = 512;

    // Initialize reference pointers of compatible type that will not be stripmined
    float* phasePtr = (float*)phase;
    float* phaseIncPtr = (float*)phase_inc;

    // Initialize pointers of compatible type to track progress as stripmine
    short* outPtr = (short*)outVector;
    const short* inPtr = (const short*)inVector;

    for (int _ = 0; _ < num_points / ROTATOR_RELOAD; _++)
        {
            size_t n = ROTATOR_RELOAD;

            for (size_t vl; n > 0; n -= vl, outPtr += vl * 2, inPtr += vl * 2)
                {
                    // Record how many elements will actually be processed
                    vl = __riscv_vsetvl_e16m2(n);

                    // Splat phase
                    vfloat32m4_t phaseRealVal = __riscv_vfmv_v_f_f32m4(phasePtr[0], vl);
                    vfloat32m4_t phaseImagVal = __riscv_vfmv_v_f_f32m4(phasePtr[1], vl);

                    // Splat phaseInc
                    vfloat32m4_t phaseIncRealVal = __riscv_vfmv_v_f_f32m4(phaseIncPtr[0], vl);
                    vfloat32m4_t phaseIncImagVal = __riscv_vfmv_v_f_f32m4(phaseIncPtr[1], vl);

                    // iter[i] = i
                    vuint32m4_t iterVal = __riscv_vid_v_u32m4(vl);

                    // phase[i] = phase[i] * ( phaseInc[i] ^ i )
                    for (int j = 1; j < vl; j++)
                        {
                            vbool8_t maskVal = __riscv_vmsgtu_vx_u32m4_b8(iterVal, 0, vl);

                            // Initialize as copies of phase so that can target masked
                            // operations onto these copies instead of the original vectors
                            vfloat32m4_t prodRealVal = phaseRealVal;
                            vfloat32m4_t prodImagVal = phaseImagVal;

                            // For more details on cross product,
                            // check `volk_gnsssdr_8ic_x2_multiply_8ic_rvv`,
                            // for instance
                            prodRealVal = __riscv_vfmul_vv_f32m4_mu(maskVal, prodRealVal, phaseRealVal, phaseIncRealVal, vl);
                            prodRealVal = __riscv_vfnmsac_vv_f32m4_mu(maskVal, prodRealVal, phaseImagVal, phaseIncImagVal, vl);
                            prodImagVal = __riscv_vfmul_vv_f32m4_mu(maskVal, prodImagVal, phaseRealVal, phaseIncImagVal, vl);
                            prodImagVal = __riscv_vfmacc_vv_f32m4_mu(maskVal, prodImagVal, phaseImagVal, phaseIncRealVal, vl);

                            phaseRealVal = prodRealVal;
                            phaseImagVal = prodImagVal;

                            iterVal = __riscv_vsub_vx_u32m4_mu(maskVal, iterVal, iterVal, 1, vl);
                        }

                    // Load inReal[0..vl), inImag[0..vl)
                    vint16m2x2_t inVal = __riscv_vlseg2e16_v_i16m2x2(inPtr, vl);
                    vint16m2_t inRealVal = __riscv_vget_v_i16m2x2_i16m2(inVal, 0);
                    vint16m2_t inImagVal = __riscv_vget_v_i16m2x2_i16m2(inVal, 1);

                    // w(ide)Out[i] = ( (float) in[i] ) * phase[i]
                    vfloat32m4_t fInRealVal = __riscv_vfwcvt_f_x_v_f32m4(inRealVal, vl);
                    vfloat32m4_t fInImagVal = __riscv_vfwcvt_f_x_v_f32m4(inImagVal, vl);

                    vfloat32m4_t wOutRealVal = __riscv_vfmul_vv_f32m4(fInRealVal, phaseRealVal, vl);
                    wOutRealVal = __riscv_vfnmsac_vv_f32m4(wOutRealVal, fInImagVal, phaseImagVal, vl);
                    vfloat32m4_t wOutImagVal = __riscv_vfmul_vv_f32m4(fInRealVal, phaseImagVal, vl);
                    wOutImagVal = __riscv_vfmacc_vv_f32m4(wOutImagVal, fInImagVal, phaseRealVal, vl);

                    // out[i] = (int16_t) wOut[i]
                    vint16m2_t outRealVal = __riscv_vfncvt_x_f_w_i16m2(wOutRealVal, vl);
                    vint16m2_t outImagVal = __riscv_vfncvt_x_f_w_i16m2(wOutImagVal, vl);

                    // Store out[0..vl)
                    vint16m2x2_t outVal = __riscv_vcreate_v_i16m2x2(outRealVal, outImagVal);
                    __riscv_vsseg2e16_v_i16m2x2(outPtr, outVal, vl);

                    // Store phase[vl - 1]
                    phaseRealVal = __riscv_vslidedown_vx_f32m4(phaseRealVal, vl - 1, vl);
                    phasePtr[0] = __riscv_vfmv_f_s_f32m4_f32(phaseRealVal);
                    phaseImagVal = __riscv_vslidedown_vx_f32m4(phaseImagVal, vl - 1, vl);
                    phasePtr[1] = __riscv_vfmv_f_s_f32m4_f32(phaseImagVal);

                    // Account for multiplication after last calculation
                    (*phase) *= *phase_inc;

                    // In looping, decrement the number of
                    // elements left and increment the pointers
                    // by the number of elements processed,
                    // taking into account how the `vl` complex
                    // numbers are each stored as two 16-bit numbers
                }

                // Regenerate phase
#ifdef __cplusplus
            (*phase) /= std::abs((*phase));
#else
            (*phase) /= hypotf(lv_creal(*phase), lv_cimag(*phase));
#endif
        }

    size_t n = num_points % ROTATOR_RELOAD;

    for (size_t vl; n > 0; n -= vl, outPtr += vl * 2, inPtr += vl * 2)
        {
            // Record how many elements will actually be processed
            vl = __riscv_vsetvl_e16m2(n);

            // Splat phase
            vfloat32m4_t phaseRealVal = __riscv_vfmv_v_f_f32m4(phasePtr[0], vl);
            vfloat32m4_t phaseImagVal = __riscv_vfmv_v_f_f32m4(phasePtr[1], vl);

            // Splat phaseInc
            vfloat32m4_t phaseIncRealVal = __riscv_vfmv_v_f_f32m4(phaseIncPtr[0], vl);
            vfloat32m4_t phaseIncImagVal = __riscv_vfmv_v_f_f32m4(phaseIncPtr[1], vl);

            // iter[i] = i
            vuint32m4_t iterVal = __riscv_vid_v_u32m4(vl);

            // phase[i] = phase[i] * ( phaseInc[i] ^ i )
            for (int j = 1; j < vl; j++)
                {
                    vbool8_t maskVal = __riscv_vmsgtu_vx_u32m4_b8(iterVal, 0, vl);

                    // Initialize as copies of phase so that can target masked
                    // operations onto these copies instead of the original vectors
                    vfloat32m4_t prodRealVal = phaseRealVal;
                    vfloat32m4_t prodImagVal = phaseImagVal;

                    // For more details on cross product,
                    // check `volk_gnsssdr_8ic_x2_multiply_8ic_rvv`,
                    // for instance
                    prodRealVal = __riscv_vfmul_vv_f32m4_mu(maskVal, prodRealVal, phaseRealVal, phaseIncRealVal, vl);
                    prodRealVal = __riscv_vfnmsac_vv_f32m4_mu(maskVal, prodRealVal, phaseImagVal, phaseIncImagVal, vl);
                    prodImagVal = __riscv_vfmul_vv_f32m4_mu(maskVal, prodImagVal, phaseRealVal, phaseIncImagVal, vl);
                    prodImagVal = __riscv_vfmacc_vv_f32m4_mu(maskVal, prodImagVal, phaseImagVal, phaseIncRealVal, vl);

                    phaseRealVal = prodRealVal;
                    phaseImagVal = prodImagVal;

                    iterVal = __riscv_vsub_vx_u32m4_mu(maskVal, iterVal, iterVal, 1, vl);
                }

            // Load inReal[0..vl), inImag[0..vl)
            vint16m2x2_t inVal = __riscv_vlseg2e16_v_i16m2x2(inPtr, vl);
            vint16m2_t inRealVal = __riscv_vget_v_i16m2x2_i16m2(inVal, 0);
            vint16m2_t inImagVal = __riscv_vget_v_i16m2x2_i16m2(inVal, 1);

            // w(ide)Out[i] = ( (float) in[i] ) * phase[i]
            vfloat32m4_t fInRealVal = __riscv_vfwcvt_f_x_v_f32m4(inRealVal, vl);
            vfloat32m4_t fInImagVal = __riscv_vfwcvt_f_x_v_f32m4(inImagVal, vl);

            vfloat32m4_t wOutRealVal = __riscv_vfmul_vv_f32m4(fInRealVal, phaseRealVal, vl);
            wOutRealVal = __riscv_vfnmsac_vv_f32m4(wOutRealVal, fInImagVal, phaseImagVal, vl);
            vfloat32m4_t wOutImagVal = __riscv_vfmul_vv_f32m4(fInRealVal, phaseImagVal, vl);
            wOutImagVal = __riscv_vfmacc_vv_f32m4(wOutImagVal, fInImagVal, phaseRealVal, vl);

            // out[i] = (int16_t) wOut[i]
            vint16m2_t outRealVal = __riscv_vfncvt_x_f_w_i16m2(wOutRealVal, vl);
            vint16m2_t outImagVal = __riscv_vfncvt_x_f_w_i16m2(wOutImagVal, vl);

            // Store out[0..vl)
            vint16m2x2_t outVal = __riscv_vcreate_v_i16m2x2(outRealVal, outImagVal);
            __riscv_vsseg2e16_v_i16m2x2(outPtr, outVal, vl);

            // Store phase[vl - 1]
            phaseRealVal = __riscv_vslidedown_vx_f32m4(phaseRealVal, vl - 1, vl);
            phasePtr[0] = __riscv_vfmv_f_s_f32m4_f32(phaseRealVal);
            phaseImagVal = __riscv_vslidedown_vx_f32m4(phaseImagVal, vl - 1, vl);
            phasePtr[1] = __riscv_vfmv_f_s_f32m4_f32(phaseImagVal);

            // Account for multiplication after last calculation
            (*phase) *= *phase_inc;

            // In looping, decrement the number of
            // elements left and increment the pointers
            // by the number of elements processed,
            // taking into account how the `vl` complex
            // numbers are each stored as two 16-bit numbers
        }
}

#endif /* LV_HAVE_RVV */

#endif /* INCLUDED_volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_H */
