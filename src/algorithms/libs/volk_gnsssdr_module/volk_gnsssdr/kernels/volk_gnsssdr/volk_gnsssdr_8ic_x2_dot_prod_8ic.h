/*!
 * \file volk_gnsssdr_8ic_x2_dot_prod_8ic.h
 * \brief Volk protokernel: multiplies two 16 bits vectors and accumulates them
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that multiplies two 16 bits vectors (8 bits the real part 
 * and 8 bits the imaginary part) and accumulates them
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef INCLUDED_volk_gnsssdr_8ic_x2_dot_prod_8ic_u_H
#define INCLUDED_volk_gnsssdr_8ic_x2_dot_prod_8ic_u_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <stdio.h>
#include <string.h>

#ifdef LV_HAVE_GENERIC
/*!
 \brief Multiplies the two input complex vectors and accumulates them, storing the result in the third vector
 \param cVector The vector where the accumulated result will be stored
 \param aVector One of the vectors to be multiplied and accumulated
 \param bVector One of the vectors to be multiplied and accumulated
 \param num_points The number of complex values in aVector and bVector to be multiplied together, accumulated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_generic(lv_8sc_t* result, const lv_8sc_t* input, const lv_8sc_t* taps, unsigned int num_points)
{
    /*lv_8sc_t* cPtr = result;
     const lv_8sc_t* aPtr = input;
     const lv_8sc_t* bPtr = taps;

     for(int number = 0; number < num_points; number++){
     *cPtr += (*aPtr++) * (*bPtr++);
     }*/

    char * res = (char*) result;
    char * in = (char*) input;
    char * tp = (char*) taps;
    unsigned int n_2_ccomplex_blocks = num_points/2;
    unsigned int isodd = num_points & 1;

    char sum0[2] = {0,0};
    char sum1[2] = {0,0};
    unsigned int i = 0;

    for(i = 0; i < n_2_ccomplex_blocks; ++i)
        {
            sum0[0] += in[0] * tp[0] - in[1] * tp[1];
            sum0[1] += in[0] * tp[1] + in[1] * tp[0];
            sum1[0] += in[2] * tp[2] - in[3] * tp[3];
            sum1[1] += in[2] * tp[3] + in[3] * tp[2];

            in += 4;
            tp += 4;
        }

    res[0] = sum0[0] + sum1[0];
    res[1] = sum0[1] + sum1[1];

    // Cleanup if we had an odd number of points
    for(i = 0; i < isodd; ++i)
        {
            *result += input[num_points - 1] * taps[num_points - 1];
        }
}

#endif /*LV_HAVE_GENERIC*/

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/*!
 \brief Multiplies the two input complex vectors and accumulates them, storing the result in the third vector
 \param cVector The vector where the accumulated result will be stored
 \param aVector One of the vectors to be multiplied and accumulated
 \param bVector One of the vectors to be multiplied and accumulated
 \param num_points The number of complex values in aVector and bVector to be multiplied together, accumulated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_u_sse2(lv_8sc_t* result, const lv_8sc_t* input, const lv_8sc_t* taps, unsigned int num_points)
{
    lv_8sc_t dotProduct;
    memset(&dotProduct, 0x0, 2*sizeof(char));

    const lv_8sc_t* a = input;
    const lv_8sc_t* b = taps;

    const unsigned int sse_iters = num_points/8;

    if (sse_iters>0)
        {
            __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc, realcacc, imagcacc;

            mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);
            realcacc = _mm_setzero_si128();
            imagcacc = _mm_setzero_si128();

            for(unsigned int number = 0; number < sse_iters; number++)
                {
                    x = _mm_loadu_si128((__m128i*)a);
                    y = _mm_loadu_si128((__m128i*)b);

                    imagx = _mm_srli_si128 (x, 1);
                    imagx = _mm_and_si128 (imagx, mult1);
                    realx = _mm_and_si128 (x, mult1);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (realx, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imagx, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (realx, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imagx, realy);

                    realc = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imagc = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    realcacc = _mm_add_epi16 (realcacc, realc);
                    imagcacc = _mm_add_epi16 (imagcacc, imagc);

                    a += 8;
                    b += 8;
                }

            realcacc = _mm_and_si128 (realcacc, mult1);
            imagcacc = _mm_and_si128 (imagcacc, mult1);
            imagcacc = _mm_slli_si128 (imagcacc, 1);

            totalc = _mm_or_si128 (realcacc, imagcacc);

            __VOLK_ATTR_ALIGNED(16) lv_8sc_t dotProductVector[8];

            _mm_storeu_si128((__m128i*)dotProductVector,totalc); // Store the results back into the dot product vector

            for (int i = 0; i<8; ++i)
                {
                    dotProduct += dotProductVector[i];
                }
        }

    for (unsigned int i = 0; i<(num_points % 8); ++i)
        {
            dotProduct += (*a++) * (*b++);
        }

    *result = dotProduct;
}

#endif /*LV_HAVE_SSE2*/

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
/*!
 \brief Multiplies the two input complex vectors and accumulates them, storing the result in the third vector
 \param cVector The vector where the accumulated result will be stored
 \param aVector One of the vectors to be multiplied and accumulated
 \param bVector One of the vectors to be multiplied and accumulated
 \param num_points The number of complex values in aVector and bVector to be multiplied together, accumulated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_u_sse4_1(lv_8sc_t* result, const lv_8sc_t* input, const lv_8sc_t* taps, unsigned int num_points)
{
    lv_8sc_t dotProduct;
    memset(&dotProduct, 0x0, 2*sizeof(char));

    const lv_8sc_t* a = input;
    const lv_8sc_t* b = taps;

    const unsigned int sse_iters = num_points/8;

    if (sse_iters>0)
        {
            __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc, realcacc, imagcacc;

            mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);
            realcacc = _mm_setzero_si128();
            imagcacc = _mm_setzero_si128();

            for(unsigned int number = 0; number < sse_iters; number++)
                {
                    x = _mm_lddqu_si128((__m128i*)a);
                    y = _mm_lddqu_si128((__m128i*)b);

                    imagx = _mm_srli_si128 (x, 1);
                    imagx = _mm_and_si128 (imagx, mult1);
                    realx = _mm_and_si128 (x, mult1);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (realx, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imagx, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (realx, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imagx, realy);

                    realc = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imagc = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    realcacc = _mm_add_epi16 (realcacc, realc);
                    imagcacc = _mm_add_epi16 (imagcacc, imagc);

                    a += 8;
                    b += 8;
                }

            imagcacc = _mm_slli_si128 (imagcacc, 1);

            totalc = _mm_blendv_epi8 (imagcacc, realcacc, mult1);

            __VOLK_ATTR_ALIGNED(16) lv_8sc_t dotProductVector[8];

            _mm_storeu_si128((__m128i*)dotProductVector,totalc); // Store the results back into the dot product vector

            for (unsigned int i = 0; i<8; ++i)
                {
                    dotProduct += dotProductVector[i];
                }
        }

    for (unsigned int i = 0; i<(num_points % 8); ++i)
        {
            dotProduct += (*a++) * (*b++);
        }

    *result = dotProduct;
}

#endif /*LV_HAVE_SSE4_1*/

#endif /*INCLUDED_volk_gnsssdr_8ic_x2_dot_prod_8ic_u_H*/


#ifndef INCLUDED_volk_gnsssdr_8ic_x2_dot_prod_8ic_a_H
#define INCLUDED_volk_gnsssdr_8ic_x2_dot_prod_8ic_a_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <stdio.h>
#include <string.h>


#ifdef LV_HAVE_GENERIC
/*!
 \brief Multiplies the two input complex vectors and accumulates them, storing the result in the third vector
 \param cVector The vector where the accumulated result will be stored
 \param aVector One of the vectors to be multiplied and accumulated
 \param bVector One of the vectors to be multiplied and accumulated
 \param num_points The number of complex values in aVector and bVector to be multiplied together, accumulated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_a_generic(lv_8sc_t* result, const lv_8sc_t* input, const lv_8sc_t* taps, unsigned int num_points)
{
    /*lv_8sc_t* cPtr = result;
     const lv_8sc_t* aPtr = input;
     const lv_8sc_t* bPtr = taps;

     for(int number = 0; number < num_points; number++){
     *cPtr += (*aPtr++) * (*bPtr++);
     }*/

    char * res = (char*) result;
    char * in = (char*) input;
    char * tp = (char*) taps;
    unsigned int n_2_ccomplex_blocks = num_points/2;
    unsigned int isodd = num_points & 1;

    char sum0[2] = {0,0};
    char sum1[2] = {0,0};
    unsigned int i = 0;

    for(i = 0; i < n_2_ccomplex_blocks; ++i)
        {
            sum0[0] += in[0] * tp[0] - in[1] * tp[1];
            sum0[1] += in[0] * tp[1] + in[1] * tp[0];
            sum1[0] += in[2] * tp[2] - in[3] * tp[3];
            sum1[1] += in[2] * tp[3] + in[3] * tp[2];

            in += 4;
            tp += 4;
        }

    res[0] = sum0[0] + sum1[0];
    res[1] = sum0[1] + sum1[1];

    // Cleanup if we had an odd number of points
    for(i = 0; i < isodd; ++i)
        {
            *result += input[num_points - 1] * taps[num_points - 1];
        }
}

#endif /*LV_HAVE_GENERIC*/

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/*!
 \brief Multiplies the two input complex vectors and accumulates them, storing the result in the third vector
 \param cVector The vector where the accumulated result will be stored
 \param aVector One of the vectors to be multiplied and accumulated
 \param bVector One of the vectors to be multiplied and accumulated
 \param num_points The number of complex values in aVector and bVector to be multiplied together, accumulated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_a_sse2(lv_8sc_t* result, const lv_8sc_t* input, const lv_8sc_t* taps, unsigned int num_points)
{
    lv_8sc_t dotProduct;
    memset(&dotProduct, 0x0, 2*sizeof(char));

    const lv_8sc_t* a = input;
    const lv_8sc_t* b = taps;

    const unsigned int sse_iters = num_points/8;

    if (sse_iters>0)
        {
            __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc, realcacc, imagcacc;

            mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);
            realcacc = _mm_setzero_si128();
            imagcacc = _mm_setzero_si128();

            for(unsigned int number = 0; number < sse_iters; number++)
                {
                    x = _mm_load_si128((__m128i*)a);
                    y = _mm_load_si128((__m128i*)b);

                    imagx = _mm_srli_si128 (x, 1);
                    imagx = _mm_and_si128 (imagx, mult1);
                    realx = _mm_and_si128 (x, mult1);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (realx, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imagx, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (realx, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imagx, realy);

                    realc = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imagc = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    realcacc = _mm_add_epi16 (realcacc, realc);
                    imagcacc = _mm_add_epi16 (imagcacc, imagc);

                    a += 8;
                    b += 8;
                }

            realcacc = _mm_and_si128 (realcacc, mult1);
            imagcacc = _mm_and_si128 (imagcacc, mult1);
            imagcacc = _mm_slli_si128 (imagcacc, 1);

            totalc = _mm_or_si128 (realcacc, imagcacc);

            __VOLK_ATTR_ALIGNED(16) lv_8sc_t dotProductVector[8];

            _mm_store_si128((__m128i*)dotProductVector,totalc); // Store the results back into the dot product vector

            for (unsigned int i = 0; i<8; ++i)
                {
                    dotProduct += dotProductVector[i];
                }
        }

    for (unsigned int i = 0; i<(num_points % 8); ++i)
        {
            dotProduct += (*a++) * (*b++);
        }

    *result = dotProduct;
}

#endif /*LV_HAVE_SSE2*/

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
/*!
 \brief Multiplies the two input complex vectors and accumulates them, storing the result in the third vector
 \param cVector The vector where the accumulated result will be stored
 \param aVector One of the vectors to be multiplied and accumulated
 \param bVector One of the vectors to be multiplied and accumulated
 \param num_points The number of complex values in aVector and bVector to be multiplied together, accumulated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_a_sse4_1(lv_8sc_t* result, const lv_8sc_t* input, const lv_8sc_t* taps, unsigned int num_points)
{
    lv_8sc_t dotProduct;
    memset(&dotProduct, 0x0, 2*sizeof(char));

    const lv_8sc_t* a = input;
    const lv_8sc_t* b = taps;

    const unsigned int sse_iters = num_points/8;

    if (sse_iters>0)
        {
            __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc, realcacc, imagcacc;

            mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);
            realcacc = _mm_setzero_si128();
            imagcacc = _mm_setzero_si128();

            for(unsigned int number = 0; number < sse_iters; number++)
                {
                    x = _mm_load_si128((__m128i*)a);
                    y = _mm_load_si128((__m128i*)b);

                    imagx = _mm_srli_si128 (x, 1);
                    imagx = _mm_and_si128 (imagx, mult1);
                    realx = _mm_and_si128 (x, mult1);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (realx, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imagx, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (realx, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imagx, realy);

                    realc = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imagc = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    realcacc = _mm_add_epi16 (realcacc, realc);
                    imagcacc = _mm_add_epi16 (imagcacc, imagc);

                    a += 8;
                    b += 8;
                }

            imagcacc = _mm_slli_si128 (imagcacc, 1);

            totalc = _mm_blendv_epi8 (imagcacc, realcacc, mult1);

            __VOLK_ATTR_ALIGNED(16) lv_8sc_t dotProductVector[8];

            _mm_store_si128((__m128i*)dotProductVector,totalc); // Store the results back into the dot product vector

            for (unsigned int i = 0; i<8; ++i)
                {
                    dotProduct += dotProductVector[i];
                }
        }

    for (unsigned int i = 0; i<(num_points % 8); ++i)
        {
            dotProduct += (*a++) * (*b++);
        }

    *result = dotProduct;
}

#endif /*LV_HAVE_SSE4_1*/

#ifdef LV_HAVE_ORC
/*!
 \brief Multiplies the two input complex vectors and accumulates them, storing the result in the third vector
 \param cVector The vector where the accumulated result will be stored
 \param aVector One of the vectors to be multiplied and accumulated
 \param bVector One of the vectors to be multiplied and accumulated
 \param num_points The number of complex values in aVector and bVector to be multiplied together, accumulated and stored into cVector
 */
extern void volk_gnsssdr_8ic_x2_dot_prod_8ic_a_orc_impl(short* resRealShort, short* resImagShort, const lv_8sc_t* input, const lv_8sc_t* taps, unsigned int num_points);
static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_u_orc(lv_8sc_t* result, const lv_8sc_t* input, const lv_8sc_t* taps, unsigned int num_points)
{
    short resReal = 0;
    char* resRealChar = (char*)&resReal;
    resRealChar++;

    short resImag = 0;
    char* resImagChar = (char*)&resImag;
    resImagChar++;

    volk_gnsssdr_8ic_x2_dot_prod_8ic_a_orc_impl(&resReal, &resImag, input, taps, num_points);

    *result = lv_cmake(*resRealChar, *resImagChar);
}
#endif /* LV_HAVE_ORC */

#endif /*INCLUDED_volk_gnsssdr_8ic_x2_dot_prod_8ic_a_H*/
