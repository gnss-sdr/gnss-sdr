/*!
 * \file CommonMacros_16ic_cw_corr_32fc.h
 * \brief Common macros used inside the 16ic_cw_corr_32fc volk protokernels.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
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
#ifndef INCLUDED_gnsssdr_CommonMacros_16ic_cw_corr_32fc_u_H
#define INCLUDED_gnsssdr_CommonMacros_16ic_cw_corr_32fc_u_H
#include "CommonMacros/CommonMacros.h"

    #ifdef LV_HAVE_SSE4_1
      /*!
        \brief Macros for U_SSE4_1
      */

        #ifndef CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1
        #define CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1(y1, y2, realy, imagy, real_bb_signal_sample, imag_bb_signal_sample,realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, input_i_1, input_i_2, output_i32, real_output_ps, imag_output_ps)\
        CM_16IC_X2_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE4_1(y1, y2, realy, imagy)\
        CM_16IC_X4_SCALAR_PRODUCT_16IC_X2_U_SSE2(real_bb_signal_sample, imag_bb_signal_sample, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output)\
        CM_16IC_CONVERT_AND_ACC_32FC_U_SSE4_1(real_output, input_i_1, input_i_2, output_i32, real_output_ps)\
        CM_16IC_CONVERT_AND_ACC_32FC_U_SSE4_1(imag_output, input_i_1, input_i_2, output_i32, imag_output_ps)
        #endif /* CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1 */

    #endif /* LV_HAVE_SSE4_1 */

    #ifdef LV_HAVE_GENERIC
    /*!
     \brief Macros for U_GENERIC
     */

    #endif /* LV_HAVE_GENERIC */
#endif /* INCLUDED_gnsssdr_CommonMacros_16ic_cw_corr_32fc_u_H */


#ifndef INCLUDED_gnsssdr_CommonMacros_16ic_cw_corr_32fc_a_H
#define INCLUDED_gnsssdr_CommonMacros_16ic_cw_corr_32fc_a_H

    #ifdef LV_HAVE_SSE4_1
    /*!
     \brief Macros for A_SSE4_1
     */

    #endif /* LV_HAVE_SSE4_1 */

    #ifdef LV_HAVE_GENERIC
    /*!
     \brief Macros for A_GENERIC
     */

    #endif /* LV_HAVE_GENERIC */
#endif /* INCLUDED_gnsssdr_CommonMacros_16ic_cw_corr_32fc_a_H */
