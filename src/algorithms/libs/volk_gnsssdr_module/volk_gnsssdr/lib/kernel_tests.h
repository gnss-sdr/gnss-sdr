/*!
 * \file kernel_tests.h
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
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


#include "qa_utils.h"
#include <vector>
#include <volk_gnsssdr/volk_gnsssdr.h>


// macros for initializing volk_gnsssdr_test_case_t. Macros are needed to generate
// function names of the pattern kernel_name_*

// for puppets we need to get all the func_variants for the puppet and just
// keep track of the actual function name to write to results
#define VOLK_INIT_PUPP(func, puppet_master_func, test_params)                                       \
    volk_gnsssdr_test_case_t(func##_get_func_desc(), (void (*)())func##_manual, std::string(#func), \
        std::string(#puppet_master_func), test_params)

#define VOLK_INIT_TEST(func, test_params)                                                           \
    volk_gnsssdr_test_case_t(func##_get_func_desc(), (void (*)())func##_manual, std::string(#func), \
        test_params)

#define QA(test) test_cases.push_back(test);

std::vector<volk_gnsssdr_test_case_t> init_test_list(volk_gnsssdr_test_params_t test_params)
{
    // Some kernels need a lower tolerance
    volk_gnsssdr_test_params_t test_params_inacc = volk_gnsssdr_test_params_t(1e-3, test_params.scalar(),
        test_params.vlen(), test_params.iter(), test_params.benchmark_mode(), test_params.kernel_regex());
    volk_gnsssdr_test_params_t test_params_int1 = volk_gnsssdr_test_params_t(1, test_params.scalar(),
        test_params.vlen(), test_params.iter(), test_params.benchmark_mode(), test_params.kernel_regex());
    // some others need more iterations *****  ADDED BY GNSS-SDR
    volk_gnsssdr_test_params_t test_params_more_iters = volk_gnsssdr_test_params_t(test_params.tol(), test_params.scalar(),
        test_params.vlen(), 100000, test_params.benchmark_mode(), test_params.kernel_regex());
    // ... or more tolerance *****  ADDED BY GNSS-SDR
    volk_gnsssdr_test_params_t test_params_int16 = volk_gnsssdr_test_params_t(16, test_params.scalar(),
        test_params.vlen(), test_params.iter(), test_params.benchmark_mode(), test_params.kernel_regex());
    volk_gnsssdr_test_params_t test_params_inacc2 = volk_gnsssdr_test_params_t(2e-1, test_params.scalar(),
        test_params.vlen(), test_params.iter(), test_params.benchmark_mode(), test_params.kernel_regex());

    std::vector<volk_gnsssdr_test_case_t> test_cases;

    QA(VOLK_INIT_TEST(volk_gnsssdr_8i_accumulator_s8i, test_params_more_iters))
    QA(VOLK_INIT_TEST(volk_gnsssdr_8i_index_max_16u, test_params_more_iters))
    QA(VOLK_INIT_TEST(volk_gnsssdr_8i_max_s8i, test_params_more_iters))
    QA(VOLK_INIT_TEST(volk_gnsssdr_8i_x2_add_8i, test_params_more_iters))
    QA(VOLK_INIT_TEST(volk_gnsssdr_8ic_conjugate_8ic, test_params_more_iters))
    QA(VOLK_INIT_TEST(volk_gnsssdr_8ic_magnitude_squared_8i, test_params_more_iters))
    QA(VOLK_INIT_TEST(volk_gnsssdr_8ic_x2_dot_prod_8ic, test_params))
    QA(VOLK_INIT_TEST(volk_gnsssdr_8ic_x2_multiply_8ic, test_params))
    QA(VOLK_INIT_TEST(volk_gnsssdr_8ic_s8ic_multiply_8ic, test_params))
    QA(VOLK_INIT_TEST(volk_gnsssdr_8u_x2_multiply_8u, test_params_more_iters))
    QA(VOLK_INIT_TEST(volk_gnsssdr_64f_accumulator_64f, test_params))
    QA(VOLK_INIT_TEST(volk_gnsssdr_32f_sincos_32fc, test_params_inacc))
    QA(VOLK_INIT_TEST(volk_gnsssdr_32f_index_max_32u, test_params))
    QA(VOLK_INIT_TEST(volk_gnsssdr_32fc_convert_8ic, test_params))
    QA(VOLK_INIT_TEST(volk_gnsssdr_32fc_convert_16ic, test_params_more_iters))
    QA(VOLK_INIT_TEST(volk_gnsssdr_16ic_x2_dot_prod_16ic, test_params))
    QA(VOLK_INIT_TEST(volk_gnsssdr_16ic_x2_multiply_16ic, test_params_more_iters))
    QA(VOLK_INIT_TEST(volk_gnsssdr_16ic_convert_32fc, test_params_more_iters))
    QA(VOLK_INIT_TEST(volk_gnsssdr_16ic_conjugate_16ic, test_params_more_iters))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_s32f_sincospuppet_32fc, volk_gnsssdr_s32f_sincos_32fc, test_params_inacc2))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_16ic_rotatorpuppet_16ic, volk_gnsssdr_16ic_s32fc_x2_rotator_16ic, test_params_int1))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_16ic_resamplerfastpuppet_16ic, volk_gnsssdr_16ic_resampler_fast_16ic, test_params))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_16ic_resamplerfastxnpuppet_16ic, volk_gnsssdr_16ic_xn_resampler_fast_16ic_xn, test_params))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_16ic_resamplerxnpuppet_16ic, volk_gnsssdr_16ic_xn_resampler_16ic_xn, test_params))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_16i_resamplerxnpuppet_16i, volk_gnsssdr_16i_xn_resampler_16i_xn, test_params))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_32fc_resamplerxnpuppet_32fc, volk_gnsssdr_32fc_xn_resampler_32fc_xn, test_params))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_32f_resamplerxnpuppet_32f, volk_gnsssdr_32f_xn_resampler_32f_xn, test_params))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic, volk_gnsssdr_16ic_x2_dot_prod_16ic_xn, test_params))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic, volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn, test_params_int16))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_16ic_16i_rotator_dotprodxnpuppet_16ic, volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn, test_params_int16))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc, volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn, test_params_int1))
    QA(VOLK_INIT_PUPP(volk_gnsssdr_32fc_32f_rotator_dotprodxnpuppet_32fc, volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn, test_params_int1));

    return test_cases;
}
