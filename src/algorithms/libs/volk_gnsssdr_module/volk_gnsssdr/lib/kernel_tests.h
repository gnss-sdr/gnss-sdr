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
#include <boost/assign/list_of.hpp>
#include <volk_gnsssdr/volk_gnsssdr.h>


// macros for initializing volk_gnsssdr_test_case_t. Macros are needed to generate
// function names of the pattern kernel_name_*

// for puppets we need to get all the func_variants for the puppet and just
// keep track of the actual function name to write to results
#define VOLK_INIT_PUPP(func, puppet_master_func, test_params)\
    volk_gnsssdr_test_case_t(func##_get_func_desc(), (void(*)())func##_manual, std::string(#func),\
    std::string(#puppet_master_func), test_params)

#define VOLK_INIT_TEST(func, test_params)\
    volk_gnsssdr_test_case_t(func##_get_func_desc(), (void(*)())func##_manual, std::string(#func),\
    test_params)

std::vector<volk_gnsssdr_test_case_t> init_test_list(volk_gnsssdr_test_params_t test_params)
{

    // Some kernels need a lower tolerance
    volk_gnsssdr_test_params_t test_params_inacc = volk_gnsssdr_test_params_t(1e-3, test_params.scalar(),
            test_params.vlen(), test_params.iter(), test_params.benchmark_mode(), test_params.kernel_regex());
    volk_gnsssdr_test_params_t test_params_int1 = volk_gnsssdr_test_params_t(1, test_params.scalar(),
            test_params.vlen(), test_params.iter(), test_params.benchmark_mode(), test_params.kernel_regex());

    std::vector<volk_gnsssdr_test_case_t> test_cases = boost::assign::list_of
        // no one uses these, so don't test them
        //VOLK_PROFILE(volk_gnsssdr_16i_x5_add_quad_16i_x4, 1e-4, 2046, 10000, &results, benchmark_mode, kernel_regex);
        //VOLK_PROFILE(volk_gnsssdr_16i_branch_4_state_8, 1e-4, 2046, 10000, &results, benchmark_mode, kernel_regex);
        //VOLK_PROFILE(volk_gnsssdr_16i_max_star_16i, 0, 0, 204602, 10000, &results, benchmark_mode, kernel_regex);
        //VOLK_PROFILE(volk_gnsssdr_16i_max_star_horizontal_16i, 0, 0, 204602, 10000, &results, benchmark_mode, kernel_regex);
        //VOLK_PROFILE(volk_gnsssdr_16i_permute_and_scalar_add, 1e-4, 0, 2046, 10000, &results, benchmark_mode, kernel_regex);
        //VOLK_PROFILE(volk_gnsssdr_16i_x4_quad_max_star_16i, 1e-4, 0, 2046, 10000, &results, benchmark_mode, kernel_regex);
        // we need a puppet for this one
        //(VOLK_INIT_TEST(volk_gnsssdr_32fc_s32f_x2_power_spectral_density_32f,   test_params))
        //(VOLK_INIT_TEST(volk_gnsssdr_32f_null_32f, test_params))
        (VOLK_INIT_TEST(volk_gnsssdr_8i_accumulator_s8i, test_params))
        (VOLK_INIT_TEST(volk_gnsssdr_8i_index_max_16u, test_params))
        (VOLK_INIT_TEST(volk_gnsssdr_8i_max_s8i, test_params))
        (VOLK_INIT_TEST(volk_gnsssdr_8i_x2_add_8i, test_params))
        (VOLK_INIT_TEST(volk_gnsssdr_8ic_conjugate_8ic, test_params))
        (VOLK_INIT_TEST(volk_gnsssdr_8ic_magnitude_squared_8i, test_params))
        (VOLK_INIT_TEST(volk_gnsssdr_8ic_x2_dot_prod_8ic, test_params))
        (VOLK_INIT_TEST(volk_gnsssdr_8ic_x2_multiply_8ic, test_params))
        (VOLK_INIT_TEST(volk_gnsssdr_8u_x2_multiply_8u, test_params))
        (VOLK_INIT_TEST(volk_gnsssdr_64f_accumulator_64f, test_params))
        (VOLK_INIT_TEST(volk_gnsssdr_32fc_convert_8ic, test_params))
        ;

    return test_cases;
}
