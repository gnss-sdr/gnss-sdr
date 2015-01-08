/*
 * Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
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
 */


#include "qa_utils.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <boost/test/unit_test.hpp>

//GNSS-SDR PROTO-KERNELS
VOLK_RUN_TESTS(volk_gnsssdr_8ic_x2_multiply_8ic, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8u_x2_multiply_8u, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8ic_x2_dot_prod_8ic, 1e-4, 0, 204603, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8ic_s8ic_multiply_8ic, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8ic_conjugate_8ic, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8i_x2_add_8i, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8i_index_max_16u, 3, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8i_accumulator_s8i, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8ic_magnitude_squared_8i, 1e-4, 0, 20462, 1);

VOLK_RUN_TESTS(volk_gnsssdr_8i_max_s8i, 3, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_64f_accumulator_64f, 3, 0, 20462, 1);

VOLK_RUN_TESTS(volk_gnsssdr_32fc_convert_16ic, 3, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_32fc_s32f_convert_8ic, 3, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_32fc_convert_8ic, 3, 0, 20462, 1);

VOLK_RUN_TESTS(volk_gnsssdr_32fc_x5_cw_epl_corr_32fc_x3, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_16ic_x5_cw_epl_corr_32fc_x3, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8ic_x5_cw_epl_corr_32fc_x3, 1e-4, 0, 20462, 1);

VOLK_RUN_TESTS(volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8ic_x7_cw_vepl_corr_safe_32fc_x5, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8ic_x7_cw_vepl_corr_unsafe_32fc_x5, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8ic_x7_cw_vepl_corr_32fc_x5, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5, 1e-4, 0, 20462, 1);

VOLK_RUN_TESTS(volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc, 1e-4, 0, 20462, 1);
VOLK_RUN_TESTS(volk_gnsssdr_s32f_x2_update_local_carrier_32fc, 1e-4, 0, 20462, 1);







//VOLK_RUN_TESTS(volk_gnsssdr_16i_x5_add_quad_16i_x4, 1e-4, 2046, 10000);
//VOLK_RUN_TESTS(volk_gnsssdr_16i_branch_4_state_8, 1e-4, 2046, 10000);
//VOLK_RUN_TESTS(volk_gnsssdr_16i_max_star_16i, 0, 0, 20462, 10000);
//VOLK_RUN_TESTS(volk_gnsssdr_16i_max_star_horizontal_16i, 0, 0, 20462, 10000);
//VOLK_RUN_TESTS(volk_gnsssdr_16i_permute_and_scalar_add, 1e-4, 0, 2046, 1000);
//VOLK_RUN_TESTS(volk_gnsssdr_16i_x4_quad_max_star_16i, 1e-4, 0, 2046, 1000);
//VOLK_RUN_TESTS(volk_gnsssdr_16i_32fc_dot_prod_32fc, 1e-4, 0, 204602, 1);
//VOLK_RUN_TESTS(volk_gnsssdr_32fc_x2_conjugate_dot_prod_32fc, 1e-4, 0, 2046, 10000);
//VOLK_RUN_TESTS(volk_gnsssdr_32fc_s32f_x2_power_spectral_density_32f, 1e-4, 2046, 10000);
//VOLK_RUN_TESTS(volk_gnsssdr_32f_s32f_32f_fm_detect_32f, 1e-4, 2046, 10000);
//VOLK_RUN_TESTS(volk_gnsssdr_32u_popcnt, 0, 0, 2046, 10000);
//VOLK_RUN_TESTS(volk_gnsssdr_64u_popcnt, 0, 0, 2046, 10000);
