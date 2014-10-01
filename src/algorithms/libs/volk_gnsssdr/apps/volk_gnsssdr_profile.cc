/* -*- c++ -*- */
/*
 * Copyright 2012-2014 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "qa_utils.h"

#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_prefs.h>

#include <ciso646>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace fs = boost::filesystem;

int main(int argc, char *argv[]) {
    // Adding program options
    boost::program_options::options_description desc("Options");
    desc.add_options()
      ("help,h", "Print help messages")
      ("benchmark,b",
            boost::program_options::value<bool>()->default_value( false )
                                                ->implicit_value( true ),
            "Run all kernels (benchmark mode)")
      ("tests-regex,R",
            boost::program_options::value<std::string>(),
            "Run tests matching regular expression.")
      ;

    // Handle the options that were given
    boost::program_options::variables_map vm;
    bool benchmark_mode;
    std::string kernel_regex;
    bool store_results = true;
    try {
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);
        benchmark_mode = vm.count("benchmark")?vm["benchmark"].as<bool>():false;
        if ( vm.count("tests-regex" ) ) {
            kernel_regex = vm["tests-regex"].as<std::string>();
            store_results = false;
            std::cout << "Warning: using a regexp will not save results to a config" << std::endl;
        }
        else {
            kernel_regex = ".*";
            store_results = true;
        }
    } catch (boost::program_options::error& error) {
        std::cerr << "Error: " << error.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }
    /** --help option
*/
    if ( vm.count("help") )
    {
      std::cout << "The VOLK profiler." << std::endl
                << desc << std::endl;
      return 0;
    }


    // Run tests
    std::vector<std::string> results;

    //VOLK_PROFILE(volk_gnsssdr_16i_x5_add_quad_16i_x4, 1e-4, 2046, 10000, &results, benchmark_mode, kernel_regex);
    //VOLK_PROFILE(volk_gnsssdr_16i_branch_4_state_8, 1e-4, 2046, 10000, &results, benchmark_mode, kernel_regex);
    //VOLK_PROFILE(volk_gnsssdr_16i_max_star_16i, 0, 0, 204602, 10000, &results, benchmark_mode, kernel_regex);
    //VOLK_PROFILE(volk_gnsssdr_16i_max_star_horizontal_16i, 0, 0, 204602, 10000, &results, benchmark_mode, kernel_regex);
    //VOLK_PROFILE(volk_gnsssdr_16i_permute_and_scalar_add, 1e-4, 0, 2046, 10000, &results, benchmark_mode, kernel_regex);
    //VOLK_PROFILE(volk_gnsssdr_16i_x4_quad_max_star_16i, 1e-4, 0, 2046, 10000, &results, benchmark_mode, kernel_regex);
    //VOLK_PROFILE(volk_gnsssdr_32fc_x2_conjugate_dot_prod_32fc, 1e-4, 0, 2046, 10000, &results, benchmark_mode, kernel_regex);
    //VOLK_PROFILE(volk_gnsssdr_32fc_s32f_x2_power_spectral_density_32f, 1e-4, 2046, 10000, &results, benchmark_mode, kernel_regex);
    //VOLK_PROFILE(volk_gnsssdr_32f_s32f_32f_fm_detect_32f, 1e-4, 2046, 10000, &results, benchmark_mode, kernel_regex);
    //VOLK_PROFILE(volk_gnsssdr_32u_popcnt, 0, 0, 2046, 10000, &results, benchmark_mode, kernel_regex);
    //VOLK_PROFILE(volk_gnsssdr_64u_popcnt, 0, 0, 2046, 10000, &results, benchmark_mode, kernel_regex);
    //VOLK_PROFILE(volk_gnsssdr_32fc_s32fc_multiply_32fc, 1e-4, lv_32fc_t(1.0, 0.5), 204602, 1000, &results, benchmark_mode, kernel_regex);

    // Until we can update the config on a kernel by kernel basis
    // do not overwrite volk_gnsssdr_config when using a regex.
    
    //GNSS-SDR PROTO-KERNELS
    //lv_32fc_t sfv = lv_cmake((float)1, (float)2);
    //example: VOLK_PROFILE(volk_gnsssdr_8ic_s8ic_multiply_8ic, 1e-4, sfv, 204602, 1000, &results, benchmark_mode, kernel_regex);
    
     VOLK_PROFILE(volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc, 1e-4, 0, 7, 1, &results, benchmark_mode, kernel_regex);

     VOLK_PROFILE(volk_gnsssdr_8ic_x7_cw_vepl_corr_safe_32fc_x5, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8ic_x7_cw_vepl_corr_unsafe_32fc_x5, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
    
     VOLK_PROFILE(volk_gnsssdr_8ic_x7_cw_vepl_corr_32fc_x5, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
    
     VOLK_PROFILE(volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8ic_x5_cw_epl_corr_32fc_x3, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_16ic_x5_cw_epl_corr_32fc_x3, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32fc_x5_cw_epl_corr_32fc_x3, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
    
     VOLK_PROFILE(volk_gnsssdr_32fc_convert_16ic, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32fc_convert_8ic, 1e-4, 0, 16000, 250, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32fc_s32f_convert_8ic, 1e-4, 5, 16000, 250, &results, benchmark_mode, kernel_regex);
    
     /*VOLK_PROFILE(volk_gnsssdr_32f_accumulator_s32f, 1e-4, 0, 204602, 10000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8i_accumulator_s8i, 1e-4, 0, 204602, 10000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32f_index_max_16u, 3, 0, 204602, 5000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8i_index_max_16u, 3, 0, 204602, 5000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8i_max_s8i, 3, 0, 204602, 5000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32f_x2_add_32f, 1e-4, 0, 204602, 10000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8i_x2_add_8i, 1e-4, 0, 204602, 10000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32fc_conjugate_32fc, 1e-4, 0, 204602, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8ic_conjugate_8ic, 1e-4, 0, 204602, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32fc_magnitude_squared_32f, 1e-4, 0, 204602, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8ic_magnitude_squared_8i, 1e-4, 0, 204602, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32fc_s32fc_multiply_32fc, 1e-4, 0, 204602, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8ic_s8ic_multiply_8ic, 1e-4, 0, 204602, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32fc_x2_dot_prod_32fc, 1e-4, 0, 204602, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8ic_x2_dot_prod_8ic, 1e-4, 0, 204602, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32fc_x2_multiply_32fc, 1e-4, 0, 204602, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8ic_x2_multiply_8ic, 1e-4, 0, 204602, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_8u_x2_multiply_8u, 1e-4, 0, 204602, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_64f_accumulator_64f, 1e-4, 0, 16000, 1000, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_32f_s32f_convert_16i, 1e-4, 1, 204602, 250, &results, benchmark_mode, kernel_regex);
     VOLK_PROFILE(volk_gnsssdr_16i_s32f_convert_32f, 1e-4, 1, 204602, 250, &results, benchmark_mode, kernel_regex);*/
    if(store_results) {
        char path[1024];
        volk_gnsssdr_get_config_path(path);

        const fs::path config_path(path);

        if (not fs::exists(config_path.branch_path()))
        {
            std::cout << "Creating " << config_path.branch_path() << "..." << std::endl;
            fs::create_directories(config_path.branch_path());
        }

        std::cout << "Writing " << config_path << "..." << std::endl;
        std::ofstream config(config_path.string().c_str());
        if(!config.is_open()) { //either we don't have write access or we don't have the dir yet
            std::cout << "Error opening file " << config_path << std::endl;
        }

        config << "\
#this file is generated by volk_gnsssdr_profile.\n\
#the function name is followed by the preferred architecture.\n\
";

        BOOST_FOREACH(std::string result, results) {
            config << result << std::endl;
        }
        config.close();
    }
    else {
        std::cout << "Warning: config not generated" << std::endl;
    }
}
