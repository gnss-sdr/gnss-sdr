/* Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
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
#include "kernel_tests.h"
#include "volk_gnsssdr_profile.h"

#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_prefs.h>

#include <ciso646>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/xpressive/xpressive.hpp>
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
      ("tol,t",
            boost::program_options::value<float>()->default_value( 1e-6 ),
            "Set the default error tolerance for tests")
      ("vlen,v",
            boost::program_options::value<int>()->default_value( 8111 ), //it is also prime
            "Set the default vector length for tests") // default is a mersenne prime
      ("iter,i",
            boost::program_options::value<int>()->default_value( 1987 ),
            "Set the default number of test iterations per kernel")
      ("tests-regex,R",
            boost::program_options::value<std::string>(),
            "Run tests matching regular expression.")
      ("update,u",
            boost::program_options::value<bool>()->default_value( false )
                                                     ->implicit_value( true ),
            "Run only kernels missing from config; use -R to further restrict the candidates")
      ("dry-run,n",
            boost::program_options::value<bool>()->default_value( false )
                                                     ->implicit_value( true ),
            "Dry run. Respect other options, but don't write to file")
      ("json,j",
            boost::program_options::value<std::string>(),
            "JSON output file")
      ("path,p",
            boost::program_options::value<std::string>(),
            "Specify volk_config path.")
      ;

    // Handle the options that were given
    boost::program_options::variables_map vm;
    bool benchmark_mode;
    std::string kernel_regex;
    std::ofstream json_file;
    float def_tol;
    lv_32fc_t def_scalar;
    int def_iter;
    int def_vlen;
    bool def_benchmark_mode;
    std::string def_kernel_regex;
    bool update_mode = false;
    bool dry_run = false;
    std::string config_file;

    // Handle the provided options
    try {
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);
        benchmark_mode = vm.count("benchmark")?vm["benchmark"].as<bool>():false;
        if ( vm.count("tests-regex" ) ) {
            kernel_regex = vm["tests-regex"].as<std::string>();
        }
        else {
            kernel_regex = ".*";
        }

        def_tol = vm["tol"].as<float>();
        def_scalar = 327.0;
        def_vlen = vm["vlen"].as<int>();
        def_iter = vm["iter"].as<int>();
        def_benchmark_mode = benchmark_mode;
        def_kernel_regex = kernel_regex;
        update_mode = vm["update"].as<bool>();
        dry_run = vm["dry-run"].as<bool>();
    }
    catch (boost::program_options::error& error) {
        std::cerr << "Error: " << error.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }

    /** --help option */
    if ( vm.count("help") ) {
      std::cout << "The VOLK_GNSSSDR profiler." << std::endl
                << desc << std::endl;
      return 0;
    }

    if ( vm.count("json") ) {
        std::string filename;
        try {
             filename = vm["json"].as<std::string>();
        }
        catch (boost::bad_any_cast& error) {
            std::cerr << error.what() << std::endl;
            return 1;
        }
        json_file.open( filename.c_str() );
    }

    if ( vm.count("path") ) {
            try {
                    config_file = vm["path"].as<std::string>() + "/volk_config";
                }
            catch (boost::bad_any_cast& error) {
                std::cerr << error.what() << std::endl;
                return 1;
          }
     }

    volk_gnsssdr_test_params_t test_params(def_tol, def_scalar, def_vlen, def_iter,
        def_benchmark_mode, def_kernel_regex);

    // Run tests
    std::vector<volk_gnsssdr_test_results_t> results;
    if(update_mode) {
        read_results(&results);
        if( vm.count("path") ) read_results(&results, config_file);
        else read_results(&results);
    }


    // Initialize the list of tests
    // the default test parameters come from options
    std::vector<volk_gnsssdr_test_case_t> test_cases = init_test_list(test_params);
    boost::xpressive::sregex kernel_expression;
    try {
        kernel_expression = boost::xpressive::sregex::compile(kernel_regex);
    }
    catch (boost::xpressive::regex_error& error) {
        std::cerr << "Error occurred while compiling regex" << std::endl << std::endl;
        return 1;
    }

    // Iteratate through list of tests running each one
    for(unsigned int ii = 0; ii < test_cases.size(); ++ii) {
        bool regex_match = true;

        volk_gnsssdr_test_case_t test_case = test_cases[ii];
        // if the kernel name matches regex then do the test
        if(boost::xpressive::regex_search(test_case.name(), kernel_expression)) {
            regex_match = true;
        }
        else {
            regex_match = false;
        }

        // if we are in update mode check if we've already got results
        // if we have any, then no need to test that kernel
        bool update = true;
        if(update_mode) {
            for(unsigned int jj=0; jj < results.size(); ++jj) {
                if(results[jj].name == test_case.name() ||
                    results[jj].name == test_case.puppet_master_name()) {
                    update = false;
                    break;
                }
            }
        }

        if( regex_match && update ) {
            try {
            run_volk_gnsssdr_tests(test_case.desc(), test_case.kernel_ptr(), test_case.name(),
                test_case.test_parameters(), &results, test_case.puppet_master_name());
            }
            catch (std::string error) {
                std::cerr << "Caught Exception in 'run_volk_gnsssdr_tests': " << error << std::endl;
            }

        }
    }


    // Output results according to provided options
    if(vm.count("json")) {
        write_json(json_file, results);
        json_file.close();
    }

    if(!dry_run) {
        write_results(&results, false);
        if(vm.count("path")) write_results(&results, false, config_file);
        else write_results(&results, false);
    }
    else {
        std::cout << "Warning: this was a dry-run. Config not generated" << std::endl;
    }
}

void read_results(std::vector<volk_gnsssdr_test_results_t> *results)
{
    char path[1024];
    volk_gnsssdr_get_config_path(path);

    read_results(results, std::string(path));
}

void read_results(std::vector<volk_gnsssdr_test_results_t> *results, std::string path)
{
    const fs::path config_path(path);

    if(fs::exists(config_path)) {
        // a config exists and we are reading results from it
        std::ifstream config(config_path.string().c_str());
        char config_line[256];
        while(config.getline(config_line, 255)) {
            // tokenize the input line by kernel_name unaligned aligned
            // then push back in the results vector with fields filled in

            std::vector<std::string> single_kernel_result;
            std::string config_str(config_line);
            std::size_t str_size = config_str.size();
            std::size_t found = 1;

            found = config_str.find(" ");
            // Split line by spaces
            while(found && found < str_size) {
                found = config_str.find(" ");
                // kernel names MUST be less than 128 chars, which is
                // a length restricted by volk_gnsssdr/volk_gnsssdr_prefs.c
                // on the last token in the parsed string we won't find a space
                // so make sure we copy at most 128 chars.
                if(found > 127) {
                    found = 127;
                }
                str_size = config_str.size();
                char buffer[128]  = {'\0'};
                config_str.copy(buffer, found + 1, 0);
                buffer[found] = '\0';
                single_kernel_result.push_back(std::string(buffer));
                config_str.erase(0, found+1);
            }

            if(single_kernel_result.size() == 3) {
                volk_gnsssdr_test_results_t kernel_result;
                kernel_result.name = std::string(single_kernel_result[0]);
                kernel_result.config_name = std::string(single_kernel_result[0]);
                kernel_result.best_arch_u = std::string(single_kernel_result[1]);
                kernel_result.best_arch_a = std::string(single_kernel_result[2]);
                results->push_back(kernel_result);
            }
        }
    }

}

void write_results(const std::vector<volk_gnsssdr_test_results_t> *results, bool update_result)
{
    char path[1024];
    volk_gnsssdr_get_config_path(path);

    write_results( results, update_result, std::string(path));
}

void write_results(const std::vector<volk_gnsssdr_test_results_t> *results, bool update_result, const std::string path)
{
    const fs::path config_path(path);
    // Until we can update the config on a kernel by kernel basis
    // do not overwrite volk_gnsssdr_config when using a regex.
    if (not fs::exists(config_path.branch_path()))
    {
        std::cout << "Creating " << config_path.branch_path() << "..." << std::endl;
        fs::create_directories(config_path.branch_path());
    }

    std::ofstream config;
    if(update_result) {
        std::cout << "Updating " << config_path << "..." << std::endl;
        config.open(config_path.string().c_str(), std::ofstream::app);
        if (!config.is_open()) { //either we don't have write access or we don't have the dir yet
            std::cout << "Error opening file " << config_path << std::endl;
        }
    }
    else {
        std::cout << "Writing " << config_path << "..." << std::endl;
        config.open(config_path.string().c_str());
        if (!config.is_open()) { //either we don't have write access or we don't have the dir yet
            std::cout << "Error opening file " << config_path << std::endl;
        }

        config << "\
#this file is generated by volk_gnsssdr_profile.\n\
#the function name is followed by the preferred architecture.\n\
";
    }

    std::vector<volk_gnsssdr_test_results_t>::const_iterator profile_results;
    for(profile_results = results->begin(); profile_results != results->end(); ++profile_results) {
        config << profile_results->config_name << " "
            << profile_results->best_arch_a << " "
            << profile_results->best_arch_u << std::endl;
    }
    config.close();
}

void write_json(std::ofstream &json_file, std::vector<volk_gnsssdr_test_results_t> results)
{
    json_file << "{" << std::endl;
    json_file << " \"volk_gnsssdr_tests\": [" << std::endl;
    size_t len = results.size();
    size_t i = 0;
    std::vector<volk_gnsssdr_test_results_t>::iterator result;
    for(result = results.begin(); result != results.end(); ++result) {
        json_file << "  {" << std::endl;
        json_file << "   \"name\": \"" << result->name << "\"," << std::endl;
        json_file << "   \"vlen\": " << (int)(result->vlen) << "," << std::endl;
        json_file << "   \"iter\": " << result->iter << "," << std::endl;
        json_file << "   \"best_arch_a\": \"" << result->best_arch_a
            << "\"," << std::endl;
        json_file << "   \"best_arch_u\": \"" << result->best_arch_u
            << "\"," << std::endl;
        json_file << "   \"results\": {" << std::endl;
        size_t results_len = result->results.size();
        size_t ri = 0;

        std::map<std::string, volk_gnsssdr_test_time_t>::iterator kernel_time_pair;
        for(kernel_time_pair = result->results.begin(); kernel_time_pair != result->results.end(); ++kernel_time_pair) {
            volk_gnsssdr_test_time_t time = kernel_time_pair->second;
            json_file << "    \"" << time.name << "\": {" << std::endl;
            json_file << "     \"name\": \"" << time.name << "\"," << std::endl;
            json_file << "     \"time\": " << time.time << "," << std::endl;
            json_file << "     \"units\": \"" << time.units << "\"" << std::endl;
            json_file << "    }" ;
            if(ri+1 != results_len) {
                json_file << ",";
            }
            json_file << std::endl;
            ri++;
        }
        json_file << "   }" << std::endl;
        json_file << "  }";
        if(i+1 != len) {
            json_file << ",";
        }
        json_file << std::endl;
        i++;
    }
    json_file << " ]" << std::endl;
    json_file << "}" << std::endl;
}


