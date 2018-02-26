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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 */

#include "kernel_tests.h"                      // for init_test_list
#include "qa_utils.h"                          // for volk_gnsssdr_test_results_t
#include "volk_gnsssdr/volk_gnsssdr_complex.h" // for lv_32fc_t
#include "volk_gnsssdr_option_helpers.h"       // for option_list, option_t
#include "volk_gnsssdr_profile.h"
#include "volk_gnsssdr/volk_gnsssdr_prefs.h"   // for volk_gnsssdr_get_config_path
#include <boost/filesystem/operations.hpp>     // for create_directories, exists
#include <boost/filesystem/path.hpp>           // for path, operator<<
#include <boost/filesystem/path_traits.hpp>    // for filesystem
#include <sys/stat.h>                          // for stat
#include <cstddef>                             // for size_t
#include <iostream>                            // for operator<<, basic_ostream
#include <fstream>                             // IWYU pragma: keep
#include <map>                                 // for map, map<>::iterator
#include <utility>                             // for pair
#include <vector>                              // for vector, vector<>::const_..



namespace fs = boost::filesystem;

volk_gnsssdr_test_params_t test_params(1e-6f, 327.f, 8111, 1987, false, "");

void set_benchmark(bool val) { test_params.set_benchmark(val); }
void set_tolerance(float val) { test_params.set_tol(val); }
void set_vlen(int val) { test_params.set_vlen((unsigned int)val); }
void set_iter(int val) { test_params.set_iter((unsigned int)val); }
void set_substr(std::string val) { test_params.set_regex(val); }
bool update_mode = false;
void set_update(bool val) { update_mode = val; }
bool dry_run = false;
void set_dryrun(bool val) { dry_run = val; }
std::string json_filename("");
void set_json(std::string val) { json_filename = val; }
std::string volk_config_path("");
void set_volk_config(std::string val) { volk_config_path = val; }

int main(int argc, char *argv[])
{
    option_list profile_options("volk_gnsssdr_profile");
    profile_options.add(option_t("benchmark", "b", "Run all kernels (benchmark mode)", set_benchmark));
    profile_options.add(option_t("tol", "t", "Set the default tolerance for all tests", set_tolerance));
    profile_options.add(option_t("vlen", "v", "Set the default vector length for tests", set_vlen));
    profile_options.add((option_t("iter", "i", "Set the default number of test iterations per kernel", set_iter)));
    profile_options.add((option_t("tests-substr", "R", "Run tests matching substring", set_substr)));
    profile_options.add((option_t("update", "u", "Run only kernels missing from config", set_update)));
    profile_options.add((option_t("dry-run", "n", "Dry run. Respect other options, but don't write to file", set_dryrun)));
    profile_options.add((option_t("json", "j", "Write results to JSON file named as argument value", set_json)));
    profile_options.add((option_t("path", "p", "Specify the volk_config path", set_volk_config)));

    try
    {
            profile_options.parse(argc, argv);
    }
    catch(...)
    {
           return 1;
    }

    for (int arg_number = 0; arg_number < argc; ++arg_number) {
            if (std::string("--help") == std::string(argv[arg_number]) ||
                    std::string("-h") == std::string(argv[arg_number])) {
                    return 0;
            }
    }
    // Adding program options
    std::ofstream json_file;
    std::string config_file;

    if ( json_filename != "" ) {
        json_file.open( json_filename.c_str() );
    }

    if ( volk_config_path != "" ) {
        config_file = volk_config_path + "/volk_config";
    }

    // Run tests
    std::vector<volk_gnsssdr_test_results_t> results;
    if(update_mode) {
        if( config_file != "" ) read_results(&results, config_file);
        else read_results(&results);
    }

    // Initialize the list of tests
    std::vector<volk_gnsssdr_test_case_t> test_cases = init_test_list(test_params);

    // Iterate through list of tests running each one
    std::string substr_to_match(test_params.kernel_regex());
    for(unsigned int ii = 0; ii < test_cases.size(); ++ii) {
        bool regex_match = true;

        volk_gnsssdr_test_case_t test_case = test_cases[ii];
        // if the kernel name matches regex then do the test
        std::string test_case_name = test_case.name();
        if(test_case_name.find(substr_to_match) == std::string::npos) {
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
            catch (std::string &error) {
                std::cerr << "Caught Exception in 'run_volk_gnssdr_tests': " << error << std::endl;
            }
        }
    }


    // Output results according to provided options
    if(json_filename != "") {
        write_json(json_file, results);
        json_file.close();
    }

    if(!dry_run) {
        if(config_file != "") write_results(&results, false, config_file);
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
    struct stat buffer;
    bool config_status = (stat (path.c_str(), &buffer) == 0);

    if( config_status ) {
        // a config exists and we are reading results from it
        std::ifstream config(path.c_str());
        char config_line[256];
        while(config.getline(config_line, 255)) {
            // tokenize the input line by kernel_name unaligned aligned
            // then push back in the results vector with fields filled in

            std::vector<std::string> single_kernel_result;
            std::string config_str(config_line);
            std::size_t str_size = config_str.size();
            std::size_t found = 1;

            found = config_str.find(' ');
            // Split line by spaces
            while(found && found < str_size) {
                    found = config_str.find(' ');
                    // kernel names MUST be less than 128 chars, which is
                    // a length restricted by volk/volk_prefs.c
                    // on the last token in the parsed string we won't find a space
                    // so make sure we copy at most 128 chars.
                    if(found > 127) {
                            found = 127;
                    }
                    str_size = config_str.size();
                    char buffer[128] = {'\0'};
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
    if (! fs::exists(config_path.branch_path()))
    {
        std::cout << "Creating " << config_path.branch_path() << " ..." << std::endl;
        fs::create_directories(config_path.branch_path());
    }

    std::ofstream config;
    if(update_result) {
        std::cout << "Updating " << path << " ..." << std::endl;
        config.open(path.c_str(), std::ofstream::app);
        if (!config.is_open()) { //either we don't have write access or we don't have the dir yet
            std::cout << "Error opening file " << path << std::endl;
        }
    }
    else {
        std::cout << "Writing " << path << " ..." << std::endl;
        config.open(path.c_str());
        if (!config.is_open()) { //either we don't have write access or we don't have the dir yet
            std::cout << "Error opening file " << path << std::endl;
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


