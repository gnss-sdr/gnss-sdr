/*
 * Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 */


#include "kernel_tests.h"                       // for init_test_list
#include "qa_utils.h"                           // for volk_gnsssdr_test_case_t, volk_gnsssdr_test_results_t
#include "volk_gnsssdr/volk_gnsssdr_complex.h"  // for lv_32fc_t
#include <cstdbool>                             // for bool, false, true
#include <iostream>                             // for operator<<, basic_ostream, endl, char...
#include <fstream>                              // IWYU pragma: keep
#include <map>                                  // for map, map<>::iterator, _Rb_tree_iterator
#include <sstream>                              // for stringstream
#include <string>                               // for string, operator<<
#include <utility>                              // for pair
#include <vector>                               // for vector

void print_qa_xml(std::vector<volk_gnsssdr_test_results_t> results, unsigned int nfails);

int main(int argc, char* argv[])
{
    bool qa_ret_val = 0;

    float def_tol = 1e-6;
    lv_32fc_t def_scalar = 327.0;
    int def_iter = 1;
    int def_vlen = 8111;
    bool def_benchmark_mode = true;
    std::string def_kernel_regex = "";

    volk_gnsssdr_test_params_t test_params(def_tol, def_scalar, def_vlen, def_iter,
        def_benchmark_mode, def_kernel_regex);
    std::vector<volk_gnsssdr_test_case_t> test_cases = init_test_list(test_params);
    std::vector<volk_gnsssdr_test_results_t> results;
    if (argc > 1)
        {
            std::stringstream ss;
            ss << argv[1];
            if (ss.fail())
                {
                    std::cerr << "Test name not correctly set." << std::endl;
                    return 0;
                }
            for (unsigned int ii = 0; ii < test_cases.size(); ++ii)
                {
                    if (ss.str() == test_cases[ii].name())
                        {
                            volk_gnsssdr_test_case_t test_case = test_cases[ii];
                            if (run_volk_gnsssdr_tests(test_case.desc(), test_case.kernel_ptr(),
                                    test_case.name(),
                                    test_case.test_parameters(), &results,
                                    test_case.puppet_master_name()))
                                {
                                    return 1;
                                }
                            else
                                {
                                    return 0;
                                }
                        }
                }
            std::cerr << "Did not run a test for kernel: " << std::string(argv[1]) << " !" << std::endl;
            return 0;
        }
    else
        {
            std::vector<std::string> qa_failures;
            // Test every kernel reporting failures when they occur
            for (unsigned int ii = 0; ii < test_cases.size(); ++ii)
                {
                    bool qa_result = false;
                    volk_gnsssdr_test_case_t test_case = test_cases[ii];
                    try
                        {
                            qa_result = run_volk_gnsssdr_tests(test_case.desc(), test_case.kernel_ptr(), test_case.name(),
                                test_case.test_parameters(), &results, test_case.puppet_master_name());
                        }
                    catch (...)
                        {
                            // TODO: what exceptions might we need to catch and how do we handle them?
                            std::cerr << "Exception found on kernel: " << test_case.name() << std::endl;
                            qa_result = false;
                        }

                    if (qa_result)
                        {
                            std::cerr << "Failure on " << test_case.name() << std::endl;
                            qa_failures.push_back(test_case.name());
                        }
                }

            // Generate XML results
            print_qa_xml(results, qa_failures.size());

            // Summarize QA results
            std::cerr << "Kernel QA finished: " << qa_failures.size() << " failures out of "
                      << test_cases.size() << " tests." << std::endl;
            if (qa_failures.size() > 0)
                {
                    std::cerr << "The following kernels failed QA:" << std::endl;
                    for (unsigned int ii = 0; ii < qa_failures.size(); ++ii)
                        {
                            std::cerr << "    " << qa_failures[ii] << std::endl;
                        }
                    qa_ret_val = 1;
                }
        }

    return qa_ret_val;
}

/*
 * This function prints qa results as XML output similar to output
 * from Junit. For reference output see http://llg.cubic.org/docs/junit/
 */
void print_qa_xml(std::vector<volk_gnsssdr_test_results_t> results, unsigned int nfails)
{
    std::ofstream qa_file;
    qa_file.open(".unittest/kernels.xml");

    qa_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
    qa_file << "<testsuites name=\"kernels\" "
            << "tests=\"" << results.size() << "\" "
            << "failures=\"" << nfails << "\" id=\"1\">" << std::endl;

    // Results are in a vector by kernel. Each element has a result
    // map containing time and arch name with test result
    for (unsigned int ii = 0; ii < results.size(); ++ii)
        {
            volk_gnsssdr_test_results_t result = results[ii];
            qa_file << "  <testsuite name=\"" << result.name << "\">" << std::endl;

            std::map<std::string, volk_gnsssdr_test_time_t>::iterator kernel_time_pair;
            for (kernel_time_pair = result.results.begin(); kernel_time_pair != result.results.end(); ++kernel_time_pair)
                {
                    volk_gnsssdr_test_time_t test_time = kernel_time_pair->second;
                    qa_file << "    <testcase name=\"" << test_time.name << "\" "
                            << "classname=\"" << result.name << "\" "
                            << "time=\"" << test_time.time << "\">" << std::endl;
                    if (!test_time.pass)
                        qa_file << "      <failure "
                                << "message=\"fail on arch " << test_time.name << "\">"
                                << "</failure>" << std::endl;
                    qa_file << "    </testcase>" << std::endl;
                }
            qa_file << "  </testsuite>" << std::endl;
        }

    qa_file << "</testsuites>" << std::endl;
    qa_file.close();
}
