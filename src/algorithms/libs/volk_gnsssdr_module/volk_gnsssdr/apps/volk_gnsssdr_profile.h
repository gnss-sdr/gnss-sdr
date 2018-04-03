/*!
 * \file volk_gnsssdr_profile.h
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

#include <cstdbool>  // for bool
#include <iosfwd>    // for ofstream
#include <string>    // for string
#include <vector>    // for vector

class volk_gnsssdr_test_results_t;


void read_results(std::vector<volk_gnsssdr_test_results_t> *results);
void read_results(std::vector<volk_gnsssdr_test_results_t> *results, std::string path);
void write_results(const std::vector<volk_gnsssdr_test_results_t> *results, bool update_result);
void write_results(const std::vector<volk_gnsssdr_test_results_t> *results, bool update_result, const std::string path);
void write_json(std::ofstream &json_file, std::vector<volk_gnsssdr_test_results_t> results);
