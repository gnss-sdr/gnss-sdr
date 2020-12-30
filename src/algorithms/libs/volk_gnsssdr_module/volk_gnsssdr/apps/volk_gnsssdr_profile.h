/*!
 * \file volk_gnsssdr_profile.h
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include <iosfwd>  // for ofstream
#include <string>  // for string
#include <vector>  // for vector

class volk_gnsssdr_test_results_t;


void read_results(std::vector<volk_gnsssdr_test_results_t> *results);
void read_results(std::vector<volk_gnsssdr_test_results_t> *results, std::string path);
void write_results(const std::vector<volk_gnsssdr_test_results_t> *results, bool update_result);
void write_results(const std::vector<volk_gnsssdr_test_results_t> *results, bool update_result, const std::string path);
void write_json(std::ofstream &json_file, std::vector<volk_gnsssdr_test_results_t> results);
