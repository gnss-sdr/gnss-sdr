/*!
 * \file acquisition_dump_reader.h
 * \brief Helper file for unit testing
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_ACQUISITION_DUMP_READER_H
#define GNSS_SDR_ACQUISITION_DUMP_READER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class acquisition_dump_reader
{
public:
    acquisition_dump_reader(std::string & basename, unsigned int sat, unsigned int doppler_max, unsigned int doppler_step, unsigned int samples_per_code);
    ~acquisition_dump_reader();
    bool read_binary_acq();

    std::vector<int> doppler;
    std::vector<unsigned int> samples;
    std::vector<std::vector<float> > mag;

private:
    std::string d_basename;
    unsigned int d_sat;
    unsigned int d_doppler_max;
    unsigned int d_doppler_step;
    unsigned int d_samples_per_code;
    unsigned int d_num_doppler_bins;
    std::vector<std::string> d_dump_filenames;
    std::vector<std::ifstream> d_dump_files;
};

#endif // GNSS_SDR_ACQUISITION_DUMP_READER_H
