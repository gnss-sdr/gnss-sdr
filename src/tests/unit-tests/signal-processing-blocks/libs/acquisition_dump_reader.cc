/*!
 * \file acquisition_dump_reader.cc
 * \brief Helper file for unit testing
 * \author Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
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

#include <complex>
#include "acquisition_dump_reader.h"

bool acquisition_dump_reader::read_binary_acq()
{
    std::complex<float>* aux = new std::complex<float>[1];
    for(int i = 0; i < d_num_doppler_bins; i++)
        {
            try
            {
                    std::ifstream ifs;
                    ifs.exceptions( std::ifstream::failbit | std::ifstream::badbit );
                    ifs.open(d_dump_filenames.at(i).c_str(), std::ios::in | std::ios::binary);
                    d_dump_files.at(i).swap(ifs);
                    if (d_dump_files.at(i).is_open())
                        {
                            for(int k = 0; k < d_samples_per_code; k++)
                                {
                                    d_dump_files.at(i).read(reinterpret_cast<char *>(&aux[0]), sizeof(std::complex<float>));
                                    mag.at(i).at(k) = std::abs(*aux) / std::pow(d_samples_per_code, 2);
                                }
                        }
                    else
                        {
                            std::cout << "File " << d_dump_filenames.at(i).c_str() << " not found." << std::endl;
                            delete[] aux;
                            return false;
                        }
                    d_dump_files.at(i).close();
            }
            catch (const std::ifstream::failure &e)
            {
                    std::cout << e.what() << std::endl;
                    delete[] aux;
                    return false;
            }
        }
    delete[] aux;
    return true;
}


acquisition_dump_reader::acquisition_dump_reader(const std::string & basename, unsigned int sat, unsigned int doppler_max, unsigned int doppler_step, unsigned int samples_per_code)
{
    d_basename = basename;
    d_sat = sat;
    d_doppler_max = doppler_max;
    d_doppler_step = doppler_step;
    d_samples_per_code = samples_per_code;
    d_num_doppler_bins = static_cast<unsigned int>(ceil( static_cast<double>(static_cast<int>(d_doppler_max) - static_cast<int>(-d_doppler_max)) / static_cast<double>(d_doppler_step)));
    std::vector<std::vector<float> > mag_aux(d_num_doppler_bins, std::vector<float>(d_samples_per_code));
    mag = mag_aux;
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            doppler.push_back(-static_cast<int>(d_doppler_max) + d_doppler_step * doppler_index);
            d_dump_filenames.push_back(d_basename + "_sat_" + std::to_string(d_sat) + "_doppler_" + std::to_string(doppler.at(doppler_index)) + ".dat");
            std::ifstream ifs;
            d_dump_files.push_back(std::move(ifs));
        }
    for (unsigned int k = 0; k < d_samples_per_code; k++)
        {
            samples.push_back(k);
        }
}


acquisition_dump_reader::~acquisition_dump_reader()
{
    for(int i = 0; i < d_num_doppler_bins; i++)
        {
            if (d_dump_files.at(i).is_open() == true)
                {
                    d_dump_files.at(i).close();
                }
        }
}
