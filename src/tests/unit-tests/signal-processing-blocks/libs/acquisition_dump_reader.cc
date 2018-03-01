/*!
 * \file acquisition_dump_reader.cc
 * \brief Helper file for unit testing
 * \authors Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
 *                    Antonio Ramos, 2018. antonio.ramos(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#include "acquisition_dump_reader.h"
#include <matio.h>
#include <cmath>
#include <iostream>

bool acquisition_dump_reader::read_binary_acq()
{
    mat_t* matfile = Mat_Open(d_dump_filename.c_str(), MAT_ACC_RDONLY);
    if( matfile == NULL)
        {
            std::cout << "¡¡¡Unreachable Acquisition dump file!!!" << std::endl;
            return false;
        }
    matvar_t* var_= Mat_VarRead(matfile, "grid");
    if( var_ == NULL)
        {
            std::cout << "¡¡¡Unreachable grid variable into Acquisition dump file!!!" << std::endl;
            Mat_Close(matfile);
            return false;
        }
    if(var_->rank != 2)
        {
            std::cout << "Invalid Acquisition dump file: rank error" << std::endl;
            Mat_VarFree(var_);
            Mat_Close(matfile);
            return false;
        }
    if((var_->dims[0] != d_samples_per_code) or (var_->dims[1] != d_num_doppler_bins))
        {
            std::cout << "Invalid Acquisition dump file: dimension matrix error" << std::endl;
            if(var_->dims[0] != d_samples_per_code) std::cout << "Expected " << d_samples_per_code << " samples per code. Obtained " << var_->dims[0] << std::endl;
            if(var_->dims[1] != d_num_doppler_bins) std::cout << "Expected " << d_num_doppler_bins << " Doppler bins. Obtained " << var_->dims[1] << std::endl;
            Mat_VarFree(var_);
            Mat_Close(matfile);
            return false;
        }
    if(var_->data_type != MAT_T_SINGLE)
        {
            std::cout << "Invalid Acquisition dump file: data type error" << std::endl;
            Mat_VarFree(var_);
            Mat_Close(matfile);
            return false;
        }
    std::vector<std::vector<float> >::iterator it1;
    std::vector<float>::iterator it2;
    float* aux = static_cast<float*>(var_->data);
    int k = 0;
    float normalization_factor = std::pow(d_samples_per_code, 2);
    for(it1 = mag.begin(); it1 != mag.end(); it1++)
        {
            for(it2 = it1->begin(); it2 != it1->end(); it2++)
                {
                    *it2 = static_cast<float>(std::sqrt(aux[k])) / normalization_factor;
                    k++;
                }
        }
    Mat_VarFree(var_);
    Mat_Close(matfile);

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
    d_dump_filename = d_basename + "_sat_" + std::to_string(d_sat) + ".mat";
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            doppler.push_back(-static_cast<int>(d_doppler_max) + d_doppler_step * doppler_index);
        }
    for (unsigned int k = 0; k < d_samples_per_code; k++)
        {
            samples.push_back(k);
        }
}


acquisition_dump_reader::~acquisition_dump_reader()
{}
