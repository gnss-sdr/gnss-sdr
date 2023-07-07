/*!
 * \file acquisition_dump_reader.cc
 * \brief Helper file for unit testing
 * \authors Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
 *                    Antonio Ramos, 2018. antonio.ramos(at)cttc.es
 *
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

#include "acquisition_dump_reader.h"
#include <matio.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

bool Acquisition_Dump_Reader::read_binary_acq()
{
    mat_t* matfile = Mat_Open(d_dump_filename.c_str(), MAT_ACC_RDONLY);
    if (matfile == nullptr)
        {
            std::cout << "Unreachable Acquisition dump file " << d_dump_filename << '\n';
            return false;
        }
    matvar_t* var_ = Mat_VarRead(matfile, "acq_grid");
    if (var_ == nullptr)
        {
            std::cout << "Unreachable grid variable in Acquisition dump file.\n";
            Mat_Close(matfile);
            return false;
        }
    if (var_->rank != 2)
        {
            std::cout << "Invalid Acquisition dump file: rank error\n";
            Mat_VarFree(var_);
            Mat_Close(matfile);
            return false;
        }
    if ((var_->dims[0] != d_samples_per_code) or (var_->dims[1] != d_num_doppler_bins))
        {
            std::cout << "Invalid Acquisition dump file: dimension matrix error\n";
            if (var_->dims[0] != d_samples_per_code)
                {
                    std::cout << "Expected " << d_samples_per_code << " samples per code. Obtained " << var_->dims[0] << '\n';
                }
            if (var_->dims[1] != d_num_doppler_bins)
                {
                    std::cout << "Expected " << d_num_doppler_bins << " Doppler bins. Obtained " << var_->dims[1] << '\n';
                }
            Mat_VarFree(var_);
            Mat_Close(matfile);
            return false;
        }
    if (var_->data_type != MAT_T_SINGLE)
        {
            std::cout << "Invalid Acquisition dump file: data type error\n";
            Mat_VarFree(var_);
            Mat_Close(matfile);
            return false;
        }
    matvar_t* var2_ = Mat_VarRead(matfile, "doppler_max");
    d_doppler_max = *static_cast<unsigned int*>(var2_->data);
    Mat_VarFree(var2_);

    var2_ = Mat_VarRead(matfile, "doppler_step");
    d_doppler_step = *static_cast<unsigned int*>(var2_->data);
    Mat_VarFree(var2_);

    var2_ = Mat_VarRead(matfile, "input_power");
    input_power = *static_cast<float*>(var2_->data);
    Mat_VarFree(var2_);

    var2_ = Mat_VarRead(matfile, "acq_doppler_hz");
    acq_doppler_hz = *static_cast<float*>(var2_->data);
    Mat_VarFree(var2_);

    var2_ = Mat_VarRead(matfile, "acq_delay_samples");
    acq_delay_samples = *static_cast<float*>(var2_->data);
    Mat_VarFree(var2_);

    var2_ = Mat_VarRead(matfile, "test_statistic");
    test_statistic = *static_cast<float*>(var2_->data);
    Mat_VarFree(var2_);

    var2_ = Mat_VarRead(matfile, "threshold");
    threshold = *static_cast<float*>(var2_->data);
    Mat_VarFree(var2_);

    var2_ = Mat_VarRead(matfile, "sample_counter");
    sample_counter = *static_cast<uint64_t*>(var2_->data);
    Mat_VarFree(var2_);

    var2_ = Mat_VarRead(matfile, "d_positive_acq");
    positive_acq = *static_cast<int*>(var2_->data);
    Mat_VarFree(var2_);

    var2_ = Mat_VarRead(matfile, "num_dwells");
    num_dwells = *static_cast<int*>(var2_->data);
    Mat_VarFree(var2_);

    var2_ = Mat_VarRead(matfile, "PRN");
    PRN = *static_cast<int*>(var2_->data);
    Mat_VarFree(var2_);

    std::vector<std::vector<float> >::iterator it1;
    std::vector<float>::iterator it2;
    auto* aux = static_cast<float*>(var_->data);
    int k = 0;
    float normalization_factor = std::pow(d_samples_per_code, 4) * input_power;
    for (it1 = mag.begin(); it1 != mag.end(); it1++)
        {
            for (it2 = it1->begin(); it2 != it1->end(); it2++)
                {
                    *it2 = static_cast<float>(aux[k]) / normalization_factor;
                    k++;
                }
        }
    Mat_VarFree(var_);
    Mat_Close(matfile);

    return true;
}


Acquisition_Dump_Reader::Acquisition_Dump_Reader(const std::string& basename,
    int channel,
    int execution)
{
    unsigned int sat_ = 0;
    unsigned int doppler_max_ = 0;
    unsigned int doppler_step_ = 0;
    unsigned int samples_per_code_ = 0;

    mat_t* matfile = Mat_Open(d_dump_filename.c_str(), MAT_ACC_RDONLY);
    if (matfile != nullptr)
        {
            matvar_t* var_ = Mat_VarRead(matfile, "doppler_max");
            doppler_max_ = *static_cast<unsigned int*>(var_->data);
            Mat_VarFree(var_);

            var_ = Mat_VarRead(matfile, "doppler_step");
            doppler_step_ = *static_cast<unsigned int*>(var_->data);
            Mat_VarFree(var_);

            var_ = Mat_VarRead(matfile, "PRN");
            sat_ = *static_cast<int*>(var_->data);
            Mat_VarFree(var_);

            var_ = Mat_VarRead(matfile, "grid");
            samples_per_code_ = var_->dims[0];
            Mat_VarFree(var_);

            Mat_Close(matfile);
        }
    else
        {
            std::cout << "Unreachable Acquisition dump file " << d_dump_filename << '\n';
        }
    acq_doppler_hz = 0.0;
    acq_delay_samples = 0.0;
    test_statistic = 0.0;
    input_power = 0.0;
    threshold = 0.0;
    positive_acq = 0;
    sample_counter = 0;
    PRN = 0;
    d_sat = 0;
    d_doppler_max = doppler_max_;
    d_doppler_step = doppler_step_;
    d_samples_per_code = samples_per_code_;
    d_num_doppler_bins = 0;
    num_dwells = 0;

    *this = Acquisition_Dump_Reader(basename,
        sat_,
        doppler_max_,
        doppler_step_,
        samples_per_code_,
        channel,
        execution);
}


Acquisition_Dump_Reader::Acquisition_Dump_Reader(const std::string& basename,
    unsigned int sat,
    unsigned int doppler_max,
    unsigned int doppler_step,
    unsigned int samples_per_code,
    int channel,
    int execution)
    : d_basename(basename),
      d_sat(sat),
      d_doppler_max(doppler_max),
      d_doppler_step(doppler_step),
      d_samples_per_code(samples_per_code)
{
    if (d_doppler_step == 0)
        {
            d_doppler_step = 1;
        }
    d_num_doppler_bins = static_cast<unsigned int>(ceil(static_cast<double>(static_cast<int>(d_doppler_max) - static_cast<int>(-d_doppler_max)) / static_cast<double>(d_doppler_step)));
    std::vector<std::vector<float> > mag_aux(d_num_doppler_bins, std::vector<float>(d_samples_per_code));
    mag = mag_aux;
    d_dump_filename = d_basename + "_ch_" + std::to_string(channel) + "_" + std::to_string(execution) + "_sat_" + std::to_string(d_sat) + ".mat";
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            doppler.push_back(-static_cast<int>(d_doppler_max) + d_doppler_step * doppler_index);
        }
    for (unsigned int k = 0; k < d_samples_per_code; k++)
        {
            samples.push_back(k);
        }
}


// Copy assignment operator
Acquisition_Dump_Reader& Acquisition_Dump_Reader::operator=(const Acquisition_Dump_Reader& other)
{
    if (this != &other)
        {
            doppler = other.doppler;
            samples = other.samples;
            mag = other.mag;
            acq_doppler_hz = other.acq_doppler_hz;
            acq_delay_samples = other.acq_delay_samples;
            test_statistic = other.test_statistic;
            input_power = other.input_power;
            threshold = other.threshold;
            positive_acq = other.positive_acq;
            PRN = other.PRN;
            num_dwells = other.num_dwells;
            sample_counter = other.sample_counter;
            d_basename = other.d_basename;
            d_dump_filename = other.d_dump_filename;
            d_sat = other.d_sat;
            d_doppler_max = other.d_doppler_max;
            d_doppler_step = other.d_doppler_step;
            d_samples_per_code = other.d_samples_per_code;
            d_num_doppler_bins = other.d_num_doppler_bins;
        }
    return *this;
}


// Move constructor
Acquisition_Dump_Reader::Acquisition_Dump_Reader(Acquisition_Dump_Reader&& other) noexcept
    : doppler(std::move(other.doppler)),
      samples(std::move(other.samples)),
      mag(std::move(other.mag)),
      acq_doppler_hz(other.acq_doppler_hz),
      acq_delay_samples(other.acq_delay_samples),
      test_statistic(other.test_statistic),
      input_power(other.input_power),
      threshold(other.threshold),
      positive_acq(other.positive_acq),
      PRN(other.PRN),
      num_dwells(other.num_dwells),
      sample_counter(other.sample_counter),
      d_basename(std::move(other.d_basename)),
      d_dump_filename(std::move(other.d_dump_filename)),
      d_sat(other.d_sat),
      d_doppler_max(other.d_doppler_max),
      d_doppler_step(other.d_doppler_step),
      d_samples_per_code(other.d_samples_per_code),
      d_num_doppler_bins(other.d_num_doppler_bins)
{
}


// Move assignment operator
Acquisition_Dump_Reader& Acquisition_Dump_Reader::operator=(Acquisition_Dump_Reader&& other) noexcept
{
    if (this != &other)  // Check for self-assignment
        {
            // Move member variables from the other object to this object
            d_basename = std::move(other.d_basename);
            d_dump_filename = std::move(other.d_dump_filename);
            d_sat = other.d_sat;
            d_doppler_max = other.d_doppler_max;
            d_doppler_step = other.d_doppler_step;
            d_samples_per_code = other.d_samples_per_code;
            d_num_doppler_bins = other.d_num_doppler_bins;
            doppler = std::move(other.doppler);
            samples = std::move(other.samples);
            mag = std::move(other.mag);
            acq_doppler_hz = other.acq_doppler_hz;
            acq_delay_samples = other.acq_delay_samples;
            test_statistic = other.test_statistic;
            input_power = other.input_power;
            threshold = other.threshold;
            positive_acq = other.positive_acq;
            PRN = other.PRN;
            num_dwells = other.num_dwells;
            sample_counter = other.sample_counter;
        }
    return *this;
}