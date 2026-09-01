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
#include <cstdint>
#include <iostream>
#include <utility>

namespace
{
// Reads a 1x1 variable from the dump, writing fallback instead when the
// variable is absent (e.g. num_dwells in pcps_acquisition_fine_doppler_cc
// dumps).
template <typename T>
void read_scalar_or(mat_t* matfile, const char* name, T& value, T fallback)
{
    matvar_t* var = Mat_VarRead(matfile, name);
    if (var == nullptr)
        {
            value = fallback;
            return;
        }
    value = *static_cast<T*>(var->data);
    Mat_VarFree(var);
}
}  // namespace


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
    if (var_->data_type != MAT_T_SINGLE)
        {
            std::cout << "Invalid Acquisition dump file: data type error\n";
            Mat_VarFree(var_);
            Mat_Close(matfile);
            return false;
        }

    // Read the Doppler metadata before validating the grid dimensions: dumps
    // written by an assisted (narrowed) acquisition carry two grid columns (the
    // assisted candidate and its noise reference) encoded as doppler_max = 0,
    // doppler_step = <configured doppler_max>, so the expected column count
    // follows from the file's own metadata rather than from the full-grid
    // parameters this reader was constructed with.
    matvar_t* var2_ = Mat_VarRead(matfile, "doppler_max");
    if (var2_ == nullptr)
        {
            std::cout << "Unreachable doppler_max variable in Acquisition dump file.\n";
            Mat_VarFree(var_);
            Mat_Close(matfile);
            return false;
        }
    d_doppler_max = *static_cast<unsigned int*>(var2_->data);
    Mat_VarFree(var2_);

    var2_ = Mat_VarRead(matfile, "doppler_step");
    if (var2_ == nullptr)
        {
            std::cout << "Unreachable doppler_step variable in Acquisition dump file.\n";
            Mat_VarFree(var_);
            Mat_Close(matfile);
            return false;
        }
    d_doppler_step = *static_cast<unsigned int*>(var2_->data);
    Mat_VarFree(var2_);

    // Optional variables, absent from dumps written by older versions and by
    // blocks that never narrow (e.g. pcps_acquisition_fine_doppler_cc): their
    // absence means a full grid centered at 0 Hz.
    read_scalar_or(matfile, "doppler_center", doppler_center, 0);
    int32_t narrowed_flag = 0;
    read_scalar_or(matfile, "doppler_narrowed", narrowed_flag, static_cast<int32_t>(0));
    doppler_narrowed = (narrowed_flag != 0);

    if (d_doppler_step == 0)
        {
            d_doppler_step = 1;
        }
    if (doppler_narrowed)
        {
            d_num_doppler_bins = 2U;
        }
    else
        {
            // pcps_acquisition sizes its grid with ceil(2*doppler_max/doppler_step),
            // but pcps_acquisition_fine_doppler_cc sizes it with floor(): accept
            // either count when they differ.
            const double bins_exact = static_cast<double>(2 * d_doppler_max) / static_cast<double>(d_doppler_step);
            d_num_doppler_bins = static_cast<unsigned int>(std::ceil(bins_exact));
            const auto bins_floor = static_cast<unsigned int>(std::floor(bins_exact));
            if ((var_->dims[1] == bins_floor) && (bins_floor != 0U))
                {
                    d_num_doppler_bins = bins_floor;
                }
        }

    if ((var_->dims[0] != d_samples_per_code) || (var_->dims[1] != d_num_doppler_bins))
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

    // Rebuild the Doppler axis and the magnitude storage from the file's
    // metadata. doppler(i) = -doppler_max + doppler_center + doppler_step * i
    // is the same decoding pcps_acquisition::compute_statistics() uses, and it
    // maps a narrowed dump's two columns to {doppler_center, doppler_center +
    // configured doppler_max}.
    doppler.clear();
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            doppler.push_back(-static_cast<int>(d_doppler_max) + doppler_center + static_cast<int>(d_doppler_step) * static_cast<int>(doppler_index));
        }
    mag.assign(d_num_doppler_bins, std::vector<float>(d_samples_per_code));

    read_scalar_or(matfile, "input_power", input_power, 0.0F);
    read_scalar_or(matfile, "acq_doppler_hz", acq_doppler_hz, 0.0F);
    read_scalar_or(matfile, "acq_delay_samples", acq_delay_samples, 0.0F);
    read_scalar_or(matfile, "test_statistic", test_statistic, 0.0F);
    read_scalar_or(matfile, "threshold", threshold, 0.0F);
    read_scalar_or(matfile, "sample_counter", sample_counter, static_cast<uint64_t>(0));

    var2_ = Mat_VarRead(matfile, "positive_acq");
    if (var2_ == nullptr)
        {
            var2_ = Mat_VarRead(matfile, "d_positive_acq");
        }
    if (var2_ == nullptr)
        {
            std::cout << "Unreachable positive acquisition variable in Acquisition dump file.\n";
            Mat_VarFree(var_);
            Mat_Close(matfile);
            return false;
        }
    positive_acq = *static_cast<int*>(var2_->data);
    Mat_VarFree(var2_);

    read_scalar_or(matfile, "num_dwells", num_dwells, 0U);
    read_scalar_or(matfile, "PRN", PRN, 0U);

    std::vector<std::vector<float> >::iterator it1;
    std::vector<float>::iterator it2;
    auto* aux = static_cast<float*>(var_->data);
    int k = 0;
    float normalization_factor = std::pow(d_samples_per_code, 4) * input_power;
    if (!(normalization_factor > 0.0F))
        {
            // Non-CFAR pcps dumps and fine-doppler dumps store input_power = 0:
            // keep the raw grid values instead of dividing by zero.
            normalization_factor = 1.0F;
        }
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

    // The dump filename embeds the PRN, which is exactly what this constructor
    // has to discover, so probe the candidate PRNs for an existing file
    // (1..210 covers every supported system, QZSS included).
    mat_t* matfile = nullptr;
    for (unsigned int candidate_sat = 1; (candidate_sat <= 210) && (matfile == nullptr); candidate_sat++)
        {
            const std::string candidate_filename = basename + "_ch_" + std::to_string(channel) + "_" + std::to_string(execution) + "_sat_" + std::to_string(candidate_sat) + ".mat";
            matfile = Mat_Open(candidate_filename.c_str(), MAT_ACC_RDONLY);
            if (matfile != nullptr)
                {
                    sat_ = candidate_sat;
                }
        }
    if (matfile != nullptr)
        {
            matvar_t* var_ = Mat_VarRead(matfile, "doppler_max");
            if (var_ != nullptr)
                {
                    doppler_max_ = *static_cast<unsigned int*>(var_->data);
                    Mat_VarFree(var_);
                }

            var_ = Mat_VarRead(matfile, "doppler_step");
            if (var_ != nullptr)
                {
                    doppler_step_ = *static_cast<unsigned int*>(var_->data);
                    Mat_VarFree(var_);
                }

            var_ = Mat_VarRead(matfile, "acq_grid");
            if (var_ != nullptr)
                {
                    samples_per_code_ = var_->dims[0];
                    Mat_VarFree(var_);
                }

            Mat_Close(matfile);
        }
    else
        {
            std::cout << "Unreachable Acquisition dump file " << basename << "_ch_" << channel << "_" << execution << "_sat_<PRN>.mat\n";
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
    mag = std::move(mag_aux);
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
