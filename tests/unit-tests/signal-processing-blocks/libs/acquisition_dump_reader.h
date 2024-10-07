/*!
 * \file acquisition_dump_reader.h
 * \brief Helper file for unit testing
 * \authors Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
 *                    Antonio Ramos, 2018. antonio.ramos(at)cttc.es
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

#ifndef GNSS_SDR_ACQUISITION_DUMP_READER_H
#define GNSS_SDR_ACQUISITION_DUMP_READER_H

#include <cstdint>
#include <string>
#include <vector>

class Acquisition_Dump_Reader
{
public:
    Acquisition_Dump_Reader(const std::string& basename,
        unsigned int sat,
        unsigned int doppler_max,
        unsigned int doppler_step,
        unsigned int samples_per_code,
        int channel = 0,
        int execution = 1);

    Acquisition_Dump_Reader(const std::string& basename,
        int channel = 0,
        int execution = 1);

    Acquisition_Dump_Reader(const Acquisition_Dump_Reader& other) = default;       //!< Copy constructor
    Acquisition_Dump_Reader& operator=(const Acquisition_Dump_Reader& other);      //!< Copy assignment operator
    Acquisition_Dump_Reader(Acquisition_Dump_Reader&& other) noexcept;             //!< Move constructor
    Acquisition_Dump_Reader& operator=(Acquisition_Dump_Reader&& other) noexcept;  //!< Move assignment operator

    bool read_binary_acq();

    std::vector<int> doppler;
    std::vector<unsigned int> samples;
    std::vector<std::vector<float> > mag;
    float acq_doppler_hz{};
    float acq_delay_samples{};
    float test_statistic{};
    float input_power{};
    float threshold{};
    int positive_acq{};
    unsigned int PRN{};
    unsigned int num_dwells{};
    uint64_t sample_counter{};

private:
    std::string d_basename;
    std::string d_dump_filename;
    unsigned int d_sat{};
    unsigned int d_doppler_max{};
    unsigned int d_doppler_step{};
    unsigned int d_samples_per_code{};
    unsigned int d_num_doppler_bins{};
};

#endif  // GNSS_SDR_ACQUISITION_DUMP_READER_H
