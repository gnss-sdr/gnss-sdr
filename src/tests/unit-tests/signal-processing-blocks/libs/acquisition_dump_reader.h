/*!
 * \file acquisition_dump_reader.h
 * \brief Helper file for unit testing
 * \authors Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
 *                    Antonio Ramos, 2018. antonio.ramos(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
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

    ~Acquisition_Dump_Reader() = default;

    Acquisition_Dump_Reader(Acquisition_Dump_Reader&& other) noexcept;             //!< Copy constructor
    Acquisition_Dump_Reader& operator=(const Acquisition_Dump_Reader&);            //!< Copy assignment operator
    Acquisition_Dump_Reader(const Acquisition_Dump_Reader& other) noexcept;        //!< Move constructor
    Acquisition_Dump_Reader& operator=(Acquisition_Dump_Reader&& other) noexcept;  //!< Move assignment operator

    bool read_binary_acq();

    std::vector<int> doppler;
    std::vector<unsigned int> samples;
    std::vector<std::vector<float> > mag;
    float acq_doppler_hz;
    float acq_delay_samples;
    float test_statistic;
    float input_power;
    float threshold;
    int positive_acq;
    unsigned int PRN;
    unsigned int num_dwells;
    uint64_t sample_counter;

private:
    std::string d_basename;
    unsigned int d_sat;
    unsigned int d_doppler_max;
    unsigned int d_doppler_step;
    unsigned int d_samples_per_code;
    unsigned int d_num_doppler_bins;
    std::string d_dump_filename;
};

#endif  // GNSS_SDR_ACQUISITION_DUMP_READER_H
