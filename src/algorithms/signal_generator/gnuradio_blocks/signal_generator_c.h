/*!
 * \file signal_generator_c.h
 * \brief GNU Radio source block that generates synthesized GNSS signal.
 * \author Marc Molina, 2013. marc.molina.pena@gmail.com
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

#ifndef GNSS_SDR_SIGNAL_GENERATOR_C_H
#define GNSS_SDR_SIGNAL_GENERATOR_C_H

#include "gnss_block_interface.h"
#include <gnuradio/block.h>
#include <random>
#include <string>
#include <vector>


class signal_generator_c;

using signal_generator_c_sptr = gnss_shared_ptr<signal_generator_c>;

/*!
* \brief Return a shared_ptr to a new instance of gen_source.
*
* To avoid accidental use of raw pointers, gen_source's
* constructor is private. signal_make_generator_c is the public
* interface for creating new instances.
*/
signal_generator_c_sptr signal_make_generator_c(
    const std::vector<std::string> &signal1,
    const std::vector<std::string> &system,
    const std::vector<unsigned int> &PRN,
    const std::vector<float> &CN0_dB,
    const std::vector<float> &doppler_Hz,
    const std::vector<unsigned int> &delay_chips,
    const std::vector<unsigned int> &delay_sec,
    bool data_flag,
    bool noise_flag,
    unsigned int fs_in,
    unsigned int vector_length,
    float BW_BB);

/*!
* \brief This class generates synthesized GNSS signal.
* \ingroup block
*
* \sa gen_source for a version that subclasses gr_block.
*/
class signal_generator_c : public gr::block
{
public:
    ~signal_generator_c() = default;  // public destructor

    // Where all the action really happens
    int general_work(int noutput_items,
        gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend signal_generator_c_sptr signal_make_generator_c(
        const std::vector<std::string> &signal1,
        const std::vector<std::string> &system,
        const std::vector<unsigned int> &PRN,
        const std::vector<float> &CN0_dB,
        const std::vector<float> &doppler_Hz,
        const std::vector<unsigned int> &delay_chips,
        const std::vector<unsigned int> &delay_sec,
        bool data_flag,
        bool noise_flag,
        unsigned int fs_in,
        unsigned int vector_length,
        float BW_BB);

    signal_generator_c(
        std::vector<std::string> signal1,
        std::vector<std::string> system,
        const std::vector<unsigned int> &PRN,
        std::vector<float> CN0_dB,
        std::vector<float> doppler_Hz,
        std::vector<unsigned int> delay_chips,
        std::vector<unsigned int> delay_sec,
        bool data_flag,
        bool noise_flag,
        unsigned int fs_in,
        unsigned int vector_length,
        float BW_BB);

    void init();

    void generate_codes();

    std::random_device r;
    std::uniform_int_distribution<int> uniform_dist;
    std::normal_distribution<float> normal_dist;
    std::vector<std::string> signal_;
    std::vector<std::string> system_;
    std::vector<std::vector<gr_complex>> sampled_code_data_;
    std::vector<std::vector<gr_complex>> sampled_code_pilot_;
    std::vector<gr_complex> current_data_bits_;
    std::vector<gr_complex> complex_phase_;
    std::vector<float> CN0_dB_;
    std::vector<float> doppler_Hz_;
    std::vector<float> start_phase_rad_;
    std::vector<unsigned int> PRN_;
    std::vector<unsigned int> delay_chips_;
    std::vector<unsigned int> delay_sec_;
    std::vector<unsigned int> samples_per_code_;
    std::vector<unsigned int> num_of_codes_per_vector_;
    std::vector<unsigned int> data_bit_duration_ms_;
    std::vector<unsigned int> ms_counter_;
    std::vector<signed int> current_data_bit_int_;
    std::vector<signed int> data_modulation_;
    std::vector<signed int> pilot_modulation_;
    float BW_BB_;
    unsigned int work_counter_{};
    unsigned int fs_in_;
    unsigned int num_sats_;
    unsigned int vector_length_;
    bool data_flag_;
    bool noise_flag_;
};

#endif  // GNSS_SDR_SIGNAL_GENERATOR_C_H
