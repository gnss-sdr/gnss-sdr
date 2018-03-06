/*!
 * \file signal_generator_c.h
 * \brief GNU Radio source block that generates synthesized GNSS signal.
 * \author Marc Molina, 2013. marc.molina.pena@gmail.com
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

#ifndef GNSS_SDR_SIGNAL_GENERATOR_C_H
#define GNSS_SDR_SIGNAL_GENERATOR_C_H

#include "gnss_signal.h"
#include <boost/scoped_array.hpp>
#include <gnuradio/random.h>
#include <gnuradio/block.h>
#include <string>
#include <vector>
#include <random>


class signal_generator_c;

/*
* We use boost::shared_ptr's instead of raw pointers for all access
* to gr_blocks (and many other data structures). The shared_ptr gets
* us transparent reference counting, which greatly simplifies storage
* management issues.
*
* See http://www.boost.org/libs/smart_ptr/smart_ptr.htm
*
* As a convention, the _sptr suffix indicates a boost::shared_ptr
*/
typedef boost::shared_ptr<signal_generator_c> signal_generator_c_sptr;

/*!
* \brief Return a shared_ptr to a new instance of gen_source.
*
* To avoid accidental use of raw pointers, gen_source's
* constructor is private. signal_make_generator_c is the public
* interface for creating new instances.
*/
signal_generator_c_sptr
signal_make_generator_c(std::vector<std::string> signal1, std::vector<std::string> system, const std::vector<unsigned int> &PRN,
    const std::vector<float> &CN0_dB, const std::vector<float> &doppler_Hz,
    const std::vector<unsigned int> &delay_chips, const std::vector<unsigned int> &delay_sec, bool data_flag, bool noise_flag,
    unsigned int fs_in, unsigned int vector_length, float BW_BB);

/*!
* \brief This class generates synthesized GNSS signal.
* \ingroup block
*
* \sa gen_source for a version that subclasses gr_block.
*/
class signal_generator_c : public gr::block
{
private:
    // The friend declaration allows gen_source to
    // access the private constructor.

    /* Create the signal_generator_c object*/
    friend signal_generator_c_sptr
    signal_make_generator_c(std::vector<std::string> signal1, std::vector<std::string> system, const std::vector<unsigned int> &PRN,
        const std::vector<float> &CN0_dB, const std::vector<float> &doppler_Hz,
        const std::vector<unsigned int> &delay_chips, const std::vector<unsigned int> &delay_sec, bool data_flag, bool noise_flag,
        unsigned int fs_in, unsigned int vector_length, float BW_BB);

    signal_generator_c(std::vector<std::string> signal1, std::vector<std::string> system, const std::vector<unsigned int> &PRN,
        const std::vector<float> &CN0_dB, const std::vector<float> &doppler_Hz,
        const std::vector<unsigned int> &delay_chips, const std::vector<unsigned int> &delay_sec, bool data_flag, bool noise_flag,
        unsigned int fs_in, unsigned int vector_length, float BW_BB);

    void init();
    void generate_codes();

    std::vector<std::string> signal_;
    std::vector<std::string> system_;
    std::vector<unsigned int> PRN_;
    std::vector<float> CN0_dB_;
    std::vector<float> doppler_Hz_;
    std::vector<unsigned int> delay_chips_;
    std::vector<unsigned int> delay_sec_;
    bool data_flag_;
    bool noise_flag_;
    unsigned int fs_in_;
    unsigned int num_sats_;
    unsigned int vector_length_;
    float BW_BB_;

    std::vector<unsigned int> samples_per_code_;
    std::vector<unsigned int> num_of_codes_per_vector_;
    std::vector<unsigned int> data_bit_duration_ms_;
    std::vector<unsigned int> ms_counter_;
    std::vector<float> start_phase_rad_;
    std::vector<gr_complex> current_data_bits_;
    std::vector<signed int> current_data_bit_int_;
    std::vector<signed int> data_modulation_;
    std::vector<signed int> pilot_modulation_;

    boost::scoped_array<gr_complex *> sampled_code_data_;
    boost::scoped_array<gr_complex *> sampled_code_pilot_;
    gr::random *random_;
    gr_complex *complex_phase_;

    unsigned int work_counter_;
    std::random_device r;
    std::default_random_engine e1;
    std::uniform_int_distribution<int> uniform_dist;

public:
    ~signal_generator_c();  // public destructor

    // Where all the action really happens

    int general_work(int noutput_items,
        gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif /* GNSS_SDR_SIGNAL_GENERATOR_C_H */
