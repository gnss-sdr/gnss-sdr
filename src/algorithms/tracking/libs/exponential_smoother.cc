/*!
 * \file exponential_smoother.cc
 * \brief Class that implements an exponential smoother
 * \authors Carles Fernandez, 2019 cfernandez@cttc.es
 *
 * Class that implements a first-order exponential smoother.
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "exponential_smoother.h"
#include <iterator>
#include <numeric>

Exponential_Smoother::Exponential_Smoother()
{
    alpha_ = 0.001;
    old_value_ = 0.0;
    one_minus_alpha_ = 1.0 - alpha_;
    samples_for_initialization_ = 200;
    initializing_ = true;
    init_counter_ = 0;
    min_value_ = 25.0;
    offset_ = 12.0;
    init_buffer_.reserve(samples_for_initialization_);
}


void Exponential_Smoother::set_alpha(float alpha)
{
    alpha_ = alpha;
    if (alpha_ < 0)
        {
            alpha_ = 0;
        }
    if (alpha_ > 1)
        {
            alpha_ = 1;
        }
    one_minus_alpha_ = 1.0 - alpha_;
}


void Exponential_Smoother::set_offset(float offset)
{
    offset_ = offset;
}


void Exponential_Smoother::set_samples_for_initialization(int num_samples)
{
    int ns = num_samples;
    if (ns <= 0)
        {
            ns = 1;
        }
    samples_for_initialization_ = ns;
    init_buffer_.reserve(samples_for_initialization_);
}


void Exponential_Smoother::reset()
{
    initializing_ = true;
    init_counter_ = 0;
    init_buffer_.clear();
}


void Exponential_Smoother::set_min_value(float value)
{
    min_value_ = value;
}


double Exponential_Smoother::smooth(double raw)
{
    auto raw_f = static_cast<float>(raw);
    auto smooth_d = static_cast<double>((this)->smooth(raw_f));
    return smooth_d;
}


float Exponential_Smoother::smooth(float raw)
{
    float smoothed_value;
    if (initializing_ == true)
        {
            init_counter_++;
            smoothed_value = raw;
            init_buffer_.push_back(smoothed_value);
            if (init_counter_ == samples_for_initialization_)
                {
                    old_value_ = std::accumulate(std::begin(init_buffer_), std::end(init_buffer_), 0.0F) / static_cast<float>(init_buffer_.size());
                    if (old_value_ < (min_value_ + offset_))
                        {
                            // flush buffer and start again
                            init_counter_ = 0;
                            init_buffer_.clear();
                        }
                    else
                        {
                            initializing_ = false;
                        }
                }
        }
    else
        {
            smoothed_value = alpha_ * raw + one_minus_alpha_ * old_value_;
            old_value_ = smoothed_value;
        }
    return smoothed_value;
}
