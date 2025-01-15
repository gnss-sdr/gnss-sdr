/*!
 * \file exponential_smoother.cc
 * \brief Class that implements an exponential smoother
 * \authors Carles Fernandez, 2019 cfernandez@cttc.es
 *
 * Class that implements a first-order exponential smoother.
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

#include "exponential_smoother.h"
#include <iterator>
#include <numeric>

Exponential_Smoother::Exponential_Smoother()
{
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
    one_minus_alpha_ = 1.0F - alpha_;
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
