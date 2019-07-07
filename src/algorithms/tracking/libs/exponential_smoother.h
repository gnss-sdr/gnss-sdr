/*!
 * \file exponential_smoother.h
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


#ifndef GNSS_SDR_EXPONENTIAL_SMOOTHER_H_
#define GNSS_SDR_EXPONENTIAL_SMOOTHER_H_

#include <vector>

/*! \brief
 * Class that implements a first-order exponential smoother.
 *
 * smoothed_value[k] = alpha * raw + (1-alpha) * smoothed_value[k-1]
 *
 * The length of the initialization can be controlled with
 * set_samples_for_initialization(int num_samples)
 */
class Exponential_Smoother
{
public:
    Exponential_Smoother();                                //!< Constructor
    ~Exponential_Smoother() = default;                     //!< Destructor
    void set_alpha(float alpha);                           //!< 0 < alpha < 1. The higher, the most responsive, but more variance. Default value: 0.001
    void set_samples_for_initialization(int num_samples);  //!< Number of samples averaged for initialization. Default value: 200
    void reset();
    void set_min_value(float value);
    void set_offset(float offset);
    float smooth(float raw);
    double smooth(double raw);
    Exponential_Smoother(Exponential_Smoother&&) = default;                       //!< Move operator
    Exponential_Smoother& operator=(Exponential_Smoother&& /*other*/) = default;  //!< Move assignment operator
private:
    float alpha_;  // takes value 0.0001 if not set
    int samples_for_initialization_;
    float one_minus_alpha_;
    float old_value_;
    float min_value_;
    float offset_;
    bool initializing_;
    int init_counter_;
    std::vector<float> init_buffer_;
};

#endif  // GNSS_SDR_EXPONENTIAL_SMOOTHER_H_
