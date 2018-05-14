/*!
 * \file tracking_loop_filter.cc
 * \brief Generic 1st to 3rd order loop filter implementation
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
 *
 * Class implementing a generic 1st, 2nd or 3rd order loop filter. Based
 * on the bilinear transform of the standard Weiner filter.
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "tracking_loop_filter.h"
#include <cmath>
#include <glog/logging.h>


Tracking_loop_filter::Tracking_loop_filter(float update_interval,
    float noise_bandwidth,
    int loop_order,
    bool include_last_integrator)
    : d_loop_order(loop_order),
      d_current_index(0),
      d_include_last_integrator(include_last_integrator),
      d_noise_bandwidth(noise_bandwidth),
      d_update_interval(update_interval)
{
    d_inputs.resize(MAX_LOOP_HISTORY_LENGTH, 0.0);
    d_outputs.resize(MAX_LOOP_HISTORY_LENGTH, 0.0);
    update_coefficients();
}

Tracking_loop_filter::Tracking_loop_filter()
    : d_loop_order(2),
      d_current_index(0),
      d_include_last_integrator(false),
      d_noise_bandwidth(15.0),
      d_update_interval(0.001)
{
    d_inputs.resize(MAX_LOOP_HISTORY_LENGTH, 0.0);
    d_outputs.resize(MAX_LOOP_HISTORY_LENGTH, 0.0);
    update_coefficients();
}

Tracking_loop_filter::~Tracking_loop_filter()
{
    // Don't need to do anything here
}

float Tracking_loop_filter::apply(float current_input)
{
    // Now apply the filter coefficients:
    float result = 0.0;

    // Hanlde the old outputs first:
    for (unsigned int ii = 0; ii < d_output_coefficients.size(); ++ii)
        {
            result += d_output_coefficients[ii] * d_outputs[(d_current_index + ii) % MAX_LOOP_HISTORY_LENGTH];
        }

    // Now update the index to handle the inputs.
    // DO NOT CHANGE THE ORDER OF THE ABOVE AND BELOW CODE
    // SNIPPETS!!!!!!!

    // Implementing a sort of circular buffer for the inputs and outputs
    // the current input/output is at d_current_index, the nth previous
    // input/output is at (d_current_index+n)%d_loop_order
    d_current_index--;
    if (d_current_index < 0)
        {
            d_current_index += MAX_LOOP_HISTORY_LENGTH;
        }

    d_inputs[d_current_index] = current_input;


    for (unsigned int ii = 0; ii < d_input_coefficients.size(); ++ii)
        {
            result += d_input_coefficients[ii] * d_inputs[(d_current_index + ii) % MAX_LOOP_HISTORY_LENGTH];
        }


    d_outputs[d_current_index] = result;


    return result;
}

void Tracking_loop_filter::update_coefficients(void)
{
    // Analog gains:
    float g1;
    float g2;
    float g3;

    // Natural frequency
    float wn;
    float T = d_update_interval;

    float zeta = 1.0 / std::sqrt(2.0);

    // The following is based on the bilinear transform approximation of
    // the analog integrator. The loop format is from Kaplan & Hegarty
    // Table 5.6. The basic concept is that the loop has a cascade of
    // integrators:
    // 1 for a 1st order loop
    // 2 for a 2nd order loop
    // 3 for a 3rd order loop
    // The bilinear transform approximates 1/s as
    // T/2(1 + z^-1)/(1-z^-1) in the z domain.

    switch (d_loop_order)
        {
        case 1:
            wn = d_noise_bandwidth * 4.0;
            g1 = wn;
            if (d_include_last_integrator)
                {
                    d_input_coefficients.resize(2);
                    d_input_coefficients[0] = g1 * T / 2.0;
                    d_input_coefficients[1] = g1 * T / 2.0;

                    d_output_coefficients.resize(1);
                    d_output_coefficients[0] = 1.0;
                }
            else
                {
                    d_input_coefficients.resize(1);
                    d_input_coefficients[0] = g1;

                    d_output_coefficients.resize(0);
                }
            break;
        case 2:
            wn = d_noise_bandwidth * (8.0 * zeta) / (4.0 * zeta * zeta + 1.0);
            g1 = wn * wn;
            g2 = wn * 2.0 * zeta;
            if (d_include_last_integrator)
                {
                    d_input_coefficients.resize(3);
                    d_input_coefficients[0] = T / 2.0 * (g1 * T / 2.0 + g2);
                    d_input_coefficients[1] = T * T / 2.0 * g1;
                    d_input_coefficients[2] = T / 2.0 * (g1 * T / 2.0 - g2);

                    d_output_coefficients.resize(2);
                    d_output_coefficients[0] = 2.0;
                    d_output_coefficients[1] = -1.0;
                }
            else
                {
                    d_input_coefficients.resize(2);
                    d_input_coefficients[0] = (g1 * T / 2.0 + g2);
                    d_input_coefficients[1] = g1 * T / 2.0 - g2;

                    d_output_coefficients.resize(1);
                    d_output_coefficients[0] = 1.0;
                }
            break;

        case 3:
            wn = d_noise_bandwidth / 0.7845;  // From Kaplan
            float a3 = 1.1;
            float b3 = 2.4;
            g1 = wn * wn * wn;
            g2 = a3 * wn * wn;
            g3 = b3 * wn;

            if (d_include_last_integrator)
                {
                    d_input_coefficients.resize(4);
                    d_input_coefficients[0] = T / 2.0 * (g3 + T / 2.0 * (g2 + T / 2.0 * g1));
                    d_input_coefficients[1] = T / 2.0 * (-g3 + T / 2.0 * (g2 + 3.0 * T / 2.0 * g1));
                    d_input_coefficients[2] = T / 2.0 * (-g3 - T / 2.0 * (g2 - 3.0 * T / 2.0 * g1));
                    d_input_coefficients[3] = T / 2.0 * (g3 - T / 2.0 * (g2 - T / 2.0 * g1));

                    d_output_coefficients.resize(3);
                    d_output_coefficients[0] = 3.0;
                    d_output_coefficients[1] = -3.0;
                    d_output_coefficients[2] = 1.0;
                }
            else
                {
                    d_input_coefficients.resize(3);
                    d_input_coefficients[0] = g3 + T / 2.0 * (g2 + T / 2.0 * g1);
                    d_input_coefficients[1] = g1 * T * T / 2.0 - 2.0 * g3;
                    d_input_coefficients[2] = g3 + T / 2.0 * (-g2 + T / 2.0 * g1);


                    d_output_coefficients.resize(2);
                    d_output_coefficients[0] = 2.0;
                    d_output_coefficients[1] = -1.0;
                }
            break;
        };
}

void Tracking_loop_filter::set_noise_bandwidth(float noise_bandwidth)
{
    d_noise_bandwidth = noise_bandwidth;
    update_coefficients();
}

float Tracking_loop_filter::get_noise_bandwidth(void) const
{
    return d_noise_bandwidth;
}

void Tracking_loop_filter::set_update_interval(float update_interval)
{
    d_update_interval = update_interval;
    update_coefficients();
}

float Tracking_loop_filter::get_update_interval(void) const
{
    return d_update_interval;
}

void Tracking_loop_filter::set_include_last_integrator(bool include_last_integrator)
{
    d_include_last_integrator = include_last_integrator;
    update_coefficients();
}

bool Tracking_loop_filter::get_include_last_integrator(void) const
{
    return d_include_last_integrator;
}

void Tracking_loop_filter::set_order(int loop_order)
{
    if (loop_order < 1 or loop_order > MAX_LOOP_ORDER)
        {
            LOG(ERROR) << "Ignoring attempt to set loop order to " << loop_order
                       << ". Maximum allowed order is: " << MAX_LOOP_ORDER
                       << ". Not changing current value of " << d_loop_order;

            return;
        }

    d_loop_order = loop_order;
    update_coefficients();
}

int Tracking_loop_filter::get_order(void) const
{
    return d_loop_order;
}

void Tracking_loop_filter::initialize(float initial_output)
{
    d_inputs.assign(MAX_LOOP_HISTORY_LENGTH, 0.0);
    d_outputs.assign(MAX_LOOP_HISTORY_LENGTH, initial_output);
    d_current_index = MAX_LOOP_HISTORY_LENGTH - 1;
}
