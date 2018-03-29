/*!
 * \file tracking_loop_filter.h
 * \brief Generic 1st to 3rd order loop filter implementation
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
 *
 * Class implementing a generic 1st, 2nd or 3rd order loop filter. Based
 * on the bilinear transform of the standard Weiner filter.
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

#ifndef GNSS_SDR_TRACKING_LOOP_FILTER_H_
#define GNSS_SDR_TRACKING_LOOP_FILTER_H_
#define MAX_LOOP_ORDER 3
#define MAX_LOOP_HISTORY_LENGTH 4

#include <vector>


/*!
 * \brief This class implements a generic 1st, 2nd or 3rd order loop filter
 *
 */
class Tracking_loop_filter
{
private:
    // Store the last inputs and outputs:
    std::vector<float> d_inputs;
    std::vector<float> d_outputs;

    // Store the filter coefficients:
    std::vector<float> d_input_coefficients;
    std::vector<float> d_output_coefficients;

    // The loop order:
    int d_loop_order;

    // The current index in the i/o arrays:
    int d_current_index;

    // Should the last integrator be included?
    bool d_include_last_integrator;

    // The noise bandwidth (in Hz)
    // Note this is an approximation only valid when the product of this
    // number and the update interval (T) is small.
    float d_noise_bandwidth;

    // Loop update interval
    float d_update_interval;

    // Compute the filter coefficients:
    void update_coefficients(void);


public:
    float get_noise_bandwidth(void) const;
    float get_update_interval(void) const;
    bool get_include_last_integrator(void) const;
    int get_order(void) const;

    void set_noise_bandwidth(float noise_bandwidth);
    void set_update_interval(float update_interval);
    void set_include_last_integrator(bool include_last_integrator);
    void set_order(int loop_order);

    void initialize(float initial_output = 0.0);
    float apply(float current_input);

    Tracking_loop_filter(float update_interval, float noise_bandwidth,
        int loop_order = 2,
        bool include_last_integrator = false);

    Tracking_loop_filter();
    ~Tracking_loop_filter();
};

#endif
