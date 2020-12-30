/*!
 * \file tracking_loop_filter.h
 * \brief Generic 1st to 3rd order loop filter implementation
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
 *
 * Class implementing a generic 1st, 2nd or 3rd order loop filter. Based
 * on the bilinear transform of the standard Wiener filter.
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

#ifndef GNSS_SDR_TRACKING_LOOP_FILTER_H
#define GNSS_SDR_TRACKING_LOOP_FILTER_H

#include <vector>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


/*!
 * \brief This class implements a generic 1st, 2nd or 3rd order loop filter
 *
 */
class Tracking_loop_filter
{
public:
    Tracking_loop_filter();
    ~Tracking_loop_filter() = default;

    Tracking_loop_filter(float update_interval, float noise_bandwidth,
        int loop_order = 2,
        bool include_last_integrator = false);

    Tracking_loop_filter(Tracking_loop_filter&&) = default;                       //!< Move operator
    Tracking_loop_filter& operator=(Tracking_loop_filter&& /*other*/) = default;  //!< Move assignment operator

    float get_noise_bandwidth() const;
    float get_update_interval() const;
    bool get_include_last_integrator() const;
    int get_order() const;

    void set_noise_bandwidth(float noise_bandwidth);
    void set_update_interval(float update_interval);
    void set_include_last_integrator(bool include_last_integrator);
    void set_order(int loop_order);

    void initialize(float initial_output = 0.0);
    float apply(float current_input);

private:
    // Compute the filter coefficients:
    void update_coefficients();

    // Store the last inputs and outputs:
    std::vector<float> d_inputs;
    std::vector<float> d_outputs;

    // Store the filter coefficients:
    std::vector<float> d_input_coefficients;
    std::vector<float> d_output_coefficients;

    // The noise bandwidth (in Hz)
    // Note this is an approximation only valid when the product of this
    // number and the update interval (T) is small.
    float d_noise_bandwidth;

    // Loop update interval
    float d_update_interval;

    // The loop order:
    int d_loop_order;

    // The current index in the i/o arrays:
    int d_current_index;

    // Should the last integrator be included?
    bool d_include_last_integrator;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_TRACKING_LOOP_FILTER_H
