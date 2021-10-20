/*!
 * \file exponential_smoother.h
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


#ifndef GNSS_SDR_EXPONENTIAL_SMOOTHER_H
#define GNSS_SDR_EXPONENTIAL_SMOOTHER_H

#include <vector>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


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
    Exponential_Smoother();             //!< Constructor
    ~Exponential_Smoother() = default;  //!< Destructor

    Exponential_Smoother(Exponential_Smoother&&) = default;                       //!< Move operator
    Exponential_Smoother& operator=(Exponential_Smoother&& /*other*/) = default;  //!< Move assignment operator

    void set_alpha(float alpha);                           //!< 0 < alpha < 1. The higher, the most responsive, but more variance. Default value: 0.001
    void set_samples_for_initialization(int num_samples);  //!< Number of samples averaged for initialization. Default value: 200
    void reset();
    void set_min_value(float value);
    void set_offset(float offset);
    float smooth(float raw);
    double smooth(double raw);

private:
    std::vector<float> init_buffer_;
    float alpha_{0.001};
    float one_minus_alpha_{0.999};
    float old_value_{0.0};
    float min_value_{25.0};
    float offset_{12.0};
    int samples_for_initialization_{200};
    int init_counter_{0};
    bool initializing_{true};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_EXPONENTIAL_SMOOTHER_H
