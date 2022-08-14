/*!
 * \file ad936x_iio_samples.cc
 * \brief A class that holds a custom sample buffer for Analog Devices AD936x family front-ends.
 * \author Javier Arribas, jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "ad936x_iio_samples.h"

ad936x_iio_samples::ad936x_iio_samples()
{
    for (int n = 0; n < IIO_MAX_CH; n++)
        {
            n_bytes[n] = 0;
            n_samples[n] = 0;
        }
}
