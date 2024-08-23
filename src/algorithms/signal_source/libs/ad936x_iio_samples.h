/*!
 * \file ad936x_iio_samples.h
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


#ifndef GNSS_SDR_AD936X_IIO_SAMPLES_H
#define GNSS_SDR_AD936X_IIO_SAMPLES_H

#define IIO_DEFAULTAD936XAPIFIFOSIZE_SAMPLES 32768 * 4
#define IIO_INPUTRAMFIFOSIZE 256

#include <memory>
#include <stdint.h>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */

class ad936x_iio_samples
{
public:
    ad936x_iio_samples() = default;
    uint32_t n_bytes{0};
    uint32_t n_interleaved_iq_samples{0};
    uint16_t n_channels{0};
    uint16_t step_bytes{0};
    char buffer[IIO_DEFAULTAD936XAPIFIFOSIZE_SAMPLES * 4 * 4];  // max 16 bits samples per buffer (4 channels, 2-bytes per I + 2-bytes per Q)
};

/** \} */
/** \} */
#endif
