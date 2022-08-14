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


#ifndef SRC_LIBS_ad936x_iio_samples_H_
#define SRC_LIBS_ad936x_iio_samples_H_

#define IIO_DEFAULTAD936XAPIFIFOSIZE_SAMPLES 32768 * 2

#define IIO_INPUTRAMFIFOSIZE 512

#define IIO_MAX_CH 4
#define IIO_MAX_BYTES_PER_CHANNEL IIO_DEFAULTAD936XAPIFIFOSIZE_SAMPLES * 2 * 2  //(2-bytes per I + 2-bytes per Q)

#include <memory>
#include <stdint.h>
#include <vector>

class ad936x_iio_samples
{
public:
    ad936x_iio_samples();
    uint32_t n_bytes[IIO_MAX_CH];
    uint32_t n_samples[IIO_MAX_CH];
    int16_t buffer[IIO_MAX_CH][IIO_DEFAULTAD936XAPIFIFOSIZE_SAMPLES * 2];  //16 bits I,Q samples buffers
};

#endif
