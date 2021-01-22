/*!
 * \file gnss_sdr_fft.h
 * \brief Helper file for FFT interface
 * \author Carles Fernandez Prades, 2021. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GNSS_SDR_FFT_H
#define GNSS_SDR_GNSS_SDR_FFT_H

#include "gnss_sdr_make_unique.h"
#include <gnuradio/fft/fft.h>

#if GNURADIO_FFT_USES_TEMPLATES
using gnss_fft_complex_fwd = gr::fft::fft_complex_fwd;
using gnss_fft_complex_rev = gr::fft::fft_complex_rev;
template <typename T>
using gnss_fft_fwd_unique_ptr = std::unique_ptr<T>;
template <typename... Args>
gnss_fft_fwd_unique_ptr<gr::fft::fft_complex_fwd> gnss_fft_fwd_make_unique(Args&&... args)
{
    return std::make_unique<gr::fft::fft_complex_fwd>(std::forward<Args>(args)...);
}
template <typename T>
using gnss_fft_rev_unique_ptr = std::unique_ptr<T>;
template <typename... Args>
gnss_fft_rev_unique_ptr<gr::fft::fft_complex_rev> gnss_fft_rev_make_unique(Args&&... args)
{
    return std::make_unique<gr::fft::fft_complex_rev>(std::forward<Args>(args)...);
}

#else

using gnss_fft_complex_fwd = gr::fft::fft_complex;
using gnss_fft_complex_rev = gr::fft::fft_complex;
template <typename T>
using gnss_fft_fwd_unique_ptr = std::unique_ptr<T>;
template <typename... Args>
gnss_fft_fwd_unique_ptr<gr::fft::fft_complex> gnss_fft_fwd_make_unique(Args&&... args)
{
    return std::make_unique<gr::fft::fft_complex>(std::forward<Args>(args)..., true);
}
template <typename T>
using gnss_fft_rev_unique_ptr = std::unique_ptr<T>;
template <typename... Args>
gnss_fft_rev_unique_ptr<gr::fft::fft_complex> gnss_fft_rev_make_unique(Args&&... args)
{
    return std::make_unique<gr::fft::fft_complex>(std::forward<Args>(args)..., false);
}

#endif

#endif  // GNSS_SDR_GNSS_SDR_FFT_H
