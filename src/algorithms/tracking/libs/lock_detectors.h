/*!
 * \file lock_detectors.h
 * \brief Interface of a library with a set of code and carrier phase lock detectors.
 *
 * SNV_CN0 is a Carrier-to-Noise (CN0) estimator
 * based on the Signal-to-Noise Variance (SNV) estimator [1].
 * Carrier lock detector using normalised estimate of the cosine
 * of twice the carrier phase error [2].
 *
 * [1] Marco Pini, Emanuela Falletti and Maurizio Fantino, "Performance
 * Evaluation of C/N0 Estimators using a Real Time GNSS Software Receiver,"
 * IEEE 10th International Symposium on Spread Spectrum Techniques and
 * Applications, pp.28-30, August 2008.
 *
 * [2] Van Dierendonck, A.J. (1996), Global Positioning System: Theory and
 * Applications,
 * Volume I, Chapter 8: GPS Receivers, AJ Systems, Los Altos, CA 94024.
 * Inc.: 329-407.
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
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

#ifndef GNSS_SDR_LOCK_DETECTORS_H
#define GNSS_SDR_LOCK_DETECTORS_H

#include <gnuradio/gr_complex.h>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


/*! \brief cn0_svn_estimator is a Carrier-to-Noise (CN0) estimator
 * based on the Signal-to-Noise Variance (SNV) estimator
 *
 * Signal-to-Noise (SNR) (\f$ \rho \f$) estimator using the Signal-to-Noise Variance (SNV) estimator:
 * \f{equation}
 *  \hat{\rho}=\frac{\hat{P}_s}{\hat{P}_n}=\frac{\hat{P}_s}{\hat{P}_{tot}-\hat{P}_s},
 * \f}
 *  where \f$ \hat{P}_s=\left(\frac{1}{N}\sum^{N-1}_{i=0}|Re(Pc(i))|\right)^2 \f$ is the estimation of the signal power,
 * \f$ \hat{P}_{tot}=\frac{1}{N}\sum^{N-1}_{i=0}|Pc(i)|^2 \f$ is the estimator of the total power, \f$ |\cdot| \f$ is the absolute value,
 * \f$ Re(\cdot) \f$ stands for the real part of the value, and \f$ Pc(i) \f$ is the prompt correlator output for the sample index i.
 *
 * The SNR value is converted to CN0 [dB-Hz], taking into account the coherent integration time, using the following formula:
 * \f{equation}
 *     CN0_{dB}=10*log(\hat{\rho})-10*log(T_{int}),
 * \f}
 * where \f$ T_{int} \f$ is the coherent integration time, in seconds.
 *
 * Ref: Marco Pini, Emanuela Falletti and Maurizio Fantino, "Performance
 * Evaluation of C/N0 Estimators using a Real Time GNSS Software Receiver,"
 * IEEE 10th International Symposium on Spread Spectrum Techniques and
 * Applications, pp.28-30, August 2008.
 */
float cn0_svn_estimator(const gr_complex* Prompt_buffer, int length, float coh_integration_time_s);


/*! \brief cn0_m2m4_estimator is a Carrier-to-Noise (CN0) estimator
 * based on the Second- and Fourth-Order Moments Method (M2M4)
 *
 * Signal-to-Noise (SNR) (\f$ \rho \f$) estimator using the Moments Method:
 * \f{equation}
 *  \hat{\rho}=\frac{\sqrt{2 \hat{M}_2^2 - \hat{M}_4 }}{\hat{M}_2-\sqrt{2 \hat{M}_2^2 - \hat{M}_4 }},
 * \f}
 * where
 * \f$ \hat{M}_2=\frac{1}{N}\sum^{K-1}_{k=0}|P[k]|^2 \f$, \f$ \hat{M}_4 = \frac{1}{K}\sum^{K-1}_{k=0}|P[k]|^4 \f$, \f$ |\cdot| \f$ is the absolute value,
 * and \f$ P[k] \f$ is the prompt correlator output for the sample index k.
 *
 * The SNR value is converted to CN0 [dB-Hz] taking into account the coherent integration time, using the following formula:
 * \f{equation}
 *     CN0_{dB}=10*log(\hat{\rho})-10*log(T_{int}),
 * \f}
 * where \f$ T_{int} \f$ is the coherent integration time, in seconds.
 *
 * Ref: D. R. Pauluzzi, N. C. Beaulieu, "A comparison of SNR estimation
 * techniques for the AWGN channel," IEEE Trans. on Comm., vol. 48,
 * no. 10, pp. 1681–1691, Oct. 2000.
 */
float cn0_m2m4_estimator(const gr_complex* Prompt_buffer, int length, float coh_integration_time_s);


/*! \brief A carrier lock detector
 *
 * The Carrier Phase Lock Detector block uses the estimate of the cosine of twice the carrier phase error is given by
 * \f{equation}
 *     C2\phi=\frac{NBD}{NBP},
 * \f}
 *  where \f$ NBD=(\sum^{N-1}_{i=0}|Im(Pc(i))|)^2+(\sum^{N-1}_{i=0}|Re(Pc(i))|)^2 \f$,
 *  \f$ NBP=\sum^{N-1}_{i=0}Im(Pc(i))^2-\sum^{N-1}_{i=0}Re(Pc(i))^2 \f$, and
 *  \f$ Pc(i) \f$ is the prompt correlator output for the sample index i.
 * Ref: Van Dierendonck, A.J. (1996), Global Positioning System: Theory and
 * Applications,
 * Volume I, Chapter 8: GPS Receivers, AJ Systems, Los Altos, CA 94024.
 * Inc.: 329-407.
 */
float carrier_lock_detector(const gr_complex* Prompt_buffer, int length);


/** \} */
/** \} */
#endif  // GNSS_SDR_LOCK_DETECTORS_H
