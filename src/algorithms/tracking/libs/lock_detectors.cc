/*!
 * \file lock_detectors.cc
 * \brief Implementation of a library with a set of code and carrier phase lock detectors.
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

#include "lock_detectors.h"
#include <cmath>

/*
 * Signal-to-Noise (SNR) (\f$\rho\f$) estimator using the Signal-to-Noise Variance (SNV) estimator:
 * \f{equation}
 *     \hat{\rho}=\frac{\hat{P}_s}{\hat{P}_n}=\frac{\hat{P}_s}{\hat{P}_{tot}-\hat{P}_s},
 * \f}
 *  where \f$\hat{P}_s=\left(\frac{1}{N}\sum^{N-1}_{i=0}|Re(Pc(i))|\right)^2\f$ is the estimation of the signal power,
 * \f$\hat{P}_{tot}=\frac{1}{N}\sum^{N-1}_{i=0}|Pc(i)|^2\f$ is the estimator of the total power, \f$|\cdot|\f$ is the absolute value,
 * \f$Re(\cdot)\f$ stands for the real part of the value, and \f$Pc(i)\f$ is the prompt correlator output for the sample index i.
 *
 * The SNR value is converted to CN0 [dB-Hz], taking to account the coherent integration time, using the following formula:
 * \f{equation}
 *     CN0_{dB}=10*log(\hat{\rho})-10*log(T_{int}),
 * \f}
 * where \f$T_{int}\f$ is the coherent integration time, in seconds.
 *
 */
float cn0_svn_estimator(const gr_complex* Prompt_buffer, int length, float coh_integration_time_s)
{
    float SNR = 0.0;
    float SNR_dB_Hz = 0.0;
    float Psig = 0.0;
    float Ptot = 0.0;
    for (int i = 0; i < length; i++)
        {
            Psig += std::abs(Prompt_buffer[i].real());
            Ptot += Prompt_buffer[i].imag() * Prompt_buffer[i].imag() + Prompt_buffer[i].real() * Prompt_buffer[i].real();
        }
    Psig /= static_cast<float>(length);
    Psig = Psig * Psig;
    Ptot /= static_cast<float>(length);
    SNR = Psig / (Ptot - Psig);
    SNR_dB_Hz = 10.0F * std::log10(SNR) - 10.0F * std::log10(coh_integration_time_s);
    return SNR_dB_Hz;
}


/*
 * Signal-to-Noise (SNR) (\f$\rho\f$) estimator using the Moments Method:
 * \f{equation}
 *  \hat{\rho}=\frac{\hat{P}_s}{\hat{P}_n}=\frac{\sqrt{2*\hat{M}_2^2 - \hat{M}_4 }}{\hat{M}_2-\sqrt{2*\hat{M}_2^2 - \hat{M}_4 }},
 * \f}
 *  where \f$\hat{P}_s=\left(\frac{1}{N}\sum^{N-1}_{i=0}|Re(Pc(i))|\right)^2\f$ is the estimation of the signal power,
 * \f$ \hat{M}_2=\frac{1}{N}\sum^{N-1}_{i=0}|Pc(i)|^2 \f$, \f$\hat{M}_4 = \frac{1}{N}\sum^{N-1}_{i=0}|Pc(i)|^4 \f$, \f$|\cdot|\f$ is the absolute value,
 * \f$Re(\cdot)\f$ stands for the real part of the value, and \f$Pc(i)\f$ is the prompt correlator output for the sample index i.
 *
 * The SNR value is converted to CN0 [dB-Hz], taking to account the coherent integration time, using the following formula:
 * \f{equation}
 *     CN0_{dB}=10*log(\hat{\rho})-10*log(T_{int}),
 * \f}
 * where \f$T_{int}\f$ is the coherent integration time, in seconds.
 *
 */
float cn0_m2m4_estimator(const gr_complex* Prompt_buffer, int length, float coh_integration_time_s)
{
    float SNR_aux = 0.0;
    float SNR_dB_Hz = 0.0;
    float Psig = 0.0;
    float m_2 = 0.0;
    float m_4 = 0.0;
    float aux;
    const auto n = static_cast<float>(length);
    for (int i = 0; i < length; i++)
        {
            Psig += std::abs(Prompt_buffer[i].real());
            aux = Prompt_buffer[i].imag() * Prompt_buffer[i].imag() + Prompt_buffer[i].real() * Prompt_buffer[i].real();
            m_2 += aux;
            m_4 += (aux * aux);
        }
    Psig /= n;
    Psig = Psig * Psig;
    m_2 /= n;
    m_4 /= n;
    aux = std::sqrt(2.0F * m_2 * m_2 - m_4);
    if (std::isnan(aux))
        {
            SNR_aux = Psig / (m_2 - Psig);
        }
    else
        {
            SNR_aux = aux / (m_2 - aux);
        }
    SNR_dB_Hz = 10.0F * std::log10(SNR_aux) - 10.0F * std::log10(coh_integration_time_s);

    return SNR_dB_Hz;
}


/*
 * The estimate of the cosine of twice the carrier phase error is given by
 * \f{equation}
 *     \cos(2\phi)=\frac{NBD}{NBP},
 * \f}
 *  where \f$NBD=(\sum^{N-1}_{i=0}Im(Pc(i)))^2-(\sum^{N-1}_{i=0}Re(Pc(i)))^2\f$,
 *  \f$NBP=(\sum^{N-1}_{i=0}Im(Pc(i)))^2+(\sum^{N-1}_{i=0}Re(Pc(i)))^2\f$, and
 *  \f$Pc(i)\f$ is the prompt correlator output for the sample index i.
 */
float carrier_lock_detector(const gr_complex* Prompt_buffer, int length)
{
    float tmp_sum_I = 0.0;
    float tmp_sum_Q = 0.0;
    float NBD = 0.0;
    float NBP = 0.0;
    for (int i = 0; i < length; i++)
        {
            tmp_sum_I += Prompt_buffer[i].real();
            tmp_sum_Q += Prompt_buffer[i].imag();
        }
    NBP = tmp_sum_I * tmp_sum_I + tmp_sum_Q * tmp_sum_Q;
    NBD = tmp_sum_I * tmp_sum_I - tmp_sum_Q * tmp_sum_Q;
    return NBD / NBP;
}
