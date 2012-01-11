/*!
 * \file CN_estimators.cc
 * \brief Implementation of a library with a set of Carrier to Noise
 * estimators and lock detectors. SNV_CN0 is a Carrier-to-Noise (CN0) estimator
 * based on the Signal-to-Noise Variance (SNV) estimator [1].
 * Carrier lock detector using normalised estimate of the cosine
 * of twice the carrier phase error [2].
 * [1] Marco Pini, Emanuela Falletti and Maurizio Fantino, "Performance
 * Evaluation of C/N0 Estimators using a Real Time GNSS Software Receiver,"
 * IEEE 10th International Symposium on Spread Spectrum Techniques and
 * Applications, pp.28-30, August 2008.
 * [2] Van Dierendonck, A.J. (1996), Global Positioning System: Theory and
 * Applications,
 * Volume I, Chapter 8: GPS Receivers, AJ Systems, Los Altos, CA 94024.
 * Inc.: 329-407.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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
#include "CN_estimators.h"
#include "GPS_L1_CA.h"
#include <gnuradio/gr_complex.h>
#include <math.h>

/*
 * Signal-to-Noise (SNR) (\f$\rho\f$) estimator using the Signal-to-Noise Variance (SNV) estimator:
 * \f{equation}
 * 	\hat{\rho}=\frac{\hat{P}_s}{\hat{P}_n}=\frac{\hat{P}_s}{\hat{P}_{tot}-\hat{P}_s},
 * \f}
 *  where \f$\hat{P}_s=\left(\frac{1}{N}\sum^{N-1}_{i=0}|Re(Pc(i))|\right)^2\f$ is the estimation of the signal power,
 * \f$\hat{P}_{tot}=\frac{1}{N}\sum^{N-1}_{i=0}|Pc(i)|^2\f$ is the estimator of the total power, \f$|\cdot|\f$ is the absolute value,
 * \f$Re(\cdot)\f$ stands for the real part of the value, and \f$Pc(i)\f$ is the prompt correlator output for the sample index i.
 *
 * The SNR value is converted to CN0 [dB-Hz], taking to account the receiver bandwidth and the PRN code gain, using the following formula:
 * \f{equation}
 * 	CN0_{dB}=10*log(\hat{\rho})+10*log(\frac{f_s}{2})-10*log(L_{PRN}),
 * \f}
 * where \f$f_s\f$ is the sampling frequency and \f$L_{PRN}\f$ is the PRN sequence length.
 *
 */
float gps_l1_ca_CN0_SNV(gr_complex* Prompt_buffer, int length, long fs_in)
{
    // estimate CN0 using buffered values
    // MATLAB CODE
    //Psig=((1/N)*sum(abs(imag(x((n-N+1):n)))))^2;
    //Ptot=(1/N)*sum(abs(x((n-N+1):n)).^2);
    //SNR_SNV(count)=Psig/(Ptot-Psig);
    //CN0_SNV_dB=10*log10(SNR_SNV)+10*log10(BW)-10*log10(PRN_length);
    float SNR, SNR_dB_Hz;
    float tmp_abs_I, tmp_abs_Q;
    float Psig, Ptot;
    //float M2,M4;
    Psig = 0;
    Ptot = 0;
    for (int i=0; i<length; i++)
        {
            tmp_abs_I = std::abs(Prompt_buffer[i].imag());
            tmp_abs_Q = std::abs(Prompt_buffer[i].real());
            Psig += tmp_abs_I;
            Ptot += Prompt_buffer[i].imag()*Prompt_buffer[i].imag() + Prompt_buffer[i].real()*Prompt_buffer[i].real();
        }
    Psig = Psig / (float)length;
    Psig = Psig*Psig;
    SNR = Psig / (Ptot / (float)length - Psig);
    SNR_dB_Hz = 10*log10(SNR) + 10*log10(fs_in/2) - 10*log10(GPS_L1_CA_CODE_LENGTH_CHIPS);
    return SNR_dB_Hz;
}

/*
 * The Carrier Phase Lock Detector block uses the normalised estimate of the cosine of twice the carrier phase error is given by
 * \f{equation}
 * 	C2\phi=\frac{NBD}{NBP},
 * \f}
 *  where \f$NBD=(\sum^{N-1}_{i=0}|Im(Pc(i))|)^2+(\sum^{N-1}_{i=0}|Re(Pc(i))|)^2\f$,
 *  \f$NBP=\sum^{N-1}_{i=0}Im(Pc(i))^2-\sum^{N-1}_{i=0}Re(Pc(i))^2\f$, and
 *  \f$Pc(i)\f$ is the prompt correlator output for the sample index i.
 */
float carrier_lock_detector(gr_complex* Prompt_buffer, int length)
{
    /*!
     * \todo Code lock detector
     */
    // estimate using buffered values
    // MATLAB CODE
    // lock detector operation
    //NBD=sum(abs(imag(x((n-N+1):n))))^2 + sum(abs(real(x((n-N+1):n))))^2;
    //NBP=sum(imag(x((n-N+1):n)).^2) - sum(real(x((n-N+1):n)).^2);
    //LOCK(count)=NBD/NBP;
    float tmp_abs_I, tmp_abs_Q;
    float tmp_sum_abs_I, tmp_sum_abs_Q;
    float tmp_sum_sqr_I, tmp_sum_sqr_Q;
    tmp_sum_abs_I=0;
    tmp_sum_abs_Q=0;
    tmp_sum_sqr_I=0;
    tmp_sum_sqr_Q=0;
    float NBD,NBP;
    for (int i=0; i<length; i++)
        {
            tmp_abs_I = std::abs(Prompt_buffer[i].imag());
            tmp_abs_Q = std::abs(Prompt_buffer[i].real());
            tmp_sum_abs_I += tmp_abs_I;
            tmp_sum_abs_Q += tmp_abs_Q;
            tmp_sum_sqr_I += (Prompt_buffer[i].imag()*Prompt_buffer[i].imag());
            tmp_sum_sqr_Q += (Prompt_buffer[i].real()*Prompt_buffer[i].real());
        }
    NBD = tmp_sum_abs_I*tmp_sum_abs_I + tmp_sum_abs_Q*tmp_sum_abs_Q;
    NBP = tmp_sum_sqr_I - tmp_sum_sqr_Q;
    return NBD/NBP;
}

