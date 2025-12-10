/*!
 * \file gps_l5i_pcps_acquisition_fpga.cc
 * \brief Adapts a PCPS acquisition block to an Acquisition Interface for
 *  GPS L5i signals for the FPGA
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.es
 *          <li> Javier Arribas, 2019. jarribas(at)cttc.es
 *          </ul>
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

#include "gps_l5i_pcps_acquisition_fpga.h"
#include "GPS_L5.h"
#include "configuration_interface.h"
#include "gnss_sdr_fft.h"
#include "gps_l5_signal_replica.h"
#include <volk/volk.h>  // for volk_32fc_conjugate_32fc
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>  // for copy_n
#include <cmath>      // for abs, pow, floor
#include <complex>    // for complex

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GpsL5iPcpsAcquisitionFpga::GpsL5iPcpsAcquisitionFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisitionFpga(configuration,
          role,
          GPS_L5I_CODE_RATE_CPS,
          GPS_L5I_CODE_LENGTH_CHIPS,
          0.0,
          DEFAULT_FPGA_BLK_EXP,
          ACQ_BUFF_1,
          in_streams,
          out_streams)
{
    generate_gps_l5i_prn_codes();
    init();
    DLOG(INFO) << "Initialized FPGA acquisition adapter for role " << role;
}


void GpsL5iPcpsAcquisitionFpga::generate_gps_l5i_prn_codes()
{
    const uint32_t code_length = acq_parameters_.code_length;
    const uint32_t nsamples_total = acq_parameters_.fft_size;
    const uint32_t NUM_PRNs = 32;

    // compute all the GPS L5 PRN Codes (this is done only once upon the class constructor in order to avoid re-computing the PRN codes every time
    // a channel is assigned)
    auto fft_if = gnss_fft_fwd_make_unique(nsamples_total);  // Direct FFT
    volk_gnsssdr::vector<std::complex<float>> code(nsamples_total);
    volk_gnsssdr::vector<std::complex<float>> fft_codes(nsamples_total);
    d_all_fft_codes_ = volk_gnsssdr::vector<uint32_t>(nsamples_total * NUM_PRNs);  // memory containing all the possible fft codes for PRN 0 to 32

    float max;
    int32_t tmp;
    int32_t tmp2;
    int32_t local_code;
    int32_t fft_data;

    for (uint32_t prn = 1; prn <= NUM_PRNs; prn++)
        {
            gps_l5i_code_gen_complex_sampled(code, prn, acq_parameters_.fs_in);

            if (acq_parameters_.enable_zero_padding)
                {
                    // Duplicate the code sequence
                    std::copy(code.begin(), code.begin() + code_length, code.begin() + code_length);
                    // Fill in zero padding for the rest
                    std::fill(code.begin() + (acq_parameters_.enable_zero_padding ? 2 * code_length : code_length), code.end(), std::complex<float>(0.0, 0.0));
                }

            std::copy_n(code.data(), nsamples_total, fft_if->get_inbuf());                     // copy to FFT buffer
            fft_if->execute();                                                                 // Run the FFT of local code
            volk_32fc_conjugate_32fc(fft_codes.data(), fft_if->get_outbuf(), nsamples_total);  // conjugate values

            max = 0;                                       // initialize maximum value
            for (uint32_t i = 0; i < nsamples_total; i++)  // search for maxima
                {
                    if (std::abs(fft_codes[i].real()) > max)
                        {
                            max = std::abs(fft_codes[i].real());
                        }
                    if (std::abs(fft_codes[i].imag()) > max)
                        {
                            max = std::abs(fft_codes[i].imag());
                        }
                }
            // map the FFT to the dynamic range of the fixed point values an copy to buffer containing all FFTs
            // and package codes in a format that is ready to be written to the FPGA
            for (uint32_t i = 0; i < nsamples_total; i++)
                {
                    tmp = static_cast<int32_t>(floor(fft_codes[i].real() * (pow(2, QUANT_BITS_LOCAL_CODE - 1) - 1) / max));
                    tmp2 = static_cast<int32_t>(floor(fft_codes[i].imag() * (pow(2, QUANT_BITS_LOCAL_CODE - 1) - 1) / max));
                    local_code = (tmp & SELECT_LSBITS) | ((tmp2 * SHL_CODE_BITS) & SELECT_MSBITS);  // put together the real part and the imaginary part
                    fft_data = local_code & SELECT_ALL_CODE_BITS;
                    d_all_fft_codes_[i + (nsamples_total * (prn - 1))] = fft_data;
                }
        }

    acq_parameters_.all_fft_codes = d_all_fft_codes_.data();
}
