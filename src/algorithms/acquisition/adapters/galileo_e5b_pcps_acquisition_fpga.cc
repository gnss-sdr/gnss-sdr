/*!
 * \file galileo_e5b_pcps_acquisition_fpga.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5b data and pilot Signals for the FPGA
 * \author Piyush Gupta, 2020. piyush04111999@gmail.com
 * \note Code added as part of GSoC 2020 Program.
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

#include "galileo_e5b_pcps_acquisition_fpga.h"
#include "Galileo_E5b.h"
#include "configuration_interface.h"
#include "galileo_e5_signal_replica.h"
#include "gnss_sdr_fft.h"
#include <volk/volk.h>  // for volk_32fc_conjugate_32fc
#include <algorithm>    // for copy_n
#include <array>        // for array
#include <cmath>        // for abs, pow, floor
#include <complex>      // for complex

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GalileoE5bPcpsAcquisitionFpga::GalileoE5bPcpsAcquisitionFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisitionFpga(configuration,
          role,
          GALILEO_E5B_CODE_CHIP_RATE_CPS,
          GALILEO_E5B_CODE_LENGTH_CHIPS,
          0.0,
          DEFAULT_FPGA_BLK_EXP,
          ACQ_BUFF_1,
          in_streams,
          out_streams),
      acq_pilot_(configuration->property(role + ".acquire_pilot", false)),
      acq_iq_(configuration->property(role + ".acquire_iq", false))
{
    generate_galileo_e5b_prn_codes();
    init();
    DLOG(INFO) << "Initialized FPGA acquisition adapter for role " << role;
}


void GalileoE5bPcpsAcquisitionFpga::generate_galileo_e5b_prn_codes()
{
    const uint32_t code_length = acq_parameters_.code_length;
    const uint32_t nsamples_total = acq_parameters_.fft_size;

    // compute all the GALILEO E5b PRN Codes (this is done only once in the class constructor in order to avoid re-computing the PRN codes every time
    // a channel is assigned)
    auto fft_if = gnss_fft_fwd_make_unique(nsamples_total);          // Direct FFT
    volk_gnsssdr::vector<std::complex<float>> code(nsamples_total);  // Buffer for local code
    volk_gnsssdr::vector<std::complex<float>> fft_code(nsamples_total);
    d_all_fft_codes_ = volk_gnsssdr::vector<uint32_t>(nsamples_total * GALILEO_E5B_NUMBER_OF_CODES);  // memory containing all the possible fft codes for PRN 0 to 32

    if (acq_iq_)
        {
            acq_pilot_ = false;
        }

    float max;
    int32_t tmp;
    int32_t tmp2;
    int32_t local_code;
    int32_t fft_data;

    for (uint32_t prn = 1; prn <= GALILEO_E5B_NUMBER_OF_CODES; prn++)
        {
            std::array<char, 3> signal_;
            signal_[0] = '7';
            signal_[2] = '\0';

            if (acq_iq_)
                {
                    signal_[1] = 'X';
                }
            else if (acq_pilot_)
                {
                    signal_[1] = 'Q';
                }
            else
                {
                    signal_[1] = 'I';
                }

            galileo_e5_b_code_gen_complex_sampled(code, prn, signal_, acq_parameters_.fs_in, 0);

            if (acq_parameters_.enable_zero_padding)
                {
                    // Duplicate the code sequence
                    std::copy(code.begin(), code.begin() + code_length, code.begin() + code_length);
                    // Fill in zero padding for the rest
                    std::fill(code.begin() + (acq_parameters_.enable_zero_padding ? 2 * code_length : code_length), code.end(), std::complex<float>(0.0, 0.0));
                }

            std::copy_n(code.data(), nsamples_total, fft_if->get_inbuf());                    // copy to FFT buffer
            fft_if->execute();                                                                // Run the FFT of local code
            volk_32fc_conjugate_32fc(fft_code.data(), fft_if->get_outbuf(), nsamples_total);  // conjugate values

            max = 0;                                       // initialize maximum value
            for (uint32_t i = 0; i < nsamples_total; i++)  // search for maxima
                {
                    if (std::abs(fft_code[i].real()) > max)
                        {
                            max = std::abs(fft_code[i].real());
                        }
                    if (std::abs(fft_code[i].imag()) > max)
                        {
                            max = std::abs(fft_code[i].imag());
                        }
                }
            // map the FFT to the dynamic range of the fixed point values an copy to buffer containing all FFTs
            // and package codes in a format that is ready to be written to the FPGA
            for (uint32_t i = 0; i < nsamples_total; i++)
                {
                    tmp = static_cast<int32_t>(floor(fft_code[i].real() * (pow(2, QUANT_BITS_LOCAL_CODE - 1) - 1) / max));
                    tmp2 = static_cast<int32_t>(floor(fft_code[i].imag() * (pow(2, QUANT_BITS_LOCAL_CODE - 1) - 1) / max));
                    local_code = (tmp & SELECT_LSBITS) | ((tmp2 * SHL_CODE_BITS) & SELECT_MSBITS);  // put together the real part and the imaginary part
                    fft_data = local_code & SELECT_ALL_CODE_BITS;
                    d_all_fft_codes_[i + (nsamples_total * (prn - 1))] = fft_data;
                }
        }

    acq_parameters_.all_fft_codes = d_all_fft_codes_.data();
}
