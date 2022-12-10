/*!
 * \file galileo_e1_pcps_ambiguous_acquisition_fpga.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.es
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

#include "galileo_e1_pcps_ambiguous_acquisition_fpga.h"
#include "Galileo_E1.h"
#include "configuration_interface.h"
#include "galileo_e1_signal_replica.h"
#include "gnss_sdr_fft.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <gnuradio/fft/fft.h>     // for fft_complex
#include <gnuradio/gr_complex.h>  // for gr_complex
#include <volk/volk.h>            // for volk_32fc_conjugate_32fc
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>  // for copy_n
#include <cmath>      // for abs, pow, floor
#include <complex>    // for complex

GalileoE1PcpsAmbiguousAcquisitionFpga::GalileoE1PcpsAmbiguousAcquisitionFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : gnss_synchro_(nullptr),
      role_(role),
      doppler_center_(0),
      channel_(0),
      doppler_step_(0),
      in_streams_(in_streams),
      out_streams_(out_streams),
      acquire_pilot_(configuration->property(role + ".acquire_pilot", false))
{
    acq_parameters_.SetFromConfiguration(configuration, role_, fpga_downsampling_factor, fpga_buff_num, fpga_blk_exp, GALILEO_E1_CODE_CHIP_RATE_CPS, GALILEO_E1_B_CODE_LENGTH_CHIPS);

    if (FLAGS_doppler_max != 0)
        {
            acq_parameters_.doppler_max = FLAGS_doppler_max;
        }
    doppler_max_ = acq_parameters_.doppler_max;
    doppler_step_ = static_cast<unsigned int>(acq_parameters_.doppler_step);
    fs_in_ = acq_parameters_.fs_in;

    uint32_t code_length = acq_parameters_.code_length;
    uint32_t nsamples_total = acq_parameters_.samples_per_code;

    // compute all the GALILEO E1 PRN Codes (this is done only once in the class constructor in order to avoid re-computing the PRN codes every time
    // a channel is assigned)
    auto fft_if = gnss_fft_fwd_make_unique(nsamples_total);          // Direct FFT
    volk_gnsssdr::vector<std::complex<float>> code(nsamples_total);  // buffer for the local code
    volk_gnsssdr::vector<gr_complex> fft_codes_padded(nsamples_total);
    d_all_fft_codes_ = volk_gnsssdr::vector<uint32_t>(nsamples_total * GALILEO_E1_NUMBER_OF_CODES);  // memory containing all the possible fft codes for PRN 0 to 32

    float max;  // temporary maxima search
    int32_t tmp;
    int32_t tmp2;
    int32_t local_code;
    int32_t fft_data;

    for (uint32_t PRN = 1; PRN <= GALILEO_E1_NUMBER_OF_CODES; PRN++)
        {
            bool cboc = false;  // cboc is set to 0 when using the FPGA

            if (acquire_pilot_ == true)
                {
                    // set local signal generator to Galileo E1 pilot component (1C)
                    std::array<char, 3> pilot_signal = {{'1', 'C', '\0'}};
                    galileo_e1_code_gen_complex_sampled(code, pilot_signal,
                        cboc, PRN, fs_in_, 0, false);
                }
            else
                {
                    std::array<char, 3> data_signal = {{'1', 'B', '\0'}};
                    galileo_e1_code_gen_complex_sampled(code, data_signal,
                        cboc, PRN, fs_in_, 0, false);
                }

            for (uint32_t s = code_length; s < 2 * code_length; s++)
                {
                    code[s] = code[s - code_length];
                }

            // fill in zero padding
            for (uint32_t s = 2 * code_length; s < nsamples_total; s++)
                {
                    code[s] = std::complex<float>(0.0, 0.0);
                }

            std::copy_n(code.data(), nsamples_total, fft_if->get_inbuf());                            // copy to FFT buffer
            fft_if->execute();                                                                        // Run the FFT of local code
            volk_32fc_conjugate_32fc(fft_codes_padded.data(), fft_if->get_outbuf(), nsamples_total);  // conjugate values

            // normalize the code
            max = 0;                                       // initialize maximum value
            for (uint32_t i = 0; i < nsamples_total; i++)  // search for maxima
                {
                    if (std::abs(fft_codes_padded[i].real()) > max)
                        {
                            max = std::abs(fft_codes_padded[i].real());
                        }
                    if (std::abs(fft_codes_padded[i].imag()) > max)
                        {
                            max = std::abs(fft_codes_padded[i].imag());
                        }
                }
            // map the FFT to the dynamic range of the fixed point values an copy to buffer containing all FFTs
            // and package codes in a format that is ready to be written to the FPGA
            for (uint32_t i = 0; i < nsamples_total; i++)
                {
                    tmp = static_cast<int32_t>(floor(fft_codes_padded[i].real() * (pow(2, quant_bits_local_code - 1) - 1) / max));
                    tmp2 = static_cast<int32_t>(floor(fft_codes_padded[i].imag() * (pow(2, quant_bits_local_code - 1) - 1) / max));
                    local_code = (tmp & select_lsbits) | ((tmp2 * shl_code_bits) & select_msbits);  // put together the real part and the imaginary part
                    fft_data = local_code & select_all_code_bits;
                    d_all_fft_codes_[i + (nsamples_total * (PRN - 1))] = fft_data;
                }
        }

    acq_parameters_.all_fft_codes = d_all_fft_codes_.data();

    DLOG(INFO) << "role " << role_;
    acquisition_fpga_ = pcps_make_acquisition_fpga(acq_parameters_);

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::stop_acquisition()
{
    // stop the acquisition and the other FPGA modules.
    acquisition_fpga_->stop_acquisition();
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_threshold(float threshold)
{
    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold;
    acquisition_fpga_->set_threshold(threshold);
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    acquisition_fpga_->set_doppler_max(doppler_max_);
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    acquisition_fpga_->set_doppler_step(doppler_step_);
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_doppler_center(int doppler_center)
{
    doppler_center_ = doppler_center;

    acquisition_fpga_->set_doppler_center(doppler_center_);
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_fpga_->set_gnss_synchro(gnss_synchro_);
}


signed int GalileoE1PcpsAmbiguousAcquisitionFpga::mag()
{
    return acquisition_fpga_->mag();
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::init()
{
    acquisition_fpga_->init();
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_local_code()
{
    acquisition_fpga_->set_local_code();
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::reset()
{
    // This command starts the acquisition process
    acquisition_fpga_->set_active(true);
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_state(int state)
{
    acquisition_fpga_->set_state(state);
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr GalileoE1PcpsAmbiguousAcquisitionFpga::get_left_block()
{
    return nullptr;
}


gr::basic_block_sptr GalileoE1PcpsAmbiguousAcquisitionFpga::get_right_block()
{
    return nullptr;
}
