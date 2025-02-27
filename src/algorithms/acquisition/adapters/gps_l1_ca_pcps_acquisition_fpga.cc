/*!
 * \file gps_l1_ca_pcps_acquisition_fpga.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface
 *  for GPS L1 C/A signals for the FPGA
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

#include "gps_l1_ca_pcps_acquisition_fpga.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_fft.h"
#include "gnss_sdr_flags.h"
#include "gps_sdr_signal_replica.h"
#include <gnuradio/gr_complex.h>  // for gr_complex
#include <volk/volk.h>            // for volk_32fc_conjugate_32fc
#include <algorithm>              // for copy_n
#include <cmath>                  // for abs, pow, floor
#include <complex>                // for complex

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GpsL1CaPcpsAcquisitionFpga::GpsL1CaPcpsAcquisitionFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : gnss_synchro_(nullptr),
                                role_(role),
                                doppler_center_(0),
                                channel_(0),
                                doppler_step_(0),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    // set acquisition parameters
    acq_parameters_.SetFromConfiguration(configuration, role_, DEFAULT_FPGA_BLK_EXP, GPS_L1_CA_CODE_RATE_CPS, GPS_L1_CA_CODE_LENGTH_CHIPS);

    // Query the capabilities of the instantiated FPGA Acquisition IP Core
    std::vector<std::pair<uint32_t, uint32_t>> downsampling_filter_specs;
    uint32_t max_FFT_size;
    acquisition_fpga_ = pcps_make_acquisition_fpga(&acq_parameters_, ACQ_BUFF_0, downsampling_filter_specs, max_FFT_size);

    // Configure the automatic resampler according to the capabilities of the instantiated FPGA Acquisition IP Core.
    // When the FPGA is in use, the acquisition resampler operates only in the L1/E1 frequency band.
    bool acq_configuration_valid = acq_parameters_.ConfigureAutomaticResampler(downsampling_filter_specs, max_FFT_size, GPS_L1_CA_OPT_ACQ_FS_SPS);

    if (!acq_configuration_valid)
        {
            std::cout << "The FPGA does not support the required sampling frequency of " << acq_parameters_.fs_in << " SPS for the L1/E1 band. Please update the sampling frequency in the configuration file." << std::endl;
            exit(0);
        }

    DLOG(INFO) << "role " << role;

    generate_gps_l1_ca_prn_codes();

#if USE_GLOG_AND_GFLAGS
    if (FLAGS_doppler_max != 0)
        {
            acq_parameters_.doppler_max = FLAGS_doppler_max;
        }
#else
    if (absl::GetFlag(FLAGS_doppler_max) != 0)
        {
            acq_parameters_.doppler_max = absl::GetFlag(FLAGS_doppler_max);
        }
#endif
    doppler_max_ = acq_parameters_.doppler_max;
    doppler_step_ = static_cast<unsigned int>(acq_parameters_.doppler_step);

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}

void GpsL1CaPcpsAcquisitionFpga::generate_gps_l1_ca_prn_codes()
{
    uint32_t code_length = acq_parameters_.code_length;
    uint32_t nsamples_total = acq_parameters_.fft_size;

    // compute all the GPS L1 PRN Codes (this is done only once upon the class constructor in order to avoid re-computing the PRN codes every time
    // a channel is assigned)
    auto fft_if = gnss_fft_fwd_make_unique(nsamples_total);
    // allocate memory to compute all the PRNs and compute all the possible codes
    volk_gnsssdr::vector<std::complex<float>> code(nsamples_total);
    volk_gnsssdr::vector<std::complex<float>> fft_codes_padded(nsamples_total);
    d_all_fft_codes_ = volk_gnsssdr::vector<uint32_t>(nsamples_total * NUM_PRNs);  // memory containing all the possible fft codes for PRN 0 to 32
    float max;
    int32_t tmp;
    int32_t tmp2;
    int32_t local_code;
    int32_t fft_data;
    // temporary maxima search
    for (uint32_t PRN = 1; PRN <= NUM_PRNs; PRN++)
        {
            gps_l1_ca_code_gen_complex_sampled(code, PRN, acq_parameters_.resampled_fs, 0);  // generate PRN code

            if (acq_parameters_.enable_zero_padding)
                {
                    // Duplicate the code sequence
                    std::copy(code.begin(), code.begin() + code_length, code.begin() + code_length);
                }

            // Fill in zero padding for the rest
            std::fill(code.begin() + (acq_parameters_.enable_zero_padding ? 2 * code_length : code_length), code.end(), std::complex<float>(0.0, 0.0));

            std::copy_n(code.data(), nsamples_total, fft_if->get_inbuf());                            // copy to FFT buffer
            fft_if->execute();                                                                        // Run the FFT of local code
            volk_32fc_conjugate_32fc(fft_codes_padded.data(), fft_if->get_outbuf(), nsamples_total);  // conjugate values

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
                    tmp = static_cast<int32_t>(floor(fft_codes_padded[i].real() * (pow(2, QUANT_BITS_LOCAL_CODE - 1) - 1) / max));
                    tmp2 = static_cast<int32_t>(floor(fft_codes_padded[i].imag() * (pow(2, QUANT_BITS_LOCAL_CODE - 1) - 1) / max));
                    local_code = (tmp & SELECT_LSBITS) | ((tmp2 * SHL_CODE_BITS) & SELECT_MSBITS);  // put together the real part and the imaginary part
                    fft_data = local_code & SELECT_ALL_CODE_BITS;
                    d_all_fft_codes_[i + (nsamples_total * (PRN - 1))] = fft_data;
                }
        }

    acq_parameters_.all_fft_codes = d_all_fft_codes_.data();
}

void GpsL1CaPcpsAcquisitionFpga::stop_acquisition()
{
    // stop the acquisition and the other FPGA modules.
    acquisition_fpga_->stop_acquisition();
}


void GpsL1CaPcpsAcquisitionFpga::set_threshold(float threshold)
{
    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold;
    acquisition_fpga_->set_threshold(threshold);
}


void GpsL1CaPcpsAcquisitionFpga::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    acquisition_fpga_->set_doppler_max(doppler_max_);
}


void GpsL1CaPcpsAcquisitionFpga::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    acquisition_fpga_->set_doppler_step(doppler_step_);
}


void GpsL1CaPcpsAcquisitionFpga::set_doppler_center(int doppler_center)
{
    doppler_center_ = doppler_center;

    acquisition_fpga_->set_doppler_center(doppler_center_);
}


void GpsL1CaPcpsAcquisitionFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_fpga_->set_gnss_synchro(gnss_synchro_);
}


signed int GpsL1CaPcpsAcquisitionFpga::mag()
{
    return acquisition_fpga_->mag();
}


void GpsL1CaPcpsAcquisitionFpga::init()
{
    acquisition_fpga_->init();
}


void GpsL1CaPcpsAcquisitionFpga::set_local_code()
{
    acquisition_fpga_->set_local_code();
}


void GpsL1CaPcpsAcquisitionFpga::reset()
{
    // this function starts the acquisition process
    acquisition_fpga_->set_active(true);
}


void GpsL1CaPcpsAcquisitionFpga::set_state(int state)
{
    acquisition_fpga_->set_state(state);
}


void GpsL1CaPcpsAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect
}


void GpsL1CaPcpsAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr GpsL1CaPcpsAcquisitionFpga::get_left_block()
{
    return nullptr;
}


gr::basic_block_sptr GpsL1CaPcpsAcquisitionFpga::get_right_block()
{
    return nullptr;
}
