/*!
 * \file gps_l2_m_pcps_acquisition_fpga.cc
 * \brief Adapts an FPGA-offloaded PCPS acquisition block
 * to an AcquisitionInterface for GPS L2 M signals
 * \authors <ul>
 *          <li> Javier Arribas, 2019. jarribas(at)cttc.es
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

#include "gps_l2_m_pcps_acquisition_fpga.h"
#include "GPS_L2C.h"
#include "configuration_interface.h"
#include "gnss_sdr_fft.h"
#include "gnss_sdr_flags.h"
#include "gnss_synchro.h"
#include "gps_l2c_signal_replica.h"
#include "uio_fpga.h"
#include <glog/logging.h>
#include <gnuradio/gr_complex.h>  // for gr_complex
#include <volk/volk.h>            // for volk_32fc_conjugate_32fc
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>  // for copy_n
#include <cmath>      // for abs, pow, floor
#include <complex>    // for complex

GpsL2MPcpsAcquisitionFpga::GpsL2MPcpsAcquisitionFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    pcpsconf_fpga_t acq_parameters;
    std::string default_dump_filename = "./acquisition.mat";

    LOG(INFO) << "role " << role;

    int64_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in_ = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    acq_parameters.fs_in = fs_in_;

    acq_parameters.repeat_satellite = configuration->property(role + ".repeat_satellite", false);
    DLOG(INFO) << role << " satellite repeat = " << acq_parameters.repeat_satellite;

    doppler_max_ = configuration->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0)
        {
            doppler_max_ = FLAGS_doppler_max;
        }
    acq_parameters.doppler_max = doppler_max_;

    auto code_length = static_cast<unsigned int>(std::round(static_cast<double>(fs_in_) / (GPS_L2_M_CODE_RATE_CPS / static_cast<double>(GPS_L2_M_CODE_LENGTH_CHIPS))));
    acq_parameters.code_length = code_length;
    // The FPGA can only use FFT lengths that are a power of two.
    float nbits = ceilf(log2f(static_cast<float>(code_length)));
    unsigned int nsamples_total = pow(2, nbits);
    unsigned int select_queue_Fpga = configuration->property(role + ".select_queue_Fpga", 0);
    acq_parameters.select_queue_Fpga = select_queue_Fpga;

    // UIO device file
    std::string device_io_name;
    // find the uio device file corresponding to the acquisition
    if (find_uio_dev_file_name(device_io_name, acquisition_device_name, 0) < 0)
        {
            std::cout << "Cannot find the FPGA uio device file corresponding to device name " << acquisition_device_name << std::endl;
            throw std::exception();
        }
    acq_parameters.device_name = device_io_name;

    acq_parameters.samples_per_code = nsamples_total;
    acq_parameters.downsampling_factor = configuration->property(role + ".downsampling_factor", 1.0);
    acq_parameters.total_block_exp = configuration->property(role + ".total_block_exp", 14);
    acq_parameters.excludelimit = static_cast<uint32_t>(std::round(static_cast<double>(fs_in_) / GPS_L2_M_CODE_RATE_CPS));

    // compute all the GPS L2C PRN Codes (this is done only once upon the class constructor in order to avoid re-computing the PRN codes every time
    // a channel is assigned)
    auto fft_if = gnss_fft_fwd_make_unique(nsamples_total);  // Direct FFT
    // allocate memory to compute all the PRNs and compute all the possible codes
    volk_gnsssdr::vector<std::complex<float>> code(nsamples_total);
    volk_gnsssdr::vector<std::complex<float>> fft_codes_padded(nsamples_total);
    d_all_fft_codes_ = std::vector<uint32_t>(nsamples_total * NUM_PRNs);  // memory containing all the possible fft codes for PRN 0 to 32

    float max;  // temporary maxima search
    int32_t tmp;
    int32_t tmp2;
    int32_t local_code;
    int32_t fft_data;

    for (unsigned int PRN = 1; PRN <= NUM_PRNs; PRN++)
        {
            gps_l2c_m_code_gen_complex_sampled(code, PRN, fs_in_);
            // fill in zero padding
            for (unsigned int s = code_length; s < nsamples_total; s++)
                {
                    code[s] = std::complex<float>(0.0, 0.0);
                }
            std::copy_n(code.data(), nsamples_total, fft_if->get_inbuf());                            // copy to FFT buffer
            fft_if->execute();                                                                        // Run the FFT of local code
            volk_32fc_conjugate_32fc(fft_codes_padded.data(), fft_if->get_outbuf(), nsamples_total);  // conjugate values
            max = 0;                                                                                  // initialize maximum value
            for (unsigned int i = 0; i < nsamples_total; i++)                                         // search for maxima
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
                    local_code = (tmp & SELECT_LSBits) | ((tmp2 * SHL_CODE_BITS) & SELECT_MSBbits);  // put together the real part and the imaginary part
                    fft_data = local_code & SELECT_ALL_CODE_BITS;
                    d_all_fft_codes_[i + (nsamples_total * (PRN - 1))] = fft_data;

                    // d_all_fft_codes_[i + nsamples_total * (PRN - 1)] = lv_16sc_t(static_cast<int32_t>(floor(fft_codes_padded[i].real() * (pow(2, QUANT_BITS_LOCAL_CODE - 1) - 1) / max)),
                    // static_cast<int32_t>(floor(fft_codes_padded[i].imag() * (pow(2, QUANT_BITS_LOCAL_CODE - 1) - 1) / max)));
                }
        }

    acq_parameters.all_fft_codes = d_all_fft_codes_.data();

    acquisition_fpga_ = pcps_make_acquisition_fpga(acq_parameters);

    channel_ = 0;
    doppler_step_ = 0;
    gnss_synchro_ = nullptr;

    threshold_ = 0.0;

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


void GpsL2MPcpsAcquisitionFpga::stop_acquisition()
{
    // stop the acquisition and the other FPGA modules.
    acquisition_fpga_->stop_acquisition();
}


void GpsL2MPcpsAcquisitionFpga::set_threshold(float threshold)
{
    threshold_ = threshold;
    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold_;
    acquisition_fpga_->set_threshold(threshold_);
}


void GpsL2MPcpsAcquisitionFpga::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    acquisition_fpga_->set_doppler_max(doppler_max_);
}


// Be aware that Doppler step should be set to 2/(3T) Hz, where T is the coherent integration time (GPS L2 period is 0.02s)
// Doppler bin minimum size= 33 Hz
void GpsL2MPcpsAcquisitionFpga::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;

    acquisition_fpga_->set_doppler_step(doppler_step_);
}


void GpsL2MPcpsAcquisitionFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;

    acquisition_fpga_->set_gnss_synchro(gnss_synchro_);
}


signed int GpsL2MPcpsAcquisitionFpga::mag()
{
    return acquisition_fpga_->mag();
}


void GpsL2MPcpsAcquisitionFpga::init()
{
    acquisition_fpga_->init();
}


void GpsL2MPcpsAcquisitionFpga::set_local_code()
{
    acquisition_fpga_->set_local_code();
}


void GpsL2MPcpsAcquisitionFpga::reset()
{
    acquisition_fpga_->set_active(true);
}


void GpsL2MPcpsAcquisitionFpga::set_state(int state)
{
    acquisition_fpga_->set_state(state);
}


void GpsL2MPcpsAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect
}


void GpsL2MPcpsAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to diconnect
}


gr::basic_block_sptr GpsL2MPcpsAcquisitionFpga::get_left_block()
{
    return nullptr;
}


gr::basic_block_sptr GpsL2MPcpsAcquisitionFpga::get_right_block()
{
    return nullptr;
}
