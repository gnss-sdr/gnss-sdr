/*!
 * \file gps_l5i pcps_acquisition_fpga.cc
 * \brief Adapts a PCPS acquisition block to an Acquisition Interface for
 *  GPS L5i signals for the FPGA
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.es
 *          <li> Javier Arribas, 2019. jarribas(at)cttc.es
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
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

#include "gps_l5i_pcps_acquisition_fpga.h"
#include "GPS_L5.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gnss_synchro.h"
#include "gps_l5_signal.h"
#include <glog/logging.h>
#include <gnuradio/fft/fft.h>     // for fft_complex
#include <gnuradio/gr_complex.h>  // for gr_complex
#include <volk/volk.h>            // for volk_32fc_conjugate_32fc
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cmath>    // for abs, pow, floor
#include <complex>  // for complex
#include <cstring>  // for memcpy

#define NUM_PRNs 32


GpsL5iPcpsAcquisitionFpga::GpsL5iPcpsAcquisitionFpga(
    ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    pcpsconf_fpga_t acq_parameters;
    configuration_ = configuration;
    std::string default_dump_filename = "./data/acquisition.dat";

    LOG(INFO) << "role " << role;

    int64_t fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 2048000);
    int64_t fs_in = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);

    float downsampling_factor = configuration_->property(role + ".downsampling_factor", 1.0);
    acq_parameters.downsampling_factor = downsampling_factor;

    fs_in = fs_in / downsampling_factor;

    acq_parameters.fs_in = fs_in;
    doppler_max_ = configuration->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0) doppler_max_ = FLAGS_doppler_max;
    acq_parameters.doppler_max = doppler_max_;
    uint32_t sampled_ms = configuration_->property(role + ".coherent_integration_time_ms", 1);
    acq_parameters.sampled_ms = sampled_ms;

    //--- Find number of samples per spreading code -------------------------
    auto code_length = static_cast<uint32_t>(std::round(static_cast<double>(fs_in) / (GPS_L5I_CODE_RATE_HZ / static_cast<double>(GPS_L5I_CODE_LENGTH_CHIPS))));
    acq_parameters.code_length = code_length;
    // The FPGA can only use FFT lengths that are a power of two.
    float nbits = ceilf(log2f((float)code_length * 2));
    uint32_t nsamples_total = pow(2, nbits);
    uint32_t select_queue_Fpga = configuration_->property(role + ".select_queue_Fpga", 1);
    acq_parameters.select_queue_Fpga = select_queue_Fpga;
    std::string default_device_name = "/dev/uio0";
    std::string device_name = configuration_->property(role + ".devicename", default_device_name);
    acq_parameters.device_name = device_name;
    acq_parameters.samples_per_ms = nsamples_total / sampled_ms;
    acq_parameters.samples_per_code = nsamples_total;

    acq_parameters.excludelimit = static_cast<uint32_t>(ceil((1.0 / GPS_L5I_CODE_RATE_HZ) * static_cast<float>(acq_parameters.fs_in)));

    // compute all the GPS L5 PRN Codes (this is done only once upon the class constructor in order to avoid re-computing the PRN codes every time
    // a channel is assigned)
    auto* fft_if = new gr::fft::fft_complex(nsamples_total, true);  // Direct FFT
    auto* code = new gr_complex[nsamples_total];
    auto* fft_codes_padded = static_cast<gr_complex*>(volk_gnsssdr_malloc(nsamples_total * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_all_fft_codes_ = new lv_16sc_t[nsamples_total * NUM_PRNs];  // memory containing all the possible fft codes for PRN 0 to 32

    float max;  // temporary maxima search
    for (uint32_t PRN = 1; PRN <= NUM_PRNs; PRN++)
        {
            gps_l5i_code_gen_complex_sampled(code, PRN, fs_in);

            for (uint32_t s = code_length; s < 2 * code_length; s++)
                {
                    code[s] = code[s - code_length];
                }

            for (uint32_t s = 2 * code_length; s < nsamples_total; s++)
                {
                    // fill in zero padding
                    code[s] = std::complex<float>(0.0, 0.0);
                }
            memcpy(fft_if->get_inbuf(), code, sizeof(gr_complex) * nsamples_total);            // copy to FFT buffer
            fft_if->execute();                                                                 // Run the FFT of local code
            volk_32fc_conjugate_32fc(fft_codes_padded, fft_if->get_outbuf(), nsamples_total);  // conjugate values

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
            for (uint32_t i = 0; i < nsamples_total; i++)  // map the FFT to the dynamic range of the fixed point values an copy to buffer containing all FFTs
                {
                    d_all_fft_codes_[i + nsamples_total * (PRN - 1)] = lv_16sc_t(static_cast<int32_t>(floor(fft_codes_padded[i].real() * (pow(2, 9) - 1) / max)),
                        static_cast<int32_t>(floor(fft_codes_padded[i].imag() * (pow(2, 9) - 1) / max)));
                }
        }

    //acq_parameters
    acq_parameters.all_fft_codes = d_all_fft_codes_;

    // reference for the FPGA FFT-IFFT attenuation factor
    acq_parameters.total_block_exp = configuration_->property(role + ".total_block_exp", 14);

    acquisition_fpga_ = pcps_make_acquisition_fpga(acq_parameters);
    DLOG(INFO) << "acquisition(" << acquisition_fpga_->unique_id() << ")";

    channel_ = 0;
    doppler_step_ = 0;
    gnss_synchro_ = nullptr;

    // temporary buffers that we can delete
    delete[] code;
    delete fft_if;
    delete[] fft_codes_padded;
}


GpsL5iPcpsAcquisitionFpga::~GpsL5iPcpsAcquisitionFpga()
{
    //delete[] code_;
    delete[] d_all_fft_codes_;
}


void GpsL5iPcpsAcquisitionFpga::stop_acquisition()
{
    // this command causes the SW to reset the HW.
    acquisition_fpga_->reset_acquisition();
}


void GpsL5iPcpsAcquisitionFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    acquisition_fpga_->set_channel(channel_);
}


void GpsL5iPcpsAcquisitionFpga::set_threshold(float threshold)
{
    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold;
    acquisition_fpga_->set_threshold(threshold);
}


void GpsL5iPcpsAcquisitionFpga::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    acquisition_fpga_->set_doppler_max(doppler_max_);
}


// Be aware that Doppler step should be set to 2/(3T) Hz, where T is the coherent integration time (GPS L2 period is 0.02s)
// Doppler bin minimum size= 33 Hz
void GpsL5iPcpsAcquisitionFpga::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    acquisition_fpga_->set_doppler_step(doppler_step_);
}


void GpsL5iPcpsAcquisitionFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_fpga_->set_gnss_synchro(gnss_synchro_);
}


signed int GpsL5iPcpsAcquisitionFpga::mag()
{
    return acquisition_fpga_->mag();
}


void GpsL5iPcpsAcquisitionFpga::init()
{
    acquisition_fpga_->init();
}

void GpsL5iPcpsAcquisitionFpga::set_local_code()
{
    acquisition_fpga_->set_local_code();
}


void GpsL5iPcpsAcquisitionFpga::reset()
{
    acquisition_fpga_->set_active(true);
}


void GpsL5iPcpsAcquisitionFpga::set_state(int state)
{
    acquisition_fpga_->set_state(state);
}


void GpsL5iPcpsAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect
}


void GpsL5iPcpsAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr GpsL5iPcpsAcquisitionFpga::get_left_block()
{
    return nullptr;
}


gr::basic_block_sptr GpsL5iPcpsAcquisitionFpga::get_right_block()
{
    return acquisition_fpga_;
}
