/*!
 * \file gps_l1_ca_pcps_acquisition_fpga.cc
 * \brief Adapts a PCPS acquisition block to an FPGA AcquisitionInterface
 *  for GPS L1 C/A signals
 * \authors <ul>
 *          <li> Marc Majoral, 2018. mmajoral(at)cttc.es
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena(at)gmail.com
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
#include <new>
#include <gnuradio/fft/fft.h>
#include <volk/volk.h>
#include <glog/logging.h>
#include "gps_l1_ca_pcps_acquisition_fpga.h"
#include "configuration_interface.h"
#include "gps_sdr_signal_processing.h"
#include "GPS_L1_CA.h"
#include "gnss_sdr_flags.h"

#define NUM_PRNs 32

using google::LogMessage;

GpsL1CaPcpsAcquisitionFpga::GpsL1CaPcpsAcquisitionFpga(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    pcpsconf_fpga_t acq_parameters;
    configuration_ = configuration;
    std::string default_item_type = "gr_complex";

    DLOG(INFO) << "role " << role;

    long fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 2048000);
    long fs_in = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    acq_parameters.fs_in = fs_in;
    long ifreq = configuration_->property(role + ".if", 0);
    acq_parameters.freq = ifreq;
    doppler_max_ = configuration_->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0) doppler_max_ = FLAGS_doppler_max;
    acq_parameters.doppler_max = doppler_max_;
    unsigned int sampled_ms = configuration_->property(role + ".coherent_integration_time_ms", 1);
    acq_parameters.sampled_ms = sampled_ms;
    unsigned int code_length = static_cast<unsigned int>(std::round(static_cast<double>(fs_in) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));

    // The FPGA can only use FFT lengths that are a power of two.
    float nbits = ceilf(log2f((float) code_length));
    unsigned int nsamples_total = pow(2, nbits);
    unsigned int vector_length = nsamples_total * sampled_ms;
    unsigned int select_queue_Fpga = configuration_->property(role + ".select_queue_Fpga",0);
    acq_parameters.select_queue_Fpga = select_queue_Fpga;
    std::string default_device_name = "/dev/uio0";
    std::string device_name = configuration_->property(role + ".devicename", default_device_name);
    acq_parameters.device_name = device_name;
    acq_parameters.samples_per_ms = nsamples_total;
    acq_parameters.samples_per_code = nsamples_total;

    // compute all the GPS L1 PRN Codes (this is done only once upon the class constructor in order to avoid re-computing the PRN codes every time
    // a channel is assigned)

    gr::fft::fft_complex* fft_if = new gr::fft::fft_complex(vector_length, true); // Direct FFT
    // allocate memory to compute all the PRNs and compute all the possible codes
    std::complex<float>* code = new std::complex<float>[nsamples_total]; // buffer for the local code
    gr_complex* fft_codes_padded = static_cast<gr_complex*>(volk_gnsssdr_malloc(nsamples_total * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_all_fft_codes_ = new lv_16sc_t[nsamples_total * NUM_PRNs]; // memory containing all the possible fft codes for PRN 0 to 32
    float max; // temporary maxima search

    for (unsigned int PRN = 1; PRN <= NUM_PRNs; PRN++)
        {
            gps_l1_ca_code_gen_complex_sampled(code, PRN, fs_in, 0); // generate PRN code
            // fill in zero padding
            for (int s=code_length;s<nsamples_total;s++)
                {
                    code[s] = 0;
                }
            int offset = 0;
            memcpy(fft_if->get_inbuf() + offset, code, sizeof(gr_complex) * nsamples_total); // copy to FFT buffer
            fft_if->execute(); // Run the FFT of local code
            volk_32fc_conjugate_32fc(fft_codes_padded, fft_if->get_outbuf(), nsamples_total); // conjugate values
            max = 0; // initialize maximum value
            for (unsigned int i = 0; i < nsamples_total; i++) // search for maxima
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
            for (unsigned int i = 0; i < nsamples_total; i++) // map the FFT to the dynamic range of the fixed point values an copy to buffer containing all FFTs
                {
                    d_all_fft_codes_[i + nsamples_total * (PRN -1)] = lv_16sc_t(static_cast<int>(floor(fft_codes_padded[i].real() * (pow(2, 7) - 1) / max)),
                            static_cast<int>(floor(fft_codes_padded[i].imag() * (pow(2, 7) - 1) / max)));

                }
            }

    //acq_parameters

    acq_parameters.all_fft_codes = d_all_fft_codes_;

    // temporary buffers that we can delete
    delete[] code;
    delete fft_if;
    delete[] fft_codes_padded;

    acquisition_fpga_ = pcps_make_acquisition(acq_parameters);
    DLOG(INFO) << "acquisition(" << acquisition_fpga_->unique_id() << ")";

    channel_ = 0;
    doppler_step_ = 0;
    gnss_synchro_ = 0;
}


GpsL1CaPcpsAcquisitionFpga::~GpsL1CaPcpsAcquisitionFpga()
{
    delete[] d_all_fft_codes_;
}


void GpsL1CaPcpsAcquisitionFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    acquisition_fpga_->set_channel(channel_);
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
    acquisition_fpga_->set_active(true);
}


void GpsL1CaPcpsAcquisitionFpga::set_state(int state)
{
    acquisition_fpga_->set_state(state);
}

void GpsL1CaPcpsAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
    // nothing to connect
}


void GpsL1CaPcpsAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
    // nothing to disconnect
}


gr::basic_block_sptr GpsL1CaPcpsAcquisitionFpga::get_left_block()
{
    return acquisition_fpga_;
}


gr::basic_block_sptr GpsL1CaPcpsAcquisitionFpga::get_right_block()
{
    return acquisition_fpga_;
}
