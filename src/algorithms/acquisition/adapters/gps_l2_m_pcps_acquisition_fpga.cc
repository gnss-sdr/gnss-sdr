/*!
 * \file gps_l2_m_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L2 M signals
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
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

#include "gps_l2_m_pcps_acquisition_fpga.h"
#include "configuration_interface.h"
#include "gps_l2c_signal.h"
#include "GPS_L2C.h"
#include "gnss_sdr_flags.h"
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>

#define NUM_PRNs 32

using google::LogMessage;

GpsL2MPcpsAcquisitionFpga::GpsL2MPcpsAcquisitionFpga(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    //pcpsconf_t acq_parameters;
    pcpsconf_fpga_t acq_parameters;
    configuration_ = configuration;
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./data/acquisition.dat";

    LOG(INFO) << "role " << role;

    item_type_ = configuration_->property(role + ".item_type", default_item_type);
    //float pfa =  configuration_->property(role + ".pfa", 0.0);

    long fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    acq_parameters.fs_in = fs_in_;
    if_ = configuration_->property(role + ".if", 0);
    acq_parameters.freq = if_;
    //dump_ = configuration_->property(role + ".dump", false);
    //acq_parameters.dump = dump_;
    //blocking_ = configuration_->property(role + ".blocking", true);
    //acq_parameters.blocking = blocking_;
    doppler_max_ = configuration->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0) doppler_max_ = FLAGS_doppler_max;
    acq_parameters.doppler_max = doppler_max_;
    //bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);
    //acq_parameters.bit_transition_flag = bit_transition_flag_;
    //use_CFAR_algorithm_flag_ = configuration_->property(role + ".use_CFAR_algorithm", true);  //will be false in future versions
    //acq_parameters.use_CFAR_algorithm_flag = use_CFAR_algorithm_flag_;
    //max_dwells_ = configuration_->property(role + ".max_dwells", 1);
    //acq_parameters.max_dwells = max_dwells_;
    //dump_filename_ = configuration_->property(role + ".dump_filename", default_dump_filename);
    //acq_parameters.dump_filename = dump_filename_;
    //--- Find number of samples per spreading code -------------------------
    //code_length_ = std::round(static_cast<double>(fs_in_) / (GPS_L2_M_CODE_RATE_HZ / static_cast<double>(GPS_L2_M_CODE_LENGTH_CHIPS)));

    acq_parameters.sampled_ms = 20;
    unsigned code_length = std::round(static_cast<double>(fs_in_) / (GPS_L2_M_CODE_RATE_HZ / static_cast<double>(GPS_L2_M_CODE_LENGTH_CHIPS)));
    acq_parameters.code_length = code_length;
    // The FPGA can only use FFT lengths that are a power of two.
    float nbits = ceilf(log2f((float)code_length));
    unsigned int nsamples_total = pow(2, nbits);
    unsigned int vector_length = nsamples_total;
    unsigned int select_queue_Fpga = configuration_->property(role + ".select_queue_Fpga", 0);
    acq_parameters.select_queue_Fpga = select_queue_Fpga;
    std::string default_device_name = "/dev/uio0";
    std::string device_name = configuration_->property(role + ".devicename", default_device_name);
    acq_parameters.device_name = device_name;
    acq_parameters.samples_per_ms = nsamples_total/acq_parameters.sampled_ms;
    //acq_parameters.samples_per_ms = static_cast<int>(std::round(static_cast<double>(fs_in_) * 0.001));
    acq_parameters.samples_per_code = nsamples_total;

    // compute all the GPS L1 PRN Codes (this is done only once upon the class constructor in order to avoid re-computing the PRN codes every time
    // a channel is assigned)
    gr::fft::fft_complex* fft_if = new gr::fft::fft_complex(vector_length, true);  // Direct FFT
    // allocate memory to compute all the PRNs and compute all the possible codes
    std::complex<float>* code = new std::complex<float>[nsamples_total];  // buffer for the local code
    gr_complex* fft_codes_padded = static_cast<gr_complex*>(volk_gnsssdr_malloc(nsamples_total * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_all_fft_codes_ = new lv_16sc_t[nsamples_total * NUM_PRNs];  // memory containing all the possible fft codes for PRN 0 to 32
    float max;                                                    // temporary maxima search
    for (unsigned int PRN = 1; PRN <= NUM_PRNs; PRN++)
        {
            gps_l2c_m_code_gen_complex_sampled(code, PRN, fs_in_);
            // fill in zero padding
            for (int s = code_length; s < nsamples_total; s++)
                {
                    code[s] = std::complex<float>(static_cast<float>(0,0));
                    //code[s] = 0;
                }
            memcpy(fft_if->get_inbuf(), code, sizeof(gr_complex) * nsamples_total);   // copy to FFT buffer
            fft_if->execute();                                                                 // Run the FFT of local code
            volk_32fc_conjugate_32fc(fft_codes_padded, fft_if->get_outbuf(), nsamples_total);  // conjugate values
            max = 0;                                                                           // initialize maximum value
            for (unsigned int i = 0; i < nsamples_total; i++)                                  // search for maxima
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
            for (unsigned int i = 0; i < nsamples_total; i++)  // map the FFT to the dynamic range of the fixed point values an copy to buffer containing all FFTs
                {
                    d_all_fft_codes_[i + nsamples_total * (PRN - 1)] = lv_16sc_t(static_cast<int>(floor(fft_codes_padded[i].real() * (pow(2, 7) - 1) / max)),
                        static_cast<int>(floor(fft_codes_padded[i].imag() * (pow(2, 7) - 1) / max)));
                }

        }

    //acq_parameters
    acq_parameters.all_fft_codes = d_all_fft_codes_;

    // temporary buffers that we can delete
    delete[] code;
    delete fft_if;
    delete[] fft_codes_padded;

    acquisition_fpga_ = pcps_make_acquisition_fpga(acq_parameters);
    DLOG(INFO) << "acquisition(" << acquisition_fpga_->unique_id() << ")";

    channel_ = 0;
    doppler_step_ = 0;
    gnss_synchro_ = 0;





//    vector_length_ = code_length_;
//
//    if (bit_transition_flag_)
//        {
//            vector_length_ *= 2;
//        }

//    code_ = new gr_complex[vector_length_];
//
//    if (item_type_.compare("cshort") == 0)
//        {
//            item_size_ = sizeof(lv_16sc_t);
//        }
//    else
//        {
//            item_size_ = sizeof(gr_complex);
//        }
    //acq_parameters.samples_per_ms = static_cast<int>(std::round(static_cast<double>(fs_in_) * 0.001));
    //acq_parameters.samples_per_code = code_length_;
    //acq_parameters.it_size = item_size_;
    //acq_parameters.sampled_ms = 20;
    //acq_parameters.num_doppler_bins_step2 = configuration_->property(role + ".second_nbins", 4);
    //acq_parameters.doppler_step2 = configuration_->property(role + ".second_doppler_step", 125.0);
    //acq_parameters.make_2_steps = configuration_->property(role + ".make_two_steps", true);
    //acquisition_ = pcps_make_acquisition(acq_parameters);
    DLOG(INFO) << "acquisition(" << acquisition_fpga_->unique_id() << ")";

//    stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, vector_length_);
//    DLOG(INFO) << "stream_to_vector(" << stream_to_vector_->unique_id() << ")";
//
//    if (item_type_.compare("cbyte") == 0)
//        {
//            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();
//            float_to_complex_ = gr::blocks::float_to_complex::make();
//        }

//    channel_ = 0;
    threshold_ = 0.0;
//    doppler_step_ = 0;
//    gnss_synchro_ = 0;
}


GpsL2MPcpsAcquisitionFpga::~GpsL2MPcpsAcquisitionFpga()
{
    //delete[] code_;
    delete[] d_all_fft_codes_;
}


void GpsL2MPcpsAcquisitionFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    acquisition_fpga_->set_channel(channel_);
}


void GpsL2MPcpsAcquisitionFpga::set_threshold(float threshold)
{
//    float pfa = configuration_->property(role_ + boost::lexical_cast<std::string>(channel_) + ".pfa", 0.0);
//
//    if (pfa == 0.0)
//        {
//            pfa = configuration_->property(role_ + ".pfa", 0.0);
//        }
//    if (pfa == 0.0)
//        {
//            threshold_ = threshold;
//        }
//    else
//        {
//            threshold_ = calculate_threshold(pfa);
//        }

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
    //set_local_code();
}


void GpsL2MPcpsAcquisitionFpga::set_local_code()
{
    //gps_l2c_m_code_gen_complex_sampled(code_, gnss_synchro_->PRN, fs_in_);

    //acquisition_fpga_->set_local_code(code_);
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


//float GpsL2MPcpsAcquisitionFpga::calculate_threshold(float pfa)
//{
//    //Calculate the threshold
//    unsigned int frequency_bins = 0;
//    for (int doppler = static_cast<int>(-doppler_max_); doppler <= static_cast<int>(doppler_max_); doppler += doppler_step_)
//        {
//            frequency_bins++;
//        }
//    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;
//    unsigned int ncells = vector_length_ * frequency_bins;
//    double exponent = 1.0 / static_cast<double>(ncells);
//    double val = pow(1.0 - pfa, exponent);
//    double lambda = double(vector_length_);
//    boost::math::exponential_distribution<double> mydist(lambda);
//    float threshold = static_cast<float>(quantile(mydist, val));
//
//    return threshold;
//}


void GpsL2MPcpsAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
//    if (item_type_.compare("gr_complex") == 0)
//        {
//            top_block->connect(stream_to_vector_, 0, acquisition_, 0);
//        }
//    else if (item_type_.compare("cshort") == 0)
//        {
//            top_block->connect(stream_to_vector_, 0, acquisition_, 0);
//        }
//    else if (item_type_.compare("cbyte") == 0)
//        {
//            top_block->connect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
//            top_block->connect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
//            top_block->connect(float_to_complex_, 0, stream_to_vector_, 0);
//            top_block->connect(stream_to_vector_, 0, acquisition_, 0);
//        }
//    else
//        {
//            LOG(WARNING) << item_type_ << " unknown acquisition item type";
//        }

    // nothing to connect
}


void GpsL2MPcpsAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
//    if (item_type_.compare("gr_complex") == 0)
//        {
//            top_block->disconnect(stream_to_vector_, 0, acquisition_, 0);
//        }
//    else if (item_type_.compare("cshort") == 0)
//        {
//            top_block->disconnect(stream_to_vector_, 0, acquisition_, 0);
//        }
//    else if (item_type_.compare("cbyte") == 0)
//        {
//            // Since a byte-based acq implementation is not available,
//            // we just convert cshorts to gr_complex
//            top_block->disconnect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
//            top_block->disconnect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
//            top_block->disconnect(float_to_complex_, 0, stream_to_vector_, 0);
//            top_block->disconnect(stream_to_vector_, 0, acquisition_, 0);
//        }
//    else
//        {
//            LOG(WARNING) << item_type_ << " unknown acquisition item type";
//        }

    // nothing to disconnect
}


gr::basic_block_sptr GpsL2MPcpsAcquisitionFpga::get_left_block()
{
//    if (item_type_.compare("gr_complex") == 0)
//        {
//            return stream_to_vector_;
//        }
//    else if (item_type_.compare("cshort") == 0)
//        {
//            return stream_to_vector_;
//        }
//    else if (item_type_.compare("cbyte") == 0)
//        {
//            return cbyte_to_float_x2_;
//        }
//    else
//        {
//            LOG(WARNING) << item_type_ << " unknown acquisition item type";
//            return nullptr;
//        }
    return nullptr;
}


gr::basic_block_sptr GpsL2MPcpsAcquisitionFpga::get_right_block()
{
    return acquisition_fpga_;
}
