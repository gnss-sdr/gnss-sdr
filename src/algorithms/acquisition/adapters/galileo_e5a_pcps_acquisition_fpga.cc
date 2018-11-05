/*!
 * \file galileo_e5a_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Antonio Ramos, 2018. antonio.ramos(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#include "galileo_e5a_pcps_acquisition_fpga.h"
#include "configuration_interface.h"
#include "galileo_e5_signal_processing.h"
#include "Galileo_E5a.h"
#include "gnss_sdr_flags.h"
#include <boost/lexical_cast.hpp>
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>


using google::LogMessage;

void GalileoE5aPcpsAcquisitionFpga::stop_acquisition()
{
}

GalileoE5aPcpsAcquisitionFpga::GalileoE5aPcpsAcquisitionFpga(ConfigurationInterface* configuration,
    std::string role, unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    //printf("creating the E5A acquisition");
    pcpsconf_fpga_t acq_parameters;
    configuration_ = configuration;
    std::string default_item_type = "cshort";
    std::string default_dump_filename = "../data/acquisition.dat";

    DLOG(INFO) << "Role " << role;

    //item_type_ = configuration_->property(role + ".item_type", default_item_type);

    long fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 32000000);
    long fs_in = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);

    float downsampling_factor = configuration_->property("GNSS-SDR.downsampling_factor", 1.0);
    acq_parameters.downsampling_factor = downsampling_factor;

    fs_in = fs_in/downsampling_factor;

    acq_parameters.fs_in = fs_in;
    //acq_parameters.freq = 0;


    //dump_ = configuration_->property(role + ".dump", false);
    //acq_parameters.dump = dump_;
    doppler_max_ = configuration_->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0) doppler_max_ = FLAGS_doppler_max;
    acq_parameters.doppler_max = doppler_max_;

    unsigned int sampled_ms = configuration_->property(role + ".coherent_integration_time_ms", 1);
    acq_parameters.sampled_ms = sampled_ms;
    //max_dwells_ = configuration_->property(role + ".max_dwells", 1);
    //acq_parameters.max_dwells = max_dwells_;
    //dump_filename_ = configuration_->property(role + ".dump_filename", default_dump_filename);
    //acq_parameters.dump_filename = dump_filename_;
    //bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);
    //acq_parameters.bit_transition_flag = bit_transition_flag_;
    //use_CFAR_ = configuration_->property(role + ".use_CFAR_algorithm", false);
    //acq_parameters.use_CFAR_algorithm_flag = use_CFAR_;
    //blocking_ = configuration_->property(role + ".blocking", true);
    //acq_parameters.blocking = blocking_;
    //--- Find number of samples per spreading code (1ms)-------------------------

    acq_pilot_ = configuration_->property(role + ".acquire_pilot", false);
    acq_iq_ = configuration_->property(role + ".acquire_iq", false);
    if (acq_iq_)
        {
            acq_pilot_ = false;
        }

    unsigned int code_length = static_cast<unsigned int>(std::round(static_cast<double>(fs_in) / Galileo_E5a_CODE_CHIP_RATE_HZ * static_cast<double>(Galileo_E5a_CODE_LENGTH_CHIPS)));
    acq_parameters.code_length = code_length;
    // The FPGA can only use FFT lengths that are a power of two.
    float nbits = ceilf(log2f((float)code_length*2));
    unsigned int nsamples_total = pow(2, nbits);
    unsigned int vector_length = nsamples_total;
    unsigned int select_queue_Fpga = configuration_->property(role + ".select_queue_Fpga", 1);
    printf("select queue = %d\n", select_queue_Fpga);
    //printf("select_queue_Fpga = %d\n", select_queue_Fpga);
    acq_parameters.select_queue_Fpga = select_queue_Fpga;
    std::string default_device_name = "/dev/uio0";
    std::string device_name = configuration_->property(role + ".devicename", default_device_name);
    acq_parameters.device_name = device_name;
    acq_parameters.samples_per_ms = nsamples_total / sampled_ms;
    acq_parameters.samples_per_code = nsamples_total;

    //vector_length_ = code_length_ * sampled_ms_;

    // compute all the GALILEO E5 PRN Codes (this is done only once upon the class constructor in order to avoid re-computing the PRN codes every time
    // a channel is assigned)
    gr::fft::fft_complex* fft_if = new gr::fft::fft_complex(nsamples_total, true);  // Direct FFT
    std::complex<float>* code = new std::complex<float>[nsamples_total];            // buffer for the local code
    gr_complex* fft_codes_padded = static_cast<gr_complex*>(volk_gnsssdr_malloc(nsamples_total * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_all_fft_codes_ = new lv_16sc_t[nsamples_total * Galileo_E5a_NUMBER_OF_CODES];  // memory containing all the possible fft codes for PRN 0 to 32
    float max;                                                                       // temporary maxima search

    //printf("creating the E5A acquisition CONT");
    //printf("nsamples_total = %d\n", nsamples_total);

    for (unsigned int PRN = 1; PRN <= Galileo_E5a_NUMBER_OF_CODES; PRN++)
        {
            //    gr_complex* code = new gr_complex[code_length_];
            char signal_[3];

            if (acq_iq_)
                {
                    strcpy(signal_, "5X");
                }
            else if (acq_pilot_)
                {
                    strcpy(signal_, "5Q");
                }
            else
                {
                    strcpy(signal_, "5I");
                }


            galileo_e5_a_code_gen_complex_sampled(code, signal_, PRN, fs_in, 0);

            for (int s = code_length; s < 2*code_length; s++)
                {
                    code[s] = code[s - code_length];
                    //code[s] = 0;
                }

            // fill in zero padding
            for (int s = 2*code_length; s < nsamples_total; s++)
                {
                    code[s] = std::complex<float>(static_cast<float>(0, 0));
                    //code[s] = 0;
                }

            memcpy(fft_if->get_inbuf(), code, sizeof(gr_complex) * nsamples_total);            // copy to FFT buffer
            fft_if->execute();                                                                 // Run the FFT of local code
            volk_32fc_conjugate_32fc(fft_codes_padded, fft_if->get_outbuf(), nsamples_total);  // conjugate values

            max = 0;                                           // initialize maximum value
            for (unsigned int i = 0; i < nsamples_total; i++)  // search for maxima
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
                    d_all_fft_codes_[i + nsamples_total * (PRN - 1)] = lv_16sc_t(static_cast<int>(floor(fft_codes_padded[i].real() * (pow(2, 5) - 1) / max)),
                        static_cast<int>(floor(fft_codes_padded[i].imag() * (pow(2, 5) - 1) / max)));
                }
        }


    acq_parameters.all_fft_codes = d_all_fft_codes_;

    // temporary buffers that we can delete
    delete[] code;
    delete fft_if;
    delete[] fft_codes_padded;

    //code_ = new gr_complex[vector_length_];

    //    if (item_type_.compare("gr_complex") == 0)
    //        {
    //            item_size_ = sizeof(gr_complex);
    //        }
    //    else if (item_type_.compare("cshort") == 0)
    //        {
    //            item_size_ = sizeof(lv_16sc_t);
    //        }
    //    else
    //        {
    //            item_size_ = sizeof(gr_complex);
    //            LOG(WARNING) << item_type_ << " unknown acquisition item type";
    //        }
    //acq_parameters.it_size = item_size_;
    //acq_parameters.samples_per_code = code_length_;
    //acq_parameters.samples_per_ms = code_length_;
    //acq_parameters.sampled_ms = sampled_ms_;
    //acq_parameters.num_doppler_bins_step2 = configuration_->property(role + ".second_nbins", 4);
    //acq_parameters.doppler_step2 = configuration_->property(role + ".second_doppler_step", 125.0);
    //acq_parameters.make_2_steps = configuration_->property(role + ".make_two_steps", false);
    //acquisition_ = pcps_make_acquisition(acq_parameters);
    //acquisition_fpga_ = pcps_make_acquisition_fpga(acq_parameters);
    //DLOG(INFO) << "acquisition(" << acquisition_fpga_->unique_id() << ")";

    acq_parameters.total_block_exp = 9;

    acquisition_fpga_ = pcps_make_acquisition_fpga(acq_parameters);
    DLOG(INFO) << "acquisition(" << acquisition_fpga_->unique_id() << ")";

    //stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, vector_length_);
    channel_ = 0;
    //threshold_ = 0.0;
    doppler_step_ = 0;
    gnss_synchro_ = 0;
    //printf("creating the E5A acquisition end");
}


GalileoE5aPcpsAcquisitionFpga::~GalileoE5aPcpsAcquisitionFpga()
{
    //delete[] code_;
    delete[] d_all_fft_codes_;
}


void GalileoE5aPcpsAcquisitionFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    //acquisition_->set_channel(channel_);
    acquisition_fpga_->set_channel(channel_);
}


void GalileoE5aPcpsAcquisitionFpga::set_threshold(float threshold)
{
    //    float pfa = configuration_->property(role_ + boost::lexical_cast<std::string>(channel_) + ".pfa", 0.0);
    //
    //    if (pfa == 0.0)
    //        {
    //            pfa = configuration_->property(role_ + ".pfa", 0.0);
    //        }
    //
    //    if (pfa == 0.0)
    //        {
    //            threshold_ = threshold;
    //        }
    //
    //    else
    //        {
    //            threshold_ = calculate_threshold(pfa);
    //        }

    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold;

    //acquisition_->set_threshold(threshold_);
    acquisition_fpga_->set_threshold(threshold);
}


void GalileoE5aPcpsAcquisitionFpga::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    //acquisition_->set_doppler_max(doppler_max_);
    acquisition_fpga_->set_doppler_max(doppler_max_);
}


void GalileoE5aPcpsAcquisitionFpga::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    //acquisition_->set_doppler_step(doppler_step_);
    acquisition_fpga_->set_doppler_step(doppler_step_);
}


void GalileoE5aPcpsAcquisitionFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    //acquisition_->set_gnss_synchro(gnss_synchro_);
    acquisition_fpga_->set_gnss_synchro(gnss_synchro_);
}


signed int GalileoE5aPcpsAcquisitionFpga::mag()
{
    //return acquisition_->mag();
    return acquisition_fpga_->mag();
}


void GalileoE5aPcpsAcquisitionFpga::init()
{
    //acquisition_->init();
    acquisition_fpga_->init();
}


void GalileoE5aPcpsAcquisitionFpga::set_local_code()
{
    //    gr_complex* code = new gr_complex[code_length_];
    //    char signal_[3];
    //
    //    if (acq_iq_)
    //        {
    //            strcpy(signal_, "5X");
    //        }
    //    else if (acq_pilot_)
    //        {
    //            strcpy(signal_, "5Q");
    //        }
    //    else
    //        {
    //            strcpy(signal_, "5I");
    //        }
    //
    //    galileo_e5_a_code_gen_complex_sampled(code, signal_, gnss_synchro_->PRN, fs_in_, 0);
    //
    //    for (unsigned int i = 0; i < sampled_ms_; i++)
    //        {
    //            memcpy(code_ + (i * code_length_), code, sizeof(gr_complex) * code_length_);
    //        }

    //acquisition_->set_local_code(code_);
    acquisition_fpga_->set_local_code();
    //    delete[] code;
}


void GalileoE5aPcpsAcquisitionFpga::reset()
{
    //acquisition_->set_active(true);
    acquisition_fpga_->set_active(true);
}


//float GalileoE5aPcpsAcquisitionFpga::calculate_threshold(float pfa)
//{
//    unsigned int frequency_bins = 0;
//    for (int doppler = static_cast<int>(-doppler_max_); doppler <= static_cast<int>(doppler_max_); doppler += doppler_step_)
//        {
//            frequency_bins++;
//        }
//    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;
//    unsigned int ncells = vector_length_ * frequency_bins;
//    double exponent = 1 / static_cast<double>(ncells);
//    double val = pow(1.0 - pfa, exponent);
//    double lambda = double(vector_length_);
//    boost::math::exponential_distribution<double> mydist(lambda);
//    float threshold = static_cast<float>(quantile(mydist, val));
//
//    return threshold;
//}


void GalileoE5aPcpsAcquisitionFpga::set_state(int state)
{
    //acquisition_->set_state(state);
    acquisition_fpga_->set_state(state);
}

// this function is only used for the unit tests
void GalileoE5aPcpsAcquisitionFpga::set_single_doppler_flag(unsigned int single_doppler_flag)
{
	acquisition_fpga_->set_single_doppler_flag(single_doppler_flag);
}
// this function is only used for the unit tests
void GalileoE5aPcpsAcquisitionFpga::read_acquisition_results(uint32_t *max_index,
    float *max_magnitude, float *second_magnitude, uint64_t *initial_sample, uint32_t *doppler_index, uint32_t *total_fft_scaling_factor)

{
	acquisition_fpga_->read_acquisition_results(max_index, max_magnitude, second_magnitude,
	        initial_sample, doppler_index, total_fft_scaling_factor);
}

// this function is only used for the unit tests
void GalileoE5aPcpsAcquisitionFpga::reset_acquisition(void)
{
	acquisition_fpga_->reset_acquisition();
}

// this function is only used for the unit tests
void GalileoE5aPcpsAcquisitionFpga::read_fpga_total_scale_factor(uint32_t *total_scale_factor, uint32_t *fw_scale_factor)
{
	acquisition_fpga_->read_fpga_total_scale_factor(total_scale_factor, fw_scale_factor);
}

void GalileoE5aPcpsAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
    //    if (item_type_.compare("gr_complex") == 0)
    //        {
    //            top_block->connect(stream_to_vector_, 0, acquisition_, 0);
    //        }
    //    else if (item_type_.compare("cshort") == 0)
    //        {
    //            top_block->connect(stream_to_vector_, 0, acquisition_, 0);
    //        }
    //    else
    //        {
    //            LOG(WARNING) << item_type_ << " unknown acquisition item type";
    //        }
}


void GalileoE5aPcpsAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
    //    if (item_type_.compare("gr_complex") == 0)
    //        {
    //            top_block->disconnect(stream_to_vector_, 0, acquisition_, 0);
    //        }
    //    else if (item_type_.compare("cshort") == 0)
    //        {
    //            top_block->disconnect(stream_to_vector_, 0, acquisition_, 0);
    //        }
    //    else
    //        {
    //            LOG(WARNING) << item_type_ << " unknown acquisition item type";
    //        }
}


gr::basic_block_sptr GalileoE5aPcpsAcquisitionFpga::get_left_block()
{
    //return stream_to_vector_;
    return nullptr;
}


gr::basic_block_sptr GalileoE5aPcpsAcquisitionFpga::get_right_block()
{
    //return acquisition_;
    return acquisition_fpga_;
}
