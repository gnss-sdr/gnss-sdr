/*!
 * \file galileo_e1_pcps_ambiguous_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "galileo_e1_pcps_ambiguous_acquisition_fpga.h"
#include "configuration_interface.h"
#include "galileo_e1_signal_processing.h"
#include "Galileo_E1.h"
#include "gnss_sdr_flags.h"
#include <boost/lexical_cast.hpp>
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>



using google::LogMessage;

GalileoE1PcpsAmbiguousAcquisitionFpga::GalileoE1PcpsAmbiguousAcquisitionFpga(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    //printf("top acq constructor start\n");
    pcpsconf_fpga_t acq_parameters;
    configuration_ = configuration;
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./data/acquisition.dat";

    DLOG(INFO) << "role " << role;

//    item_type_ = configuration_->property(role + ".item_type", default_item_type);

    long fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 4000000);
    long fs_in = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);

    float downsampling_factor = configuration_->property("GNSS-SDR.downsampling_factor", 1.0);
    acq_parameters.downsampling_factor = downsampling_factor;
    //fs_in = fs_in/2.0; // downampling filter
    //printf("fs_in pre downsampling = %ld\n", fs_in);

    fs_in = fs_in/downsampling_factor;
    //printf("fs_in post downsampling = %ld\n", fs_in);


    acq_parameters.fs_in = fs_in;
    //if_ = configuration_->property(role + ".if", 0);
    //acq_parameters.freq = if_;

  //  dump_ = configuration_->property(role + ".dump", false);
  //  acq_parameters.dump = dump_;
  //  blocking_ = configuration_->property(role + ".blocking", true);
//    acq_parameters.blocking = blocking_;
    doppler_max_ = configuration_->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0) doppler_max_ = FLAGS_doppler_max;
    acq_parameters.doppler_max = doppler_max_;
    //unsigned int sampled_ms = 4;
    //acq_parameters.sampled_ms = sampled_ms;
    unsigned int sampled_ms = configuration_->property(role + ".coherent_integration_time_ms", 4);
    acq_parameters.sampled_ms = sampled_ms;

 //   bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);
 //   acq_parameters.bit_transition_flag = bit_transition_flag_;
 //   use_CFAR_algorithm_flag_ = configuration_->property(role + ".use_CFAR_algorithm", true);  //will be false in future versions
 //   acq_parameters.use_CFAR_algorithm_flag = use_CFAR_algorithm_flag_;
    acquire_pilot_ = configuration_->property(role + ".acquire_pilot", false);  //will be true in future versions

 //   max_dwells_ = configuration_->property(role + ".max_dwells", 1);
 //   acq_parameters.max_dwells = max_dwells_;
 //   dump_filename_ = configuration_->property(role + ".dump_filename", default_dump_filename);
 //   acq_parameters.dump_filename = dump_filename_;
    //--- Find number of samples per spreading code (4 ms)  -----------------
    unsigned int code_length = static_cast<unsigned int>(std::round(static_cast<double>(fs_in) / (Galileo_E1_CODE_CHIP_RATE_HZ / Galileo_E1_B_CODE_LENGTH_CHIPS)));
    //acq_parameters.samples_per_code = code_length_;
    //int samples_per_ms = static_cast<int>(std::round(static_cast<double>(fs_in_) * 0.001));
    //acq_parameters.samples_per_ms = samples_per_ms;
    //unsigned int vector_length = sampled_ms * samples_per_ms;

//    if (bit_transition_flag_)
//        {
//            vector_length_ *= 2;
//        }

    //printf("fs_in = %d\n", fs_in);
    //printf("Galileo_E1_B_CODE_LENGTH_CHIPS = %f\n", Galileo_E1_B_CODE_LENGTH_CHIPS);
    //printf("Galileo_E1_CODE_CHIP_RATE_HZ = %f\n", Galileo_E1_CODE_CHIP_RATE_HZ);
    //printf("acq adapter code_length = %d\n", code_length);
    acq_parameters.code_length = code_length;
    // The FPGA can only use FFT lengths that are a power of two.
    float nbits = ceilf(log2f((float)code_length));
    unsigned int nsamples_total = pow(2, nbits);
    unsigned int vector_length = nsamples_total;
    //printf("acq adapter nsamples_total (= vector_length) = %d\n", vector_length);
    unsigned int select_queue_Fpga = configuration_->property(role + ".select_queue_Fpga", 0);
    acq_parameters.select_queue_Fpga = select_queue_Fpga;
    std::string default_device_name = "/dev/uio0";
    std::string device_name = configuration_->property(role + ".devicename", default_device_name);
    acq_parameters.device_name = device_name;
    acq_parameters.samples_per_ms = nsamples_total/sampled_ms;
    acq_parameters.samples_per_code = nsamples_total;

    // compute all the GALILEO E1 PRN Codes (this is done only once upon the class constructor in order to avoid re-computing the PRN codes every time
    // a channel is assigned)
    gr::fft::fft_complex* fft_if = new gr::fft::fft_complex(nsamples_total, true);  // Direct FFT
    std::complex<float>* code = new std::complex<float>[nsamples_total];  // buffer for the local code
    gr_complex* fft_codes_padded = static_cast<gr_complex*>(volk_gnsssdr_malloc(nsamples_total * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_all_fft_codes_ = new lv_16sc_t[nsamples_total * Galileo_E1_NUMBER_OF_CODES];  // memory containing all the possible fft codes for PRN 0 to 32
    float max;                                                        // temporary maxima search

    //int tmp_re, tmp_im;

    for (unsigned int PRN = 1; PRN <= Galileo_E1_NUMBER_OF_CODES; PRN++)
        {

        //code_ = new gr_complex[vector_length_];

        bool cboc = false; // cboc is set to 0 when using the FPGA

        //std::complex<float>* code = new std::complex<float>[code_length_];

        if (acquire_pilot_ == true)
            {
                //printf("yes acquiring pilot!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1\n");
                //set local signal generator to Galileo E1 pilot component (1C)
                char pilot_signal[3] = "1C";
                galileo_e1_code_gen_complex_sampled(code, pilot_signal,
                    cboc, PRN, fs_in, 0, false);
            }
        else
            {
                char data_signal[3] = "1B";
                galileo_e1_code_gen_complex_sampled(code, data_signal,
                    cboc, PRN, fs_in, 0, false);
            }

//        for (unsigned int i = 0; i < sampled_ms / 4; i++)
//            {
//                //memcpy(&(code_[i * code_length_]), code, sizeof(gr_complex) * code_length_);
//                memcpy(&(d_all_fft_codes_[i * code_length_]), code, sizeof(gr_complex) * code_length_);
//            }


//                // debug
//                char filename[25];
//                FILE *fid;
//                sprintf(filename,"gal_prn%d.txt", PRN);
//                fid = fopen(filename, "w");
//                for (unsigned int kk=0;kk< nsamples_total; kk++)
//                    {
//                        fprintf(fid, "%f\n", code[kk].real());
//                        fprintf(fid, "%f\n", code[kk].imag());
//                    }
//                fclose(fid);


//        // fill in zero padding
        for (int s = code_length; s < nsamples_total; s++)
            {
                code[s] = std::complex<float>(static_cast<float>(0,0));
                //code[s] = 0;
            }

        memcpy(fft_if->get_inbuf(), code, sizeof(gr_complex) * nsamples_total);   // copy to FFT buffer
        fft_if->execute();                                                                 // Run the FFT of local code
        volk_32fc_conjugate_32fc(fft_codes_padded, fft_if->get_outbuf(), nsamples_total);  // conjugate values

//        // debug
//        char filename[25];
//        FILE *fid;
//        sprintf(filename,"fft_gal_prn%d.txt", PRN);
//        fid = fopen(filename, "w");
//        for (unsigned int kk=0;kk< nsamples_total; kk++)
//            {
//                fprintf(fid, "%f\n", fft_codes_padded[kk].real());
//                fprintf(fid, "%f\n", fft_codes_padded[kk].imag());
//            }
//        fclose(fid);


        // normalize the code
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
                //d_all_fft_codes_[i + nsamples_total * (PRN - 1)] = lv_16sc_t(static_cast<int>(floor(4096*fft_codes_padded[i].real() * (pow(2, 3) - 1) / max)),
                //    static_cast<int>(floor(4096*fft_codes_padded[i].imag() * (pow(2, 3) - 1) / max)));
//                d_all_fft_codes_[i + nsamples_total * (PRN - 1)] = lv_16sc_t(static_cast<int>(floor(1024*fft_codes_padded[i].real() * (pow(2, 5) - 1) / max)),
//                    static_cast<int>(floor(1024*fft_codes_padded[i].imag() * (pow(2, 5) - 1) / max)));
 //               d_all_fft_codes_[i + nsamples_total * (PRN - 1)] = lv_16sc_t(static_cast<int>(floor(256*fft_codes_padded[i].real() * (pow(2, 7) - 1) / max)),
 //                   static_cast<int>(floor(256*fft_codes_padded[i].imag() * (pow(2, 7) - 1) / max)));
//                d_all_fft_codes_[i + nsamples_total * (PRN - 1)] = lv_16sc_t(static_cast<int>(floor(16*fft_codes_padded[i].real() * (pow(2, 11) - 1) / max)),
//                    static_cast<int>(floor(16*fft_codes_padded[i].imag() * (pow(2, 11) - 1) / max)));
                d_all_fft_codes_[i + nsamples_total * (PRN - 1)] = lv_16sc_t(static_cast<int>(floor(fft_codes_padded[i].real() * (pow(2, 15) - 1) / max)),
                    static_cast<int>(floor(fft_codes_padded[i].imag() * (pow(2, 15) - 1) / max)));

//                tmp_re = static_cast<int>(floor(fft_codes_padded[i].real() * (pow(2, 7) - 1) / max));
//                tmp_im = static_cast<int>(floor(fft_codes_padded[i].imag() * (pow(2, 7) - 1) / max));

//                if (tmp_re > 127)
//                    {
//                        tmp_re = 127;
//                    }
//                if (tmp_re < -128)
//                    {
//                        tmp_re = -128;
//                    }
//                if (tmp_im > 127)
//                    {
//                        tmp_im = 127;
//                    }
//                if (tmp_im < -128)
//                    {
//                        tmp_im = -128;
//                    }
//                d_all_fft_codes_[i + nsamples_total * (PRN - 1)] = lv_16sc_t(static_cast<int>(tmp_re), static_cast<int>(tmp_im));
//
            }

//        // debug
//        char filename2[25];
//        FILE *fid2;
//        sprintf(filename2,"fft_gal_prn%d_norm.txt", PRN);
//        fid2 = fopen(filename2, "w");
//        for (unsigned int kk=0;kk< nsamples_total; kk++)
//            {
//                fprintf(fid2, "%d\n", d_all_fft_codes_[kk + nsamples_total * (PRN - 1)].real());
//                fprintf(fid2, "%d\n", d_all_fft_codes_[kk + nsamples_total * (PRN - 1)].imag());
//            }
//        fclose(fid2);


        }


//    for (unsigned int PRN = 1; PRN <= Galileo_E1_NUMBER_OF_CODES; PRN++)
//        {
//                    // debug
//                    char filename2[25];
//                    FILE *fid2;
//                    sprintf(filename2,"fft_gal_prn%d_norm_last.txt", PRN);
//                    fid2 = fopen(filename2, "w");
//                    for (unsigned int kk=0;kk< nsamples_total; kk++)
//                        {
//                            fprintf(fid2, "%d\n", d_all_fft_codes_[kk + nsamples_total * (PRN - 1)].real());
//                            fprintf(fid2, "%d\n", d_all_fft_codes_[kk + nsamples_total * (PRN - 1)].imag());
//                        }
//                    fclose(fid2);
//        }

    //acq_parameters

    acq_parameters.all_fft_codes = d_all_fft_codes_;

    // temporary buffers that we can delete
    delete[] code;
    delete fft_if;
    delete[] fft_codes_padded;

    acquisition_fpga_ = pcps_make_acquisition_fpga(acq_parameters);
    DLOG(INFO) << "acquisition(" << acquisition_fpga_->unique_id() << ")";

//    stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, vector_length_);
//    DLOG(INFO) << "stream_to_vector(" << stream_to_vector_->unique_id() << ")";

//    if (item_type_.compare("cbyte") == 0)
//        {
//            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();
//            float_to_complex_ = gr::blocks::float_to_complex::make();
//        }

    channel_ = 0;
    //threshold_ = 0.0;
    doppler_step_ = 0;
    gnss_synchro_ = 0;
    //printf("top acq constructor end\n");
}


GalileoE1PcpsAmbiguousAcquisitionFpga::~GalileoE1PcpsAmbiguousAcquisitionFpga()
{
    //printf("top acq destructor start\n");
    //delete[] code_;
    delete[] d_all_fft_codes_;
    //printf("top acq destructor end\n");
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_channel(unsigned int channel)
{
    //printf("top acq set channel start\n");
    channel_ = channel;
    acquisition_fpga_->set_channel(channel_);
    //printf("top acq set channel end\n");
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_threshold(float threshold)
{
    //printf("top acq set threshold start\n");
    // the .pfa parameter and the threshold calculation is only used for the CFAR algorithm.
    // We don't use the CFAR algorithm in the FPGA. Therefore the threshold is set as such.

//    float pfa = configuration_->property(role_ + boost::lexical_cast<std::string>(channel_) + ".pfa", 0.0);
//
//    if (pfa == 0.0) pfa = configuration_->property(role_ + ".pfa", 0.0);
//
//    if (pfa == 0.0)
//        {
//            threshold_ = threshold;
//        }
//    else
//        {
//            threshold_ = calculate_threshold(pfa);
//        }

    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold;
    acquisition_fpga_->set_threshold(threshold);
//    acquisition_fpga_->set_threshold(threshold_);
    //printf("top acq set threshold end\n");
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_doppler_max(unsigned int doppler_max)
{
    //printf("top acq set doppler max start\n");
    doppler_max_ = doppler_max;

    acquisition_fpga_->set_doppler_max(doppler_max_);
    //printf("top acq set doppler max end\n");
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_doppler_step(unsigned int doppler_step)
{
    //printf("top acq set doppler step start\n");
    doppler_step_ = doppler_step;

    acquisition_fpga_->set_doppler_step(doppler_step_);
    //printf("top acq set doppler step end\n");
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    //printf("top acq set gnss synchro start\n");
    gnss_synchro_ = gnss_synchro;

    acquisition_fpga_->set_gnss_synchro(gnss_synchro_);
    //printf("top acq set gnss synchro end\n");
}


signed int GalileoE1PcpsAmbiguousAcquisitionFpga::mag()
{
   // printf("top acq mag start\n");
    return acquisition_fpga_->mag();
    //printf("top acq mag end\n");
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::init()
{
   // printf("top acq init start\n");
    acquisition_fpga_->init();
   // printf("top acq init end\n");
    //set_local_code();
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_local_code()
{
   // printf("top acq set local code start\n");
//    bool cboc = configuration_->property(
//        "Acquisition" + boost::lexical_cast<std::string>(channel_) + ".cboc", false);
//
//    std::complex<float>* code = new std::complex<float>[code_length_];
//
//    if (acquire_pilot_ == true)
//        {
//            //set local signal generator to Galileo E1 pilot component (1C)
//            char pilot_signal[3] = "1C";
//            galileo_e1_code_gen_complex_sampled(code, pilot_signal,
//                cboc, gnss_synchro_->PRN, fs_in_, 0, false);
//        }
//    else
//        {
//            galileo_e1_code_gen_complex_sampled(code, gnss_synchro_->Signal,
//                cboc, gnss_synchro_->PRN, fs_in_, 0, false);
//        }
//
//
//    for (unsigned int i = 0; i < sampled_ms_ / 4; i++)
//        {
//            memcpy(&(code_[i * code_length_]), code, sizeof(gr_complex) * code_length_);
//        }

    //acquisition_fpga_->set_local_code(code_);
    acquisition_fpga_->set_local_code();
//    delete[] code;
  //  printf("top acq set local code end\n");
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::reset()
{
 //   printf("top acq reset start\n");
    acquisition_fpga_->set_active(true);
  //  printf("top acq reset end\n");
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::set_state(int state)
{
  //  printf("top acq set state start\n");
    acquisition_fpga_->set_state(state);
  //  printf("top acq set state end\n");
}


//float GalileoE1PcpsAmbiguousAcquisitionFpga::calculate_threshold(float pfa)
//{
//    unsigned int frequency_bins = 0;
//    for (int doppler = static_cast<int>(-doppler_max_); doppler <= static_cast<int>(doppler_max_); doppler += doppler_step_)
//        {
//            frequency_bins++;
//        }
//
//    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;
//
//    unsigned int ncells = vector_length_ * frequency_bins;
//    double exponent = 1 / static_cast<double>(ncells);
//    double val = pow(1.0 - pfa, exponent);
//    double lambda = double(vector_length_);
//    boost::math::exponential_distribution<double> mydist(lambda);
//    float threshold = static_cast<float>(quantile(mydist, val));
//
//    return threshold;
//}

// this function is only used for the unit tests
void GalileoE1PcpsAmbiguousAcquisitionFpga::set_single_doppler_flag(unsigned int single_doppler_flag)
{
	acquisition_fpga_->set_single_doppler_flag(single_doppler_flag);
}
// this function is only used for the unit tests
void GalileoE1PcpsAmbiguousAcquisitionFpga::read_acquisition_results(uint32_t *max_index,
    float *max_magnitude, uint64_t *initial_sample, float *power_sum, uint32_t *doppler_index)
{
	acquisition_fpga_->read_acquisition_results(max_index, max_magnitude,
	        initial_sample, power_sum, doppler_index);
}

// this function is only used for the unit tests
void GalileoE1PcpsAmbiguousAcquisitionFpga::reset_acquisition(void)
{
	acquisition_fpga_->reset_acquisition();
}

// this function is only used for the unit tests
void GalileoE1PcpsAmbiguousAcquisitionFpga::read_fpga_total_scale_factor(uint32_t *total_scale_factor, uint32_t *fw_scale_factor)
{
	acquisition_fpga_->read_fpga_total_scale_factor(total_scale_factor, fw_scale_factor);
}

void GalileoE1PcpsAmbiguousAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
  //  printf("top acq connect\n");
//    if (item_type_.compare("gr_complex") == 0)
//        {
//            top_block->connect(stream_to_vector_, 0, acquisition_fpga_, 0);
//        }
//    else if (item_type_.compare("cshort") == 0)
//        {
//            top_block->connect(stream_to_vector_, 0, acquisition_fpga_, 0);
//        }
//    else if (item_type_.compare("cbyte") == 0)
//        {
//            top_block->connect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
//            top_block->connect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
//            top_block->connect(float_to_complex_, 0, stream_to_vector_, 0);
//            top_block->connect(stream_to_vector_, 0, acquisition_fpga_, 0);
//        }
//    else
//        {
//            LOG(WARNING) << item_type_ << " unknown acquisition item type";
//        }

    // nothing to connect
}


void GalileoE1PcpsAmbiguousAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
//    if (item_type_.compare("gr_complex") == 0)
//        {
//            top_block->disconnect(stream_to_vector_, 0, acquisition_fpga_, 0);
//        }
//    else if (item_type_.compare("cshort") == 0)
//        {
//            top_block->disconnect(stream_to_vector_, 0, acquisition_fpga_, 0);
//        }
//    else if (item_type_.compare("cbyte") == 0)
//        {
//            // Since a byte-based acq implementation is not available,
//            // we just convert cshorts to gr_complex
//            top_block->disconnect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
//            top_block->disconnect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
//            top_block->disconnect(float_to_complex_, 0, stream_to_vector_, 0);
//            top_block->disconnect(stream_to_vector_, 0, acquisition_fpga_, 0);
//        }
//    else
//        {
//            LOG(WARNING) << item_type_ << " unknown acquisition item type";
//        }

    // nothing to disconnect
 //   printf("top acq disconnect\n");
}


gr::basic_block_sptr GalileoE1PcpsAmbiguousAcquisitionFpga::get_left_block()
{
 //   printf("top acq get left block start\n");
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
            return nullptr;
//        }
   //         printf("top acq get left block end\n");
}


gr::basic_block_sptr GalileoE1PcpsAmbiguousAcquisitionFpga::get_right_block()
{
 //   printf("top acq get right block start\n");
    return acquisition_fpga_;
 //   printf("top acq get right block end\n");
}

