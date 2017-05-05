/*!
 * \file gps_l1_ca_pcps_acquisition_fpga.cc
 * \brief Adapts a PCPS acquisition block to an FPGA Acquisition Interface for
 *  GPS L1 C/A signals. This file is based on the file gps_l1_ca_pcps_acquisition.cc
 * \authors <ul>
 * 			<li> Marc Majoral, 2017. mmajoral(at)cttc.cat
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#include "gps_l1_ca_pcps_acquisition_fpga.h"
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include "gps_sdr_signal_processing.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"


using google::LogMessage;

GpsL1CaPcpsAcquisitionFpga::GpsL1CaPcpsAcquisitionFpga(
        ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams) :
    role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    configuration_ = configuration;

    std::string default_item_type = "cshort";
    std::string default_dump_filename = "./data/acquisition.dat";

    DLOG(INFO) << "role " << role;

    item_type_ = configuration_->property(role + ".item_type", default_item_type);

    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_hz", 2048000);
    if_ = configuration_->property(role + ".if", 0);
    dump_ = configuration_->property(role + ".dump", false);
    doppler_max_ = configuration_->property(role + ".doppler_max", 5000);
    sampled_ms_ = configuration_->property(role + ".coherent_integration_time_ms", 1);

    // note : the FPGA is implemented according to bit transition flag = 0. Setting bit transition flag to 1 has no effect.
    bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);

    // note : the FPGA is implemented according to use_CFAR_algorithm = 0. Setting use_CFAR_algorithm to 1 has no effect.
    use_CFAR_algorithm_flag_=configuration_->property(role + ".use_CFAR_algorithm", false);

    max_dwells_ = configuration_->property(role + ".max_dwells", 1);

    dump_filename_ = configuration_->property(role + ".dump_filename", default_dump_filename);

    //--- Find number of samples per spreading code -------------------------
    code_length_ = round(fs_in_ / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));

    // code length has the same value as d_fft_size
	float nbits;
	nbits = ceilf(log2f(code_length_));
	nsamples_total_ = pow(2,nbits);

    //vector_length_ = code_length_ * sampled_ms_;
    vector_length_ = nsamples_total_ * sampled_ms_;


    if( bit_transition_flag_ )
        {
            vector_length_ *= 2;
        }

    code_ = new gr_complex[vector_length_];

    if (item_type_.compare("cshort") == 0 )
        {
            item_size_ = sizeof(lv_16sc_t);
            gps_acquisition_fpga_sc_ = gps_pcps_make_acquisition_fpga_sc(sampled_ms_, max_dwells_,
                    doppler_max_, if_, fs_in_, code_length_, code_length_, vector_length_,
                    bit_transition_flag_, use_CFAR_algorithm_flag_, dump_, dump_filename_);
            DLOG(INFO) << "acquisition(" << gps_acquisition_fpga_sc_->unique_id() << ")";

        }
    else{
    		LOG(FATAL) << item_type_ << " FPGA only accepts chsort";
    	}

    channel_ = 0;
    threshold_ = 0.0;
    doppler_step_ = 0;
    gnss_synchro_ = 0;
}


GpsL1CaPcpsAcquisitionFpga::~GpsL1CaPcpsAcquisitionFpga()
{
    delete[] code_;
}


void GpsL1CaPcpsAcquisitionFpga::set_channel(unsigned int channel)
{
    channel_ = channel;

    gps_acquisition_fpga_sc_->set_channel(channel_);

}


void GpsL1CaPcpsAcquisitionFpga::set_threshold(float threshold)
{
    float pfa = configuration_->property(role_ + ".pfa", 0.0);

    if(pfa == 0.0)
        {
            threshold_ = threshold;
        }
    else
        {
            threshold_ = calculate_threshold(pfa);
        }

    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold_;


    gps_acquisition_fpga_sc_->set_threshold(threshold_);

}


void GpsL1CaPcpsAcquisitionFpga::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    gps_acquisition_fpga_sc_->set_doppler_max(doppler_max_);

}


void GpsL1CaPcpsAcquisitionFpga::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;

    gps_acquisition_fpga_sc_->set_doppler_step(doppler_step_);

}

void GpsL1CaPcpsAcquisitionFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;

    gps_acquisition_fpga_sc_->set_gnss_synchro(gnss_synchro_);
}


signed int GpsL1CaPcpsAcquisitionFpga::mag()
{

	return gps_acquisition_fpga_sc_->mag();
}


void GpsL1CaPcpsAcquisitionFpga::init()
{

	gps_acquisition_fpga_sc_->init();

    set_local_code();
}


void GpsL1CaPcpsAcquisitionFpga::set_local_code()
{

    std::complex<float>* code = new std::complex<float>[vector_length_];


    //init to zeros for the zero padding of the fft
    for (uint s=0;s<vector_length_;s++)
    {
    	code[s] = std::complex<float>(0, 0);
    }

    unsigned long long interpolated_sampling_frequency; // warning: we need a long long to do this conversion to avoid running out of bits

    gps_l1_ca_code_gen_complex_sampled(code, gnss_synchro_->PRN, fs_in_ , 0);

    for (unsigned int i = 0; i < sampled_ms_; i++)
	{
		memcpy(&(code_[i*vector_length_]), code, sizeof(gr_complex)*vector_length_);

	}

    gps_acquisition_fpga_sc_->set_local_code(code_);

    delete[] code;
}


void GpsL1CaPcpsAcquisitionFpga::reset()
{

	gps_acquisition_fpga_sc_->set_active(true);

}


void GpsL1CaPcpsAcquisitionFpga::set_state(int state)
{

	gps_acquisition_fpga_sc_->set_state(state);
}



float GpsL1CaPcpsAcquisitionFpga::calculate_threshold(float pfa)
{
    //Calculate the threshold
    unsigned int frequency_bins = 0;
    for (int doppler = (int)(-doppler_max_); doppler <= (int)doppler_max_; doppler += doppler_step_)
        {
            frequency_bins++;
        }
    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;
    unsigned int ncells = vector_length_ * frequency_bins;
    double exponent = 1 / static_cast<double>(ncells);
    double val = pow(1.0 - pfa, exponent);
    double lambda = double(vector_length_);
    boost::math::exponential_distribution<double> mydist (lambda);
    float threshold = (float)quantile(mydist,val);

    return threshold;
}


void GpsL1CaPcpsAcquisitionFpga::connect(gr::top_block_sptr top_block)
{

	//nothing to connect
}


void GpsL1CaPcpsAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{

	//nothing to disconnect
}


gr::basic_block_sptr GpsL1CaPcpsAcquisitionFpga::get_left_block()
{

	return gps_acquisition_fpga_sc_;

}


gr::basic_block_sptr GpsL1CaPcpsAcquisitionFpga::get_right_block()
{

	return gps_acquisition_fpga_sc_;

}

