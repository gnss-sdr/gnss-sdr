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
#include <stdexcept>
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"

using google::LogMessage;

GpsL1CaPcpsAcquisitionFpga::GpsL1CaPcpsAcquisitionFpga(
        ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams) :
        role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    unsigned int code_length;
    bool bit_transition_flag;
    bool use_CFAR_algorithm_flag;
    unsigned int sampled_ms;
    long fs_in;
    long ifreq;
    bool dump;
    std::string dump_filename;
    unsigned int nsamples_total;
    unsigned int select_queue_Fpga;
    std::string device_name;

    configuration_ = configuration;

    std::string default_item_type = "cshort";
    std::string default_dump_filename = "./data/acquisition.dat";

    DLOG(INFO) << "role " << role;

    item_type_ = configuration_->property(role + ".item_type", default_item_type);

    long fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    ifreq = configuration_->property(role + ".if", 0);
    dump = configuration_->property(role + ".dump", false);
    doppler_max_ = configuration_->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0 ) doppler_max_ = FLAGS_doppler_max;
    sampled_ms = configuration_->property(role + ".coherent_integration_time_ms", 1);

    // note : the FPGA is implemented according to bit transition flag = 0. Setting bit transition flag to 1 has no effect.
    bit_transition_flag = configuration_->property(role + ".bit_transition_flag", false);

    // note : the FPGA is implemented according to use_CFAR_algorithm = 0. Setting use_CFAR_algorithm to 1 has no effect.
    use_CFAR_algorithm_flag = configuration_->property(role + ".use_CFAR_algorithm", false);

    // note : the FPGA does not use the max_dwells variable.
    max_dwells_ = configuration_->property(role + ".max_dwells", 1);

    dump_filename = configuration_->property(role + ".dump_filename", default_dump_filename);

    //--- Find number of samples per spreading code -------------------------
    code_length = round(
            fs_in / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));

    // code length has the same value as d_fft_size
    float nbits;
    nbits = ceilf(log2f(code_length));
    nsamples_total = pow(2, nbits);

    //vector_length_ = code_length_ * sampled_ms_;
    vector_length_ = nsamples_total * sampled_ms;

    //    if( bit_transition_flag_ )
    //        {
    //            vector_length_ *= 2;
    //        }

    select_queue_Fpga = configuration_->property(role + ".select_queue_Fpga", 0);

    std::string default_device_name = "/dev/uio0";
    device_name = configuration_->property(role + ".devicename", default_device_name);

    if (item_type_.compare("cshort") == 0)
        {
            item_size_ = sizeof(lv_16sc_t);
            gps_acquisition_fpga_sc_ = gps_pcps_make_acquisition_fpga_sc(
                    sampled_ms, max_dwells_, doppler_max_, ifreq, fs_in,
                    code_length, code_length, vector_length_, nsamples_total,
                    bit_transition_flag, use_CFAR_algorithm_flag,
                    select_queue_Fpga, device_name, dump, dump_filename);
            DLOG(INFO) << "acquisition("
                    << gps_acquisition_fpga_sc_->unique_id() << ")";
        }
    else
        {
            LOG(WARNING) << "item_type configured to " << item_type_ << "but FPGA implementation only accepts cshort";
            throw std::invalid_argument( "Wrong input_type configuration. Should be cshort" );
        }

    channel_ = 0;
    threshold_ = 0.0;
    doppler_step_ = 0;
    gnss_synchro_ = 0;
}


GpsL1CaPcpsAcquisitionFpga::~GpsL1CaPcpsAcquisitionFpga()
{
}


void GpsL1CaPcpsAcquisitionFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    gps_acquisition_fpga_sc_->set_channel(channel_);
}


void GpsL1CaPcpsAcquisitionFpga::set_threshold(float threshold)
{
    float pfa = configuration_->property(role_ + ".pfa", 0.0);

    if (pfa == 0.0)
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
    gps_acquisition_fpga_sc_->set_local_code();
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
    for (int doppler = static_cast<int>(-doppler_max_); doppler <= static_cast<int>(doppler_max_);
            doppler += doppler_step_)
        {
            frequency_bins++;
        }
    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;
    unsigned int ncells = vector_length_ * frequency_bins;
    double exponent = 1 / static_cast<double>(ncells);
    double val = pow(1.0 - pfa, exponent);
    double lambda = double(vector_length_);
    boost::math::exponential_distribution<double> mydist(lambda);
    float threshold = static_cast<float>(quantile(mydist, val));

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
