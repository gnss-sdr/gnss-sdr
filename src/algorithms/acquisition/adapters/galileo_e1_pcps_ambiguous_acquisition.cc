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

#include "galileo_e1_pcps_ambiguous_acquisition.h"
#include <boost/lexical_cast.hpp>
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include "galileo_e1_signal_processing.h"
#include "Galileo_E1.h"
#include "configuration_interface.h"

using google::LogMessage;

GalileoE1PcpsAmbiguousAcquisition::GalileoE1PcpsAmbiguousAcquisition(
        ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams) :
        role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    configuration_ = configuration;
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./data/acquisition.dat";

    DLOG(INFO) << "role " << role;

    item_type_ = configuration_->property(role + ".item_type", default_item_type);

    long fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 4000000);
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    if_ = configuration_->property(role + ".if", 0);
    dump_ = configuration_->property(role + ".dump", false);
    blocking_ = configuration_->property(role + ".blocking", true);
    doppler_max_ = configuration_->property(role + ".doppler_max", 5000);
    sampled_ms_ = configuration_->property(role + ".coherent_integration_time_ms", 4);

    if (sampled_ms_ % 4 != 0)
        {
            sampled_ms_ = static_cast<int>(sampled_ms_ / 4) * 4;
            LOG(WARNING) << "coherent_integration_time should be multiple of "
                         << "Galileo code length (4 ms). coherent_integration_time = "
                         << sampled_ms_ << " ms will be used.";
        }

    bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);
    use_CFAR_algorithm_flag_ = configuration_->property(role + ".use_CFAR_algorithm", true); //will be false in future versions
    acquire_pilot_= configuration_->property(role + ".acquire_pilot", false); //will be true in future versions

    max_dwells_ = configuration_->property(role + ".max_dwells", 1);

    dump_filename_ = configuration_->property(role + ".dump_filename", default_dump_filename);

    //--- Find number of samples per spreading code (4 ms)  -----------------
    code_length_ = round(fs_in_ / (Galileo_E1_CODE_CHIP_RATE_HZ / Galileo_E1_B_CODE_LENGTH_CHIPS));
    int samples_per_ms = round(code_length_ / 4.0);
    vector_length_ = sampled_ms_ * samples_per_ms;

    if( bit_transition_flag_ )
        {
            vector_length_ *= 2;
        }

    code_ = new gr_complex[vector_length_];

    if (item_type_.compare("cshort") == 0 )
        {
            item_size_ = sizeof(lv_16sc_t);
            acquisition_sc_ = pcps_make_acquisition_sc(sampled_ms_, max_dwells_,
                    doppler_max_, if_, fs_in_, samples_per_ms, code_length_,
                    bit_transition_flag_, use_CFAR_algorithm_flag_, dump_, blocking_,
                    dump_filename_);
            DLOG(INFO) << "acquisition(" << acquisition_sc_->unique_id() << ")";

        }
    else
        {
            item_size_ = sizeof(gr_complex);
            acquisition_cc_ = pcps_make_acquisition_cc(sampled_ms_, max_dwells_,
                    doppler_max_, if_, fs_in_, samples_per_ms, code_length_,
                    bit_transition_flag_, use_CFAR_algorithm_flag_, dump_, blocking_,
                    dump_filename_);
            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }

    stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, vector_length_);
    DLOG(INFO) << "stream_to_vector(" << stream_to_vector_->unique_id() << ")";
    
    if (item_type_.compare("cbyte") == 0)
        {
            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();
            float_to_complex_ = gr::blocks::float_to_complex::make();
        }

    channel_ = 0;
    threshold_ = 0.0;
    doppler_step_ = 0;
    gnss_synchro_ = 0;
}


GalileoE1PcpsAmbiguousAcquisition::~GalileoE1PcpsAmbiguousAcquisition()
{
    delete[] code_;
}


void GalileoE1PcpsAmbiguousAcquisition::set_channel(unsigned int channel)
{
    channel_ = channel;
    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_channel(channel_);
        }
    else
        {
            acquisition_cc_->set_channel(channel_);
        }
}


void GalileoE1PcpsAmbiguousAcquisition::set_threshold(float threshold)
{
    float pfa = configuration_->property(role_+ boost::lexical_cast<std::string>(channel_) + ".pfa", 0.0);

    if(pfa == 0.0) pfa = configuration_->property(role_ + ".pfa", 0.0);

    if(pfa == 0.0)
        {
            threshold_ = threshold;
        }
    else
        {
            threshold_ = calculate_threshold(pfa);
        }

    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold_;

    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_threshold(threshold_);
        }
    else
        {
            acquisition_cc_->set_threshold(threshold_);
        }
}


void GalileoE1PcpsAmbiguousAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_doppler_max(doppler_max_);
        }
    else
        {
            acquisition_cc_->set_doppler_max(doppler_max_);
        }
}


void GalileoE1PcpsAmbiguousAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;

    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_doppler_step(doppler_step_);
        }
    else
        {
            acquisition_cc_->set_doppler_step(doppler_step_);
        }
}


void GalileoE1PcpsAmbiguousAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;

    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_gnss_synchro(gnss_synchro_);
        }
    else
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro_);
        }
}


signed int GalileoE1PcpsAmbiguousAcquisition::mag()
{
    if (item_type_.compare("cshort") == 0)
        {
            return acquisition_sc_->mag();
        }
    else
        {
            return acquisition_cc_->mag();
        }
}


void GalileoE1PcpsAmbiguousAcquisition::init()
{
    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->init();
        }
    else
        {
            acquisition_cc_->init();
        }

    //set_local_code();
}


void GalileoE1PcpsAmbiguousAcquisition::set_local_code()
{
    bool cboc = configuration_->property(
                    "Acquisition" + boost::lexical_cast<std::string>(channel_)
                    + ".cboc", false);

    std::complex<float> * code = new std::complex<float>[code_length_];

    if (acquire_pilot_==true)
    {
        //set local signal generator to Galileo E1 pilot component (1C)
        char pilot_signal[3]="1C";
        galileo_e1_code_gen_complex_sampled(code, pilot_signal,
                        cboc, gnss_synchro_->PRN, fs_in_, 0, false);
    }else
    {
        galileo_e1_code_gen_complex_sampled(code, gnss_synchro_->Signal,
                        cboc, gnss_synchro_->PRN, fs_in_, 0, false);
    }


    for (unsigned int i = 0; i < sampled_ms_ / 4; i++)
        {
            memcpy(&(code_[i*code_length_]), code, sizeof(gr_complex)*code_length_);
        }

    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_local_code(code_);
        }
    else
        {
            acquisition_cc_->set_local_code(code_);
        }

    delete[] code;
}


void GalileoE1PcpsAmbiguousAcquisition::reset()
{
    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_active(true);
        }
    else
        {
            acquisition_cc_->set_active(true);
        }
}


void GalileoE1PcpsAmbiguousAcquisition::set_state(int state)
{
    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_state(state);
        }
    else
        {
            acquisition_cc_->set_state(state);
        }
}


float GalileoE1PcpsAmbiguousAcquisition::calculate_threshold(float pfa)
{
    unsigned int frequency_bins = 0;
    for (int doppler = static_cast<int>(-doppler_max_); doppler <= static_cast<int>(doppler_max_); doppler += doppler_step_)
        {
            frequency_bins++;
        }

    DLOG(INFO) <<"Channel "<<channel_<<"  Pfa = "<< pfa;

    unsigned int ncells = vector_length_ * frequency_bins;
    double exponent = 1 / static_cast<double>(ncells);
    double val = pow(1.0 - pfa,exponent);
    double lambda = double(vector_length_);
    boost::math::exponential_distribution<double> mydist (lambda);
    float threshold = static_cast<float>(quantile(mydist,val));

    return threshold;
}


void GalileoE1PcpsAmbiguousAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
    else if (item_type_.compare("cshort") == 0)
        {
            top_block->connect(stream_to_vector_, 0, acquisition_sc_, 0);
        }
    else if (item_type_.compare("cbyte") == 0)
        {
            top_block->connect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->connect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->connect(float_to_complex_, 0, stream_to_vector_, 0);
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


void GalileoE1PcpsAmbiguousAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
    else if (item_type_.compare("cshort") == 0)
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_sc_, 0);
        }
    else if (item_type_.compare("cbyte") == 0)
        {
            // Since a byte-based acq implementation is not available,
            // we just convert cshorts to gr_complex
            top_block->disconnect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->disconnect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->disconnect(float_to_complex_, 0, stream_to_vector_, 0);
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


gr::basic_block_sptr GalileoE1PcpsAmbiguousAcquisition::get_left_block()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            return stream_to_vector_;
        }
    else if (item_type_.compare("cshort") == 0)
        {
            return stream_to_vector_;
        }
    else if (item_type_.compare("cbyte") == 0)
        {
            return cbyte_to_float_x2_;
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
            return nullptr;
        }
}


gr::basic_block_sptr GalileoE1PcpsAmbiguousAcquisition::get_right_block()
{
    if (item_type_.compare("cshort") == 0)
        {
            return acquisition_sc_;
        }
    else
        {
            return acquisition_cc_;
        }
}

