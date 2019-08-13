/*!
 * \file gps_l1_ca_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A Signals
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "GPS_L1_CA.h"
#include "acq_conf.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gps_sdr_signal_processing.h"
#include <glog/logging.h>


GpsL1CaPcpsAcquisitionFineDoppler::GpsL1CaPcpsAcquisitionFineDoppler(
    ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./acquisition.mat";

    DLOG(INFO) << "role " << role;
    Acq_Conf acq_parameters = Acq_Conf();

    item_type_ = configuration->property(role + ".item_type", default_item_type);
    int64_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in_ = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    acq_parameters.fs_in = fs_in_;
    acq_parameters.samples_per_chip = static_cast<unsigned int>(ceil(GPS_L1_CA_CHIP_PERIOD * static_cast<float>(acq_parameters.fs_in)));
    dump_ = configuration->property(role + ".dump", false);
    acq_parameters.dump = dump_;
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    acq_parameters.dump_filename = dump_filename_;
    doppler_max_ = configuration->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0)
        {
            doppler_max_ = FLAGS_doppler_max;
        }
    acq_parameters.doppler_max = doppler_max_;
    sampled_ms_ = configuration->property(role + ".coherent_integration_time_ms", 1);
    acq_parameters.sampled_ms = sampled_ms_;
    max_dwells_ = configuration->property(role + ".max_dwells", 1);
    acq_parameters.max_dwells = max_dwells_;

    acq_parameters.blocking_on_standby = configuration->property(role + ".blocking_on_standby", false);

    //--- Find number of samples per spreading code -------------------------
    vector_length_ = round(fs_in_ / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));
    acq_parameters.samples_per_ms = vector_length_;
    code_ = std::vector<std::complex<float>>(vector_length_);

    if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            acquisition_cc_ = pcps_make_acquisition_fine_doppler_cc(acq_parameters);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }

    channel_ = 0;
    threshold_ = 0.0;
    doppler_step_ = 0;
    gnss_synchro_ = nullptr;

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


void GpsL1CaPcpsAcquisitionFineDoppler::stop_acquisition()
{
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_threshold(float threshold)
{
    threshold_ = threshold;
    acquisition_cc_->set_threshold(threshold_);
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;
    acquisition_cc_->set_doppler_max(doppler_max_);
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;
    acquisition_cc_->set_doppler_step(doppler_step_);
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;
    acquisition_cc_->set_gnss_synchro(gnss_synchro_);
}


signed int GpsL1CaPcpsAcquisitionFineDoppler::mag()
{
    return acquisition_cc_->mag();
}


void GpsL1CaPcpsAcquisitionFineDoppler::init()
{
    acquisition_cc_->init();
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_local_code()
{
    gps_l1_ca_code_gen_complex_sampled(code_, gnss_synchro_->PRN, fs_in_, 0);
    acquisition_cc_->set_local_code(code_.data());
}


void GpsL1CaPcpsAcquisitionFineDoppler::reset()
{
    acquisition_cc_->set_active(true);
}


void GpsL1CaPcpsAcquisitionFineDoppler::set_state(int state)
{
    acquisition_cc_->set_state(state);
}


void GpsL1CaPcpsAcquisitionFineDoppler::connect(boost::shared_ptr<gr::top_block> top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}


void GpsL1CaPcpsAcquisitionFineDoppler::disconnect(boost::shared_ptr<gr::top_block> top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}


boost::shared_ptr<gr::basic_block> GpsL1CaPcpsAcquisitionFineDoppler::get_left_block()
{
    return acquisition_cc_;
}


boost::shared_ptr<gr::basic_block> GpsL1CaPcpsAcquisitionFineDoppler::get_right_block()
{
    return acquisition_cc_;
}
