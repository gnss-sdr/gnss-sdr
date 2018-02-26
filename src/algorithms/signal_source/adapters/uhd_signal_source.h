/*!
 * \file uhd_signal_source.h
 * \brief Interface for the Universal Hardware Driver signal source
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_UHD_SIGNAL_SOURCE_H_
#define GNSS_SDR_UHD_SIGNAL_SOURCE_H_

#include "gnss_block_interface.h"
#include <boost/shared_ptr.hpp>
#include <gnuradio/hier_block2.h>
#include <gnuradio/uhd/usrp_source.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/msg_queue.h>
#include <string>
#include <vector>


class ConfigurationInterface;

/*!
 * \brief This class reads samples from a UHD device (see http://code.ettus.com/redmine/ettus/projects/uhd/wiki)
 */
class UhdSignalSource: public GNSSBlockInterface
{

public:
    UhdSignalSource(ConfigurationInterface* configuration,
            std::string role, unsigned int in_stream,
            unsigned int out_stream, boost::shared_ptr<gr::msg_queue> queue);

    virtual ~UhdSignalSource();

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "UHD_Signal_Source"
     */
    inline std::string implementation() override
    {
        return "UHD_Signal_Source";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;
    gr::basic_block_sptr get_right_block(int RF_channel) override;

private:
    std::string role_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    gr::uhd::usrp_source::sptr uhd_source_;

    // UHD SETTINGS
    uhd::stream_args_t uhd_stream_args_;
    std::string device_address_;
    double sample_rate_;
    int RF_channels_;
    std::string item_type_;
    size_t item_size_;

    std::string subdevice_;
    std::string clock_source_;

    std::vector<double> freq_;
    std::vector<double> gain_;
    std::vector<double> IF_bandwidth_hz_;
    std::vector<long> samples_;
    std::vector<bool> dump_;
    std::vector<std::string> dump_filename_;

    std::vector<boost::shared_ptr<gr::block>> valve_;
    std::vector<gr::blocks::file_sink::sptr> file_sink_;

    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif /*GNSS_SDR_UHD_SIGNAL_SOURCE_H_*/
