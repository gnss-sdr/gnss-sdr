/*!
 * \file gps_l2_m_telemetry_decoder.h
 * \brief Interface of an adapter of a GPS L1 C/A NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_L2_M_TELEMETRY_DECODER_H_
#define GNSS_SDR_GPS_L2_M_TELEMETRY_DECODER_H_

#include <string>
#include <gnuradio/msg_queue.h>
#include "telemetry_decoder_interface.h"
#include "gps_l2_m_telemetry_decoder_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements a NAV data decoder for GPS L2 M
 */
class GpsL2MTelemetryDecoder : public TelemetryDecoderInterface
{
public:
    GpsL2MTelemetryDecoder(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams,
            boost::shared_ptr<gr::msg_queue> queue);

    virtual ~GpsL2MTelemetryDecoder();
    std::string role()
    {
        return role_;
    }

    //! Returns "gps_l2_m_telemetry_decoder"
    std::string implementation()
    {
        return "gps_l2_m_telemetry_decoder";
    }
    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();
    void set_satellite(Gnss_Satellite satellite);
    void set_channel(int channel){telemetry_decoder_->set_channel(channel);}
    void reset()
    {
        return;
    }
    size_t item_size()
    {
        return 0;
    }

private:
    gps_l2_m_telemetry_decoder_cc_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    int channel_;
    unsigned int vector_length_;
    std::string item_type_;
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif
