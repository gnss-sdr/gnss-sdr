/*!
 * \file beidou_b1i_telemetry_decoder.h
 * \brief Interface of an adapter of a BEIDOU B1I NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Enric Juan, 2016. enric.juan.92(at)gmail.com
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


#ifndef GNSS_SDR_BEIDOU_B1I_TELEMETRY_DECODER_H_
#define GNSS_SDR_BEIDOU_B1I_TELEMETRY_DECODER_H_

#include <string>
#include "telemetry_decoder_interface.h"
#include "beidou_b1i_telemetry_decoder_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements a NAV data decoder for beidou b1i
 */
class BeiDouB1iTelemetryDecoder : public TelemetryDecoderInterface
{
public:
    BeiDouB1iTelemetryDecoder(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams);

    virtual ~BeiDouB1iTelemetryDecoder();
    std::string role()
    {
        return role_;
    }

    //! Returns "BeiDou_B1I_Telemetry_Decoder"
    std::string implementation()
    {
        return "BeiDou_B1I_Telemetry_Decoder";
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
    beidou_b1i_telemetry_decoder_cc_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    int channel_;
    std::string item_type_;
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif /* GNSS_SDR_BEIDOU_B1I_TELEMETRY_DECODER_H_ */
