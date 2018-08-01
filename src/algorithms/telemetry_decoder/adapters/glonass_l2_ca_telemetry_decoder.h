/*!
 * \file glonass_l2_ca_telemetry_decoder.h
 * \brief Interface of an adapter of a GLONASS L2 C/A NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Damian Miralles, 2018. dmiralles2009(at)gmail.com
 *
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GLONASS_L2_CA_TELEMETRY_DECODER_H_
#define GNSS_SDR_GLONASS_L2_CA_TELEMETRY_DECODER_H_

#include "telemetry_decoder_interface.h"
#include "glonass_l2_ca_telemetry_decoder_cc.h"
#include <string>

class ConfigurationInterface;

/*!
 * \brief This class implements a NAV data decoder for GLONASS L2 C/A
 */
class GlonassL2CaTelemetryDecoder : public TelemetryDecoderInterface
{
public:
    GlonassL2CaTelemetryDecoder(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams);

    virtual ~GlonassL2CaTelemetryDecoder();
    std::string role() override
    {
        return role_;
    }

    //! Returns "GLONASS_L2_CA_Telemetry_Decoder"
    std::string implementation() override
    {
        return "GLONASS_L2_CA_Telemetry_Decoder";
    }
    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;
    void set_satellite(const Gnss_Satellite& satellite) override;
    void set_channel(int channel) override { telemetry_decoder_->set_channel(channel); }
    void reset() override
    {
        return;
    }
    size_t item_size() override
    {
        return 0;
    }

private:
    glonass_l2_ca_telemetry_decoder_cc_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    int channel_;
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif
