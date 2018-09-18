/*!
 * \file galileo_e5a_telemetry_decoder.h
 * \brief Interface of an adapter of a GALILEO E5a FNAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *         <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          </ul>
 *
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

#ifndef GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_H_
#define GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_H_

#include "galileo_telemetry_decoder_cc.h"
#include "telemetry_decoder_interface.h"
#include <string>

class ConfigurationInterface;

/*!
 * \brief This class implements a NAV data decoder for Galileo INAV frames in E1B radio link
 */
class GalileoE5aTelemetryDecoder : public TelemetryDecoderInterface
{
public:
    GalileoE5aTelemetryDecoder(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams);

    virtual ~GalileoE5aTelemetryDecoder();

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Galileo_E5a_Telemetry_Decoder"
     */
    inline std::string implementation() override
    {
        return "Galileo_E5A_Telemetry_Decoder";
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    void set_satellite(const Gnss_Satellite& satellite) override;
    inline void set_channel(int channel) override { telemetry_decoder_->set_channel(channel); }
    inline void reset() override
    {
        return;
    }

    inline size_t item_size() override
    {
        return 0;
    }

private:
    galileo_telemetry_decoder_cc_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    int channel_;
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif /* GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_H_ */
