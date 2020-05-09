/*!
 * \file glonass_l2_ca_telemetry_decoder.h
 * \brief Interface of an adapter of a GLONASS L2 C/A NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Damian Miralles, 2018. dmiralles2009(at)gmail.com
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
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GLONASS_L2_CA_TELEMETRY_DECODER_H
#define GNSS_SDR_GLONASS_L2_CA_TELEMETRY_DECODER_H

#include "glonass_l2_ca_telemetry_decoder_gs.h"
#include "gnss_satellite.h"  // for Gnss_Satellite
#include "telemetry_decoder_interface.h"
#include <gnuradio/runtime_types.h>  // for basic_block_sptr, top_block_sptr
#include <cstddef>                   // for size_t
#include <string>

class ConfigurationInterface;

/*!
 * \brief This class implements a NAV data decoder for GLONASS L2 C/A
 */
class GlonassL2CaTelemetryDecoder : public TelemetryDecoderInterface
{
public:
    GlonassL2CaTelemetryDecoder(ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GlonassL2CaTelemetryDecoder() = default;
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
    inline void reset() override
    {
        telemetry_decoder_->reset();
        return;
    }
    size_t item_size() override
    {
        return 0;
    }

private:
    glonass_l2_ca_telemetry_decoder_gs_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    int channel_;
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif
