/*!
 * \file sbas_l1_telemetry_decoder.h
 * \brief Interface of an adapter of a SBAS telemetry data decoder block
 * to a TelemetryDecoderInterface
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_SBAS_L1_TELEMETRY_DECODER_H
#define GNSS_SDR_SBAS_L1_TELEMETRY_DECODER_H


#include "gnss_satellite.h"  // for Gnss_Satellite
#include "gnss_synchro.h"
#include "sbas_l1_telemetry_decoder_gs.h"
#include "telemetry_decoder_interface.h"
#include <gnuradio/runtime_types.h>  // for basic_block_sptr, top_block_sptr
#include <cstddef>                   // for size_t
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */

class ConfigurationInterface;

/*!
 * \brief This class implements a NAV data decoder for SBAS frames in L1 radio link
 */
class SbasL1TelemetryDecoder : public TelemetryDecoderInterface
{
public:
    SbasL1TelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "SBAS_L1_Telemetry_Decoder"
     */
    inline std::string implementation() override
    {
        return "SBAS_L1_Telemetry_Decoder";
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    void set_satellite(const Gnss_Satellite& satellite) override;

    inline void set_channel(int channel) override { telemetry_decoder_->set_channel(channel); }

    inline void reset() override
    {
        telemetry_decoder_->reset();
    }

    inline size_t item_size() override
    {
        return sizeof(Gnss_Synchro);
    }

private:
    sbas_l1_telemetry_decoder_gs_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SBAS_L1_TELEMETRY_DECODER_H
