/*!
 * \file galileo_e6_telemetry_decoder.h
 * \brief Interface of an adapter of a GALILEO E6 CNAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Carles Fernandez-Prades, 2020 cfernandez@cttc.es
 *
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


#ifndef GNSS_SDR_GALILEO_E6_TELEMETRY_DECODER_H
#define GNSS_SDR_GALILEO_E6_TELEMETRY_DECODER_H

#include "galileo_telemetry_decoder_gs.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "telemetry_decoder_interface.h"
#include "tlm_conf.h"
#include <gnuradio/runtime_types.h>  // for basic_block_sptr, top_block_sptr
#include <cstddef>                   // for size_t
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class implements a NAV data decoder for Galileo CNAV frames in E6 radio link
 */
class GalileoE6TelemetryDecoder : public TelemetryDecoderInterface
{
public:
    GalileoE6TelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GalileoE6TelemetryDecoder() = default;

    /*!
     * \brief Returns "Galileo_E6_Telemetry_Decoder"
     */
    inline std::string implementation() override
    {
        return "Galileo_E6_Telemetry_Decoder";
    }

    /*!
     * \brief Connect
     */
    void connect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Disconnect
     */
    void disconnect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Get left block
     */
    gr::basic_block_sptr get_left_block() override;

    /*!
     * \brief Get right block
     */
    gr::basic_block_sptr get_right_block() override;

    void set_satellite(const Gnss_Satellite& satellite) override;

    inline std::string role() override
    {
        return role_;
    }

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
    galileo_telemetry_decoder_gs_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    Tlm_Conf tlm_parameters_;
    std::string role_;
    int channel_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E6_TELEMETRY_DECODER_H
