/*!
 * \file telemetry_decoder_adapter_base.h
 * \brief Common functionality for telemetry decoder adapters
 * \author Carles Fernandez-Prades, 2025 cfernandez@cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_TELEMETRY_DECODER_ADAPTER_BASE_H
#define GNSS_SDR_TELEMETRY_DECODER_ADAPTER_BASE_H

#include "configuration_interface.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "telemetry_decoder_interface.h"
#include "telemetry_impl_base.h"
#include "tlm_conf.h"
#include <gnuradio/runtime_types.h>
#include <cstddef>
#include <string>

class ConfigurationInterface;

/** \addtogroup Telemetry_Decoder
 * \{
 */
/** \addtogroup Telemetry_Decoder_adapters
 * \{
 */

 /*!
 * \brief Base class for Telemetry Decoder adapters
 */
class TelemetryDecoderAdapterBase : public TelemetryDecoderInterface
{
public:
    TelemetryDecoderAdapterBase(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~TelemetryDecoderAdapterBase() override = default;

    void connect(gr::top_block_sptr top_block) override;

    void disconnect(gr::top_block_sptr top_block) override;

    gr::basic_block_sptr get_left_block() override;

    gr::basic_block_sptr get_right_block() override;

    void set_satellite(const Gnss_Satellite& satellite) override;

    std::string role() override;

    void set_channel(int channel) override;

    void reset() override;

    size_t item_size() override;

protected:
    void InitializeDecoder(telemetry_impl_base_sptr decoder);

    const Gnss_Satellite& satellite() const;

    Tlm_Conf tlm_parameters_;

private:
    telemetry_impl_base_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    std::string role_;
    unsigned int in_streams_ = 0;
    unsigned int out_streams_ = 0;
};

/** \} */
/** \} */

#endif  // GNSS_SDR_TELEMETRY_DECODER_ADAPTER_BASE_H
