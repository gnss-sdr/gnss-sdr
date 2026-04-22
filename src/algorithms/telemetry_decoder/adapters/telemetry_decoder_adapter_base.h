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
#include "telemetry_impl_interface.h"
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

inline Tlm_Conf get_tlm_conf(const ConfigurationInterface* configuration, const std::string& role)
{
    Tlm_Conf conf;

    if (configuration != nullptr)
        {
            conf.SetFromConfiguration(configuration, role);
        }

    return conf;
}

/*!
 * \brief Base class for Telemetry Decoder adapters
 */
class TelemetryDecoderAdapterBase : public TelemetryDecoderInterface
{
public:
    TelemetryDecoderAdapterBase(
        const std::string& role,
        const std::string& implementation,
        unsigned int in_streams,
        unsigned int out_streams,
        telemetry_impl_interface_sptr decoder);

    ~TelemetryDecoderAdapterBase() override = default;

    void connect(gr::top_block_sptr top_block) override;

    void disconnect(gr::top_block_sptr top_block) override;

    gr::basic_block_sptr get_left_block() override;

    gr::basic_block_sptr get_right_block() override;

    void set_satellite(const Gnss_Satellite& satellite) override;

    std::string role() override;

    std::string implementation() override;

    void set_channel(int channel) override;

    void reset() override;

    size_t item_size() override;

private:
    telemetry_impl_interface_sptr telemetry_decoder_;
    const std::string role_;
    const std::string implementation_;
};

/** \} */
/** \} */

#endif  // GNSS_SDR_TELEMETRY_DECODER_ADAPTER_BASE_H
