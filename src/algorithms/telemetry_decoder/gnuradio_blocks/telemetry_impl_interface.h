/*!
 * \file telemetry_impl_interface.h
 * \brief Base class for telemetry decoder GNU Radio blocks.
 * \author Carles Fernandez-Prades, 2025 cfernandez@cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_TELEMETRY_IMPL_INTERFACE_H
#define GNSS_SDR_TELEMETRY_IMPL_INTERFACE_H

#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include <gnuradio/block.h>
#include <gnuradio/io_signature.h>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

class Tlm_Conf;
class Tlm_CRC_Stats;

/** \addtogroup Telemetry_Decoder
 * \{
 */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks telemetry_decoder_gr_blocks
 * \{
 */

class telemetry_impl_interface;
using telemetry_impl_interface_sptr = gnss_shared_ptr<telemetry_impl_interface>;

/*!
 * \brief Common base class for telemetry decoder GNU Radio implementations.
 */
class telemetry_impl_interface : public gr::block
{
public:
    telemetry_impl_interface(const std::string& name,
        gr::io_signature::sptr input_signature,
        gr::io_signature::sptr output_signature)
        : gr::block(name,
              std::move(input_signature),
              std::move(output_signature)) {}

    ~telemetry_impl_interface() override = default;

    virtual void set_satellite(const Gnss_Satellite& satellite) = 0;
    virtual void set_channel(int channel) = 0;
    virtual void reset() = 0;

protected:
    void configure_basic_outputs();

    void configure_dump_file(int32_t channel,
        bool enable_dump,
        std::string& dump_filename,
        std::ofstream& dump_file) const;

    void configure_crc_stats_channel(int32_t channel,
        bool& dump_crc_stats,
        std::unique_ptr<Tlm_CRC_Stats>& crc_stats) const;
};

/** \} */
/** \} */

#endif  // GNSS_SDR_TELEMETRY_IMPL_INTERFACE_H
