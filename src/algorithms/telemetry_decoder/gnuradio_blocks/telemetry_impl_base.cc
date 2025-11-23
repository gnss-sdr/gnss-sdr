/*!
 * \file telemetry_impl_base.cc
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

#include "telemetry_impl_base.h"
#include "tlm_crc_stats.h"
#include <pmt/pmt_sugar.h>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

void telemetry_impl_base::configure_basic_outputs()
{
    // prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
}


void telemetry_impl_base::configure_dump_file(int32_t channel,
    bool enable_dump,
    std::string& dump_filename,
    std::ofstream& dump_file) const
{
    if (enable_dump && (dump_file.is_open() == false))
        {
            try
                {
                    dump_filename.append(std::to_string(channel));
                    dump_filename.append(".dat");
                    dump_file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
                    dump_file.open(dump_filename.c_str(), std::ios::out | std::ios::binary);
                    LOG(INFO) << "Telemetry decoder dump enabled on channel "
                              << channel << ". Dump file: " << dump_filename.c_str();
                }
            catch (const std::ofstream::failure& e)
                {
                    LOG(WARNING) << "Channel " << channel
                                 << ": exception opening Telemetry dump file. " << e.what();
                }
        }
}


void telemetry_impl_base::configure_crc_stats_channel(int32_t channel,
    bool& dump_crc_stats,
    std::unique_ptr<Tlm_CRC_Stats>& crc_stats) const
{
    if (dump_crc_stats && crc_stats)
        {
            // set the channel number for the telemetry CRC statistics
            // disable the telemetry CRC statistics if there is a problem opening the output file
            dump_crc_stats = crc_stats->set_channel(channel);
        }
}