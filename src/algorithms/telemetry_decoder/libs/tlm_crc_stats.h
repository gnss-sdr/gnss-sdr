/*!
 * \file tlm_crc_stats.h
 * \brief Class that computes the telemetry CRC statistics
 * \author Marc Majoral, 2021. mmajoral(at)cttc.es
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

#ifndef GNSS_SDR_CRC_STATS_H
#define GNSS_SDR_CRC_STATS_H

#include <fstream>  // for std::ofstream
#include <string>   // for std::string

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libs telemetry_decoder_libs
 * \{ */

/*!
 * \brief Class that computes the telemetry CRC statistics
 */
class Tlm_CRC_Stats
{
public:
    Tlm_CRC_Stats();

    ~Tlm_CRC_Stats();

    /*!
	 * \brief Initialize the telemetry CRC statistics
	 */
    void initialize(std::string dump_crc_stats_filename_);

    /*!
	 * \brief Initialize the channel number and output file
	 */
    bool set_channel(int32_t channel_);

    /*!
	 * \brief Update the CRC statistics
	 */
    void update_CRC_stats(bool CRC);

private:
    bool enable_crc_stats;
    uint32_t num_crc_ok;
    uint32_t num_crc_not_ok;
    int32_t channel;

    std::ofstream d_dump_file;
    std::string d_dump_crc_stats_filename;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CRC_STATS_H
