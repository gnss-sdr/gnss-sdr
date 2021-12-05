/*!
 * \file file_timestamp_signal_source.h
 * \brief This class reads samples stored in a file and generate stream tags with its timestamp information stored in separated file
 * \author Javier Arribas, jarribas(at)cttc.es
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


#ifndef GNSS_SDR_FILE_TIMESTAMP_SIGNAL_SOURCE_H
#define GNSS_SDR_FILE_TIMESTAMP_SIGNAL_SOURCE_H

#include "configuration_interface.h"
#include "file_source_base.h"
#include "gnss_sdr_timestamp.h"
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


/*!
 * \brief Class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 */
class FileTimestampSignalSource : public FileSourceBase
{
public:
    FileTimestampSignalSource(const ConfigurationInterface* configuration, const std::string& role,
        unsigned int in_streams, unsigned int out_streams,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~FileTimestampSignalSource() = default;

protected:
    // std::tuple<size_t, bool> itemTypeToSize() override;
    // double packetsPerSample() const override;
    gnss_shared_ptr<gr::block> source() const override;
    void create_file_source_hook() override;
    void pre_connect_hook(gr::top_block_sptr top_block) override;
    void pre_disconnect_hook(gr::top_block_sptr top_block) override;

private:
    gnss_shared_ptr<Gnss_Sdr_Timestamp> timestamp_block_;
    std::string timestamp_file_;
    double timestamp_clock_offset_ms_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FILE_TIMESTAMP_SIGNAL_SOURCE_H
