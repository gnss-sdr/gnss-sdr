/*!
 * \file ntlab_file_signal_source.h
 * \brief Interface of a class that reads signal samples from a file. Each
 * sample is two bits from multiple channels.
 *
 * \author Pedro Pereira, 2025 pereirapedrocp (at) gmail.com
 *
 * This class represents a file signal source.
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

#ifndef GNSS_SDR_NTLAB_FILE_SIGNAL_SOURCE_H
#define GNSS_SDR_NTLAB_FILE_SIGNAL_SOURCE_H

#include "file_source_base.h"
#include "unpack_ntlab_2bit_samples.h"
#include <cstddef>
#include <string>
#include <tuple>


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief Class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 */
class MultiChannelTwoBitPackedFileSignalSource : public FileSourceBase
{
public:
    MultiChannelTwoBitPackedFileSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_streams,
        unsigned int out_streams, Concurrent_Queue<pmt::pmt_t>* queue);

    ~MultiChannelTwoBitPackedFileSignalSource() = default;

    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

protected:
    std::tuple<size_t, bool> itemTypeToSize() override;
    double packetsPerSample() const override;
    gnss_shared_ptr<gr::block> source() const override;
    void create_file_source_hook() override;
    void pre_connect_hook(gr::top_block_sptr top_block) override;
    void pre_disconnect_hook(gr::top_block_sptr top_block) override;

private:
    std::string sample_type_;
    unpack_ntlab_2bit_samples_sptr unpack_samples_;
    int n_channels_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NTLAB_FILE_SIGNAL_SOURCE_H
