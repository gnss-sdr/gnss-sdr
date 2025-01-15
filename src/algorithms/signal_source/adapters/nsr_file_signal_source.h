/*!
 * \file nsr_file_signal_source.h
 * \brief Implementation of a class that reads signals samples from a NSR 2 bits sampler front-end file
 * and adapts it to a SignalSourceInterface. More information about the front-end here
 * http://www.ifen.com/products/sx-scientific-gnss-solutions/nsr-software-receiver.html
 * \author Javier Arribas, 2013 jarribas(at)cttc.es
 *
 * This class represents a file signal source.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NSR_FILE_SIGNAL_SOURCE_H
#define GNSS_SDR_NSR_FILE_SIGNAL_SOURCE_H

#include "file_source_base.h"
#include "unpack_byte_2bit_samples.h"
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
class NsrFileSignalSource : public FileSourceBase
{
public:
    NsrFileSignalSource(const ConfigurationInterface* configuration, const std::string& role,
        unsigned int in_streams, unsigned int out_streams,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~NsrFileSignalSource() = default;

protected:
    std::tuple<size_t, bool> itemTypeToSize() override;
    double packetsPerSample() const override;
    gnss_shared_ptr<gr::block> source() const override;
    void create_file_source_hook() override;
    void pre_connect_hook(gr::top_block_sptr top_block) override;
    void pre_disconnect_hook(gr::top_block_sptr top_block) override;

private:
    unpack_byte_2bit_samples_sptr unpack_byte_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NSR_FILE_SIGNAL_SOURCE_H
