/*!
 * \file two_bit_cpx_file_signal_source.h
 * \brief Interface of a class that reads signals samples from a 2 bit complex sampler front-end file
 * and adapts it to a SignalSourceInterface.
 * \author Javier Arribas, 2015 jarribas(at)cttc.es
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

#ifndef GNSS_SDR_TWO_BIT_CPX_FILE_SIGNAL_SOURCE_H
#define GNSS_SDR_TWO_BIT_CPX_FILE_SIGNAL_SOURCE_H

#include "file_source_base.h"
#include "unpack_byte_2bit_cpx_samples.h"
#include <gnuradio/blocks/interleaved_short_to_complex.h>
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
class TwoBitCpxFileSignalSource : public FileSourceBase
{
public:
    TwoBitCpxFileSignalSource(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~TwoBitCpxFileSignalSource() = default;

protected:
    std::tuple<size_t, bool> itemTypeToSize() override;
    double packetsPerSample() const override;
    gnss_shared_ptr<gr::block> source() const override;
    void create_file_source_hook() override;
    void pre_connect_hook(gr::top_block_sptr top_block) override;
    void pre_disconnect_hook(gr::top_block_sptr top_block) override;

private:
    unpack_byte_2bit_cpx_samples_sptr unpack_byte_;
    gr::blocks::interleaved_short_to_complex::sptr inter_shorts_to_cpx_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_TWO_BIT_CPX_FILE_SIGNAL_SOURCE_H
