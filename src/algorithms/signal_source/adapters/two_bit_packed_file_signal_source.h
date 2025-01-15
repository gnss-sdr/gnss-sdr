/*!
 * \file two_bit_packed_file_signal_source.h
 * \brief Interface of a class that reads signals samples from a file. Each
 * sample is two bits, which are packed into bytes or shorts.
 *
 * \author Cillian O'Driscoll, 2015 cillian.odriscoll (at) gmail.com
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

#ifndef GNSS_SDR_TWO_BIT_PACKED_FILE_SIGNAL_SOURCE_H
#define GNSS_SDR_TWO_BIT_PACKED_FILE_SIGNAL_SOURCE_H

#include "file_source_base.h"
#include "unpack_2bit_samples.h"
#include <gnuradio/blocks/interleaved_char_to_complex.h>
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
class TwoBitPackedFileSignalSource : public FileSourceBase
{
public:
    TwoBitPackedFileSignalSource(const ConfigurationInterface* configuration, const std::string& role,
        unsigned int in_streams, unsigned int out_streams,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~TwoBitPackedFileSignalSource() = default;

private:
    inline bool big_endian_items() const
    {
        return big_endian_items_;
    }

    inline bool big_endian_bytes() const
    {
        return big_endian_bytes_;
    }


    inline bool reverse_interleaving() const
    {
        return reverse_interleaving_;
    }

protected:
    std::tuple<size_t, bool> itemTypeToSize() override;
    double packetsPerSample() const override;
    gnss_shared_ptr<gr::block> source() const override;
    void create_file_source_hook() override;
    void pre_connect_hook(gr::top_block_sptr top_block) override;
    void pre_disconnect_hook(gr::top_block_sptr top_block) override;

private:
    std::string sample_type_;
    bool big_endian_items_;
    bool big_endian_bytes_;
    bool reverse_interleaving_;
    unpack_2bit_samples_sptr unpack_samples_;
    gnss_shared_ptr<gr::block> char_to_float_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_TWO_BIT_CPX_FILE_SIGNAL_SOURCE_H
