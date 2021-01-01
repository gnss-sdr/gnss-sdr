/*!
 * \file beamformer_filter.h
 * \brief Interface of an adapter of a digital beamformer
 * \author Javier Arribas jarribas (at) cttc.es
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


#ifndef GNSS_SDR_BEAMFORMER_FILTER_H
#define GNSS_SDR_BEAMFORMER_FILTER_H

#include "gnss_block_interface.h"
#include <gnuradio/hier_block2.h>
#include <cstdint>
#include <string>

/** \addtogroup Input_Filter
 * \{ */
/** \addtogroup Input_filter_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief Interface of an adapter of a digital beamformer block
 * to a GNSSBlockInterface
 */
class BeamformerFilter : public GNSSBlockInterface
{
public:
    BeamformerFilter(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream);

    ~BeamformerFilter() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! returns "Beamformer_Filte"
    inline std::string implementation() override
    {
        return "Beamformer_Filter";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    gr::block_sptr beamformer_;
    gr::block_sptr file_sink_;
    std::string role_;
    std::string item_type_;
    std::string dump_filename_;
    size_t item_size_;
    uint64_t samples_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BEAMFORMER_FILTER_H
