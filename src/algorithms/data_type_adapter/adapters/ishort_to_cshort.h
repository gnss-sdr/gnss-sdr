/*!
 * \file ishort_to_cshort.h
 * \brief Adapts a short interleaved sample stream into a std::complex<short> stream
 * \author Carles Fernandez-Prades, cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ISHORT_TO_CSHORT_H
#define GNSS_SDR_ISHORT_TO_CSHORT_H

#include "conjugate_sc.h"
#include "gnss_block_interface.h"
#include "interleaved_short_to_complex_short.h"
#include <gnuradio/blocks/file_sink.h>
#include <string>


class ConfigurationInterface;

/*!
 * \brief Adapts a short integer (16 bits) interleaved sample stream into a std::complex<short> stream
 *
 */
class IshortToCshort : public GNSSBlockInterface
{
public:
    IshortToCshort(ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_streams,
        unsigned int out_streams);

    ~IshortToCshort() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Ishort_To_Cshort"
    inline std::string implementation() override
    {
        return "Ishort_To_Cshort";
    }

    inline size_t item_size() override
    {
        return 0;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    interleaved_short_to_complex_short_sptr interleaved_short_to_complex_short_;
    conjugate_sc_sptr conjugate_sc_;
    gr::blocks::file_sink::sptr file_sink_;
    ConfigurationInterface* config_;
    std::string dump_filename_;
    std::string input_item_type_;
    std::string output_item_type_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool inverted_spectrum;
    bool dump_;
};

#endif
