/*!
 * \file ishort_to_complex.h
 * \brief Adapts an I/Q interleaved short integer sample stream to a gr_complex (float) stream
 * \author Javier Arribas, jarribas(at)cttc.es
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

#ifndef GNSS_SDR_ISHORT_TO_COMPLEX_H
#define GNSS_SDR_ISHORT_TO_COMPLEX_H

#include "conjugate_cc.h"
#include "gnss_block_interface.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/interleaved_short_to_complex.h>
#include <string>

class ConfigurationInterface;

/*!
 * \brief Adapts an I/Q interleaved short integer sample stream to a gr_complex (float) stream
 *
 */
class IshortToComplex : public GNSSBlockInterface
{
public:
    IshortToComplex(ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_streams,
        unsigned int out_streams);

    ~IshortToComplex() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Ishort_To_Complex"
    inline std::string implementation() override
    {
        return "Ishort_To_Complex";
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
    gr::blocks::interleaved_short_to_complex::sptr gr_interleaved_short_to_complex_;
    ConfigurationInterface* config_;
    bool dump_;
    std::string dump_filename_;
    std::string input_item_type_;
    std::string output_item_type_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    gr::blocks::file_sink::sptr file_sink_;
    conjugate_cc_sptr conjugate_cc_;
    bool inverted_spectrum;
};

#endif
