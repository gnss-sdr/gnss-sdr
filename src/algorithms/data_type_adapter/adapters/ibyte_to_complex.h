/*!
 * \file ibyte_to_complex.h
 * \brief Adapts an I/Q interleaved byte integer sample stream to a gr_complex (float) stream
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

#ifndef GNSS_SDR_IBYTE_TO_COMPLEX_H
#define GNSS_SDR_IBYTE_TO_COMPLEX_H

#include "conjugate_cc.h"
#include "gnss_block_interface.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <cstdint>
#include <string>

/** \addtogroup Data_Type
 * \{ */
/** \addtogroup Data_type_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief Adapts an I/Q interleaved byte integer sample stream to a gr_complex (float) stream
 *
 */
class IbyteToComplex : public GNSSBlockInterface
{
public:
    IbyteToComplex(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_streams,
        unsigned int out_streams);

    ~IbyteToComplex() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Ibyte_To_Complex"
    inline std::string implementation() override
    {
        return "Ibyte_To_Complex";
    }

    inline size_t item_size() override
    {
        return 2 * sizeof(int8_t);
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    gr::blocks::interleaved_char_to_complex::sptr gr_interleaved_char_to_complex_;
    conjugate_cc_sptr conjugate_cc_;
    gr::blocks::file_sink::sptr file_sink_;
    std::string dump_filename_;
    std::string input_item_type_;
    std::string output_item_type_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool inverted_spectrum;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_IBYTE_TO_COMPLEX_H
