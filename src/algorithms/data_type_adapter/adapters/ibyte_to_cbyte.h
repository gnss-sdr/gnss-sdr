/*!
 * \file ibyte_to_cbyte.h
 * \brief \brief Adapts an I/Q interleaved byte (unsigned char) sample stream
 * into a std::complex<unsigned char> stream
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_IBYTE_TO_CBYTE_H
#define GNSS_SDR_IBYTE_TO_CBYTE_H

#include "conjugate_ic.h"
#include "gnss_block_interface.h"
#include "interleaved_byte_to_complex_byte.h"
#include <gnuradio/blocks/file_sink.h>
#include <cstdint>
#include <string>

/** \addtogroup Data_Type
 * \{ */
/** \addtogroup Data_type_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \briefAdapts an I/Q interleaved byte (unsigned char) sample stream
 * into a std::complex<unsigned char> stream
 */
class IbyteToCbyte : public GNSSBlockInterface
{
public:
    IbyteToCbyte(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_streams,
        unsigned int out_streams);

    ~IbyteToCbyte() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Ibyte_To_Cbyte"
    inline std::string implementation() override
    {
        return "Ibyte_To_Cbyte";
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
    interleaved_byte_to_complex_byte_sptr ibyte_to_cbyte_;
    conjugate_ic_sptr conjugate_ic_;
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
#endif  // GNSS_SDR_IBYTE_TO_CBYTE_H
