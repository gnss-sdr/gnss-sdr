/*!
 * \file cshort_to_grcomplex.h
 * \brief Adapts an 16-bits complex sample stream to a float complex stream
 * \author Carles Fernandez Prades, 2014 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CSHORT_TO_GRCOMPLEX_H
#define GNSS_SDR_CSHORT_TO_GRCOMPLEX_H

#include "cshort_to_gr_complex.h"
#include "gnss_block_interface.h"
#include <gnuradio/blocks/file_sink.h>
#include <cstdint>
#include <string>

/** \addtogroup Data_Type Data Type Adapters
 * Classes for data type conversion
 * \{ */
/** \addtogroup Data_type_adapters data_type_adapters
 * Wrap GNU Radio data type adapter blocks with a GNSSBlockInterface
 * \{ */


class ConfigurationInterface;

/*!
 * \brief Adapts an 16-bits complex sample stream to a float complex stream
 *
 */
class CshortToGrComplex : public GNSSBlockInterface
{
public:
    CshortToGrComplex(const ConfigurationInterface* configuration,
        std::string role, unsigned int in_streams,
        unsigned int out_streams);

    ~CshortToGrComplex() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Cshort_To_Gr_Complex"
    inline std::string implementation() override
    {
        return "Cshort_To_Gr_Complex";
    }

    inline size_t item_size() override
    {
        return 2 * sizeof(float);
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    cshort_to_gr_complex_sptr cshort_to_gr_complex_;
    gr::blocks::file_sink::sptr file_sink_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CSHORT_TO_GRCOMPLEX_H
