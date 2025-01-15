/*!
 * \file pass_through.h
 * \brief Interface of a block that just puts its input in its
 *        output.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
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

#ifndef GNSS_SDR_PASS_THROUGH_H
#define GNSS_SDR_PASS_THROUGH_H

#include "conjugate_cc.h"
#include "conjugate_ic.h"
#include "conjugate_sc.h"
#include "gnss_block_interface.h"
#include <gnuradio/blocks/copy.h>
#include <gnuradio/runtime_types.h>
#include <cstddef>
#include <string>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class implements a block that connects input and output (does nothing)
 */
class Pass_Through : public GNSSBlockInterface
{
public:
    Pass_Through(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_stream,
        unsigned int out_stream);

    ~Pass_Through() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Pass_Through"
    inline std::string implementation() override
    {
        return "Pass_Through";
    }

    inline std::string item_type() const
    {
        return item_type_;
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
    gr::blocks::copy::sptr kludge_copy_;
    conjugate_cc_sptr conjugate_cc_;
    conjugate_sc_sptr conjugate_sc_;
    conjugate_ic_sptr conjugate_ic_;
    std::string item_type_;
    std::string role_;
    size_t item_size_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool inverted_spectrum;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PASS_THROUGH_H
