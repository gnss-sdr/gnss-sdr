/*!
 * \file fir_filter.h
 * \brief Adapts a gnuradio gr_fir_filter designed with pm_remez
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#ifndef GNSS_SDR_FIR_FILTER_H
#define GNSS_SDR_FIR_FILTER_H

#include "byte_x2_to_complex_byte.h"
#include "complex_byte_to_float_x2.h"
#include "cshort_to_float_x2.h"
#include "gnss_block_interface.h"
#include "short_x2_to_cshort.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/float_to_char.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <gnuradio/blocks/float_to_short.h>
#include <gnuradio/gr_complex.h>
#ifdef GR_GREATER_38
#include <gnuradio/filter/fir_filter_blk.h>
#else
#include <gnuradio/filter/fir_filter_ccf.h>
#include <gnuradio/filter/fir_filter_fff.h>
#endif
#include <cmath>
#include <string>
#include <vector>

/** \addtogroup Input_Filter Input Filter
 * Classes for input signal filtering
 * \{ */
/** \addtogroup Input_filter_adapters input_filter_adapters
 * Classes that wrap GNU Radio input filters with a GNSSBlockInterface
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class adapts a GNU Radio gr_fir_filter designed with pm_remez
 *
 * See Parks-McClellan FIR filter design, https://en.wikipedia.org/wiki/Parks-McClellan_filter_design_algorithm
 * Calculates the optimal (in the Chebyshev/minimax sense) FIR filter impulse response
 * given a set of band edges, the desired response on those bands, and the weight given
 * to the error in those bands.
 */
class FirFilter : public GNSSBlockInterface
{
public:
    //! Constructor
    FirFilter(const ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams);

    //! Destructor
    ~FirFilter() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Fir_Filter"
    inline std::string implementation() override
    {
        return "Fir_Filter";
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
    void init();

    gr::filter::fir_filter_ccf::sptr fir_filter_ccf_;
    gr::filter::fir_filter_fff::sptr fir_filter_fff_1_;
    gr::filter::fir_filter_fff::sptr fir_filter_fff_2_;
    gr::blocks::float_to_complex::sptr float_to_complex_;
    gr::blocks::float_to_short::sptr float_to_short_1_;
    gr::blocks::float_to_short::sptr float_to_short_2_;
    short_x2_to_cshort_sptr short_x2_to_cshort_;
    complex_byte_to_float_x2_sptr cbyte_to_float_x2_;
    byte_x2_to_complex_byte_sptr char_x2_cbyte_;
    cshort_to_float_x2_sptr cshort_to_float_x2_;
    gr::blocks::float_to_char::sptr float_to_char_1_;
    gr::blocks::float_to_char::sptr float_to_char_2_;
    gr::blocks::file_sink::sptr file_sink_;
    const ConfigurationInterface* config_;
    std::vector<float> taps_;
    std::string dump_filename_;
    std::string input_item_type_;
    std::string output_item_type_;
    std::string taps_item_type_;
    std::string role_;
    size_t item_size_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FIR_FILTER_H
