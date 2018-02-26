/*!
 * \file fir_filter.h
 * \brief Adapts a gnuradio gr_fir_filter designed with pm_remez
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FIR_FILTER_H_
#define GNSS_SDR_FIR_FILTER_H_

#include "gnss_block_interface.h"
#include "complex_byte_to_float_x2.h"
#include "byte_x2_to_complex_byte.h"
#include "short_x2_to_cshort.h"
#include "cshort_to_float_x2.h"
#include <gnuradio/gr_complex.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/float_to_char.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <gnuradio/blocks/float_to_short.h>
#include <gnuradio/filter/fir_filter_ccf.h>
#include <gnuradio/filter/fir_filter_fff.h>
#include <cmath>
#include <string>
#include <vector>

class ConfigurationInterface;

/*!
 * \brief This class adapts a GNU Radio gr_fir_filter designed with pm_remez
 *
 * See Parks-McClellan FIR filter design, http://en.wikipedia.org/wiki/Parks-McClellan_filter_design_algorithm
 * Calculates the optimal (in the Chebyshev/minimax sense) FIR filter impulse response
 * given a set of band edges, the desired response on those bands, and the weight given
 * to the error in those bands.
 */
class FirFilter: public GNSSBlockInterface
{
public:
    //! Constructor
    FirFilter(ConfigurationInterface* configuration,
              std::string role,
              unsigned int in_streams,
              unsigned int out_streams);

    //! Destructor
    virtual ~FirFilter();

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
        return 0;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    gr::filter::fir_filter_ccf::sptr fir_filter_ccf_;
    ConfigurationInterface* config_;
    bool dump_;
    std::string dump_filename_;
    std::string input_item_type_;
    std::string output_item_type_;
    std::string taps_item_type_;
    std::vector <float> taps_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    gr::blocks::file_sink::sptr file_sink_;
    void init();
    complex_byte_to_float_x2_sptr cbyte_to_float_x2_;
    gr::filter::fir_filter_fff::sptr fir_filter_fff_1_;
    gr::filter::fir_filter_fff::sptr fir_filter_fff_2_;
    gr::blocks::float_to_char::sptr float_to_char_1_;
    gr::blocks::float_to_char::sptr float_to_char_2_;
    byte_x2_to_complex_byte_sptr char_x2_cbyte_;
    gr::blocks::float_to_complex::sptr float_to_complex_;
    cshort_to_float_x2_sptr cshort_to_float_x2_;
    gr::blocks::float_to_short::sptr float_to_short_1_;
    gr::blocks::float_to_short::sptr float_to_short_2_;
    short_x2_to_cshort_sptr short_x2_to_cshort_;
};

#endif
