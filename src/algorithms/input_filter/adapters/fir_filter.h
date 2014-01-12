/*!
 * \file fir_filter.h
 * \brief Adapts a gnuradio gr_fir_filter designed with pm_remez
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#include <cmath>
#include <string>
#include <vector>
#include <gnuradio/gr_complex.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/filter/fir_filter_ccf.h>
#include <gnuradio/msg_queue.h>
#include "gnss_synchro.h"
#include "gnss_block_interface.h"


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
              unsigned int out_streams,
              boost::shared_ptr<gr::msg_queue> queue);

    //! Destructor
    virtual ~FirFilter();
    std::string role()
    {
        return role_;
    }

    //! Returns "Fir_Filter"
    std::string implementation()
    {
        return "Fir_Filter";
    }
    size_t item_size()
    {
        return 0;
    }
    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();

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
    boost::shared_ptr<gr::msg_queue> queue_;
    gr::blocks::file_sink::sptr file_sink_;
    void init();
};

#endif
