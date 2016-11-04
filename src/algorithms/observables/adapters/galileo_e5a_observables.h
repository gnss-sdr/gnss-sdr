/*!
 * \file galileo_e5a_observables.h
 * \brief Implementation of an adapter of a Galileo E5a observables block
 * to a ObservablesInterface
 * \author Carles Fernandez 2016. carles.fernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2016  (see AUTHORS file for a list of contributors)
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


#ifndef GNSS_SDR_GALILEO_E5A_OBSERVABLES_H_
#define GNSS_SDR_GALILEO_E5A_OBSERVABLES_H_

#include <string>
#include "observables_interface.h"
#include "hybrid_observables_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements an ObservablesInterface for Galileo E5A
 */
class GalileoE5aObservables : public ObservablesInterface
{
public:
    GalileoE5aObservables(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams);
    virtual ~GalileoE5aObservables();
    std::string role()
    {
        return role_;
    }

    //!  Returns "Galileo_E5A_Observables"
    std::string implementation()
    {
        return "Galileo_E5A_Observables";
    }
    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();
    void reset()
    {
        return;
    }

    //! All blocks must have an item_size() function implementation
    size_t item_size()
    {
        return sizeof(gr_complex);
    }

private:
    hybrid_observables_cc_sptr observables_;
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif
