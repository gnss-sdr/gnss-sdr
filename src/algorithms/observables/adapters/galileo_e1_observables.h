/*!
 * \file galileo_e1_observables.h
 * \brief Implementation of an adapter of a Galileo E1 observables block
 * to a ObservablesInterface
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas 2013. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GALILEO_E1_OBSERVABLES_H_
#define GNSS_SDR_GALILEO_E1_OBSERVABLES_H_

#include <string>
#include "observables_interface.h"
#include "galileo_e1_observables_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements an ObservablesInterface for Galileo E1B
 */
class GalileoE1Observables : public ObservablesInterface
{
public:
    GalileoE1Observables(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams);
    virtual ~GalileoE1Observables();
    std::string role()
    {
        return role_;
    }

    //!  Returns "Galileo_E1B_Observables"
    std::string implementation()
    {
        return "Galileo_E1B_Observables";
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
    galileo_e1_observables_cc_sptr observables_;
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif
