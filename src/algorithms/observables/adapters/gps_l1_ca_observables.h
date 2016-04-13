/*!
 * \file gps_l1_ca_observables.h
 * \brief Interface of an adapter of a GPS L1 C/A observables block
 * to a ObservablesInterface
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_L1_CA_OBSERVABLES_H_
#define GNSS_SDR_GPS_L1_CA_OBSERVABLES_H_

#include <string>
#include "observables_interface.h"
#include "gps_l1_ca_observables_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements an ObservablesInterface for GPS L1 C/A
 */
class GpsL1CaObservables : public ObservablesInterface
{
public:
    GpsL1CaObservables(ConfigurationInterface* configuration,
                       std::string role,
                       unsigned int in_streams,
                       unsigned int out_streams);
    virtual ~GpsL1CaObservables();
    std::string role()
    {
        return role_;
    }

    //!  Returns "GPS_L1_CA_Observables"
    std::string implementation()
    {
        return "GPS_L1_CA_Observables";
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
    gps_l1_ca_observables_cc_sptr observables_;
    bool dump_;
    //unsigned int fs_in_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif
