/*!
 * \file gnss_block_factory.h
 * \brief This class implements a factory that returns instances of GNSS blocks.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * This class encapsulates the complexity behind the instantiation
 * of GNSS blocks.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_BLOCK_FACTORY_H_
#define GNSS_BLOCK_FACTORY_H_

#include <vector>
#include <string>

#include <gnuradio/gr_msg_queue.h>

class ConfigurationInterface;
class GNSSBlockInterface;

class GNSSBlockFactory
{

public:

    GNSSBlockFactory();
    virtual ~GNSSBlockFactory();

    GNSSBlockInterface* GetSignalSource(
            ConfigurationInterface *configuration, gr_msg_queue_sptr queue);
    GNSSBlockInterface* GetSignalConditioner(
            ConfigurationInterface *configuration, gr_msg_queue_sptr queue);
    GNSSBlockInterface* GetPVT(ConfigurationInterface *configuration,
            gr_msg_queue_sptr queue);
    GNSSBlockInterface* GetObservables(ConfigurationInterface *configuration,
            gr_msg_queue_sptr queue);
    GNSSBlockInterface* GetOutputFilter(
            ConfigurationInterface *configuration, gr_msg_queue_sptr queue);
    GNSSBlockInterface* GetChannel(ConfigurationInterface *configuration,
            std::string acq, std::string trk, std::string tlm, int channel,
            gr_msg_queue_sptr queue);
    std::vector<GNSSBlockInterface*>* GetChannels(
            ConfigurationInterface *configuration, gr_msg_queue_sptr queue);
    GNSSBlockInterface* GetBlock(ConfigurationInterface* configuration,
            std::string role, std::string implementation,
            unsigned int in_streams, unsigned int out_streams,
            gr_msg_queue_sptr queue);

};

#endif /*GNSS_BLOCK_FACTORY_H_*/
