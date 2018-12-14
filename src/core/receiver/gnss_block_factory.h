/*!
 * \file gnss_block_factory.h
 * \brief Interface of a factory that returns smart pointers to GNSS blocks.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *         Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *
 * This class encapsulates the complexity behind the instantiation
 * of GNSS blocks.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_BLOCK_FACTORY_H_
#define GNSS_SDR_BLOCK_FACTORY_H_

#include <gnuradio/msg_queue.h>
#include <memory>
#include <string>
#include <vector>


class ConfigurationInterface;
class GNSSBlockInterface;
class AcquisitionInterface;
class TrackingInterface;
class TelemetryDecoderInterface;

/*!
 * \brief Class that produces all kinds of GNSS blocks
 */
class GNSSBlockFactory
{
public:
    GNSSBlockFactory();
    virtual ~GNSSBlockFactory();
    std::unique_ptr<GNSSBlockInterface> GetSignalSource(std::shared_ptr<ConfigurationInterface> configuration,
        gr::msg_queue::sptr queue, int ID = -1);

    std::unique_ptr<GNSSBlockInterface> GetSignalConditioner(std::shared_ptr<ConfigurationInterface> configuration, int ID = -1);

    std::unique_ptr<GNSSBlockInterface> GetPVT(std::shared_ptr<ConfigurationInterface> configuration);

    std::unique_ptr<GNSSBlockInterface> GetObservables(std::shared_ptr<ConfigurationInterface> configuration);

    std::unique_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> GetChannels(std::shared_ptr<ConfigurationInterface> configuration,
        gr::msg_queue::sptr queue);

    /*
     * \brief Returns the block with the required configuration and implementation
     */
    std::unique_ptr<GNSSBlockInterface> GetBlock(std::shared_ptr<ConfigurationInterface> configuration,
        std::string role, std::string implementation,
        unsigned int in_streams, unsigned int out_streams,
        gr::msg_queue::sptr queue = nullptr);

private:
    std::unique_ptr<GNSSBlockInterface> GetChannel_1C(std::shared_ptr<ConfigurationInterface> configuration,
        std::string acq, std::string trk, std::string tlm, int channel,
        gr::msg_queue::sptr queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_2S(std::shared_ptr<ConfigurationInterface> configuration,
        std::string acq, std::string trk, std::string tlm, int channel,
        gr::msg_queue::sptr queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_1B(std::shared_ptr<ConfigurationInterface> configuration,
        std::string acq, std::string trk, std::string tlm, int channel,
        gr::msg_queue::sptr queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_5X(std::shared_ptr<ConfigurationInterface> configuration,
        std::string acq, std::string trk, std::string tlm, int channel,
        gr::msg_queue::sptr queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_L5(std::shared_ptr<ConfigurationInterface> configuration,
        std::string acq, std::string trk, std::string tlm, int channel,
        gr::msg_queue::sptr queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_1G(std::shared_ptr<ConfigurationInterface> configuration,
        std::string acq, std::string trk, std::string tlm, int channel,
        boost::shared_ptr<gr::msg_queue> queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_2G(std::shared_ptr<ConfigurationInterface> configuration,
        std::string acq, std::string trk, std::string tlm, int channel,
        boost::shared_ptr<gr::msg_queue> queue);

    std::unique_ptr<AcquisitionInterface> GetAcqBlock(
        std::shared_ptr<ConfigurationInterface> configuration,
        std::string role,
        std::string implementation, unsigned int in_streams,
        unsigned int out_streams);

    std::unique_ptr<TrackingInterface> GetTrkBlock(
        std::shared_ptr<ConfigurationInterface> configuration,
        std::string role,
        std::string implementation, unsigned int in_streams,
        unsigned int out_streams);

    std::unique_ptr<TelemetryDecoderInterface> GetTlmBlock(
        std::shared_ptr<ConfigurationInterface> configuration,
        std::string role,
        std::string implementation, unsigned int in_streams,
        unsigned int out_streams);
};

#endif /*GNSS_SDR_BLOCK_FACTORY_H_*/
