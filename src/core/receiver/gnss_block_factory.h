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
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#include "concurrent_queue.h"
#include <pmt/pmt.h>
#include <memory>  // for unique_ptr, shared_ptr
#include <string>  // for string
#include <vector>  // for vector


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
    GNSSBlockFactory() = default;
    ~GNSSBlockFactory() = default;

    std::unique_ptr<GNSSBlockInterface> GetSignalSource(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue, int ID = -1);  // NOLINT(performance-unnecessary-value-param)

    std::unique_ptr<GNSSBlockInterface> GetSignalConditioner(const std::shared_ptr<ConfigurationInterface>& configuration, int ID = -1);

    std::unique_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> GetChannels(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);  // NOLINT(performance-unnecessary-value-param)

    std::unique_ptr<GNSSBlockInterface> GetObservables(const std::shared_ptr<ConfigurationInterface>& configuration);

    std::unique_ptr<GNSSBlockInterface> GetPVT(const std::shared_ptr<ConfigurationInterface>& configuration);

    /*!
     * \brief Returns the block with the required configuration and implementation
     */
    std::unique_ptr<GNSSBlockInterface> GetBlock(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& role, const std::string& implementation,
        unsigned int in_streams, unsigned int out_streams,
        const std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue = nullptr);  // NOLINT(performance-unnecessary-value-param)

private:
    std::unique_ptr<GNSSBlockInterface> GetChannel_1C(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_2S(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_1B(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_5X(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_L5(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_1G(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_2G(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_B1(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    std::unique_ptr<GNSSBlockInterface> GetChannel_B3(const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    std::unique_ptr<AcquisitionInterface> GetAcqBlock(
        const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& role,
        const std::string& implementation, unsigned int in_streams,
        unsigned int out_streams);

    std::unique_ptr<TrackingInterface> GetTrkBlock(
        const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& role,
        const std::string& implementation, unsigned int in_streams,
        unsigned int out_streams);

    std::unique_ptr<TelemetryDecoderInterface> GetTlmBlock(
        const std::shared_ptr<ConfigurationInterface>& configuration,
        const std::string& role,
        const std::string& implementation, unsigned int in_streams,
        unsigned int out_streams);
};

#endif  // GNSS_SDR_BLOCK_FACTORY_H_
