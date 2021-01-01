/*!
 * \file gnss_block_factory.h
 * \brief Interface of a factory that returns smart pointers to GNSS blocks.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *         Carles Fernandez-Prades, 2014-2020. cfernandez(at)cttc.es
 *
 * This class encapsulates the complexity behind the instantiation
 * of GNSS blocks.
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

#ifndef GNSS_SDR_BLOCK_FACTORY_H
#define GNSS_SDR_BLOCK_FACTORY_H

#include "concurrent_queue.h"
#include <pmt/pmt.h>
#include <memory>  // for unique_ptr
#include <string>  // for string
#include <vector>  // for vector

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver
 * \{ */


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

    std::unique_ptr<GNSSBlockInterface> GetSignalSource(const ConfigurationInterface* configuration,
        Concurrent_Queue<pmt::pmt_t>* queue, int ID = -1);

    std::unique_ptr<GNSSBlockInterface> GetSignalConditioner(const ConfigurationInterface* configuration, int ID = -1);

    std::unique_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> GetChannels(const ConfigurationInterface* configuration,
        Concurrent_Queue<pmt::pmt_t>* queue);

    std::unique_ptr<GNSSBlockInterface> GetObservables(const ConfigurationInterface* configuration);

    std::unique_ptr<GNSSBlockInterface> GetPVT(const ConfigurationInterface* configuration);

    /*!
     * \brief Returns the block with the required role implementation and its configuration parameters
     */
    std::unique_ptr<GNSSBlockInterface> GetBlock(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams,
        Concurrent_Queue<pmt::pmt_t>* queue = nullptr);

private:
    std::unique_ptr<GNSSBlockInterface> GetChannel(
        const ConfigurationInterface* configuration,
        const std::string& signal,
        int channel,
        Concurrent_Queue<pmt::pmt_t>* queue);

    std::unique_ptr<AcquisitionInterface> GetAcqBlock(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    std::unique_ptr<TrackingInterface> GetTrkBlock(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    std::unique_ptr<TelemetryDecoderInterface> GetTlmBlock(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BLOCK_FACTORY_H
