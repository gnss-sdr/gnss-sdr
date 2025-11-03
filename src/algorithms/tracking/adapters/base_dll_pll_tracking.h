/*!
 * \file base_dll_pll_tracking.h
 * \brief Base class providing shared logic for DLL+PLL VEML tracking adapters.
 * \authors Carles Fernandez, 2025. carles.fernandez(at)cttc.cat
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_BASE_DLL_PLL_TRACKING_H
#define GNSS_SDR_BASE_DLL_PLL_TRACKING_H

#include "dll_pll_conf.h"
#include "dll_pll_veml_tracking.h"
#include "tracking_interface.h"
#include <cstddef>
#include <string>

/** \addtogroup Tracking
 * Classes for GNSS signal tracking.
 * \{ */
/** \addtogroup Tracking_adapters tracking_adapters
 * Wrap GNU Radio blocks for GNSS signal tracking with a TrackingInterface
 * \{ */

class ConfigurationInterface;

/*!
 * \brief Base class providing shared logic for DLL+PLL tracking loop adapters
 * for GNSS signals.
 */
class BaseDllPllTracking : public TrackingInterface
{
public:
    /*!
     * \brief Base constructor of a Tracking block adapter
     */
    explicit BaseDllPllTracking(const ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Default destructor of the Tracking block adapter
     */
    ~BaseDllPllTracking() override = default;

    /*!
     * \brief Get role from the Tracking block adapter
     */
    inline std::string role() override final { return role_; }

    /*!
     * \brief Get item_size from the Tracking block adapter
     */
    inline size_t item_size() override final { return item_size_; }

    /*!
     * \brief Connect the Tracking block adapter
     */
    void connect(gr::top_block_sptr top_block) override final;

    /*!
     * \brief Disconnect the sTracking block adapter
     */
    void disconnect(gr::top_block_sptr top_block) override final;

    /*!
     * \brief Get left block from the Tracking block adapter
     */
    gr::basic_block_sptr get_left_block() override final;

    /*!
     * \brief Get right block from the Tracking block adapter
     */
    gr::basic_block_sptr get_right_block() override final;

    /*!
     * \brief Set tracking channel unique ID
     */
    void set_channel(unsigned int channel) override final;

    /*!
     * \brief Set acquisition Gnss_Synchro object pointer
     * to exchange synchronization data between acquisition and tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override final;

    /*!
     * \brief Start the Tracking block
     */
    void start_tracking() override final;

    /*!
     * \brief Stop the Tracking block
     */
    void stop_tracking() override final;

protected:
    // Can be used by each derived class
    inline Dll_Pll_Conf& config_params() { return trk_params_; }
    inline const Dll_Pll_Conf& config_params() const { return trk_params_; }
    inline void set_item_size(size_t item_size) { item_size_ = item_size; }

    // Must be implemented / filled by each derived class
    virtual void configure_tracking_parameters(const ConfigurationInterface* configuration) = 0;
    virtual void create_tracking_block() = 0;
    dll_pll_veml_tracking_sptr tracking_sptr_;

private:
    // Managed by the base class
    Dll_Pll_Conf trk_params_;
    std::string role_;
    size_t item_size_;
    unsigned int channel_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_BASE_DLL_PLL_TRACKING_H
