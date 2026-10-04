/*!
 * \file dll_pll_tracking_adapter.h
 * \brief Base class providing shared logic for DLL+PLL VEML tracking adapters.
 * \authors Carles Fernandez, 2025. carles.fernandez(at)cttc.cat
 *          Mathieu Favreau, 2026. favreau.mathieu(at)hotmail.com
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

#ifndef GNSS_SDR_DLL_PLL_TRACKING_ADAPTER_H
#define GNSS_SDR_DLL_PLL_TRACKING_ADAPTER_H

#include "dll_pll_conf.h"
#include "dll_pll_veml_tracking.h"
#include "signal_flag.h"
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
class DllPllTrackingAdapter : public TrackingInterface
{
public:
    /*!
     * \brief Base constructor of a Tracking block adapter
     */
    explicit DllPllTrackingAdapter(const ConfigurationInterface* configuration,
        std::string role,
        std::string implementation,
        unsigned int in_streams,
        unsigned int out_streams,
        signal_flag sig_flag);

    /*!
     * \brief Default destructor of the Tracking block adapter
     */
    ~DllPllTrackingAdapter() override = default;

    /*!
     * \brief Get role from the Tracking block adapter
     */
    inline std::string role() override { return role_; }

    /*!
     * \brief Get implementation from the Tracking block adapter
     */
    inline std::string implementation() override { return implementation_; }

    /*!
     * \brief Get item_size from the Tracking block adapter
     */
    inline size_t item_size() override
    {
        return item_size_;
    }

    /*!
     * \brief Connect the Tracking block adapter
     */
    void connect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Disconnect the sTracking block adapter
     */
    void disconnect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Get left block from the Tracking block adapter
     */
    gr::basic_block_sptr get_left_block() override;

    /*!
     * \brief Get right block from the Tracking block adapter
     */
    gr::basic_block_sptr get_right_block() override;

    /*!
     * \brief Set tracking channel unique ID
     */
    void set_channel(unsigned int channel) override;

    /*!
     * \brief Set acquisition Gnss_Synchro object pointer
     * to exchange synchronization data between acquisition and tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    /*!
     * \brief Start the Tracking block
     */
    void start_tracking() override;

    /*!
     * \brief Stop the Tracking block
     */
    void stop_tracking() override;

private:
    dll_pll_veml_tracking_sptr tracking_sptr_;
    Dll_Pll_Conf trk_params_;
    const std::string role_;
    const std::string implementation_;
    size_t item_size_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_DLL_PLL_TRACKING_ADAPTER_H
