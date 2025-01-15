/*!
 * \file galileo_e6_dll_pll_tracking.h
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E6 signals
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_GALILEO_E6_DLL_PLL_TRACKING_H
#define GNSS_SDR_GALILEO_E6_DLL_PLL_TRACKING_H

#include "dll_pll_veml_tracking.h"
#include "tracking_interface.h"
#include <string>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_adapters
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GalileoE6DllPllTracking : public TrackingInterface
{
public:
    GalileoE6DllPllTracking(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~GalileoE6DllPllTracking() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "Galileo_E6_DLL_PLL_Tracking"
    inline std::string implementation() override
    {
        return "Galileo_E6_DLL_PLL_Tracking";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    /*!
     * \brief Connect
     */
    void connect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Disconnect
     */
    void disconnect(gr::top_block_sptr top_block) override;

    /*!
     * \brief Get left block
     */
    gr::basic_block_sptr get_left_block() override;

    /*!
     * \brief Get right block
     */
    gr::basic_block_sptr get_right_block() override;

    /*!
     * \brief Set tracking channel unique ID
     */
    void set_channel(unsigned int channel) override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    void start_tracking() override;

    /*!
     * \brief Stop running tracking
     */
    void stop_tracking() override;

private:
    dll_pll_veml_tracking_sptr tracking_sptr_;
    std::string role_;
    size_t item_size_;
    unsigned int channel_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E6_DLL_PLL_TRACKING_H
