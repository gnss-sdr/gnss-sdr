/*!
 * \file gnss_block_interface.h
 * \brief This interface represents a GNSS block.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * Abstract class for GNSS block interfaces. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
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


#ifndef GNSS_SDR_GNSS_BLOCK_INTERFACE_H
#define GNSS_SDR_GNSS_BLOCK_INTERFACE_H

#include <gnuradio/top_block.h>
#include <cassert>
#include <string>
#include <utility>  // for std::forward

/** \addtogroup Core
 * \{ */
/** \addtogroup GNSS_Block_Interfaces
 * \{ */

// clang-format off
#if GNURADIO_USES_STD_POINTERS
#include <memory>
template <typename T>
using gnss_shared_ptr = std::shared_ptr<T>;
template <typename C, typename... Args>
gnss_shared_ptr<C> gnss_make_shared(Args &&... args)
{
    return std::make_shared<C>(std::forward<Args>(args)...);
}
#else
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
template <typename T>
using gnss_shared_ptr = boost::shared_ptr<T>;
template <typename C, typename... Args>
gnss_shared_ptr<C> gnss_make_shared(Args &&... args)
{
    return boost::make_shared<C>(std::forward<Args>(args)...);
}
#endif
// clang-format on


/*!
 * \brief This abstract class represents an interface to GNSS blocks.
 *
 * Abstract class for GNSS block interfaces. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 */
class GNSSBlockInterface
{
public:
    virtual ~GNSSBlockInterface() = default;

    /*!
     * \brief Returns the role of the block in the flowgraph.
     * \return A string identifying the block's role (e.g. "Acquisition", "Tracking").
     */
    virtual std::string role() = 0;

    /*!
     * \brief Returns the concrete implementation identifier.
     * \return A string identifying the implementation (e.g. "GPS_L1_CA_PCPS_Acquisition").
     */
    virtual std::string implementation() = 0;

    /*!
     * \brief Returns the size of each input/output item in bytes.
     * \return Item size in bytes.
     */
    virtual size_t item_size() = 0;

    /*!
     * \brief Connects the block to the GNU Radio flowgraph.
     * \param[in] top_block Shared pointer to the GNU Radio top block.
     */
    virtual void connect(gr::top_block_sptr top_block) = 0;

    /*!
     * \brief Disconnects the block from the GNU Radio flowgraph.
     * \param[in] top_block Shared pointer to the GNU Radio top block.
     */
    virtual void disconnect(gr::top_block_sptr top_block) = 0;

    /*!
     * \brief Returns the leftmost GNU Radio block of this processing block.
     * \return Shared pointer to the leftmost basic block.
     */
    virtual gr::basic_block_sptr get_left_block() = 0;

    /*!
     * \brief Returns the rightmost GNU Radio block of this processing block.
     * \return Shared pointer to the rightmost basic block.
     */
    virtual gr::basic_block_sptr get_right_block() = 0;

    /*!
     * \brief Returns the leftmost GNU Radio block for a given RF channel.
     * \param[in] RF_channel RF channel index.
     * \return Shared pointer to the leftmost basic block, or nullptr if not implemented.
     */
    virtual gr::basic_block_sptr get_left_block(int RF_channel)
    {
        assert(RF_channel >= 0);
        if (RF_channel == 0)
            {
            };
        return nullptr;
    }

    /*!
     * \brief Returns the rightmost GNU Radio block for a given RF channel.
     * \param[in] RF_channel RF channel index.
     * \return Shared pointer to the rightmost basic block, or nullptr if not implemented.
     */
    virtual gr::basic_block_sptr get_right_block(int RF_channel)
    {
        assert(RF_channel >= 0);
        if (RF_channel == 0)
            {
            };
        return nullptr;
    }

    /*!
     * \brief Start the flow of samples if needed.
     */
    virtual void start() {};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_BLOCK_INTERFACE_H
