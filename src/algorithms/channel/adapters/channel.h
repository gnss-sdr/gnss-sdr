/*!
 * \file channel.h
 * \brief Interface of a GNSS channel.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * It holds blocks for acquisition, tracking,
 * navigation data extraction and pseudorange calculation.
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

#ifndef GNSS_SDR_CHANNEL_H
#define GNSS_SDR_CHANNEL_H

#include "channel_fsm.h"
#include "channel_interface.h"
#include "channel_msg_receiver_cc.h"
#include "concurrent_queue.h"
#include "gnss_signal.h"
#include "gnss_synchro.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

/** \addtogroup Channel
 * Classes containing a GNSS channel.
 * \{ */
/** \addtogroup Channel_adapters channel_adapters
 * Classes that wrap an AcquisitionInterface,
 * a TrackingInterface and a TelemetryDecoderInterface, and handles
 * their interaction.
 * \{ */


class ConfigurationInterface;
class AcquisitionInterface;
class TrackingInterface;
class TelemetryDecoderInterface;


/*!
 * \brief This class represents a GNSS channel. It wraps an AcquisitionInterface,
 * a TrackingInterface and a TelemetryDecoderInterface, and handles
 * their interaction through a Finite State Machine
 *
 */
class Channel : public ChannelInterface
{
public:
    //! Constructor
    Channel(const ConfigurationInterface* configuration,
        uint32_t channel,
        std::shared_ptr<AcquisitionInterface> acq,
        std::shared_ptr<TrackingInterface> trk,
        std::shared_ptr<TelemetryDecoderInterface> nav,
        const std::string& role,
        const std::string& signal_str,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~Channel() = default;  //!< Destructor

    void connect(gr::top_block_sptr top_block) override;  //!< Connects the tracking block to the top_block and to the telemetry
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_left_block_trk() override;   //!< Gets the GNU Radio tracking block input pointer
    gr::basic_block_sptr get_right_block_trk() override;  //!< Gets the GNU Radio tracking block output pointer
    gr::basic_block_sptr get_left_block_acq() override;   //!< Gets the GNU Radio acquisition block input pointer
    gr::basic_block_sptr get_right_block_acq() override;  //!< Gets the GNU Radio acquisition block output pointer
    gr::basic_block_sptr get_right_block() override;      //!< Gets the GNU Radio channel block output pointer

    inline std::string role() override { return role_; }
    inline std::string implementation() override { return std::string("Channel"); }  //!< Returns "Channel"
    inline size_t item_size() override { return 2 * sizeof(float); }
    inline Gnss_Signal get_signal() const override { return gnss_signal_; }
    void start_acquisition() override;                          //!< Start the State Machine
    void stop_channel() override;                               //!< Stop the State Machine
    void set_signal(const Gnss_Signal& gnss_signal_) override;  //!< Sets the channel GNSS signal

    void assist_acquisition_doppler(double Carrier_Doppler_hz) override;

    inline std::shared_ptr<AcquisitionInterface> acquisition() const { return acq_; }
    inline std::shared_ptr<TrackingInterface> tracking() const { return trk_; }
    inline std::shared_ptr<TelemetryDecoderInterface> telemetry() const { return nav_; }

private:
    bool glonass_dll_pll_c_aid_tracking_check() const;
    std::shared_ptr<ChannelFsm> channel_fsm_;
    std::shared_ptr<AcquisitionInterface> acq_;
    std::shared_ptr<TrackingInterface> trk_;
    std::shared_ptr<TelemetryDecoderInterface> nav_;
    channel_msg_receiver_cc_sptr channel_msg_rx_;
    Gnss_Synchro gnss_synchro_{};
    Gnss_Signal gnss_signal_;
    std::string role_;
    std::mutex mx_;
    uint32_t channel_;
    int glonass_extend_correlation_ms_;
    bool connected_;
    bool repeat_;
    bool flag_enable_fpga_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CHANNEL_H
