/*!
 * \file ad9361_fpga_signal_source.h
 * \brief signal source for Analog Devices front-end AD9361 connected directly to FPGA accelerators.
 * This source implements only the AD9361 control. It is NOT compatible with conventional SDR acquisition and tracking blocks.
 * Please use the fmcomms2 source if conventional SDR acquisition and tracking is selected in the configuration file.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_AD9361_FPGA_SIGNAL_SOURCE_H_
#define GNSS_SDR_AD9361_FPGA_SIGNAL_SOURCE_H_

#include "gnss_block_interface.h"
#include "fpga_switch.h"
#include <boost/shared_ptr.hpp>
#include <gnuradio/msg_queue.h>
#include <string>

class ConfigurationInterface;

class Ad9361FpgaSignalSource : public GNSSBlockInterface
{
public:
    Ad9361FpgaSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream,
        unsigned int out_stream, boost::shared_ptr<gr::msg_queue> queue);

    ~Ad9361FpgaSignalSource();

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Ad9361_Fpga_Signal_Source"
     */
    inline std::string implementation() override
    {
        return "Ad9361_Fpga_Signal_Source";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    std::string role_;

    // Front-end settings
    std::string uri_;     // device direction
    unsigned long freq_;  // frequency of local oscillator
    unsigned long sample_rate_;
    unsigned long bandwidth_;
    unsigned long buffer_size_;  // reception buffer
    bool rx1_en_;
    bool rx2_en_;
    bool quadrature_;
    bool rf_dc_;
    bool bb_dc_;
    std::string gain_mode_rx1_;
    std::string gain_mode_rx2_;
    double rf_gain_rx1_;
    double rf_gain_rx2_;
    std::string rf_port_select_;
    std::string filter_file_;
    bool filter_auto_;

    // DDS configuration for LO generation for external mixer
    bool enable_dds_lo_;
    unsigned long freq_rf_tx_hz_;
    unsigned long freq_dds_tx_hz_;
    double scale_dds_dbfs_;
    double phase_dds_deg_;
    double tx_attenuation_db_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    std::string item_type_;
    size_t item_size_;
    long samples_;
    bool dump_;
    std::string dump_filename_;

    boost::shared_ptr<gr::msg_queue> queue_;

    std::shared_ptr<fpga_switch> switch_fpga;
};

#endif /*GNSS_SDR_AD9361_FPGA_SIGNAL_SOURCE_H_*/
