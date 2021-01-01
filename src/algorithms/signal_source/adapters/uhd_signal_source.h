/*!
 * \file uhd_signal_source.h
 * \brief Interface for the Universal Hardware Driver signal source
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_UHD_SIGNAL_SOURCE_H
#define GNSS_SDR_UHD_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/uhd/usrp_source.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <string>
#include <vector>


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */

class ConfigurationInterface;

/*!
 * \brief This class reads samples from a UHD device (see http://code.ettus.com/redmine/ettus/projects/uhd/wiki)
 */
class UhdSignalSource : public GNSSBlockInterface
{
public:
    UhdSignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~UhdSignalSource() = default;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "UHD_Signal_Source"
     */
    inline std::string implementation() override
    {
        return "UHD_Signal_Source";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;
    gr::basic_block_sptr get_right_block(int RF_channel) override;

private:
    gr::uhd::usrp_source::sptr uhd_source_;

    std::vector<gnss_shared_ptr<gr::block>> valve_;
    std::vector<gr::blocks::file_sink::sptr> file_sink_;
    std::vector<double> freq_;
    std::vector<double> gain_;
    std::vector<double> IF_bandwidth_hz_;
    std::vector<uint64_t> samples_;
    std::vector<std::string> dump_filename_;
    std::vector<bool> dump_;

    uhd::stream_args_t uhd_stream_args_;  // UHD SETTINGS

    std::string device_address_;
    std::string item_type_;
    std::string subdevice_;
    std::string clock_source_;
    std::string role_;

    double sample_rate_;
    size_t item_size_;
    int RF_channels_;
    unsigned int in_stream_;
    unsigned int out_stream_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_UHD_SIGNAL_SOURCE_H
