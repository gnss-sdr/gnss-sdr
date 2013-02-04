/*!
 * \file rtlsdr_signal_source.h
 * \brief Signal source for the Realtek RTL2832U USB dongle DVB-T receiver
 * (see http://sdr.osmocom.org/trac/wiki/rtl-sdr for more information)
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef GNSS_SDR_RTLSDR_SIGNAL_SOURCE_H_
#define GNSS_SDR_RTLSDR_SIGNAL_SOURCE_H_

#include <boost/shared_ptr.hpp>
#include <osmosdr/osmosdr_source_c.h>
#include <gnuradio/gr_hier_block2.h>
#include <gnuradio/gr_msg_queue.h>
#include "gnss_block_interface.h"

class ConfigurationInterface;

/*!
 * \brief This class reads samples from Realtek RTL2832U USB dongle DVB-T receiver (see http://sdr.osmocom.org/trac/wiki/rtl-sdr)
 */
class RtlsdrSignalSource: public GNSSBlockInterface
{
public:
	RtlsdrSignalSource(ConfigurationInterface* configuration,
            std::string role, unsigned int in_stream,
            unsigned int out_stream, gr_msg_queue_sptr queue);

    virtual ~RtlsdrSignalSource();

    std::string role()
    {
        return role_;
    }

    /*!
     * \brief Returns "Rtlsdr_Signal_Source"
     */
    std::string implementation()
    {
        return "Rtlsdr_Signal_Source";
    }
    size_t item_size()
    {
        return item_size_;
    }

    void connect(gr_top_block_sptr top_block);
    void disconnect(gr_top_block_sptr top_block);
    gr_basic_block_sptr get_left_block();
    gr_basic_block_sptr get_right_block();

private:
    std::string role_;

    // UHD SETTINGS
    bool AGC_enabled_;
    double sample_rate_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    double freq_;
    double gain_;

    std::string item_type_;
    size_t item_size_;
    long samples_;
    bool dump_;
    std::string dump_filename_;

    boost::shared_ptr<osmosdr_source_c> rtlsdr_source_;

    gr_block_sptr valve_;
    gr_block_sptr file_sink_;
    gr_msg_queue_sptr queue_;
};

#endif /*GNSS_SDR_RTLSDR_SIGNAL_SOURCE_H_*/
