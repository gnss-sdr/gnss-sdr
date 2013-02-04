/*!
 * \file signal_conditioner.h
 * \brief It wraps blocks to change data type, filter and resample input data.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
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

#ifndef GNSS_SDR_SIGNAL_CONDITIONER_H_
#define GNSS_SDR_SIGNAL_CONDITIONER_H_

#include <gnuradio/gr_null_sink.h>
#include <gnuradio/gr_msg_queue.h>
#include "gnss_block_interface.h"
#include "control_message_factory.h"


class ConfigurationInterface;
class AcquisitionInterface;
class TrackingInterface;
class TelemetryDecoderInterface;

/*!
 * \brief This class wraps blocks to change data_type_adapter, input_filter and resampler
 * to be applied to the input flow of sampled signal.
 */
class SignalConditioner: public GNSSBlockInterface
{
public:
    //! Constructor
    SignalConditioner(ConfigurationInterface *configuration,
            GNSSBlockInterface *data_type_adapt, GNSSBlockInterface *in_filt,
            GNSSBlockInterface *res, std::string role, std::string implementation,
            gr_msg_queue_sptr queue);

    //! Virtual destructor
    virtual ~SignalConditioner();

    void connect(gr_top_block_sptr top_block);
    void disconnect(gr_top_block_sptr top_block);
    gr_basic_block_sptr get_left_block();
    gr_basic_block_sptr get_right_block();

    std::string role(){ return role_; }
    //! Returns "Signal_Conditioner"
    std::string implementation(){ return "Signal_Conditioner"; }
    size_t item_size(){ return 0; }

    GNSSBlockInterface *data_type_adapter(){ return data_type_adapt_; }
    GNSSBlockInterface *input_filter(){ return in_filt_; }
    GNSSBlockInterface *resampler(){ return res_; }

private:
    GNSSBlockInterface *data_type_adapt_;
    GNSSBlockInterface *in_filt_;
    GNSSBlockInterface *res_;
    std::string role_;
    std::string implementation_;
    bool connected_;
    //bool stop_;
    gr_msg_queue_sptr queue_;
};

#endif /*GNSS_SDR_SIGNAL_CONDITIONER_H_*/
