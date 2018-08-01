/*!
 * \file array_signal_conditioner.h
 * \brief It wraps blocks to change data type, filter and resample input data, adapted to array receiver
 * \author Javier Arribas jarribas (at) cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ARRAY_SIGNAL_CONDITIONER_H_
#define GNSS_SDR_ARRAY_SIGNAL_CONDITIONER_H_


#include <string>
#include <gnuradio/msg_queue.h>
#include "gnss_block_interface.h"


class ConfigurationInterface;
class AcquisitionInterface;
class TrackingInterface;
class TelemetryDecoderInterface;

/*!
 * \brief This class wraps blocks to change data_type_adapter, input_filter and resampler
 * to be applied to the input flow of sampled signal.
 */
class ArraySignalConditioner : public GNSSBlockInterface
{
public:
    //! Constructor
    ArraySignalConditioner(ConfigurationInterface *configuration,
        std::shared_ptr<GNSSBlockInterface> data_type_adapt, std::shared_ptr<GNSSBlockInterface> in_filt,
        std::shared_ptr<GNSSBlockInterface> res, std::string role, std::string implementation);

    //! Virtual destructor
    virtual ~ArraySignalConditioner();

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    inline std::string role() override { return role_; }
    //! Returns "Array_Signal_Conditioner"
    inline std::string implementation() override { return "Array_Signal_Conditioner"; }
    inline size_t item_size() override { return 0; }

    inline std::shared_ptr<GNSSBlockInterface> data_type_adapter() { return data_type_adapt_; }
    inline std::shared_ptr<GNSSBlockInterface> input_filter() { return in_filt_; }
    inline std::shared_ptr<GNSSBlockInterface> resampler() { return res_; }

private:
    std::shared_ptr<GNSSBlockInterface> data_type_adapt_;
    std::shared_ptr<GNSSBlockInterface> in_filt_;
    std::shared_ptr<GNSSBlockInterface> res_;
    std::string role_;
    std::string implementation_;
    bool connected_;
};

#endif /*GNSS_SDR_SIGNAL_CONDITIONER_H_*/
