/*!
 * \file signal_conditioner.h
 * \brief It wraps blocks to change data type, filter and resample input data.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#ifndef GNSS_SDR_SIGNAL_CONDITIONER_H
#define GNSS_SDR_SIGNAL_CONDITIONER_H

#include "gnss_block_interface.h"
#include <gnuradio/block.h>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

/** \addtogroup Signal_Conditioner Signal Conditioner
 * Signal Conditioner wrapper block
 * \{ */
/** \addtogroup Signal_Conditioner_adapters conditioner_adapters
 * Wrap a Signal Conditioner with a GNSSBlockInterface
 * \{ */


/*!
 * \brief This class wraps blocks to change data_type_adapter, input_filter and resampler
 * to be applied to the input flow of sampled signal.
 */
class SignalConditioner : public GNSSBlockInterface
{
public:
    //! Constructor
    SignalConditioner(std::shared_ptr<GNSSBlockInterface> data_type_adapt,
        std::shared_ptr<GNSSBlockInterface> in_filt,
        std::shared_ptr<GNSSBlockInterface> res,
        std::string role,
        bool remove_identity = false);

    SignalConditioner(std::shared_ptr<GNSSBlockInterface> stage, std::string role, bool remove_identity);

    //! An empty conditioning chain, resolved to its upstream endpoint by the flowgraph.
    explicit SignalConditioner(std::string role, size_t input_item_size = 0);

    //! Destructor
    ~SignalConditioner() = default;

    void set_remove_identity(bool remove_identity);
    size_t identity_stage_count() const;
    bool ends_with_identity() const;
    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    inline std::string role() override { return role_; }

    inline std::string implementation() override { return "Signal_Conditioner"; }  //!< Returns "Signal_Conditioner"

    inline size_t item_size() override { return data_type_adapt_ ? data_type_adapt_->item_size() : input_item_size_; }
    bool is_identity() const override { return stages_.empty(); }

    inline std::shared_ptr<GNSSBlockInterface> data_type_adapter() { return data_type_adapt_; }
    inline std::shared_ptr<GNSSBlockInterface> input_filter() { return in_filt_; }
    inline std::shared_ptr<GNSSBlockInterface> resampler() { return res_; }

private:
    std::vector<std::shared_ptr<GNSSBlockInterface>> stages_;
    std::shared_ptr<GNSSBlockInterface> data_type_adapt_;
    std::shared_ptr<GNSSBlockInterface> in_filt_;
    std::shared_ptr<GNSSBlockInterface> res_;
    std::string role_;
    size_t input_item_size_ = 0;
    bool connected_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SIGNAL_CONDITIONER_H
