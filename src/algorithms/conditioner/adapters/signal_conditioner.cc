/*!
 * \file signal_conditioner.cc
 * \brief It holds blocks to change data type, filter and resample input data.
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

#include "signal_conditioner.h"
#include <stdexcept>
#include <utility>

SignalConditioner::SignalConditioner(std::string role, size_t input_item_size)
    : role_(std::move(role)), input_item_size_(input_item_size), connected_(false)
{
}


SignalConditioner::SignalConditioner(std::shared_ptr<GNSSBlockInterface> data_type_adapt,
    std::shared_ptr<GNSSBlockInterface> in_filt,
    std::shared_ptr<GNSSBlockInterface> res,
    std::string role, bool remove_identity)
    : data_type_adapt_(std::move(data_type_adapt)),
      in_filt_(std::move(in_filt)),
      res_(std::move(res)),
      role_(std::move(role)),
      connected_(false)
{
    if (!data_type_adapt_ || !in_filt_ || !res_)
        {
            throw std::invalid_argument("DataTypeAdapter, InputFilter and Resampler implementations must be defined");
        }
    // Validate even stages that will be removed: optimization must not hide a
    // misconfigured sample format at an intermediate boundary.
    const std::vector<std::shared_ptr<GNSSBlockInterface>> configured{data_type_adapt_, in_filt_, res_};
    for (size_t i = 0; i < configured.size(); ++i)
        {
            if (configured[i]->item_size() == 0)
                {
                    throw std::invalid_argument("itemsize mismatch: invalid SignalConditioner stage");
                }
            if (i > 0 && configured[i - 1]->get_right_block()->output_signature()->sizeof_stream_item(0) !=
                             configured[i]->get_left_block()->input_signature()->sizeof_stream_item(0))
                {
                    throw std::invalid_argument("itemsize mismatch: incompatible SignalConditioner stages");
                }
            if (!remove_identity || !configured[i]->is_identity())
                {
                    stages_.push_back(configured[i]);
                }
        }
}


SignalConditioner::SignalConditioner(std::shared_ptr<GNSSBlockInterface> stage, std::string role, bool remove_identity)
    : data_type_adapt_(std::move(stage)), role_(std::move(role)), connected_(false)
{
    if (!data_type_adapt_)
        {
            throw std::invalid_argument("Missing SignalConditioner stage");
        }
    set_remove_identity(remove_identity);
}


bool SignalConditioner::ends_with_identity() const
{
    const auto stage = res_ ? res_ : (in_filt_ ? in_filt_ : data_type_adapt_);
    return stage && stage->is_identity();
}


void SignalConditioner::set_remove_identity(bool remove_identity)
{
    if (connected_)
        {
            throw std::logic_error("Cannot change SignalConditioner topology while connected");
        }
    stages_.clear();
    for (const auto& stage : {data_type_adapt_, in_filt_, res_})
        {
            if (stage && (!remove_identity || !stage->is_identity()))
                {
                    stages_.push_back(stage);
                }
        }
}


size_t SignalConditioner::identity_stage_count() const
{
    size_t count = 0;
    for (const auto& stage : {data_type_adapt_, in_filt_, res_})
        {
            if (stage && stage->is_identity())
                {
                    ++count;
                }
        }
    return count;
}


void SignalConditioner::connect(gr::top_block_sptr top_block)
{
    if (connected_)
        {
            return;
        }
    size_t connected_stages = 0;
    size_t connected_edges = 0;
    try
        {
            for (const auto& stage : stages_)
                {
                    stage->connect(top_block);
                    ++connected_stages;
                }
            for (size_t i = 1; i < stages_.size(); ++i)
                {
                    top_block->connect(stages_[i - 1]->get_right_block(), 0, stages_[i]->get_left_block(), 0);
                    ++connected_edges;
                }
            connected_ = true;
        }
    catch (...)
        {
            while (connected_edges > 0)
                {
                    const size_t i = connected_edges--;
                    top_block->disconnect(stages_[i - 1]->get_right_block(), 0, stages_[i]->get_left_block(), 0);
                }
            while (connected_stages > 0)
                {
                    stages_[--connected_stages]->disconnect(top_block);
                }
            throw;
        }
}


void SignalConditioner::disconnect(gr::top_block_sptr top_block)
{
    if (!connected_)
        {
            return;
        }
    for (size_t i = stages_.size(); i > 1; --i)
        {
            top_block->disconnect(stages_[i - 2]->get_right_block(), 0, stages_[i - 1]->get_left_block(), 0);
        }
    for (auto stage = stages_.rbegin(); stage != stages_.rend(); ++stage)
        {
            (*stage)->disconnect(top_block);
        }
    connected_ = false;
}


gr::basic_block_sptr SignalConditioner::get_left_block()
{
    return stages_.empty() ? gr::basic_block_sptr() : stages_.front()->get_left_block();
}


gr::basic_block_sptr SignalConditioner::get_right_block()
{
    return stages_.empty() ? gr::basic_block_sptr() : stages_.back()->get_right_block();
}
