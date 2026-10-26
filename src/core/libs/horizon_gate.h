/*!
 * \file horizon_gate.h
 * \brief Block that releases a stream up to the deterministic horizon
 * \author Carles Fernandez Prades, 2026 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_HORIZON_GATE_H
#define GNSS_SDR_HORIZON_GATE_H

#include "gnss_block_interface.h"
#include <gnuradio/block.h>
#include <chrono>
#include <cstddef>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

class horizon_gate;
using horizon_gate_sptr = gnss_shared_ptr<horizon_gate>;

/*!
 * \brief Makes a block that copies its input until the number of items set
 * by DeterministicClock::horizon(), so that no reader of its output sees an
 * item beyond it.
 * \param item_size Size of a stream item, in bytes.
 * \param batch_items Items released at once, except at the end of the stream.
 */
horizon_gate_sptr make_horizon_gate(size_t item_size, uint64_t batch_items);

/*!
 * \brief Copies a stream up to the deterministic horizon.
 */
class horizon_gate : public gr::block
{
public:
    void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items) override;

private:
    friend horizon_gate_sptr make_horizon_gate(size_t item_size, uint64_t batch_items);
    horizon_gate(size_t item_size, uint64_t batch_items);
    bool input_finished() const;

    std::chrono::steady_clock::time_point stalled_since_;
    size_t item_size_;
    bool stalled_ = false;
    bool stall_reported_ = false;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_HORIZON_GATE_H
