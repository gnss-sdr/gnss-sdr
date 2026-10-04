/*!
 * \file evk1029_source.h
 * \brief Unpacks the SAPHYRION EVK1029 raw capture files (continuous stream
 * of OBA-encoded 4-bit samples, 16 samples packed per little-endian 64-bit
 * word, no periodic block header).
 *
 * This is the successor of the old EVK10x9_source block, updated for the
 * current EVK1029 capture format: the old format packed two 3-bit OBA
 * samples per byte and inserted an 8-byte header every 2048-byte block;
 * the current format is a continuous stream with no headers, packing two
 * 4-bit OBA samples per byte (16 samples per 64-bit word, per the
 * capture's .sdrx metadata: quantization=4, packedbits=64, encoding=OBA).
 *
 * The block has a single output port. Multiple RF bands sharing the same
 * raw capture (e.g. several Freq_Xlating_Fir_Filter instances tuned to
 * different IFs) are each connected to that one port (see the
 * signal-source-to-conditioner wiring in gnss_flowgraph.cc, which takes
 * this path whenever output_signature()->max_streams() == 1): GNU Radio
 * supports several readers on one producer port, each with its own read
 * pointer over the same buffer. The scheduler still throttles this block
 * to the slowest of those readers (buffer::space_available() is bounded by
 * the reader with the most unconsumed items), exactly as it would with one
 * buffer per port, so the bands cannot drift apart by more than one output
 * buffer. What the single port saves is the N-1 extra output buffers and
 * the N-1 full-buffer memcpy calls per work() call, at up to the raw
 * capture rate, that a port-per-band design needs in order to duplicate
 * the same decoded samples.
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

#ifndef GNSS_SDR_EVK1029_SOURCE_H
#define GNSS_SDR_EVK1029_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include <gnuradio/sync_block.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class Evk1029Source;

using Evk1029Source_sptr = gnss_shared_ptr<Evk1029Source>;

// bytes_to_skip: byte position in the raw capture at which reading starts,
// so that a long capture can be jumped straight to a time of interest
// instead of being processed from the beginning (the adapter derives it
// from its .seconds_to_skip parameter and the raw sampling frequency).
// Rounded up to the next multiple of 8 bytes (64 bits) here, matching the
// capture's native packing (16 OBA samples per little-endian 64-bit word):
// this block itself only ever reads byte-by-byte and would decode correctly
// from any byte offset, but seeking to a non-word-aligned byte splits a
// 64-bit DMA word the real hardware always transferred as one unit, which
// needlessly complicates comparing a jumped-to run against a from-the-start
// one at the file/word level.
Evk1029Source_sptr evk1029_make_source(const std::string& filename, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency, uint64_t bytes_to_skip = 0);

/*!
 * \brief Reads a continuous, header-less EVK1029 raw capture file and
 * unpacks its OBA-encoded 4-bit samples (two per byte, low nibble first).
 */
class Evk1029Source : public gr::sync_block
{
public:
    ~Evk1029Source() override;

private:
    friend Evk1029Source_sptr evk1029_make_source(const std::string& filename, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency, uint64_t bytes_to_skip);
    Evk1029Source(const std::string& filename, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency, uint64_t bytes_to_skip = 0);

    int work(int noutput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items) override;

    std::ifstream binary_input_file_;
    Concurrent_Queue<pmt::pmt_t>* queue_;
    std::vector<uint8_t> buffer_;
    std::size_t buffer_valid_;  // number of valid bytes currently in buffer_
    std::size_t buffer_pos_;    // next unread byte offset within buffer_
};


/** \} */
/** \} */
#endif  // GNSS_SDR_EVK1029_SOURCE_H
