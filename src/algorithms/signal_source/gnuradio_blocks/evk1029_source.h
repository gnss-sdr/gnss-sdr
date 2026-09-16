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
 * Supports emitting multiple identical output streams (one read, N copies)
 * from a single work() call, so that two RF bands fed from the same raw
 * capture (e.g. two Freq_Xlating_Fir_Filter instances tuned to different
 * IFs) can never drift apart sample-index-wise: GNU Radio computes
 * noutput_items as the minimum available space across all declared output
 * ports, so backpressure from whichever downstream chain is slower stalls
 * production for every output port together, not just its own.
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

Evk1029Source_sptr evk1029_make_source(const std::string& filename, int n_streams, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency);

/*!
 * \brief Reads a continuous, header-less EVK1029 raw capture file and
 * unpacks its OBA-encoded 4-bit samples (two per byte, low nibble first),
 * writing the identical result to every declared output port.
 */
class Evk1029Source : public gr::sync_block
{
public:
    ~Evk1029Source() override;

private:
    friend Evk1029Source_sptr evk1029_make_source(const std::string& filename, int n_streams, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency);
    Evk1029Source(const std::string& filename, int n_streams, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency);

    int work(int noutput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items) override;

    std::ifstream binary_input_file_;
    Concurrent_Queue<pmt::pmt_t>* queue_;
    std::vector<uint8_t> buffer_;
    std::size_t buffer_valid_;  // number of valid bytes currently in buffer_
    std::size_t buffer_pos_;    // next unread byte offset within buffer_
    int n_streams_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_EVK1029_SOURCE_H
