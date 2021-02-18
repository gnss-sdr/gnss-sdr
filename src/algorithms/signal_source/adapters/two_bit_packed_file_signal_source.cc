/*!
 * \file two_bit_packed_file_signal_source.cc
 * \brief Interface of a class that reads signals samples from a file. Each
 * sample is two bits, which are packed into bytes or shorts.
 *
 * \author Cillian O'Driscoll, 2015 cillian.odriscoll (at) gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "two_bit_packed_file_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_string_literals.h"
#include <glog/logging.h>
#include <gnuradio/blocks/char_to_float.h>

using namespace std::string_literals;

TwoBitPackedFileSignalSource::TwoBitPackedFileSignalSource(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : FileSourceBase(configuration, role, "Two_Bit_Packed_File_Signal_Source"s, queue, "byte"s), sample_type_(configuration->property(role + ".sample_type", "real"s)),  // options: "real", "iq", "qi"
      big_endian_items_(configuration->property(role + ".big_endian_items", true)),
      big_endian_bytes_(configuration->property(role + ".big_endian_bytes", false)),
      reverse_interleaving_(false)
{
    if (in_streams > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


std::tuple<size_t, bool> TwoBitPackedFileSignalSource::itemTypeToSize()
{
    auto is_complex_t = false;
    auto item_size = size_t(sizeof(char));  // default

    if (item_type() == "byte")
        {
            item_size = sizeof(char);
        }
    else if (item_type() == "short")
        {
            // If we have shorts stored in little endian format, might as
            // well read them in as bytes.
            // TODO: this seems to make assumptions about the endianness of this machine
            if (big_endian_items_)
                {
                    item_size = sizeof(int16_t);
                }
            else
                {
                    // how can this be right? the number of samples is computed based on this value
                    item_size = sizeof(char);
                }
        }
    else
        {
            LOG(WARNING) << item_type() << " unrecognized item type. Using byte.";
        }

    // the complex-ness of the input is inferred from the output type
    if (sample_type_ == "real")
        {
            is_complex_t = false;
        }
    else if (sample_type_ == "iq")
        {
            is_complex_t = true;
        }
    else if (sample_type_ == "qi")
        {
            is_complex_t = true;
            reverse_interleaving_ = true;
        }
    else
        {
            LOG(WARNING) << sample_type_ << " unrecognized sample type. Assuming: "
                         << (is_complex_t ? (reverse_interleaving_ ? "qi" : "iq") : "real");
        }


    return std::make_tuple(item_size, is_complex_t);
}

// Each sample is 2 bits; if the item_type() is char, then the size is 8/2 = 4 packets per sample
// If the item_type() is short, then the size is 16/2 = 8 packets per sample
double TwoBitPackedFileSignalSource::packetsPerSample() const { return item_size() / 2.0; }
gnss_shared_ptr<gr::block> TwoBitPackedFileSignalSource::source() const { return char_to_float_; }

void TwoBitPackedFileSignalSource::create_file_source_hook()
{
    unpack_samples_ = make_unpack_2bit_samples(big_endian_bytes_, item_size(),
        big_endian_items_, reverse_interleaving_);
    DLOG(INFO) << "unpack_byte_2bit_samples(" << unpack_samples_->unique_id() << ")";

    if (is_complex())
        {
            char_to_float_ = gr::blocks::interleaved_char_to_complex::make(false);
            DLOG(INFO) << "interleaved_char_to_complex(" << char_to_float_->unique_id() << ")";
        }
    else
        {
            char_to_float_ = gr::blocks::char_to_float::make();
            DLOG(INFO) << "char_to_float(" << char_to_float_->unique_id() << ")";
        }
}

void TwoBitPackedFileSignalSource::pre_connect_hook(gr::top_block_sptr top_block)
{
    top_block->connect(file_source(), 0, unpack_samples_, 0);
    DLOG(INFO) << "connected file source to unpack samples";
    top_block->connect(unpack_samples_, 0, char_to_float_, 0);
    DLOG(INFO) << "connected unpack samples to char to float";
}

void TwoBitPackedFileSignalSource::pre_disconnect_hook(gr::top_block_sptr top_block)
{
    top_block->disconnect(file_source(), 0, unpack_samples_, 0);
    DLOG(INFO) << "disconnected file source to unpack samples";
    top_block->disconnect(unpack_samples_, 0, char_to_float_, 0);
    DLOG(INFO) << "disconnected unpack samples to char to float";
}
