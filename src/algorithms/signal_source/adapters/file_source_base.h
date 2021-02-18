/*!
 * \file file_source_base.h
 * \brief Header file of the base class to file-oriented signal_source GNSS blocks.
 * \author Jim Melton, 2021. jim.melton(at)sncorp.com
 *
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

#ifndef GNSS_SDR_FILE_SOURCE_BASE_H
#define GNSS_SDR_FILE_SOURCE_BASE_H

#include "concurrent_queue.h"
#include "signal_source_base.h"
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/throttle.h>
#include <pmt/pmt.h>
#include <tuple>

// for dump
#include <gnuradio/blocks/file_sink.h>
#include <cstddef>
#include <string>


class ConfigurationInterface;


//! \brief Base class to file-oriented SignalSourceBase GNSS blocks.
//!
//! This class supports the following properties:
//!
//!   .filename - the path to the input file
//!             - may be overridden by the -signal_source or -s command-line arguments
//!
//!   .samples  - number of samples to process (default 0)
//!             - if not specified or 0, read the entire file; otherwise stop after that many samples
//!
//!   .sampling_frequency - the frequency of the sampled data (samples/second)
//!
//!   .item_type - data type of the samples (default "short")
//!
//!   .header_size - the size of a prefixed header to skip in "samples" (default 0)
//!
//!   .seconds_to_skip - number of seconds of lead-in data to skip over (default 0)
//!
//!   .enable_throttle_control - whether to stop reading if the upstream buffer is full (default false)
//!
//!   .repeat   - whether to rewind and continue at end of file (default false)
//!
//! (probably abstracted to the base class)
//!
//!   .dump     - whether to archive input data
//!
//!   .dump_filename - if dumping, path to file for output


class FileSourceBase : public SignalSourceBase
{
public:
    // Virtual overrides
    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    //! The file to read
    std::string filename() const;

    //! The item type
    std::string item_type() const;

    //! The configured size of each item
    size_t item_size() override;
    virtual size_t item_size() const;  // what the interface **should** have declared

    //! Whether to repeat reading after end-of-file
    bool repeat() const;

    //! The sampling frequency of the source file
    int64_t sampling_frequency() const;

    //! The number of samples in the file
    uint64_t samples() const;

protected:
    //! \brief Constructor
    //!
    //! Subclasses may want to assert default item types that are appropriate to the specific file
    //! type supported. Rather than require the item type to be specified in the config file, allow
    //! sub-classes to impose their will
    FileSourceBase(ConfigurationInterface const* configuration, std::string const& role, std::string impl,
        Concurrent_Queue<pmt::pmt_t>* queue,
        std::string default_item_type = "short");

    //! Perform post-construction initialization
    void init();

    //! Compute the item size, from the item_type(). Subclasses may constrain types that don't make
    //  sense. The return of this method is a tuple of item_size and is_complex
    virtual std::tuple<size_t, bool> itemTypeToSize();

    //! The number of (possibly unpacked) samples in a (raw) file sample (default=1)
    virtual double packetsPerSample() const;

    //! Compute the number of samples to skip
    virtual size_t samplesToSkip() const;

    //! Compute the number of samples in the file
    size_t computeSamplesInFile() const;

    //! Abstracted front-end source. Sub-classes may override if they create specialized chains to
    //! decode source files into a usable format
    virtual gnss_shared_ptr<gr::block> source() const;

    //! For complex source chains, the size of the file item may not be the same as the size of the
    // "source" (decoded) item. This method allows subclasses to handle these differences
    virtual size_t source_item_size() const;
    bool is_complex() const;

    // Generic access to created objects
    gnss_shared_ptr<gr::block> file_source() const;
    gnss_shared_ptr<gr::block> valve() const;
    gnss_shared_ptr<gr::block> throttle() const;
    gnss_shared_ptr<gr::block> sink() const;

    // The methods create the various blocks, if enabled, and return access to them. The created
    // object is also held in this class
    gr::blocks::file_source::sptr create_file_source();
    gr::blocks::throttle::sptr create_throttle();
    gnss_shared_ptr<gr::block> create_valve();
    gr::blocks::file_sink::sptr create_sink();

    // Subclass hooks to augment created objects, as required
    virtual void create_file_source_hook();
    virtual void create_throttle_hook();
    virtual void create_valve_hook();
    virtual void create_sink_hook();

    // Subclass hooks for connection/disconnection
    virtual void pre_connect_hook(gr::top_block_sptr top_block);
    virtual void post_connect_hook(gr::top_block_sptr top_block);
    virtual void pre_disconnect_hook(gr::top_block_sptr top_block);
    virtual void post_disconnect_hook(gr::top_block_sptr top_block);

private:
    std::string filename_;
    gr::blocks::file_source::sptr file_source_;

    std::string item_type_;
    size_t item_size_;
    bool is_complex_;  // a misnomer; if I/Q are interleaved as integer values

    size_t header_size_;  // length (in samples) of the header (if any)
    double seconds_to_skip_;
    bool repeat_;

    // The valve allows only the configured number of samples through, then it closes.

    // The framework passes the queue as a naked pointer, rather than a shared pointer, so this
    // class has two choices: construct the valve in the ctor, or hold onto the pointer, possibly
    // beyond its lifetime. Fortunately, the queue is only used to create the valve, so the
    // likelihood of holding a stale pointer is mitigated
    uint64_t samples_;
    int64_t sampling_frequency_;  // why is this signed
    gnss_shared_ptr<gr::block> valve_;
    Concurrent_Queue<pmt::pmt_t>* queue_;

    bool enable_throttle_control_;
    gr::blocks::throttle::sptr throttle_;

    bool dump_;
    std::string dump_filename_;
    gr::blocks::file_sink::sptr sink_;
};


#endif
