/*!
 * \file fifo_reader.cc
 *
 * \brief Implementation of the class to retrieve samples from an existing Unix FIFO
 * \author Malte Lenhart, 2021. malte.lenhart(at)mailbox.org
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

#include "fifo_reader.h"
#include <glog/logging.h>
#include <volk/volk.h>

const int FIFO_SIZE = 1472000;  // Value taken from gr_complex_ip_packet_source, is it optimal?

// initial construction; pass to private constructor
FifoReader::sptr FifoReader::make(const std::string &file_name, const std::string &sample_type)
{
    return gnuradio::get_initial_sptr(new FifoReader(file_name, sample_type));
}

// private constructor called by ::make
FifoReader::FifoReader(const std::string &file_name, const std::string &sample_type)
    : gr::sync_block("fifo_reader",
          gr::io_signature::make(0, 0, 0),                    // no input
          gr::io_signature::make(1, 1, sizeof(gr_complex))),  // <+MIN_OUT+>, <+MAX_OUT+>, sizeof(<+OTYPE+>)
      file_name_(file_name),
      sample_type_(sample_type),
      fifo_buffer_(new char[FIFO_SIZE]),
      buffer_idx_(0),
      buffer_size_(0)
{
    DLOG(INFO) << "Starting FifoReader";
}

bool FifoReader::start()
{
    fifo_.open(file_name_, std::ios::binary);
    if (!fifo_.is_open())
        {
            LOG(ERROR) << "Error opening FIFO";
            return false;
        }
    return true;
}

// work loop
// taken from here: https://stackoverflow.com/questions/25546619/work-with-fifo-in-c-blocking-read
int FifoReader::work(int noutput_items,
    __attribute__((unused)) gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    if (output_items.size() > 1)
        {
            LOG(ERROR) << "FifoReader connected to too many outputs";
        }

    // read samples out
    size_t items_retrieved = 0;
    if (sample_type_ == "ishort")
        {
            items_retrieved = read_interleaved<int16_t>(noutput_items, output_items);
        }
    else if (sample_type_ == "ibyte")  // Does this also work with cbyte?
        {
            items_retrieved = read_interleaved<int8_t>(noutput_items, output_items);
        }
    else if (sample_type_ == "gr_complex")
        {
            items_retrieved = read_gr_complex(noutput_items, output_items);
        }
    else
        {
            // please see gr_complex_ip_packet_source for inspiration on how to implement other sample types
            LOG(ERROR) << sample_type_ << " is unfortunately not yet implemented as sample type";
        }

    // we return varying number of data -> call produce & return flag
    produce(0, items_retrieved);
    return this->WORK_CALLED_PRODUCE;
}


template <typename Type>
size_t FifoReader::read_interleaved(int noutput_items, gr_vector_void_star &output_items)
{
    boost::mutex::scoped_lock lock(d_mutex);  // hold mutex for duration of this function
    size_t items_retrieved = 0;
    int n;
    Type real;
    Type imag;

    if (buffer_idx_ >= buffer_size_)
        {
            fifo_.read(fifo_buffer_, FIFO_SIZE);
            buffer_idx_ = 0;
            if (fifo_.good())
                {
                    buffer_size_ = FIFO_SIZE;
                }
            else if (fifo_.eof())
                {
                    // Although we got an EOF, ensure we don't lose the other samples
                    buffer_size_ = fifo_.gcount();
                    fifo_.clear();
                }
            else
                {
                    fifo_error_output();
                    return 0;
                }
        }

    for (n = 0; n < noutput_items && buffer_idx_ < buffer_size_; n++)
        {
            memcpy(&real, &fifo_buffer_[buffer_idx_], sizeof(real));
            memcpy(&imag, &fifo_buffer_[buffer_idx_ + sizeof(imag)], sizeof(imag));
            static_cast<gr_complex *>(output_items.at(0))[n] = gr_complex(real, imag);

            buffer_idx_ += 2 * sizeof(Type);
        }

    items_retrieved = n;

    return items_retrieved;
}


// read gr_complex items from fifo
// this fct has duplicate code with the templated read_interleaved fct above
size_t FifoReader::read_gr_complex(int noutput_items, gr_vector_void_star &output_items)
{
    boost::mutex::scoped_lock lock(d_mutex);  // hold mutex for duration of this function
    size_t items_retrieved = 0;
    int n;
    gr_complex sample;

    if (buffer_idx_ >= buffer_size_)
        {
            fifo_.read(fifo_buffer_, FIFO_SIZE);
            buffer_idx_ = 0;
            if (fifo_.good())
                {
                    buffer_size_ = FIFO_SIZE;
                }
            else if (fifo_.eof())
                {
                    // Although we got an EOF, ensure we don't lose the other samples
                    buffer_size_ = fifo_.gcount();
                    fifo_.clear();
                }
            else
                {
                    fifo_error_output();
                    return 0;
                }
        }

    for (n = 0; n < noutput_items && buffer_idx_ < buffer_size_; n++)
        {
            memcpy(&sample, &fifo_buffer_[buffer_idx_], sizeof(gr_complex));
            static_cast<gr_complex *>(output_items.at(0))[n] = sample;

            buffer_idx_ += sizeof(gr_complex);
        }

    items_retrieved = n;

    return items_retrieved;
}

void FifoReader::fifo_error_output() const
{
    LOG(ERROR) << "unhandled FIFO event";
}


FifoReader::~FifoReader()
{
    delete[] fifo_buffer_;
}
