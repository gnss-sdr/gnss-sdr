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
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

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
      sample_type_(sample_type)
{
    std::cout << "Starting FifoReader\n";
    // instream: https://en.cppreference.com/w/cpp/io/basic_ifstream
}

bool FifoReader::start()
{
    fifo_.open(file_name_, std::ios::binary);
    if (!fifo_.is_open())
        {
            DLOG(ERROR) << "Error opening fifo\n";
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
            DLOG(ERROR) << "FifoReader connected to too many outputs\n";
        }

    // read samples out
    size_t items_retrieved = 0;
    if (sample_type_ == "ishort")
        {
            // ishort == int16_t
            items_retrieved = read_interleaved<int16_t>(noutput_items, output_items);
        }
    else if (sample_type_ == "gr_complex")
        {
            DLOG(WARNING) << sample_type_ << " is not yet tested. Please consider removing this warning if tested successfully\n";
            items_retrieved = read_gr_complex(noutput_items, output_items);
        }
    else
        {
            // please see gr_complex_ip_packet_source for inspiration on how to implement other sample types
            DLOG(ERROR) << sample_type_ << " is unfortunately not yet implemented as sample type\n";
        }

    // we return varying number of data -> call produce & return flag
    produce(0, items_retrieved);
    return this->WORK_CALLED_PRODUCE;
}

// read gr_complex items from fifo
size_t FifoReader::read_gr_complex(int noutput_items, gr_vector_void_star &output_items)
{
    size_t items_retrieved = 0;
    for (int n = 0; n < noutput_items; n++)
        {
            std::vector<char> buffer(4);
            fifo_.read((char *)&buffer[0], buffer.size());
            if (fifo_.good())
                {
                    gr_complex sample;
                    memcpy(&sample, &buffer[0], sizeof(sample));
                    static_cast<gr_complex *>(output_items.at(0))[n] = sample;
                    items_retrieved++;
                }
            else if (fifo_.eof() || fifo_.fail())
                {
                    // not enough samples.. what if we did not have a complete sample? need to reassemble in between loops
                    fifo_.clear();
                    break;
                }
        }
    return items_retrieved;
}
