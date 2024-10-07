/*!
 * \file fifo_reader.h
 *
 * \brief Header file to retrieve samples from an existing Unix FIFO
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

#ifndef GNSS_SDR_FIFO_READER_H_
#define GNSS_SDR_FIFO_READER_H_

#include "gnss_block_interface.h"
#include <gnuradio/sync_block.h>
#include <array>
#include <fstream>  // std::ifstream
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */
class FifoReader : virtual public gr::sync_block
{
public:
    //! \brief static function to create a class instance
    using sptr = gnss_shared_ptr<FifoReader>;
    static sptr make(const std::string &file_name, const std::string &sample_type);

    ~FifoReader() = default;

    //! initialize istream resource for FIFO
    bool start();

    // gnu radio work cycle function
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    //! \brief Constructor
    //! private constructor called by function make
    //! (gr handles this with public and private header pair)
    FifoReader(const std::string &file_name, const std::string &sample_type);

    size_t read_gr_complex(int noutput_items, gr_vector_void_star &output_items);

    //! function to read data out of FIFO which is stored as interleaved I/Q stream.
    //! template argument determines sample_type
    // Note: template definition necessary in header file
    // See also: https://stackoverflow.com/questions/495021/why-can-templates-only-be-implemented-in-the-header-file
    template <typename Type>
    size_t read_interleaved(int noutput_items, gr_vector_void_star &output_items)
    {
        size_t items_retrieved = 0;
        for (int n = 0; n < noutput_items; n++)
            {
                // TODO: try if performance increases if we copy larger chunks to vector.
                // how to read from stream: https://en.cppreference.com/w/cpp/io/basic_ifstream
                std::array<char, 2 * sizeof(Type)> buffer;
                fifo_.read(reinterpret_cast<char *>(buffer.data()), buffer.size());
                if (fifo_.good())
                    {
                        auto real = reinterpret_cast<Type const *>(&buffer[0]);
                        auto imag = reinterpret_cast<Type const *>(&buffer[sizeof(Type)]);
                        static_cast<gr_complex *>(output_items[0])[n] = gr_complex(*real, *imag);
                        items_retrieved++;
                    }
                else if (fifo_.eof())
                    {
                        fifo_.clear();
                        break;
                    }
                else
                    {
                        fifo_error_output();
                        break;
                    }
            }
        return items_retrieved;
    }

    //! this function moves logging output from this header into the source file
    //! thereby eliminating the need to include glog/logging.h in this header
    void fifo_error_output() const;

    const std::string file_name_;
    const std::string sample_type_;
    std::ifstream fifo_;
};

/** \} */
/** \} */
#endif /* GNSS_SDR_FIFO_READER_H_ */
