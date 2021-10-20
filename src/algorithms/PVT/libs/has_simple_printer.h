/*!
 * \file has_simple_printer.h
 * \brief Interface of a class that prints HAS messages content in a txt file.
 * \author Carles Fernandez-Prades, 2021. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_HAS_SIMPLE_PRINTER_H
#define GNSS_SDR_HAS_SIMPLE_PRINTER_H

#include <cstddef>  // for size_t
#include <fstream>  // for std::ofstream
#include <mutex>    // for std::mutex
#include <string>   // for std::string
#include <vector>   // for std::vector

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */

class Galileo_HAS_data;

/*!
 * \brief Prints HAS messages content in a txt file. See HAS-SIS-ICD for a
 * message description.
 */
class Has_Simple_Printer
{
public:
    explicit Has_Simple_Printer(const std::string& base_path = std::string("."), const std::string& filename = std::string("HAS_Messages"), bool time_tag_name = true);
    ~Has_Simple_Printer();
    bool print_message(const Galileo_HAS_data* const has_data);

private:
    template <class T>
    std::string print_vector(const std::vector<T>& vec, float scale_factor = 1) const;

    template <class T>
    std::string print_vector_binary(const std::vector<T>& vec, size_t bit_length) const;

    template <class T>
    std::string print_matrix(const std::vector<std::vector<T>>& mat, const std::string& filler, float scale_factor = 1) const;

    std::string print_vector_string(const std::vector<std::string>& vec) const;

    bool close_file();

    std::mutex d_mutex;
    std::ofstream d_has_file;
    std::string d_has_filename;
    std::string d_has_base_path;
    bool d_data_printed;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_HAS_SIMPLE_PRINTER_H
