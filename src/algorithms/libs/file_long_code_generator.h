/*!
 * \file file_long_code_generator.h
 * \brief Defines a code generator conforming to the long_code_interface that
 * reads the chips from a file.
 * \author Cillian O'Driscoll, 2016. cillian.odriscoll(at)gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2016  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FILE_LONG_CODE_GENERATOR_H_
#define GNSS_SDR_FILE_LONG_CODE_GENERATOR_H_

#include "long_code_interface.h"
#include <istream>

class FileLongCodeGenerator : public LongCodeInterface
{
    public:
        FileLongCodeGenerator( int sv, std::istream &is  );

        bool get_chips( uint64_t first_chip_index,
                unsigned int num_chips,
                std::vector< short >& dest );

        void set_prn( int sv );

        uint64_t get_code_length( void ) const;

    private:

        int d_sv;

        uint64_t d_first_chip_index;
        uint64_t d_num_chips;
        uint64_t d_code_length;

        std::vector< short > d_the_chips;

        void initialise( std::istream &is );

};

#endif // GNSS_SDR_PCODE_GENERATOR_H_

