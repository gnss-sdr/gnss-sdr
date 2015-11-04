/*!
 * \file spirent_prs_code_generator.h
 * \brief Defines a spirent prs code generator conforming to the long_code_interface
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_SPIRENT_PRS_CODE_GENERATOR_H_
#define GNSS_SDR_SPIRENT_PRS_CODE_GENERATOR_H_

#include "long_code_interface.h"
#include "gps_pcode.h"

class SpirentPrsCodeGenerator : public LongCodeInterface
{
    public:
        SpirentPrsCodeGenerator( int sv, bool is_e1 );

        bool get_chips( uint64_t first_chip_index,
                unsigned int num_chips,
                std::vector< short >& dest );

        void set_prn( int sv );

    private:
        int d_sv;

        unsigned int d_downsample_factor;

        unsigned int d_offset;

        GPS_P_Code_Generator d_code_gen;

        std::vector< short > d_pcode_store;

};

#endif // GNSS_SDR_SPIRENT_PRS_CODE_GENERATOR_H_
