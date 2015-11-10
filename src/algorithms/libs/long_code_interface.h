/*!
 * \file long_code_interface.h
 * \brief Defines the interface for a long code generator.
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


#ifndef GNSS_SDR_LONG_CODE_INTERFACE_H_
#define GNSS_SDR_LONG_CODE_INTERFACE_H_

#include <vector>
#include <cstdint>
#include <boost/shared_ptr.hpp>

class LongCodeInterface;

typedef boost::shared_ptr<LongCodeInterface> LongCodeInterface_sptr;

class LongCodeInterface
{
    public:
        virtual bool get_chips( uint64_t first_chip_index,
                unsigned int num_chips,
                std::vector< short > &dest ) = 0;

        virtual void set_prn( int sv ) = 0;

        virtual uint64_t get_code_length() const = 0;
};

#endif // GNSS_SDR_LONG_CODE_INTERFACE_H_
