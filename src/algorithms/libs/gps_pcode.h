/*!
 * \file gps_pcode.h
 * \brief Defines functions for dealing with the GPS P COde
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

#ifndef GNSS_SDR_GPS_PCODE_H_
#define GNSS_SDR_GPS_PCODE_H_

#include <vector>
#include <cstdint>

//!Generates the GPS X1A code as a vector of short integers
void gps_x1a_code_gen(std::vector< short > & _dest );

//!Generates the GPS X1B code as a vector of short integers
void gps_x1b_code_gen(std::vector< short > & _dest );

//!Generates the GPS X2A code as a vector of short integers
void gps_x2a_code_gen(std::vector< short > & _dest );

//!Generates the GPS X2B code as a vector of short integers
void gps_x2b_code_gen(std::vector< short > & _dest );

//! Class for generating p code chips
class GPS_P_Code_Generator
{
    public:
    GPS_P_Code_Generator();
    ~GPS_P_Code_Generator();

    void get_chips( int sv, uint64_t first_chip_index,
            unsigned N, std::vector< short > & _dest ) const;

    private:
        std::vector< short > d_x1a;
        std::vector< short > d_x1b;
        std::vector< short > d_x2a;
        std::vector< short > d_x2b;

        //! Number of chips in an X1 epoch
        static const int64_t X1_EPOCH_CHIPS;

        //! Number of chips in an X2 epoch
        static const int64_t X2_EPOCH_CHIPS;

        //! Number of X1A epochs per X1 epoch
        static const unsigned int X1A_EPOCHS_PER_X1_EPOCH;
        //! Number of X1B epochs per X1 epoch
        static const unsigned int X1B_EPOCHS_PER_X1_EPOCH;

        //! Number of X2A epochs per X2 epoch
        static const unsigned int X2A_EPOCHS_PER_X2_EPOCH;
        //! Number of X2B epochs per X2 epoch
        static const unsigned int X2B_EPOCHS_PER_X2_EPOCH;

        //! The Number of X1 epochs per week
        static const unsigned int X1_EPOCHS_PER_WEEK;

        //! The number of x2 epochs per week:
        static const unsigned int X2_EPOCHS_PER_WEEK;

        //! The extra length of the X1B code due to the hold at the end
        //of each X1 epoch:
        static const unsigned int X1B_EXTRA_LENGTH;

        //! The extra length of the X2A code due to the hold a the end
        //of the week - note that this is also sufficient for the hold
        //at the end of the x1 epoch.
        static const unsigned int X2A_EXTRA_LENGTH;

        //! The extra length of the X2B code due to the hold at the end
        //of the week - note this is also sufficient for the hold at the
        //end of each X1 epoch.
        static const unsigned int X2B_EXTRA_LENGTH;

        //! The number of X2A epochs in the last X1 epoch of the week
        static const unsigned int X2A_EPOCHS_LAST_X2_EPOCH;

        //! The number of X2B epochs in the last x1 epoch of the week
        static const unsigned int X2B_EPOCHS_LAST_X2_EPOCH;

};

#endif /* GNSS_SDR_GPS_PCODE_H_ */

