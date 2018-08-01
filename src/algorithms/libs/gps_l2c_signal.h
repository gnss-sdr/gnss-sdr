/*!
 * \file gps_l2c_signal.h
 * \brief This class implements signal generators for the GPS L2C signals
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_L2C_SIGNAL_H_
#define GNSS_SDR_GPS_L2C_SIGNAL_H_

#include <complex>


//!Generates complex GPS L2C M code for the desired SV ID
void gps_l2c_m_code_gen_complex(std::complex<float>* _dest, unsigned int _prn);
void gps_l2c_m_code_gen_float(float* _dest, unsigned int _prn);

//! Generates complex GPS L2C M code for the desired SV ID, and sampled to specific sampling frequency
void gps_l2c_m_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs);

#endif /* GNSS_GPS_L2C_SIGNAL_H_ */
