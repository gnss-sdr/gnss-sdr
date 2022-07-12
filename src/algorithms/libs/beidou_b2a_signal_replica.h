/*!
 * \file beidou_b2a_signal_replica.h
 * \brief This class implements signal generators for the BeiDou B2a signals
 * \author Sara Hrbek, 2018. sara.hrbek(at)gmail.com .
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
 * \note Initial code added as part of GSoC 2018 program
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_BEIDOU_B2A_SIGNAL_REPLICA_H_
#define GNSS_SDR_BEIDOU_B2A_SIGNAL_REPLICA_H_

#include <complex>
#include <cstdint>

#if HAS_SPAN
#include <span>
namespace gsl = std;
#else
#include <gsl/gsl>
#endif


/*!
 * Generates a string version of the BeiDou B2a pilot secondary code
 */
void beidou_b2ap_secondary_gen_string(std::string& _dest, uint32_t _prn);

/*!
 * \brief Generates complex BeiDou B2a data primary code for the desired SV ID
 */
void beidou_b2ad_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn);

/*!
 * \brief Generates complex BeiDou B2a data primary code with secondary code for the desired SV ID
 */
void beidou_b2ad_code_gen_complex_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn);

/*!
 * \brief Generates complex BeiDou B2a data primary code for the desired SV ID
 */
void beidou_b2ad_code_gen_float(gsl::span<float> _dest, uint32_t _prn);

/*!
 * \brief Generates complex BeiDou B2a data primary code with secondary code for the desired SV ID
 */
void beidou_b2ap_code_gen_complex_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn);

/*!
 * \brief Generates complex BeiDou B2a pilot primary code for the desired SV ID
 */
void beidou_b2ap_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn);

/*!
 * \brief Generates complex BeiDou B2a pilot primary code for the desired SV ID
 */
void beidou_b2ap_code_gen_float(gsl::span<float> _dest, uint32_t _prn);

/*!
 * \brief Generates complex BeiDou B2a pilot secondary code for the desired SV ID
 */
void beidou_b2ap_code_gen_float_secondary(gsl::span<float> _dest, uint32_t _prn);

/*!
 * \Generates complex BeiDou B2a data primary code for the desired SV ID, and sampled to specific sampling frequency
 */
void beidou_b2ad_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs);

/*!
 * \Generates complex BeiDou B2a data primary code  with secondary code for the desired SV ID, and sampled to specific sampling frequency
 */
void beidou_b2ad_code_gen_complex_sampled_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs);

/*!
 * \ Generates complex BeiDou B2 pilot primary code for the desired SV ID, and sampled to specific sampling frequency
 */
void beidou_b2ap_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs);

/*!
 * \Generates complex BeiDou B2a pilot primary code  with secondary code for the desired SV ID, and sampled to specific sampling frequency
 */
void beidou_b2ap_code_gen_complex_sampled_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs);

/*!
 * \Generates complex BeiDou B2a data+pilot primary code  for the desired SV ID, and sampled to specific sampling frequency
 */
void beidou_b2a_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs);

#endif /* GNSS_SDR_BEIDOU_B2A_SIGNAL_REPLICA_H_ */