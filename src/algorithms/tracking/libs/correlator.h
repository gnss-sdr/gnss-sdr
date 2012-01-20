/*!
 * \file correlator.h
 * \brief High optimized vector correlator class
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Class that implements a high optimized vector correlator class.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef CORRELATOR_H_
#define CORRELATOR_H_
/*!
 * \brief High optimized vector correlator class
 *
 */

#include <volk/volk.h>
#include <gnuradio/gr_block.h>
class correlator
{
private:
	std::string volk_32fc_x2_multiply_32fc_a_best_arch;
	std::string volk_32fc_x2_dot_prod_32fc_a_best_arch;

	unsigned long next_power_2(unsigned long v);
	void cpu_arch_test_volk_32fc_x2_dot_prod_32fc_a();
	void cpu_arch_test_volk_32fc_x2_multiply_32fc_a();

	public:
	void Carrier_wipeoff_and_EPL_generic(int signal_length_samples,const gr_complex* input, gr_complex* carrier,gr_complex* E_code, gr_complex* P_code, gr_complex* L_code,gr_complex* E_out, gr_complex* P_out, gr_complex* L_out);
	void Carrier_wipeoff_and_EPL_volk(int signal_length_samples,const gr_complex* input, gr_complex* carrier,gr_complex* E_code, gr_complex* P_code, gr_complex* L_code,gr_complex* E_out, gr_complex* P_out, gr_complex* L_out);
	correlator();
	~correlator();
};
#endif
