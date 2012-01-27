/*!
 * \file gnss_synchro.h
 * \brief  Implementation of the Gnss_Synchro class
 * \author
 *  Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *  Javier Arribas, 2012. jarribas(at)cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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
#ifndef GNSS_SDR_GNSS_SYNCHRO_H_
#define GNSS_SDR_GNSS_SYNCHRO_H_

#include "gnss_signal.h"

#include <string>

class Gnss_Synchro{
private:

public:
	Gnss_Synchro();
	~Gnss_Synchro();

	//Gnss_Signal Signal;
	// Satellite and signal info
	char System;
	char Signal[3];
	unsigned int PRN;
	// Acquisition
	double Acq_delay_samples;
	double Acq_doppler_hz;
	unsigned long int Acq_samplestamp_samples;
	bool Flag_valid_acquisition;
	//Tracking
	double Prompt_I;
	double Prompt_Q;
	double Carrier_phase_rads;
	double Code_phase_secs;
	double Tracking_timestamp_secs;
	double CN0_dB_hz;
	bool Flag_valid_tracking;
	//Telemetry Decoder
	double Preamble_delay_ms;
	double Prn_delay_ms;
	double Preamble_code_phase_ms;
	double Preamble_code_phase_correction_ms;
	int Channel_ID;
	bool Flag_valid_word;
	bool Flag_preamble;
	// Pseudorange
	double Pseudorange_m;
	double Pseudorange_timestamp_ms;
	bool Flag_valid_pseudorange;

};
#endif

