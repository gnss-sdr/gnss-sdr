/*!
 * \file gnss_synchro.cc
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

#include "gnss_synchro.h"

Gnss_Synchro::Gnss_Synchro()
{
	// Acquisition
	Acq_delay_samples=0.0;
	Acq_doppler_hz=0.0;
	Acq_samplestamp_samples=0;
	Flag_valid_acquisition=false;
	//Tracking
	Prompt_I=0.0;
	Prompt_Q=0.0;
	Carrier_phase_rads=0.0;
	Code_phase_secs=0.0;
	Tracking_timestamp_secs=0.0;
	CN0_dB_hz=0.0;
	Flag_valid_tracking=false;
	//Telemetry Decoder
	Preamble_delay_ms=0.0;
	Prn_delay_ms=0.0;
	Preamble_code_phase_ms=0.0;
	Preamble_code_phase_correction_ms=0.0;
	Channel_ID=0;
    Flag_valid_word=false;
	Flag_preamble=false;
	// Pseudorange
	Pseudorange_m=0.0;
	Pseudorange_timestamp_ms=0.0;
	Flag_valid_pseudorange=false;
}

Gnss_Synchro::~Gnss_Synchro()
{
}
