/*!
 * \file gnss_synchro.h
 * \brief  Interface of the Gnss_Synchro class
 * \author
 *  Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *  Javier Arribas, 2012. jarribas(at)cttc.es
 *  Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
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
#ifndef GNSS_SDR_GNSS_SYNCHRO_H_
#define GNSS_SDR_GNSS_SYNCHRO_H_

#include "gnss_signal.h"


/*!
 * \brief This is the class that contains the information that is shared
 * by the processing blocks.
 */
class Gnss_Synchro
{
public:
    // Satellite and signal info
    char System;       //!< Set by Channel::set_signal(Gnss_Signal gnss_signal)
    char Signal[3];    //!< Set by Channel::set_signal(Gnss_Signal gnss_signal)
    unsigned int PRN;  //!< Set by Channel::set_signal(Gnss_Signal gnss_signal)
    int Channel_ID;    //!< Set by Channel constructor
    // Acquisition
    double Acq_delay_samples;                   //!< Set by Acquisition processing block
    double Acq_doppler_hz;                      //!< Set by Acquisition processing block
    unsigned long int Acq_samplestamp_samples;  //!< Set by Acquisition processing block
    bool Flag_valid_acquisition;                //!< Set by Acquisition processing block
    //Tracking
    long int fs;                                //!< Set by Tracking processing block
    double Prompt_I;                            //!< Set by Tracking processing block
    double Prompt_Q;                            //!< Set by Tracking processing block
    double CN0_dB_hz;                           //!< Set by Tracking processing block
    double Carrier_Doppler_hz;                  //!< Set by Tracking processing block
    double Carrier_phase_rads;                  //!< Set by Tracking processing block
    double Code_phase_samples;                  //!< Set by Tracking processing block
    unsigned long int Tracking_sample_counter;  //!< Set by Tracking processing block

    bool Flag_valid_symbol_output;  //!< Set by Tracking processing block
    int correlation_length_ms;      //!< Set by Tracking processing block

    //Telemetry Decoder
    bool Flag_valid_word;                   //!< Set by Telemetry Decoder processing block
    unsigned int TOW_at_current_symbol_ms;  //!< Set by Telemetry Decoder processing block

    // Observables
    double Pseudorange_m;         //!< Set by Observables processing block
    double RX_time;               //!< Set by Observables processing block
    bool Flag_valid_pseudorange;  //!< Set by Observables processing block
    double interp_TOW_ms;         //!< Set by Observables processing block


    /*!
     * \brief This member function is necessary to serialize and restore
     * Gnss_Synchro objects from a byte stream.
     */
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        if (version)
            {
            };
        // Satellite and signal info
        ar& System;
        ar& Signal;
        ar& PRN;
        ar& Channel_ID;
        ar& Acq_delay_samples;
        ar& Acq_doppler_hz;
        ar& Acq_samplestamp_samples;
        ar& Flag_valid_acquisition;
        //Tracking
        ar& fs;
        ar& Prompt_I;
        ar& Prompt_Q;
        ar& CN0_dB_hz;
        ar& Carrier_Doppler_hz;
        ar& Carrier_phase_rads;
        ar& Code_phase_samples;
        ar& Tracking_sample_counter;
        ar& Flag_valid_symbol_output;
        ar& correlation_length_ms;
        //Telemetry Decoder
        ar& Flag_valid_word;
        ar& TOW_at_current_symbol_ms;
        // Observables
        ar& Pseudorange_m;
        ar& RX_time;
        ar& Flag_valid_pseudorange;
        ar& interp_TOW_ms;
    }
};

#endif
