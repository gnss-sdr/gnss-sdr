/*!
 * \file gnss_synchro.h
 * \brief  Interface of the Gnss_Synchro class
 * \author
 *  Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *  Javier Arribas, 2012. jarribas(at)cttc.es
 *  Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GNSS_SYNCHRO_H
#define GNSS_SDR_GNSS_SYNCHRO_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>
#include <utility>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters core_system_parameters
 * GNSS parameters
 * \{ */


/*!
 * \brief This is the class that contains the information that is shared
 * by the processing blocks.
 */
class Gnss_Synchro
{
public:
    Gnss_Synchro() = default;  //!< Default constructor

    ~Gnss_Synchro() = default;  //!< Default destructor

    // Satellite and signal info
    char System{};         //!< Set by Channel::set_signal(Gnss_Signal gnss_signal)
    char Signal[3]{};      //!< Set by Channel::set_signal(Gnss_Signal gnss_signal)
    uint32_t PRN{};        //!< Set by Channel::set_signal(Gnss_Signal gnss_signal)
    int32_t Channel_ID{};  //!< Set by Channel constructor

    // Acquisition
    double Acq_delay_samples{};          //!< Set by Acquisition processing block
    double Acq_doppler_hz{};             //!< Set by Acquisition processing block
    uint64_t Acq_samplestamp_samples{};  //!< Set by Acquisition processing block
    uint32_t Acq_doppler_step{};         //!< Set by Acquisition processing block

    // Tracking
    int64_t fs{};                        //!< Set by Tracking processing block
    double Prompt_I{};                   //!< Set by Tracking processing block
    double Prompt_Q{};                   //!< Set by Tracking processing block
    double CN0_dB_hz{};                  //!< Set by Tracking processing block
    double Carrier_Doppler_hz{};         //!< Set by Tracking processing block
    double Carrier_phase_rads{};         //!< Set by Tracking processing block
    double Code_phase_samples{};         //!< Set by Tracking processing block
    uint64_t Tracking_sample_counter{};  //!< Set by Tracking processing block
    int32_t correlation_length_ms{};     //!< Set by Tracking processing block

    // Telemetry Decoder
    uint32_t TOW_at_current_symbol_ms{};  //!< Set by Telemetry Decoder processing block

    // Observables
    double Pseudorange_m{};  //!< Set by Observables processing block
    double RX_time{};        //!< Set by Observables processing block
    double interp_TOW_ms{};  //!< Set by Observables processing block

    // Flags
    bool Flag_valid_acquisition{};         //!< Set by Acquisition processing block
    bool Flag_valid_symbol_output{};       //!< Set by Tracking processing block
    bool Flag_valid_word{};                //!< Set by Telemetry Decoder processing block
    bool Flag_valid_pseudorange{};         //!< Set by Observables processing block
    bool Flag_PLL_180_deg_phase_locked{};  //!< Set by Telemetry Decoder processing block

    /// Copy constructor
    Gnss_Synchro(const Gnss_Synchro& other) noexcept = default;

    /// Copy assignment operator
    Gnss_Synchro& operator=(const Gnss_Synchro& rhs) noexcept
    {
        // Only do assignment if RHS is a different object from this.
        if (this != &rhs)
            {
                this->System = rhs.System;
                this->Signal[0] = rhs.Signal[0];
                this->Signal[1] = rhs.Signal[1];
                this->Signal[2] = rhs.Signal[2];
                this->PRN = rhs.PRN;
                this->Channel_ID = rhs.Channel_ID;
                this->Acq_delay_samples = rhs.Acq_delay_samples;
                this->Acq_doppler_hz = rhs.Acq_doppler_hz;
                this->Acq_samplestamp_samples = rhs.Acq_samplestamp_samples;
                this->Acq_doppler_step = rhs.Acq_doppler_step;
                this->fs = rhs.fs;
                this->Prompt_I = rhs.Prompt_I;
                this->Prompt_Q = rhs.Prompt_Q;
                this->CN0_dB_hz = rhs.CN0_dB_hz;
                this->Carrier_Doppler_hz = rhs.Carrier_Doppler_hz;
                this->Carrier_phase_rads = rhs.Carrier_phase_rads;
                this->Code_phase_samples = rhs.Code_phase_samples;
                this->Tracking_sample_counter = rhs.Tracking_sample_counter;
                this->correlation_length_ms = rhs.correlation_length_ms;
                this->TOW_at_current_symbol_ms = rhs.TOW_at_current_symbol_ms;
                this->Pseudorange_m = rhs.Pseudorange_m;
                this->RX_time = rhs.RX_time;
                this->interp_TOW_ms = rhs.interp_TOW_ms;
                this->Flag_valid_acquisition = rhs.Flag_valid_acquisition;
                this->Flag_valid_symbol_output = rhs.Flag_valid_symbol_output;
                this->Flag_valid_word = rhs.Flag_valid_word;
                this->Flag_valid_pseudorange = rhs.Flag_valid_pseudorange;
                this->Flag_PLL_180_deg_phase_locked = rhs.Flag_PLL_180_deg_phase_locked;
            }
        return *this;
    };

    /// Move constructor
    Gnss_Synchro(Gnss_Synchro&& other) noexcept = default;

    /// Move assignment operator
    Gnss_Synchro& operator=(Gnss_Synchro&& other) noexcept
    {
        if (this != &other)
            {
                this->System = other.System;
                this->Signal[0] = other.Signal[0];
                this->Signal[1] = other.Signal[1];
                this->Signal[2] = other.Signal[2];
                this->PRN = other.PRN;
                this->Channel_ID = other.Channel_ID;
                this->Acq_delay_samples = other.Acq_delay_samples;
                this->Acq_doppler_hz = other.Acq_doppler_hz;
                this->Acq_samplestamp_samples = other.Acq_samplestamp_samples;
                this->Acq_doppler_step = other.Acq_doppler_step;
                this->fs = other.fs;
                this->Prompt_I = other.Prompt_I;
                this->Prompt_Q = other.Prompt_Q;
                this->CN0_dB_hz = other.CN0_dB_hz;
                this->Carrier_Doppler_hz = other.Carrier_Doppler_hz;
                this->Carrier_phase_rads = other.Carrier_phase_rads;
                this->Code_phase_samples = other.Code_phase_samples;
                this->Tracking_sample_counter = other.Tracking_sample_counter;
                this->correlation_length_ms = other.correlation_length_ms;
                this->TOW_at_current_symbol_ms = other.TOW_at_current_symbol_ms;
                this->Pseudorange_m = other.Pseudorange_m;
                this->RX_time = other.RX_time;
                this->interp_TOW_ms = other.interp_TOW_ms;
                this->Flag_valid_acquisition = other.Flag_valid_acquisition;
                this->Flag_valid_symbol_output = other.Flag_valid_symbol_output;
                this->Flag_valid_word = other.Flag_valid_word;
                this->Flag_valid_pseudorange = other.Flag_valid_pseudorange;
                this->Flag_PLL_180_deg_phase_locked = other.Flag_PLL_180_deg_phase_locked;

                // Leave the source object in a valid but unspecified state
                other.Signal[0] = '\0';
                other.Signal[1] = '\0';
                other.Signal[2] = '\0';
                other.System = 0;
                other.PRN = 0;
                other.Channel_ID = 0;
                other.Acq_delay_samples = 0.0;
                other.Acq_doppler_hz = 0.0;
                other.Acq_samplestamp_samples = 0;
                other.Acq_doppler_step = 0;
                other.fs = 0;
                other.Prompt_I = 0.0;
                other.Prompt_Q = 0.0;
                other.CN0_dB_hz = 0.0;
                other.Carrier_Doppler_hz = 0.0;
                other.Carrier_phase_rads = 0.0;
                other.Code_phase_samples = 0.0;
                other.Tracking_sample_counter = 0;
                other.correlation_length_ms = 0;
                other.TOW_at_current_symbol_ms = 0;
                other.Pseudorange_m = 0.0;
                other.RX_time = 0.0;
                other.interp_TOW_ms = 0.0;
                other.Flag_valid_acquisition = false;
                other.Flag_valid_symbol_output = false;
                other.Flag_valid_word = false;
                other.Flag_valid_pseudorange = false;
                other.Flag_PLL_180_deg_phase_locked = false;
            }
        return *this;
    };

    /*!
     * \brief This member function serializes and restores
     * Gnss_Synchro objects from a byte stream.
     */
    template <class Archive>

    void serialize(Archive& ar, const unsigned int version)
    {
        if (version)
            {
            };
        // Satellite and signal info
        ar& BOOST_SERIALIZATION_NVP(System);
        ar& BOOST_SERIALIZATION_NVP(Signal);
        ar& BOOST_SERIALIZATION_NVP(PRN);
        ar& BOOST_SERIALIZATION_NVP(Channel_ID);
        // Acquisition
        ar& BOOST_SERIALIZATION_NVP(Acq_delay_samples);
        ar& BOOST_SERIALIZATION_NVP(Acq_doppler_hz);
        ar& BOOST_SERIALIZATION_NVP(Acq_samplestamp_samples);
        ar& BOOST_SERIALIZATION_NVP(Acq_doppler_step);
        // Tracking
        ar& BOOST_SERIALIZATION_NVP(fs);
        ar& BOOST_SERIALIZATION_NVP(Prompt_I);
        ar& BOOST_SERIALIZATION_NVP(Prompt_Q);
        ar& BOOST_SERIALIZATION_NVP(CN0_dB_hz);
        ar& BOOST_SERIALIZATION_NVP(Carrier_Doppler_hz);
        ar& BOOST_SERIALIZATION_NVP(Carrier_phase_rads);
        ar& BOOST_SERIALIZATION_NVP(Code_phase_samples);
        ar& BOOST_SERIALIZATION_NVP(Tracking_sample_counter);
        ar& BOOST_SERIALIZATION_NVP(correlation_length_ms);
        // Telemetry Decoder
        ar& BOOST_SERIALIZATION_NVP(TOW_at_current_symbol_ms);
        // Observables
        ar& BOOST_SERIALIZATION_NVP(Pseudorange_m);
        ar& BOOST_SERIALIZATION_NVP(RX_time);
        ar& BOOST_SERIALIZATION_NVP(interp_TOW_ms);
        // Flags
        ar& BOOST_SERIALIZATION_NVP(Flag_valid_acquisition);
        ar& BOOST_SERIALIZATION_NVP(Flag_valid_symbol_output);
        ar& BOOST_SERIALIZATION_NVP(Flag_valid_word);
        ar& BOOST_SERIALIZATION_NVP(Flag_valid_pseudorange);
        ar& BOOST_SERIALIZATION_NVP(Flag_PLL_180_deg_phase_locked);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SYNCHRO_H
