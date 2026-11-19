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

#include "qzss.h"
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>
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

    enum class TrackingComponent : uint8_t
    {
        Unknown = 0,
        Data = 1,
        Pilot = 2
    };

    //! Actual code/carrier tracking component, after tracking implementation overrides.
    TrackingComponent tracking_component{TrackingComponent::Unknown};

    //! RINEX 4 / RTCM observation code; Signal remains the receiver's routing label.
    //! An empty code means that the measurement cannot be identified safely.
    const char* observation_code() const noexcept
    {
        struct SignalCode
        {
            char system;
            const char* signal;
            const char* data;
            const char* pilot;
        };
        static const SignalCode codes[] = {
            {'G', "1C", "1C", "1C"},
            {'G', "2S", "2S", "2S"},
            {'G', "L5", "5I", "5Q"},
            {'G', "1S", "1S", "1L"},
            {'E', "1B", "1B", "1C"},
            {'E', "5X", "5I", "5Q"},
            {'E', "7X", "7I", "7Q"},
            {'E', "E6", "6B", "6C"},
            {'R', "1G", "1C", "1C"},
            {'R', "2G", "2C", "2C"},
            {'C', "B1", "2I", "2I"},
            {'C', "B3", "6I", "6I"},
            {'C', "1D", "1D", "1P"},
            {'C', "5D", "5D", "5P"},
            {'J', "J1", "1C", "1C"},
            {'J', "J5", "5I", "5Q"},
            {'S', "S1", "1C", "1C"}};
        // QZSS L1 C/B shares the J1 routing label with L1 C/A. Its own PRNs
        // identify it until the PVT block replaces them by the nominal PRN
        // of the satellite, and Flag_qzss_l1cb from then on.
        if (System == 'J' && Signal[0] == 'J' && Signal[1] == '1' &&
            (Flag_qzss_l1cb || qzss_l1cb_prn_to_nominal_prn(PRN) != PRN))
            {
                return "1E";
            }
        for (const auto& code : codes)
            {
                if (System == code.system && Signal[0] == code.signal[0] && Signal[1] == code.signal[1])
                    {
                        if (code.data[1] == code.pilot[1] || tracking_component == TrackingComponent::Data)
                            {
                                return code.data;
                            }
                        return tracking_component == TrackingComponent::Pilot ? code.pilot : "";
                    }
            }
        return "";
    }

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
    bool Flag_cycle_slip{};                //!< Set by Observables processing block

    //! Set by Observables processing block. Signals that the carrier phase has
    //! jumped by half a cycle because the Costas loop phase ambiguity resolved
    //! by the Telemetry Decoder changed
    bool Flag_half_cycle_slip{};

    //! Set by Tracking processing block. False on the first output after the
    //! accumulated carrier phase has been (re)initialized, that is, whenever
    //! the carrier phase ambiguity of the channel has changed
    bool Flag_carrier_phase_continuous{true};

    //! Set by the PVT processing block when it replaces the PRN of a QZSS
    //! L1 C/B measurement (203-206) by the nominal PRN of its satellite, so
    //! that the signal can still be told apart from L1 C/A. Not serialized:
    //! Monitor clients receive the transmitted PRN
    bool Flag_qzss_l1cb{};

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
                this->tracking_component = rhs.tracking_component;
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
                this->Flag_cycle_slip = rhs.Flag_cycle_slip;
                this->Flag_half_cycle_slip = rhs.Flag_half_cycle_slip;
                this->Flag_carrier_phase_continuous = rhs.Flag_carrier_phase_continuous;
                this->Flag_qzss_l1cb = rhs.Flag_qzss_l1cb;
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
                this->tracking_component = other.tracking_component;
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
                this->Flag_cycle_slip = other.Flag_cycle_slip;
                this->Flag_half_cycle_slip = other.Flag_half_cycle_slip;
                this->Flag_carrier_phase_continuous = other.Flag_carrier_phase_continuous;
                this->Flag_qzss_l1cb = other.Flag_qzss_l1cb;

                // Leave the source object in a valid but unspecified state
                other.tracking_component = TrackingComponent::Unknown;
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
                other.Flag_cycle_slip = false;
                other.Flag_half_cycle_slip = false;
                other.Flag_carrier_phase_continuous = true;
                other.Flag_qzss_l1cb = false;
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
        ar& BOOST_SERIALIZATION_NVP(Flag_cycle_slip);
        ar& BOOST_SERIALIZATION_NVP(Flag_half_cycle_slip);
        ar& BOOST_SERIALIZATION_NVP(Flag_carrier_phase_continuous);
        if (version > 0)
            {
                ar& BOOST_SERIALIZATION_NVP(tracking_component);
            }
        else if (Archive::is_loading::value)
            {
                tracking_component = TrackingComponent::Unknown;
            }
    }
};


BOOST_CLASS_VERSION(Gnss_Synchro, 1)

/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SYNCHRO_H
