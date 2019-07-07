/*!
 * \file serdes_gnss_synchro.h
 * \brief Serialization / Deserialization of Gnss_Synchro objects using
 * Protocol Buffers
 * \author Carles Fernandez-Prades, 2019. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_SERDES_GNSS_SYNCHRO_H_
#define GNSS_SDR_SERDES_GNSS_SYNCHRO_H_

#include "gnss_synchro.h"
#include "gnss_synchro.pb.h"  // file created by Protocol Buffers at compile time
#include <array>
#include <vector>


/*!
 * \brief This class implements serialization and deserialization of
 * Gnss_Synchro objects using Protocol Buffers.
 */
class Serdes_Gnss_Synchro
{
public:
    Serdes_Gnss_Synchro()
    {
        // Verify that the version of the library that we linked against is
        // compatible with the version of the headers we compiled against.
        GOOGLE_PROTOBUF_VERIFY_VERSION;
    }

    ~Serdes_Gnss_Synchro()
    {
        google::protobuf::ShutdownProtobufLibrary();
    }

    inline Serdes_Gnss_Synchro(Serdes_Gnss_Synchro&& other)  //!< Copy constructor
    {
        this->observables = other.observables;
    }

    inline Serdes_Gnss_Synchro& operator=(const Serdes_Gnss_Synchro& rhs)  //!< Copy assignment operator
    {
        this->observables = rhs.observables;
        return *this;
    }

    inline Serdes_Gnss_Synchro(const Serdes_Gnss_Synchro& other)  //!< Move constructor
    {
        this->observables = std::move(other.observables);
    }

    inline Serdes_Gnss_Synchro& operator=(Serdes_Gnss_Synchro&& other)  //!< Move assignment operator
    {
        if (this != &other)
            {
                this->observables = std::move(other.observables);
            }
        return *this;
    }

    inline std::string createProtobuffer(const std::vector<Gnss_Synchro>& vgs)  //!< Serialization into a string
    {
        observables.Clear();
        std::string data;
        for (auto gs : vgs)
            {
                gnss_sdr::GnssSynchro* obs = observables.add_observable();
                char c = gs.System;
                const std::string sys(1, c);

                std::array<char, 2> cc;
                cc[0] = gs.Signal[0];
                cc[1] = gs.Signal[1];
                const std::string sig(cc.cbegin(), cc.cend());

                obs->set_system(sys);
                obs->set_signal(sig);
                obs->set_prn(gs.PRN);
                obs->set_channel_id(gs.Channel_ID);

                obs->set_acq_delay_samples(gs.Acq_delay_samples);
                obs->set_acq_doppler_hz(gs.Acq_doppler_hz);
                obs->set_acq_samplestamp_samples(gs.Acq_samplestamp_samples);
                obs->set_acq_doppler_step(gs.Acq_doppler_step);
                obs->set_flag_valid_acquisition(gs.Flag_valid_acquisition);

                obs->set_fs(gs.fs);
                obs->set_prompt_i(gs.Prompt_I);
                obs->set_prompt_q(gs.Prompt_Q);
                obs->set_cn0_db_hz(gs.CN0_dB_hz);
                obs->set_carrier_doppler_hz(gs.Carrier_Doppler_hz);
                obs->set_code_phase_samples(gs.Code_phase_samples);
                obs->set_tracking_sample_counter(gs.Tracking_sample_counter);
                obs->set_flag_valid_symbol_output(gs.Flag_valid_symbol_output);
                obs->set_correlation_length_ms(gs.correlation_length_ms);

                obs->set_flag_valid_word(gs.Flag_valid_word);
                obs->set_tow_at_current_symbol_ms(gs.TOW_at_current_symbol_ms);

                obs->set_pseudorange_m(gs.Pseudorange_m);
                obs->set_rx_time(gs.RX_time);
                obs->set_flag_valid_pseudorange(gs.Flag_valid_pseudorange);
                obs->set_interp_tow_ms(gs.interp_TOW_ms);
            }
        observables.SerializeToString(&data);
        return data;
    }

    inline std::vector<Gnss_Synchro> readProtobuffer(const gnss_sdr::Observables& obs) const  //!< Deserialization
    {
        std::vector<Gnss_Synchro> vgs;
        vgs.reserve(obs.observable_size());
        for (int i = 0; i < obs.observable_size(); ++i)
            {
                const gnss_sdr::GnssSynchro& gs_read = obs.observable(i);
                Gnss_Synchro gs = Gnss_Synchro();
                gs.System = gs_read.system()[0];
                gs.Signal[0] = gs_read.signal()[0];
                gs.Signal[1] = gs_read.signal()[1];
                gs.Signal[2] = '\0';
                gs.PRN = gs_read.prn();
                gs.Channel_ID = gs_read.channel_id();

                gs.Acq_delay_samples = gs_read.acq_delay_samples();
                gs.Acq_doppler_hz = gs_read.acq_doppler_hz();
                gs.Acq_samplestamp_samples = gs_read.acq_samplestamp_samples();
                gs.Acq_doppler_step = gs_read.acq_doppler_step();
                gs.Flag_valid_acquisition = gs_read.flag_valid_acquisition();

                gs.fs = gs_read.fs();
                gs.Prompt_I = gs_read.prompt_i();
                gs.Prompt_Q = gs_read.prompt_q();
                gs.CN0_dB_hz = gs_read.cn0_db_hz();
                gs.Carrier_Doppler_hz = gs_read.carrier_doppler_hz();
                gs.Tracking_sample_counter = gs_read.tracking_sample_counter();
                gs.Flag_valid_symbol_output = gs_read.flag_valid_symbol_output();
                gs.correlation_length_ms = gs_read.correlation_length_ms();

                gs.Flag_valid_word = gs_read.flag_valid_word();
                gs.TOW_at_current_symbol_ms = gs_read.tow_at_current_symbol_ms();

                gs.Pseudorange_m = gs_read.pseudorange_m();
                gs.RX_time = gs_read.rx_time();
                gs.Flag_valid_pseudorange = gs_read.flag_valid_pseudorange();
                gs.interp_TOW_ms = gs_read.interp_tow_ms();

                vgs.push_back(gs);
            }
        return vgs;
    }

private:
    gnss_sdr::Observables observables{};
};

#endif  // GNSS_SDR_SERDES_GNSS_SYNCHRO_H_
