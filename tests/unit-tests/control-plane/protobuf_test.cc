/*!
 * \file protobuf_test.cc
 * \brief This file implements tests for Serdes_Gnss_Synchro
 * \author Carles Fernandez-Prades, 2019. cfernandez(at)cttc.es
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


#include "serdes_gnss_synchro.h"
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <sstream>

TEST(Protobuf, Works)
{
    uint32_t prn_true = 17;
    uint32_t prn_true2 = 23;
    std::string sys = "G";
    std::string sig = "1C";

    Gnss_Synchro gs = Gnss_Synchro();
    gs.System = *sys.c_str();
    std::memcpy(static_cast<void*>(gs.Signal), sig.c_str(), 3);
    gs.PRN = prn_true;

    gs.Channel_ID = 3;

    gs.Acq_delay_samples = 431;
    gs.Acq_doppler_hz = 1234;
    gs.Acq_samplestamp_samples = 10000;
    gs.Acq_doppler_step = 125;
    gs.Flag_valid_acquisition = true;

    gs.fs = 10000000;
    gs.Prompt_I = 10000.0;
    gs.Prompt_Q = 0.01;
    gs.CN0_dB_hz = 39;
    gs.Carrier_Doppler_hz = 321;
    gs.Tracking_sample_counter = 11000;
    gs.Flag_valid_symbol_output = false;
    gs.correlation_length_ms = 1;

    gs.Flag_valid_word = false;
    gs.TOW_at_current_symbol_ms = 12345;

    gs.Pseudorange_m = 22000002.1;
    gs.RX_time = 4321;
    gs.Flag_valid_pseudorange = false;
    gs.interp_TOW_ms = 20;

    gs.Pseudorange_m = 22000002.1;
    gs.Carrier_phase_rads = 45.4;
    gs.Carrier_Doppler_hz = 321;
    gs.CN0_dB_hz = 39;

    Serdes_Gnss_Synchro serdes = Serdes_Gnss_Synchro();

    // Create a vector of Gnss_Synchro objects
    std::vector<Gnss_Synchro> vgs;
    vgs.push_back(gs);
    gs.PRN = prn_true2;
    vgs.push_back(gs);

    // Serialize data
    std::string serialized_data = serdes.createProtobuffer(vgs);

    // Recover data from serialization
    gnss_sdr::Observables obs;
    ASSERT_TRUE(obs.ParseFromString(serialized_data));

    // Check that recovered data is ok

    // We can access like this:
    std::vector<Gnss_Synchro> vgs_read = serdes.readProtobuffer(obs);
    Gnss_Synchro gs_read{};
    gs_read = vgs_read[0];
    uint32_t prn_read = gs_read.PRN;
    uint32_t prn_read2 = vgs_read[1].PRN;
    std::string system_read(1, gs_read.System);
    std::string signal_read(gs_read.Signal);

    // or without the need of gnss_synchro:
    int obs_size = obs.observable_size();
    uint32_t prn_read3 = obs.observable(0).prn();

    EXPECT_EQ(sig, signal_read);
    EXPECT_EQ(sys, system_read);
    EXPECT_EQ(prn_true2, prn_read2);
    EXPECT_EQ(prn_true, prn_read3);
    EXPECT_EQ(prn_read, prn_read3);
    EXPECT_EQ(2, obs_size);
}


TEST(Protobuf, CarrierPhaseContinuitySurvivesSerialization)
{
    Serdes_Gnss_Synchro serdes = Serdes_Gnss_Synchro();

    Gnss_Synchro gs = Gnss_Synchro();
    EXPECT_TRUE(gs.Flag_carrier_phase_continuous) << "a fresh Gnss_Synchro must not report a discontinuity";

    std::vector<Gnss_Synchro> vgs;
    vgs.push_back(gs);
    gs.Flag_carrier_phase_continuous = false;
    vgs.push_back(gs);

    gnss_sdr::Observables obs;
    ASSERT_TRUE(obs.ParseFromString(serdes.createProtobuffer(vgs)));
    const std::vector<Gnss_Synchro> vgs_read = serdes.readProtobuffer(obs);

    ASSERT_EQ(2U, vgs_read.size());
    EXPECT_TRUE(vgs_read[0].Flag_carrier_phase_continuous);
    EXPECT_FALSE(vgs_read[1].Flag_carrier_phase_continuous);

    // the flag travels negated, so that a message written by a version that does
    // not know the field is read back as "no discontinuity"
    EXPECT_FALSE(obs.observable(0).flag_carrier_phase_discontinuity());
    EXPECT_TRUE(obs.observable(1).flag_carrier_phase_discontinuity());

    gnss_sdr::Observables without_field;
    without_field.add_observable();
    EXPECT_TRUE(serdes.readProtobuffer(without_field)[0].Flag_carrier_phase_continuous);
    EXPECT_FALSE(serdes.readProtobuffer(without_field)[0].Flag_half_cycle_slip);
}


TEST(Protobuf, HalfCycleSlipSurvivesSerialization)
{
    Serdes_Gnss_Synchro serdes = Serdes_Gnss_Synchro();

    Gnss_Synchro gs = Gnss_Synchro();
    EXPECT_FALSE(gs.Flag_half_cycle_slip);

    std::vector<Gnss_Synchro> vgs;
    vgs.push_back(gs);
    gs.Flag_half_cycle_slip = true;
    vgs.push_back(gs);

    gnss_sdr::Observables obs;
    ASSERT_TRUE(obs.ParseFromString(serdes.createProtobuffer(vgs)));
    const std::vector<Gnss_Synchro> vgs_read = serdes.readProtobuffer(obs);

    ASSERT_EQ(2U, vgs_read.size());
    EXPECT_FALSE(vgs_read[0].Flag_half_cycle_slip);
    EXPECT_TRUE(vgs_read[1].Flag_half_cycle_slip);
}


TEST(Protobuf, CarrierPhaseContinuitySurvivesCopyAndMove)
{
    Gnss_Synchro gs = Gnss_Synchro();
    gs.Flag_carrier_phase_continuous = false;

    Gnss_Synchro copy_constructed(gs);
    EXPECT_FALSE(copy_constructed.Flag_carrier_phase_continuous);

    Gnss_Synchro copy_assigned = Gnss_Synchro();
    copy_assigned = gs;
    EXPECT_FALSE(copy_assigned.Flag_carrier_phase_continuous);

    Gnss_Synchro move_assigned = Gnss_Synchro();
    move_assigned = std::move(copy_constructed);
    EXPECT_FALSE(move_assigned.Flag_carrier_phase_continuous);
}


TEST(Protobuf, TrackingComponentSurvivesSerializationAndCopy)
{
    Gnss_Synchro pilot;
    pilot.System = 'E';
    std::memcpy(pilot.Signal, "1B", 3);
    pilot.tracking_component = Gnss_Synchro::TrackingComponent::Pilot;
    Gnss_Synchro data = pilot;
    data.tracking_component = Gnss_Synchro::TrackingComponent::Data;
    Gnss_Synchro copied;
    copied = pilot;
    Gnss_Synchro moved;
    moved = std::move(copied);
    EXPECT_STREQ("1C", moved.observation_code());
    Serdes_Gnss_Synchro serdes;
    gnss_sdr::Observables message;
    ASSERT_TRUE(message.ParseFromString(serdes.createProtobuffer({data, moved})));
    const auto restored = serdes.readProtobuffer(message);
    ASSERT_EQ(2U, restored.size());
    EXPECT_STREQ("1B", restored[0].observation_code());
    EXPECT_STREQ("1C", restored[1].observation_code());
    message.mutable_observable(1)->clear_tracking_component();
    EXPECT_STREQ("", serdes.readProtobuffer(message)[1].observation_code());
}


TEST(Protobuf, TrackingComponentBoostArchivesPreserveUnknownLegacyState)
{
    Gnss_Synchro pilot;
    pilot.System = 'G';
    std::memcpy(pilot.Signal, "L5", 3);
    pilot.tracking_component = Gnss_Synchro::TrackingComponent::Pilot;
    std::stringstream stream;
    {
        boost::archive::text_oarchive output(stream);
        output << pilot;
    }
    Gnss_Synchro restored;
    {
        boost::archive::text_iarchive input(stream);
        input >> restored;
    }
    EXPECT_STREQ("5Q", restored.observation_code());

    std::stringstream legacy;
    {
        boost::archive::text_oarchive output(legacy);
        pilot.serialize(output, 0);
    }
    {
        boost::archive::text_iarchive input(legacy);
        restored.serialize(input, 0);
    }
    EXPECT_EQ(Gnss_Synchro::TrackingComponent::Unknown, restored.tracking_component);
    EXPECT_STREQ("", restored.observation_code());
}
