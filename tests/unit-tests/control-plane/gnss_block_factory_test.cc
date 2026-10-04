/*!
 * \file gnss_block_factory_test.cc
 * \brief This class implements a Unit Test for the GNSSBlockFactory class.
 * \authors <ul>
 *          <li> Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
 *
 * This class test the instantiation of all blocks in gnss_block_factory
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

#include "acquisition_interface.h"
#include "concurrent_queue.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_make_unique.h"
#include "in_memory_configuration.h"
#include "signal_source_interface.h"
#include "tracking_interface.h"
#include <gtest/gtest.h>
#include <pmt/pmt.h>
#include <utility>
#include <vector>

TEST(GNSSBlockFactoryTest, InstantiateFileSignalSource)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalSource.implementation", "File_Signal_Source");
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat";
    configuration->set_property("SignalSource.filename", std::move(filename));
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    // Example of a block as a shared_ptr
    auto signal_source = block_factory::GetSignalSource(configuration.get(), queue.get());
    EXPECT_STREQ("SignalSource", signal_source->role().c_str());
    EXPECT_STREQ("File_Signal_Source", signal_source->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongSignalSource)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalSource.implementation", "Parapsychological_Source");
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    // Example of a block as a unique_ptr
    auto signal_source = block_factory::GetSignalSource(configuration.get(), queue.get());
    EXPECT_EQ(nullptr, signal_source);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongSignalSource2)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalSource.implementation", "Pass_Through");
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    // Example of a block as a unique_ptr
    auto signal_source = block_factory::GetSignalSource(configuration.get(), queue.get());
    EXPECT_EQ(nullptr, signal_source);
}


TEST(GNSSBlockFactoryTest, InstantiateSignalConditioner)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalConditioner.implementation", "Signal_Conditioner");
    auto signal_conditioner = block_factory::GetSignalConditioner(configuration.get());
    EXPECT_STREQ("SignalConditioner", signal_conditioner->role().c_str());
    EXPECT_STREQ("Signal_Conditioner", signal_conditioner->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongSignalConditioner)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalConditioner.implementation", "Signal_Ruinder");
    auto signal_conditioner = block_factory::GetSignalConditioner(configuration.get());
    EXPECT_EQ(nullptr, signal_conditioner);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongSignalConditioner2)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("SignalConditioner.implementation", "Fir_Filter");
    auto signal_conditioner = block_factory::GetSignalConditioner(configuration.get());
    EXPECT_EQ(nullptr, signal_conditioner);
}


TEST(GNSSBlockFactoryTest, InstantiateFIRFilter)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();

    configuration->set_property("InputFilter.implementation", "Fir_Filter");

    configuration->set_property("InputFilter.number_of_taps", "4");
    configuration->set_property("InputFilter.number_of_bands", "2");

    configuration->set_property("InputFilter.band1_begin", "0.0");
    configuration->set_property("InputFilter.band1_end", "0.45");
    configuration->set_property("InputFilter.band2_begin", "0.55");
    configuration->set_property("InputFilter.band2_end", "1.0");

    configuration->set_property("InputFilter.ampl1_begin", "1.0");
    configuration->set_property("InputFilter.ampl1_end", "1.0");
    configuration->set_property("InputFilter.ampl2_begin", "0.0");
    configuration->set_property("InputFilter.ampl2_end", "0.0");

    configuration->set_property("InputFilter.band1_error", "1.0");
    configuration->set_property("InputFilter.band2_error", "1.0");

    configuration->set_property("InputFilter.filter_type", "bandpass");
    configuration->set_property("InputFilter.grid_density", "16");

    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);

    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Fir_Filter", input_filter->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateFreqXlatingFIRFilter)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();

    configuration->set_property("InputFilter.implementation", "Freq_Xlating_Fir_Filter");

    configuration->set_property("InputFilter.number_of_taps", "4");
    configuration->set_property("InputFilter.number_of_bands", "2");

    configuration->set_property("InputFilter.band1_begin", "0.0");
    configuration->set_property("InputFilter.band1_end", "0.45");
    configuration->set_property("InputFilter.band2_begin", "0.55");
    configuration->set_property("InputFilter.band2_end", "1.0");

    configuration->set_property("InputFilter.ampl1_begin", "1.0");
    configuration->set_property("InputFilter.ampl1_end", "1.0");
    configuration->set_property("InputFilter.ampl2_begin", "0.0");
    configuration->set_property("InputFilter.ampl2_end", "0.0");

    configuration->set_property("InputFilter.band1_error", "1.0");
    configuration->set_property("InputFilter.band2_error", "1.0");

    configuration->set_property("InputFilter.filter_type", "bandpass");
    configuration->set_property("InputFilter.grid_density", "16");

    configuration->set_property("InputFilter.sampling_frequency", "4000000");
    configuration->set_property("InputFilter.IF", "34000");

    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);

    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Freq_Xlating_Fir_Filter", input_filter->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiatePulseBlankingFilter)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    configuration->set_property("InputFilter.implementation", "Pulse_Blanking_Filter");
    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);
    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Pulse_Blanking_Filter", input_filter->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateNotchFilter)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    configuration->set_property("InputFilter.implementation", "Notch_Filter");
    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);
    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Notch_Filter", input_filter->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateNotchFilterLite)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    configuration->set_property("InputFilter.implementation", "Notch_Filter_Lite");
    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);
    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Notch_Filter_Lite", input_filter->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongFilter)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    configuration->set_property("InputFilter.implementation", "Pollen_Filter");
    auto input_filter = block_factory::GetBlock(configuration.get(), "InputFilter", 1, 1);
    EXPECT_EQ(nullptr, input_filter);
}


TEST(GNSSBlockFactoryTest, InstantiateDirectResampler)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Resampler.implementation", "Direct_Resampler");
    auto resampler = block_factory::GetBlock(configuration.get(), "Resampler", 1, 1);
    EXPECT_STREQ("Resampler", resampler->role().c_str());
    EXPECT_STREQ("Direct_Resampler", resampler->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongResampler)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Resampler.implementation", "RaNdOm_Resampler");
    auto resampler = block_factory::GetBlock(configuration.get(), "Resampler", 1, 1);
    EXPECT_EQ(nullptr, resampler);
}


TEST(GNSSBlockFactoryTest, InstantiateGpsL1CaPcpsAcquisition)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Acquisition");
    auto acquisition = block_factory::GetBlock(configuration.get(), "Acquisition", 1, 0);
    EXPECT_STREQ("Acquisition", acquisition->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_PCPS_Acquisition", acquisition->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateGpsL1CaPcpsQuickSyncAcquisition)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_QuickSync_Acquisition");
    auto acquisition = block_factory::GetBlock(configuration.get(), "Acquisition", 1, 0);
    EXPECT_STREQ("Acquisition", acquisition->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_PCPS_QuickSync_Acquisition", acquisition->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateGalileoE1PcpsQuickSyncAmbiguousAcquisition)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Acquisition.implementation", "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition");
    auto acquisition = block_factory::GetBlock(configuration.get(), "Acquisition", 1, 0);
    EXPECT_STREQ("Acquisition", acquisition->role().c_str());
    EXPECT_STREQ("Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition", acquisition->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateGalileoE1PcpsAmbiguousAcquisition)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Acquisition.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    auto acquisition = block_factory::GetBlock(configuration.get(), "Acquisition", 1, 0);
    EXPECT_STREQ("Acquisition", acquisition->role().c_str());
    EXPECT_STREQ("Galileo_E1_PCPS_Ambiguous_Acquisition", acquisition->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongAcquisition)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Alchemy");
    auto acq_ = block_factory::GetBlock(configuration.get(), "Acquisition", 1, 0);
    EXPECT_EQ(nullptr, acq_);
}


TEST(GNSSBlockFactoryTest, InstantiateDllPllTrackingAdapterGpsL1Ca)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Tracking.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    auto tracking = block_factory::GetBlock(configuration.get(), "Tracking", 1, 1);
    EXPECT_STREQ("Tracking", tracking->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_DLL_PLL_Tracking", tracking->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateGpsL1CaTcpConnectorTracking)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Tracking.implementation", "GPS_L1_CA_TCP_CONNECTOR_Tracking");
    auto tracking = block_factory::GetBlock(configuration.get(), "Tracking", 1, 1);
    EXPECT_STREQ("Tracking", tracking->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_TCP_CONNECTOR_Tracking", tracking->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateDllPllTrackingAdapterGalileoE1)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Tracking.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    auto tracking = block_factory::GetBlock(configuration.get(), "Tracking", 1, 1);
    EXPECT_STREQ("Tracking", tracking->role().c_str());
    EXPECT_STREQ("Galileo_E1_DLL_PLL_VEML_Tracking", tracking->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongTracking)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Tracking.implementation", "The perfect tracking");
    auto trk_ = block_factory::GetBlock(configuration.get(), "Tracking", 1, 1);
    EXPECT_EQ(nullptr, trk_);
}


TEST(GNSSBlockFactoryTest, InstantiateTelemetryDecoderAdapterGpsL1Ca)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("TelemetryDecoder.implementation", "GPS_L1_CA_Telemetry_Decoder");
    auto telemetry_decoder = block_factory::GetBlock(configuration.get(), "TelemetryDecoder", 1, 1);
    EXPECT_STREQ("TelemetryDecoder", telemetry_decoder->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_Telemetry_Decoder", telemetry_decoder->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongTelemetryDecoder)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("TelemetryDecoder.implementation", "GPS_Xenomorphic_Telemetry_Decoder");
    auto telemetry_decoder = block_factory::GetBlock(configuration.get(), "TelemetryDecoder", 1, 1);
    EXPECT_EQ(nullptr, telemetry_decoder);
}


TEST(GNSSBlockFactoryTest, InstantiateEmptyTelemetryDecoder)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("TelemetryDecoder.implementation", std::string(""));
    auto telemetry_decoder = block_factory::GetBlock(configuration.get(), "TelemetryDecoder", 1, 1);
    EXPECT_EQ(nullptr, telemetry_decoder);
}


TEST(GNSSBlockFactoryTest, InstantiateChannels)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Channels_1C.count", "2");
    configuration->set_property("Channels_1E.count", "0");
    configuration->set_property("Channels.in_acquisition", "2");
    configuration->set_property("Acquisition_1C.implementation", "GPS_L1_CA_PCPS_Acquisition");
    configuration->set_property("Tracking_1C.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    configuration->set_property("TelemetryDecoder_1C.implementation", "GPS_L1_CA_Telemetry_Decoder");
    auto queue = std::make_shared<Concurrent_Queue<pmt::pmt_t>>();
    auto channels = block_factory::GetChannels(configuration.get(), queue.get());
    EXPECT_EQ(static_cast<unsigned int>(2), channels.size());
    channels.erase(channels.begin(), channels.end());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongObservables)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Observables.implementation", "Supercalifragilistic_Observables");
    auto observables = block_factory::GetObservables(configuration.get());
    EXPECT_EQ(nullptr, observables);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongObservables2)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Observables.implementation", "Pass_Through");
    auto observables = block_factory::GetObservables(configuration.get());
    EXPECT_EQ(nullptr, observables);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongObservables3)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Observables.implementation", "RTKLIB_PVT");
    auto observables = block_factory::GetObservables(configuration.get());
    EXPECT_EQ(nullptr, observables);
}


TEST(GNSSBlockFactoryTest, InstantiateObservables)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Observables.implementation", "Hybrid_Observables");
    auto observables = block_factory::GetObservables(configuration.get());
    EXPECT_STREQ("Observables", observables->role().c_str());
    EXPECT_STREQ("Hybrid_Observables", observables->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateRTKLIBPvt)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("PVT.implementation", "RTKLIB_PVT");
    auto pvt = block_factory::GetPVT(configuration.get());
    EXPECT_STREQ("PVT", pvt->role().c_str());
    EXPECT_STREQ("RTKLIB_PVT", pvt->implementation().c_str());
}


TEST(GNSSBlockFactoryTest, InstantiateWrongPvt)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("PVT.implementation", "Pepito");
    auto pvt = block_factory::GetPVT(configuration.get());
    EXPECT_EQ(nullptr, pvt);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongPvt2)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("PVT.implementation", "Pass_Through");
    auto pvt = block_factory::GetPVT(configuration.get());
    EXPECT_EQ(nullptr, pvt);
}


TEST(GNSSBlockFactoryTest, InstantiateWrongPvt3)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("PVT.implementation", "Quantum_Particle_PVT");
    auto pvt = block_factory::GetPVT(configuration.get());
    EXPECT_EQ(nullptr, pvt);
}
