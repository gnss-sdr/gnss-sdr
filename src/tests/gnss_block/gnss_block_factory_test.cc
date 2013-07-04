/*!
 * \file gnss_block_factory_test.cc
 * \brief This class implements a Unit Test for the GNSSBlockFactory class.
 * \authors <ul>
 *          <li> Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
 *
 * This class test the instantiation of all blocks in gnss_block_factory
 *
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


#include <gnuradio/msg_queue.h>
#include <vector>
#include <gtest/gtest.h>
#include "in_memory_configuration.h"
#include "gnss_block_interface.h"
#include "acquisition_interface.h"
#include "tracking_interface.h"
#include "telemetry_decoder_interface.h"
#include "observables_interface.h"
#include "pvt_interface.h"
#include "gnss_block_factory.h"


TEST(GNSS_Block_Factory_Test, InstantiateFileSignalSource)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();

    configuration->set_property("SignalSource.implementation", "File_Signal_Source");
    configuration->set_property("SignalSource.filename", "../src/tests/signal_samples/GPS_L1_CA_ID_1_Fs_4Msps_2ms.dat");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    GNSSBlockInterface *signal_source = factory->GetSignalSource(configuration, queue);

    EXPECT_STREQ("SignalSource", signal_source->role().c_str());
    EXPECT_STREQ("File_Signal_Source", signal_source->implementation().c_str());

    delete configuration;
    delete factory;
    delete signal_source;
}


TEST(GNSS_Block_Factory_Test, InstantiateUHDSignalSource)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();

    configuration->set_property("SignalSource.implementation", "UHD_Signal_Source");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    GNSSBlockInterface *signal_source = factory->GetSignalSource(configuration, queue);

    EXPECT_STREQ("SignalSource", signal_source->role().c_str());
    EXPECT_STREQ("UHD_Signal_Source", signal_source->implementation().c_str());

    delete configuration;
    delete factory;
    delete signal_source;
}


TEST(GNSS_Block_Factory_Test, InstantiateWrongSignalSource)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("SignalSource.implementation", "Pepito");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    GNSSBlockInterface *signal_source = factory->GetSignalSource(configuration, queue);

    EXPECT_EQ(NULL, signal_source);

    delete configuration;
    delete factory;
}


TEST(GNSS_Block_Factory_Test, InstantiateSignalConditioner)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("SignalConditioner.implementation", "Signal_Conditioner");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    GNSSBlockInterface *signal_conditioner = factory->GetSignalConditioner(configuration, queue);

    EXPECT_STREQ("SignalConditioner", signal_conditioner->role().c_str());
    EXPECT_STREQ("Signal_Conditioner", signal_conditioner->implementation().c_str());

    delete configuration;
    delete factory;
    delete signal_conditioner;
}


TEST(GNSS_Block_Factory_Test, InstantiateFIRFilter)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

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

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    GNSSBlockInterface *input_filter = factory->GetBlock(configuration, "InputFilter", "Fir_Filter", 1,1, queue);

    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Fir_Filter", input_filter->implementation().c_str());

    delete configuration;
    delete factory;
    delete input_filter;
}

TEST(GNSS_Block_Factory_Test, InstantiateFreqXlatingFIRFilter)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

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

    configuration->set_property("InputFilter.sampling_frequency","4000000");
    configuration->set_property("InputFilter.IF","34000");

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    GNSSBlockInterface *input_filter = factory->GetBlock(configuration, "InputFilter", "Freq_Xlating_Fir_Filter", 1,1, queue);

    EXPECT_STREQ("InputFilter", input_filter->role().c_str());
    EXPECT_STREQ("Freq_Xlating_Fir_Filter", input_filter->implementation().c_str());

    delete configuration;
    delete factory;
    delete input_filter;
}

TEST(GNSS_Block_Factory_Test, InstantiateDirectResampler)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("Resampler.implementation", "Direct_Resampler");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    GNSSBlockInterface *resampler = factory->GetBlock(configuration, "Resampler", "Direct_Resampler", 1,1, queue);

    EXPECT_STREQ("Resampler", resampler->role().c_str());
    EXPECT_STREQ("Direct_Resampler", resampler->implementation().c_str());

    delete configuration;
    delete factory;
    delete resampler;
}

TEST(GNSS_Block_Factory_Test, InstantiateGpsL1CaPcpsAcquisition)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Acquisition");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    AcquisitionInterface *acquisition = (AcquisitionInterface*)factory->GetBlock(configuration, "Acquisition", "GPS_L1_CA_PCPS_Acquisition", 1, 1, queue);

    EXPECT_STREQ("Acquisition", acquisition->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_PCPS_Acquisition", acquisition->implementation().c_str());

    delete configuration;
    delete factory;
    delete acquisition;
}


TEST(GNSS_Block_Factory_Test, InstantiateGalileoE1PcpsAmbiguousAcquisition)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("Acquisition.implementation", "Galileo_E1_PCPS_Ambiguous_Acquisition");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    AcquisitionInterface *acquisition = (AcquisitionInterface*)factory->GetBlock(configuration, "Acquisition", "Galileo_E1_PCPS_Ambiguous_Acquisition", 1, 1, queue);

    EXPECT_STREQ("Acquisition", acquisition->role().c_str());
    EXPECT_STREQ("Galileo_E1_PCPS_Ambiguous_Acquisition", acquisition->implementation().c_str());

    delete configuration;
    delete factory;
    delete acquisition;
}


TEST(GNSS_Block_Factory_Test, InstantiateGpsL1CaDllFllPllTracking)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("Tracking.implementation", "GPS_L1_CA_DLL_FLL_PLL_Tracking");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    TrackingInterface *tracking = (TrackingInterface*)factory->GetBlock(configuration, "Tracking", "GPS_L1_CA_DLL_FLL_PLL_Tracking", 1, 1, queue);

    EXPECT_STREQ("Tracking", tracking->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_DLL_FLL_PLL_Tracking", tracking->implementation().c_str());

    delete configuration;
    delete factory;
    delete tracking;
}


TEST(GNSS_Block_Factory_Test, InstantiateGpsL1CaDllPllTracking)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("Tracking.implementation", "GPS_L1_CA_DLL_PLL_Tracking");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    TrackingInterface *tracking = (TrackingInterface*)factory->GetBlock(configuration, "Tracking", "GPS_L1_CA_DLL_PLL_Tracking", 1, 1, queue);

    EXPECT_STREQ("Tracking", tracking->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_DLL_PLL_Tracking", tracking->implementation().c_str());

    delete configuration;
    delete factory;
    delete tracking;
}


TEST(GNSS_Block_Factory_Test, InstantiateGpsL1CaTcpConnectorTracking)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("Tracking.implementation", "GPS_L1_CA_TCP_CONNECTOR_Tracking");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    TrackingInterface *tracking = (TrackingInterface*)factory->GetBlock(configuration, "Tracking", "GPS_L1_CA_TCP_CONNECTOR_Tracking", 1, 1, queue);

    EXPECT_STREQ("Tracking", tracking->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_TCP_CONNECTOR_Tracking", tracking->implementation().c_str());

    delete configuration;
    delete factory;
    delete tracking;
}


TEST(GNSS_Block_Factory_Test, InstantiateGalileoE1DllPllVemlTracking)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("Tracking.implementation", "Galileo_E1_DLL_PLL_VEML_Tracking");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    TrackingInterface *tracking = (TrackingInterface*)factory->GetBlock(configuration, "Tracking", "Galileo_E1_DLL_PLL_VEML_Tracking", 1, 1, queue);

    EXPECT_STREQ("Tracking", tracking->role().c_str());
    EXPECT_STREQ("Galileo_E1_DLL_PLL_VEML_Tracking", tracking->implementation().c_str());

    delete configuration;
    delete factory;
    delete tracking;
}


TEST(GNSS_Block_Factory_Test, InstantiateGpsL1CaTelemetryDecoder)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("TelemetryDecoder.implementation", "GPS_L1_CA_Telemetry_Decoder");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    TelemetryDecoderInterface *telemetry_decoder = (TelemetryDecoderInterface*)factory->GetBlock(configuration, "TelemetryDecoder", "GPS_L1_CA_Telemetry_Decoder", 1, 1, queue);

    EXPECT_STREQ("TelemetryDecoder", telemetry_decoder->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_Telemetry_Decoder", telemetry_decoder->implementation().c_str());

    delete configuration;
    delete factory;
    delete telemetry_decoder;
}

TEST(GNSS_Block_Factory_Test, InstantiateChannels)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();

    configuration->set_property("Channels.count", "2");
    configuration->set_property("Channels.in_acquisition", "2");
    configuration->set_property("Tracking.implementation","GPS_L1_CA_DLL_FLL_PLL_Tracking");
    configuration->set_property("TelemetryDecoder.implementation","GPS_L1_CA_Telemetry_Decoder");

    configuration->set_property("Channel0.item_type", "gr_complex");
    configuration->set_property("Acquisition0.implementation", "GPS_L1_CA_PCPS_Acquisition");

    configuration->set_property("Channel1.item_type", "gr_complex");
    configuration->set_property("Acquisition1.implementation", "GPS_L1_CA_PCPS_Acquisition");

    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();

    std::vector<GNSSBlockInterface*>* channels = factory->GetChannels(configuration, queue);

    EXPECT_EQ((unsigned int) 2, channels->size());

    delete configuration;
    delete factory;
    for(unsigned int i=0 ; i<channels->size() ; i++) delete channels->at(i);
    channels->clear();
    delete channels;
}


TEST(GNSS_Block_Factory_Test, InstantiateObservables)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("Observables.implementation", "Pass_Through");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    ObservablesInterface *observables = (ObservablesInterface*)factory->GetObservables(configuration, queue);

    EXPECT_STREQ("Observables", observables->role().c_str());
    EXPECT_STREQ("Pass_Through", observables->implementation().c_str());

    delete configuration;
    delete factory;
    delete observables;
}


TEST(GNSS_Block_Factory_Test, InstantiateGpsL1CaObservables)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("Observables.implementation", "GPS_L1_CA_Observables");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    ObservablesInterface *observables = (ObservablesInterface*)factory->GetObservables(configuration, queue);

    EXPECT_STREQ("Observables", observables->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_Observables", observables->implementation().c_str());

    delete configuration;
    delete factory;
    delete observables;
}


TEST(GNSS_Block_Factory_Test, InstantiateWrongObservables)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("Observables.implementation", "Pepito");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    ObservablesInterface *observables = (ObservablesInterface*)factory->GetObservables(configuration, queue);

    EXPECT_EQ(NULL, observables);

    delete configuration;
    delete factory;
    delete observables;
}

TEST(GNSS_Block_Factory_Test, InstantiatePvt)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("PVT.implementation", "Pass_Through");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    PvtInterface *pvt = (PvtInterface*)factory->GetPVT(configuration, queue);

    EXPECT_STREQ("PVT", pvt->role().c_str());
    EXPECT_STREQ("Pass_Through", pvt->implementation().c_str());

    delete configuration;
    delete factory;
    delete pvt;
}


TEST(GNSS_Block_Factory_Test, InstantiateGpsL1CaPvt)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("PVT.implementation", "GPS_L1_CA_PVT");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    PvtInterface *pvt = (PvtInterface*)factory->GetPVT(configuration, queue);

    EXPECT_STREQ("PVT", pvt->role().c_str());
    EXPECT_STREQ("GPS_L1_CA_PVT", pvt->implementation().c_str());

    delete configuration;
    delete factory;
    delete pvt;
}


TEST(GNSS_Block_Factory_Test, InstantiateWrongPvt)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("PVT.implementation", "Pepito");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    PvtInterface *pvt = (PvtInterface*)factory->GetPVT(configuration, queue);

    EXPECT_EQ(NULL, pvt);

    delete configuration;
    delete factory;
    delete pvt;
}



TEST(GNSS_Block_Factory_Test, InstantiateNullSinkOutputFilter)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("OutputFilter.implementation", "Null_Sink_Output_Filter");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    GNSSBlockInterface *output_filter = factory->GetOutputFilter(configuration, queue);

    EXPECT_STREQ("OutputFilter", output_filter->role().c_str());
    EXPECT_STREQ("Null_Sink_Output_Filter", output_filter->implementation().c_str());

    delete configuration;
    delete factory;
    delete output_filter;
}


TEST(GNSS_Block_Factory_Test, InstantiateFileOutputFilter)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("OutputFilter.implementation", "File_Output_Filter");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    GNSSBlockInterface *output_filter = factory->GetOutputFilter(configuration, queue);

    EXPECT_STREQ("OutputFilter", output_filter->role().c_str());
    EXPECT_STREQ("File_Output_Filter", output_filter->implementation().c_str());

    delete configuration;
    delete factory;
    delete output_filter;
}


TEST(GNSS_Block_Factory_Test, InstantiateWrongOutputFilter)
{
    InMemoryConfiguration *configuration = new InMemoryConfiguration();
    configuration->set_property("OutputFilter.implementation", "Pepito");
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    GNSSBlockFactory *factory = new GNSSBlockFactory();
    GNSSBlockInterface *output_filter = factory->GetOutputFilter(configuration, queue);

    EXPECT_EQ(NULL, output_filter);

    delete configuration;
    delete factory;
}
