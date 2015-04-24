/*!
 * \file gnss_block_factory.cc
 * \brief  This class implements a factory that returns instances of GNSS blocks.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * This class encapsulates the complexity behind the instantiation
 * of GNSS blocks.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "gnss_block_factory.h"
#include <string>
#include <sstream>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include "configuration_interface.h"
#include "gnss_block_interface.h"
#include "pass_through.h"
#include "file_signal_source.h"
#include "nsr_file_signal_source.h"
#include "spir_file_signal_source.h"
#include "null_sink_output_filter.h"
#include "file_output_filter.h"
#include "channel.h"

#include "signal_conditioner.h"
#include "array_signal_conditioner.h"
#include "byte_to_short.h"
#include "ibyte_to_cbyte.h"
#include "ibyte_to_complex.h"
#include "ishort_to_cshort.h"
#include "ishort_to_complex.h"
#include "direct_resampler_conditioner.h"
#include "fir_filter.h"
#include "freq_xlating_fir_filter.h"
#include "beamformer_filter.h"
#include "gps_l1_ca_pcps_acquisition.h"
#include "gps_l2_m_pcps_acquisition.h"
#include "gps_l1_ca_pcps_multithread_acquisition.h"
#include "gps_l1_ca_pcps_tong_acquisition.h"
#include "gps_l1_ca_pcps_assisted_acquisition.h"
#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "gps_l1_ca_pcps_quicksync_acquisition.h"
#include "galileo_e1_pcps_ambiguous_acquisition.h"
#include "galileo_e1_pcps_8ms_ambiguous_acquisition.h"
#include "galileo_e1_pcps_tong_ambiguous_acquisition.h"
#include "galileo_e1_pcps_cccwsr_ambiguous_acquisition.h"
#include "galileo_e1_pcps_quicksync_ambiguous_acquisition.h"
#include "galileo_e5a_noncoherent_iq_acquisition_caf.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "gps_l1_ca_dll_pll_optim_tracking.h"
#include "gps_l1_ca_dll_fll_pll_tracking.h"
#include "gps_l1_ca_tcp_connector_tracking.h"
#include "galileo_e1_dll_pll_veml_tracking.h"
#include "galileo_volk_e1_dll_pll_veml_tracking.h"
#include "galileo_e1_tcp_connector_tracking.h"
#include "galileo_e5a_dll_pll_tracking.h"
#include "gps_l1_ca_telemetry_decoder.h"
#include "galileo_e1b_telemetry_decoder.h"
#include "galileo_e5a_telemetry_decoder.h"
#include "sbas_l1_telemetry_decoder.h"
#include "gps_l1_ca_observables.h"
#include "galileo_e1_observables.h"
#include "hybrid_observables.h"
#include "gps_l1_ca_pvt.h"
#include "galileo_e1_pvt.h"
#include "hybrid_pvt.h"

#if OPENCL_BLOCKS
    #include "gps_l1_ca_pcps_opencl_acquisition.h"
#endif

#if GN3S_DRIVER
        #include "gn3s_signal_source.h"
#endif

#if RAW_ARRAY_DRIVER
        #include "raw_array_signal_source.h"
#endif

#if OSMOSDR_DRIVER
        #include "osmosdr_signal_source.h"
#endif

#if UHD_DRIVER
        #include "uhd_signal_source.h"
#endif

#if FLEXIBAND_DRIVER
        #include "flexiband_signal_source.h"
#endif


using google::LogMessage;


GNSSBlockFactory::GNSSBlockFactory()
{}


GNSSBlockFactory::~GNSSBlockFactory()
{}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetSignalSource(
        std::shared_ptr<ConfigurationInterface> configuration, boost::shared_ptr<gr::msg_queue> queue, int ID)
{
    std::string default_implementation = "File_Signal_Source";
    std::string role = "SignalSource"; //backwards compatibility for old conf files
    if (ID != -1)
        {
            role = "SignalSource" + boost::lexical_cast<std::string>(ID);
        }
    std::string implementation = configuration->property(role + ".implementation", default_implementation);
    LOG(INFO) << "Getting SignalSource with implementation " << implementation;
    return GetBlock(configuration, role, implementation, 0, 1, queue);
}



std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetSignalConditioner(
        std::shared_ptr<ConfigurationInterface> configuration, boost::shared_ptr<gr::msg_queue> queue, int ID)
{
    std::string default_implementation = "Pass_Through";
    //backwards compatibility for old conf files
    std::string role_conditioner = "SignalConditioner" ;
    std::string role_datatypeadapter = "DataTypeAdapter";
    std::string role_inputfilter = "InputFilter";
    std::string role_resampler = "Resampler";

    if (ID != -1)
        {
            role_conditioner = "SignalConditioner" + boost::lexical_cast<std::string>(ID);
            role_datatypeadapter = "DataTypeAdapter" + boost::lexical_cast<std::string>(ID);
            role_inputfilter = "InputFilter" + boost::lexical_cast<std::string>(ID);
            role_resampler = "Resampler" + boost::lexical_cast<std::string>(ID);
        }

    std::string signal_conditioner = configuration->property(
            role_conditioner + ".implementation", default_implementation);

    std::string data_type_adapter;
    std::string input_filter;
    std::string resampler;
    if(signal_conditioner.compare("Pass_Through") == 0)
        {
            data_type_adapter = "Pass_Through";
            input_filter = "Pass_Through";
            resampler = "Pass_Through";
        }
    else
        {
            data_type_adapter = configuration->property(
                    role_datatypeadapter + ".implementation", default_implementation);
            input_filter = configuration->property(
                    role_inputfilter + ".implementation", default_implementation);
            resampler = configuration->property(
                    role_resampler + ".implementation", default_implementation);
        }

    LOG(INFO) << "Getting SignalConditioner with DataTypeAdapter implementation: "
            << data_type_adapter << ", InputFilter implementation: "
            << input_filter << ", and Resampler implementation: "
            << resampler;

    if(signal_conditioner.compare("Array_Signal_Conditioner") == 0)
        {
            //instantiate the array version
            std::unique_ptr<GNSSBlockInterface> conditioner_(new ArraySignalConditioner(configuration.get(), GetBlock(configuration,
                    role_datatypeadapter, data_type_adapter, 1, 1, queue).release(), GetBlock(
                            configuration,role_inputfilter, input_filter, 1, 1, queue).release(),
                            GetBlock(configuration,role_resampler, resampler, 1, 1, queue).release(),
                            role_conditioner, "Signal_Conditioner", queue));
            return conditioner_;
        }
    else
        {
            //single-antenna version
            std::unique_ptr<GNSSBlockInterface> conditioner_(new SignalConditioner(configuration.get(), GetBlock(configuration,
                    role_datatypeadapter, data_type_adapter, 1, 1, queue).release(), GetBlock(
                            configuration,role_inputfilter, input_filter, 1, 1, queue).release(),
                            GetBlock(configuration,role_resampler, resampler, 1, 1, queue).release(),
                            role_conditioner, "Signal_Conditioner", queue));
            return conditioner_;
        }
}



std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetObservables(std::shared_ptr<ConfigurationInterface> configuration,
        boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "GPS_L1_CA_Observables";
    std::string implementation = configuration->property("Observables.implementation", default_implementation);
    LOG(INFO) << "Getting Observables with implementation " << implementation;
    unsigned int Galileo_channels = configuration->property("Channels_Galileo.count", 0);
    unsigned int GPS_channels = configuration->property("Channels_GPS.count", 0);
    return GetBlock(configuration, "Observables", implementation, Galileo_channels + GPS_channels, Galileo_channels + GPS_channels, queue);
}



std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetPVT(std::shared_ptr<ConfigurationInterface> configuration,
        boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "Pass_Through";
    std::string implementation = configuration->property("PVT.implementation", default_implementation);
    LOG(INFO) << "Getting PVT with implementation " << implementation;
    unsigned int Galileo_channels = configuration->property("Channels_Galileo.count", 0);
    unsigned int GPS_channels = configuration->property("Channels_GPS.count", 0);
    return GetBlock(configuration, "PVT", implementation, Galileo_channels + GPS_channels, 1, queue);
}



std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetOutputFilter(std::shared_ptr<ConfigurationInterface> configuration,
        boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "Null_Sink_Output_Filter";
    std::string implementation = configuration->property("OutputFilter.implementation", default_implementation);
    LOG(INFO) << "Getting OutputFilter with implementation " << implementation;
    return GetBlock(configuration, "OutputFilter", implementation, 1, 0, queue);
}

//********* GPS CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_GPS(
        std::shared_ptr<ConfigurationInterface> configuration,
        std::string acq, std::string trk, std::string tlm, int channel,
        boost::shared_ptr<gr::msg_queue> queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << id << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk  << ", Telemetry Decoder implementation: " << tlm;

    std::unique_ptr<GNSSBlockInterface> pass_through_ = GetBlock(configuration, "Channel", "Pass_Through", 1, 1, queue);
    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_GPS", acq, 1, 1, queue);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_GPS", trk, 1, 1, queue);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_GPS", tlm, 1, 1, queue);

    std::unique_ptr<GNSSBlockInterface> channel_(new Channel(configuration.get(), channel, pass_through_.release(),
            acq_.release(),
            trk_.release(),
            tlm_.release(),
            "Channel", "GPS", queue));

    return channel_;
}


//********* GALILEO CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_Galileo(
        std::shared_ptr<ConfigurationInterface> configuration,
        std::string acq, std::string trk, std::string tlm, int channel,
        boost::shared_ptr<gr::msg_queue> queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << id << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk  << ", Telemetry Decoder implementation: " << tlm;

    std::unique_ptr<GNSSBlockInterface> pass_through_ = GetBlock(configuration, "Channel", "Pass_Through", 1, 1, queue);
    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_Galileo", acq, 1, 1, queue);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_Galileo", trk, 1, 1, queue);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_Galileo", tlm, 1, 1, queue);

    std::unique_ptr<GNSSBlockInterface> channel_(new Channel(configuration.get(), channel, pass_through_.release(),
            acq_.release(),
            trk_.release(),
            tlm_.release(),
            "Channel", "Galileo", queue));

    return channel_;
}

std::unique_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> GNSSBlockFactory::GetChannels(
        std::shared_ptr<ConfigurationInterface> configuration, boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "Pass_Through";
    unsigned int channel_count;
    std::string tracking;
    std::string telemetry_decoder;
    std::string acquisition_implementation;

    std::unique_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> channels(new std::vector<std::unique_ptr<GNSSBlockInterface>>());

    unsigned int channel_absolute_id=0;

    //**************** GPS CHANNELS **********************
    channel_count= configuration->property("Channels_GPS.count", 0);

    LOG(INFO) << "Getting " << channel_count << " GPS channels";

    tracking = configuration->property("Tracking_GPS.implementation", default_implementation);
    telemetry_decoder = configuration->property("TelemetryDecoder_GPS.implementation", default_implementation);
    acquisition_implementation = configuration->property("Acquisition_GPS.implementation", default_implementation);

    for (unsigned int i = 0; i < channel_count; i++)
        {
            std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition_GPS" + boost::lexical_cast<std::string>(i) + ".implementation",
                        default_implementation);
            if(acquisition_implementation_specific.compare(default_implementation) != 0)
            {
                acquisition_implementation = acquisition_implementation_specific;
            }

            channels->push_back(std::move(GetChannel_GPS(configuration,
                    acquisition_implementation, tracking, telemetry_decoder, channel_absolute_id, queue)));
            channel_absolute_id++;
        }

    //**************** GALILEO CHANNELS **********************
    channel_count= configuration->property("Channels_Galileo.count", 0);

    LOG(INFO) << "Getting " << channel_count << " Galileo channels";

    tracking = configuration->property("Tracking_Galileo.implementation", default_implementation);
    telemetry_decoder = configuration->property("TelemetryDecoder_Galileo.implementation", default_implementation);
    acquisition_implementation = configuration->property("Acquisition_Galileo.implementation", default_implementation);

    for (unsigned int i = 0; i < channel_count; i++)
        {
            std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition_Galileo" + boost::lexical_cast<std::string>(i) + ".implementation",
                        default_implementation);
            if(acquisition_implementation_specific.compare(default_implementation) != 0)
            {
                acquisition_implementation = acquisition_implementation_specific;
            }
            channels->push_back(std::move(GetChannel_Galileo(configuration,
                    acquisition_implementation, tracking, telemetry_decoder, channel_absolute_id, queue)));
            channel_absolute_id++;
        }
    return channels;
}


/*
 * Returns the block with the required configuration and implementation
 *
 * PLEASE ADD YOUR NEW BLOCK HERE!!
 *
 * IMPORTANT NOTE: Acquisition, Tracking and telemetry blocks are only included here for testing purposes.
 * To be included in a channel they must be also be included in GetAcqBlock(), GetTrkBlock() and GetTlmBlock()
 * (see below)
 */
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetBlock(
        std::shared_ptr<ConfigurationInterface> configuration,
        std::string role,
        std::string implementation, unsigned int in_streams,
        unsigned int out_streams, boost::shared_ptr<gr::msg_queue> queue)
{
    std::unique_ptr<GNSSBlockInterface> block;

    //PASS THROUGH ----------------------------------------------------------------
    if (implementation.compare("Pass_Through") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new Pass_Through(configuration.get(), role, in_streams, out_streams));
            block = std::move(block_);
        }

    // SIGNAL SOURCES -------------------------------------------------------------
    else if (implementation.compare("File_Signal_Source") == 0)
        {
            try
            {
                    std::unique_ptr<GNSSBlockInterface> block_(new FileSignalSource(configuration.get(), role, in_streams,
                            out_streams, queue));
                    block = std::move(block_);
            }

            catch (const std::exception &e)
            {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
            }
        }
    else if (implementation.compare("Nsr_File_Signal_Source") == 0)
        {
            try
            {
                    std::unique_ptr<GNSSBlockInterface> block_(new NsrFileSignalSource(configuration.get(), role, in_streams,
                            out_streams, queue));
                    block = std::move(block_);

            }
            catch (const std::exception &e)
            {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
            }
        }
    else if (implementation.compare("Spir_File_Signal_Source") == 0)
        {
            try
            {
                    std::unique_ptr<GNSSBlockInterface> block_(new SpirFileSignalSource(configuration.get(), role, in_streams,
                            out_streams, queue));
                    block = std::move(block_);

            }
            catch (const std::exception &e)
            {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
            }
        }
#if UHD_DRIVER
    else if (implementation.compare("UHD_Signal_Source") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new UhdSignalSource(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
#endif
#if GN3S_DRIVER
    else if (implementation.compare("GN3S_Signal_Source") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new Gn3sSignalSource(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
#endif

#if RAW_ARRAY_DRIVER
    else if (implementation.compare("Raw_Array_Signal_Source") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new RawArraySignalSource(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
#endif

#if OSMOSDR_DRIVER
    else if (implementation.compare("Osmosdr_Signal_Source") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new OsmosdrSignalSource(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
#endif

#if FLEXIBAND_DRIVER
    else if (implementation.compare("Flexiband_Signal_Source") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new FlexibandSignalSource(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
#endif

    // DATA TYPE ADAPTER -----------------------------------------------------------
    else if (implementation.compare("Byte_To_Short") == 0)
        {
            std::unique_ptr<GNSSBlockInterface>block_(new ByteToShort(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Ibyte_To_Cbyte") == 0)
        {
            std::unique_ptr<GNSSBlockInterface>block_(new IbyteToCbyte(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Ibyte_To_Complex") == 0)
        {
            std::unique_ptr<GNSSBlockInterface>block_(new IbyteToComplex(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Ishort_To_Cshort") == 0)
        {
            std::unique_ptr<GNSSBlockInterface>block_(new IshortToCshort(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Ishort_To_Complex") == 0)
        {
            std::unique_ptr<GNSSBlockInterface>block_(new IshortToComplex(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }

    // INPUT FILTER ----------------------------------------------------------------
    else if (implementation.compare("Fir_Filter") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new FirFilter(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Freq_Xlating_Fir_Filter") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new FreqXlatingFirFilter(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Beamformer_Filter") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new BeamformerFilter(configuration.get(), role, in_streams,
                    out_streams));
            block = std::move(block_);
        }

    // RESAMPLER -------------------------------------------------------------------
    else if (implementation.compare("Direct_Resampler") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new DirectResamplerConditioner(configuration.get(), role,
                    in_streams, out_streams));
            block = std::move(block_);
        }

    // ACQUISITION BLOCKS ---------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_Assisted_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsAssistedAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_Tong_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsTongAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_Multithread_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsMultithreadAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }

#if OPENCL_BLOCKS
    else if (implementation.compare("GPS_L1_CA_PCPS_OpenCl_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsOpenClAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
#endif

    else if (implementation.compare("GPS_L1_CA_PCPS_Acquisition_Fine_Doppler") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsAcquisitionFineDoppler(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_QuickSync_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_( new GpsL1CaPcpsQuickSyncAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1PcpsAmbiguousAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_8ms_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1Pcps8msAmbiguousAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_Tong_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1PcpsTongAmbiguousAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_CCCWSR_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1PcpsCccwsrAmbiguousAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_Noncoherent_IQ_Acquisition_CAF") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE5aNoncoherentIQAcquisitionCaf(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }


    else if (implementation.compare("Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_( new GalileoE1PcpsQuickSyncAmbiguousAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }




    // TRACKING BLOCKS -------------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaDllPllTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_Optim_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaDllPllOptimTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_DLL_FLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaDllFllPllTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_TCP_CONNECTOR_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaTcpConnectorTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1DllPllVemlTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_volk_E1_DLL_PLL_VEML_Tracking") == 0)
    {
        std::unique_ptr<GNSSBlockInterface> block_(new GalileoVolkE1DllPllVemlTracking(configuration.get(), role, in_streams,
                                                                                   out_streams, queue));
        block = std::move(block_);
    }
    else if (implementation.compare("Galileo_E1_TCP_CONNECTOR_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1TcpConnectorTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE5aDllPllTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }

    // TELEMETRY DECODERS ----------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaTelemetryDecoder(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1B_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1BTelemetryDecoder(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("SBAS_L1_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new SbasL1TelemetryDecoder(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE5aTelemetryDecoder(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }

    // OBSERVABLES -----------------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_Observables") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaObservables(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }

    else if (implementation.compare("Galileo_E1B_Observables") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1Observables(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Hybrid_Observables") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new HybridObservables(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    // PVT -------------------------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_PVT") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPvt(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GALILEO_E1_PVT") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1Pvt(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Hybrid_PVT") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new HybridPvt(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    // OUTPUT FILTERS --------------------------------------------------------------
    else if (implementation.compare("Null_Sink_Output_Filter") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new NullSinkOutputFilter(configuration.get(), role, in_streams,
                    out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("File_Output_Filter") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new FileOutputFilter(configuration.get(), role, in_streams,
                    out_streams));
            block = std::move(block_);
        }
    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role<<"."<<implementation << ": Undefined implementation for block";
        }
    return std::move(block);
}


/*
 *
 * PLEASE ADD YOUR NEW BLOCK HERE!!
 *
 * Not very elegant, Acq, Trk and Tlm blocks must be added here, too.
 * To be improved!
 */

std::unique_ptr<AcquisitionInterface> GNSSBlockFactory::GetAcqBlock(
        std::shared_ptr<ConfigurationInterface> configuration,
        std::string role,
        std::string implementation, unsigned int in_streams,
        unsigned int out_streams, boost::shared_ptr<gr::msg_queue> queue)
{
    std::unique_ptr<AcquisitionInterface> block;
    // ACQUISITION BLOCKS ---------------------------------------------------------
    if (implementation.compare("GPS_L1_CA_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_Assisted_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsAssistedAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_Tong_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsTongAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_Multithread_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsMultithreadAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }

#if OPENCL_BLOCKS
    else if (implementation.compare("GPS_L1_CA_PCPS_OpenCl_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsOpenClAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
#endif

    else if (implementation.compare("GPS_L1_CA_PCPS_Acquisition_Fine_Doppler") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsAcquisitionFineDoppler(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_QuickSync_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_( new GpsL1CaPcpsQuickSyncAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L2_M_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL2MPcpsAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE1PcpsAmbiguousAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_8ms_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE1Pcps8msAmbiguousAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_Tong_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE1PcpsTongAmbiguousAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_CCCWSR_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE1PcpsCccwsrAmbiguousAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }

    else if (implementation.compare("Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_( new GalileoE1PcpsQuickSyncAmbiguousAcquisition(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_Noncoherent_IQ_Acquisition_CAF") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE5aNoncoherentIQAcquisitionCaf(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role<<"."<<implementation << ": Undefined implementation for block";
        }
    return std::move(block);
}


std::unique_ptr<TrackingInterface> GNSSBlockFactory::GetTrkBlock(
        std::shared_ptr<ConfigurationInterface> configuration,
        std::string role,
        std::string implementation, unsigned int in_streams,
        unsigned int out_streams, boost::shared_ptr<gr::msg_queue> queue)
{
    std::unique_ptr<TrackingInterface> block;

    // TRACKING BLOCKS -------------------------------------------------------------
    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL1CaDllPllTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_Optim_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL1CaDllPllOptimTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_DLL_FLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL1CaDllFllPllTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_TCP_CONNECTOR_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL1CaTcpConnectorTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GalileoE1DllPllVemlTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_Volk_E1_DLL_PLL_VEML_Tracking") == 0)
    {
        std::unique_ptr<TrackingInterface> block_(new GalileoVolkE1DllPllVemlTracking(configuration.get(), role, in_streams,
                                                                                  out_streams, queue));
        block = std::move(block_);
    }
    else if (implementation.compare("Galileo_E1_TCP_CONNECTOR_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GalileoE1TcpConnectorTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GalileoE5aDllPllTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_volk_E1_DLL_PLL_VEML_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GalileoVolkE1DllPllVemlTracking(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }

    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role<<"."<<implementation << ": Undefined implementation for block";
        }
    return std::move(block);
}


std::unique_ptr<TelemetryDecoderInterface> GNSSBlockFactory::GetTlmBlock(
        std::shared_ptr<ConfigurationInterface> configuration,
        std::string role,
        std::string implementation, unsigned int in_streams,
        unsigned int out_streams, boost::shared_ptr<gr::msg_queue> queue)
{
    std::unique_ptr<TelemetryDecoderInterface> block;

    // TELEMETRY DECODERS ----------------------------------------------------------
    if (implementation.compare("GPS_L1_CA_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new GpsL1CaTelemetryDecoder(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1B_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new GalileoE1BTelemetryDecoder(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("SBAS_L1_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new SbasL1TelemetryDecoder(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new GalileoE5aTelemetryDecoder(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role<<"."<<implementation << ": Undefined implementation for block";
        }
    return std::move(block);
}
