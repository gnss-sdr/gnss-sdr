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
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
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
#include "null_sink_output_filter.h"
#include "file_output_filter.h"
#include "channel.h"
#include "uhd_signal_source.h"
#include "signal_conditioner.h"
#include "array_signal_conditioner.h"
#include "ishort_to_complex.h"
#include "direct_resampler_conditioner.h"
#include "fir_filter.h"
#include "freq_xlating_fir_filter.h"
#include "beamformer_filter.h"
#include "gps_l1_ca_pcps_acquisition.h"
#include "gps_l1_ca_pcps_multithread_acquisition.h"
#include "gps_l1_ca_pcps_tong_acquisition.h"
#include "gps_l1_ca_pcps_assisted_acquisition.h"
#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "galileo_e1_pcps_ambiguous_acquisition.h"
#include "galileo_e1_pcps_8ms_ambiguous_acquisition.h"
#include "galileo_e1_pcps_tong_ambiguous_acquisition.h"
#include "galileo_e1_pcps_cccwsr_ambiguous_acquisition.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "gps_l1_ca_dll_pll_optim_tracking.h"
#include "gps_l1_ca_dll_fll_pll_tracking.h"
#include "gps_l1_ca_tcp_connector_tracking.h"
#include "galileo_e1_dll_pll_veml_tracking.h"
#include "galileo_e1_tcp_connector_tracking.h"
#include "gps_l1_ca_telemetry_decoder.h"
#include "galileo_e1b_telemetry_decoder.h"
#include "sbas_l1_telemetry_decoder.h"
#include "gps_l1_ca_observables.h"
#include "galileo_e1_observables.h"
#include "gps_l1_ca_pvt.h"
#include "galileo_e1_pvt.h"

#if OPENCL_BLOCKS
    #include "gps_l1_ca_pcps_opencl_acquisition.h"
#endif

#if GN3S_DRIVER
        #include "gn3s_signal_source.h"
#endif

#if RAW_ARRAY_DRIVER
        #include "raw_array_signal_source.h"
#endif

#if RTLSDR_DRIVER
        #include "rtlsdr_signal_source.h"
#endif

using google::LogMessage;


GNSSBlockFactory::GNSSBlockFactory()
{}


GNSSBlockFactory::~GNSSBlockFactory()
{}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetSignalSource(
        std::shared_ptr<ConfigurationInterface> configuration, boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "File_Signal_Source";
    std::string implementation = configuration->property(
            "SignalSource.implementation", default_implementation);
    LOG(INFO) << "Getting SignalSource with implementation " << implementation;
    return GetBlock(configuration, "SignalSource", implementation, 0, 1,
            queue);
}



std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetSignalConditioner(
        std::shared_ptr<ConfigurationInterface> configuration, boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "Pass_Through";
    std::string signal_conditioner = configuration->property(
            "SignalConditioner.implementation", default_implementation);
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
                    "DataTypeAdapter.implementation", default_implementation);
            input_filter = configuration->property(
                    "InputFilter.implementation", default_implementation);
            resampler = configuration->property(
                     "Resampler.implementation", default_implementation);
        }

    LOG(INFO) << "Getting SignalConditioner with DataTypeAdapter implementation: "
            << data_type_adapter << ", InputFilter implementation: "
            << input_filter << ", and Resampler implementation: "
            << resampler;
    //std::unique_ptr<GNSSBlockInterface> conditioner_;

    if(signal_conditioner.compare("Array_Signal_Conditioner") == 0)
        {
            //instantiate the array version
            std::unique_ptr<GNSSBlockInterface> conditioner_(new ArraySignalConditioner(configuration.get(), GetBlock(configuration,
                    "DataTypeAdapter", data_type_adapter, 1, 1, queue).release(), GetBlock(
                            configuration,"InputFilter", input_filter, 1, 1, queue).release(),
                            GetBlock(configuration,"Resampler", resampler, 1, 1, queue).release(),
                            "SignalConditioner", "Signal_Conditioner", queue));
            return conditioner_;
        }
    else
        {
            //single-antenna version
            std::unique_ptr<GNSSBlockInterface> conditioner_(new SignalConditioner(configuration.get(), GetBlock(configuration,
                    "DataTypeAdapter", data_type_adapter, 1, 1, queue).release(), GetBlock(
                            configuration,"InputFilter", input_filter, 1, 1, queue).release(),
                            GetBlock(configuration,"Resampler", resampler, 1, 1, queue).release(),
                            "SignalConditioner", "Signal_Conditioner", queue));
            return conditioner_;
        }
}



std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetObservables(std::shared_ptr<ConfigurationInterface> configuration,
        boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "GPS_L1_CA_Observables";
    std::string implementation = configuration->property("Observables.implementation", default_implementation);
    LOG(INFO) << "Getting Observables with implementation " << implementation;
    unsigned int channel_count = configuration->property("Channels.count", 12);
    return GetBlock(configuration, "Observables", implementation, channel_count, channel_count, queue);
}



std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetPVT(std::shared_ptr<ConfigurationInterface> configuration,
        boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "Pass_Through";
    std::string implementation = configuration->property("PVT.implementation", default_implementation);
    LOG(INFO) << "Getting PVT with implementation " << implementation;
    unsigned int channel_count = configuration->property("Channels.count", 12);
    return GetBlock(configuration, "PVT", implementation, channel_count, 1, queue);
}



std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetOutputFilter(std::shared_ptr<ConfigurationInterface> configuration,
        boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "Null_Sink_Output_Filter";
    std::string implementation = configuration->property("OutputFilter.implementation", default_implementation);
    LOG(INFO) << "Getting OutputFilter with implementation " << implementation;
    return GetBlock(configuration, "OutputFilter", implementation, 1, 0, queue);
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel(
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
    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition", acq, 1, 1, queue);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking", trk, 1, 1, queue);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder", tlm, 1, 1, queue);

    std::unique_ptr<GNSSBlockInterface> channel_(new Channel(configuration.get(), channel, pass_through_.release(),
            acq_.release(),
            trk_.release(),
            tlm_.release(),
            "Channel", "Channel", queue));

    return channel_;
}



std::unique_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> GNSSBlockFactory::GetChannels(
        std::shared_ptr<ConfigurationInterface> configuration, boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "Pass_Through";
    unsigned int channel_count = configuration->property("Channels.count", 12);
    LOG(INFO) << "Getting " << channel_count << " channels";

    std::unique_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> channels(new std::vector<std::unique_ptr<GNSSBlockInterface>>());

    std::string tracking = configuration->property("Tracking.implementation", default_implementation);
    std::string telemetry_decoder = configuration->property("TelemetryDecoder.implementation", default_implementation);
    std::string acquisition_implementation = configuration->property("Acquisition.implementation", default_implementation);

    for (unsigned int i = 0; i < channel_count; i++)
        {
            std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition" + boost::lexical_cast<std::string>(i) + ".implementation",
                        default_implementation);
            if(acquisition_implementation_specific.compare(default_implementation) != 0)
            {
                acquisition_implementation = acquisition_implementation_specific;
            }

            channels->push_back(std::move(GetChannel(configuration,
                    acquisition_implementation, tracking, telemetry_decoder, i, queue)));
        }
    return channels;
}


/*
 * Returns the block with the required configuration and implementation
 *
 * PLEASE ADD YOUR NEW BLOCK HERE!!
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
                    LOG(ERROR) << implementation << ": Source file not found";
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
                    LOG(ERROR) << implementation << ": Source file not found";
                    exit(1);
            }
        }
    else if (implementation.compare("UHD_Signal_Source") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new UhdSignalSource(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
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

#if RTLSDR_DRIVER
    else if (implementation.compare("Rtlsdr_Signal_Source") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new RtlsdrSignalSource(configuration.get(), role, in_streams,
                    out_streams, queue));
            block = std::move(block_);
        }
#endif

    // DATA TYPE ADAPTER -----------------------------------------------------------
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
    else if (implementation.compare("Galileo_E1_TCP_CONNECTOR_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1TcpConnectorTracking(configuration.get(), role, in_streams,
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
            LOG(ERROR) << implementation << ": Undefined implementation for block";
        }
    return std::move(block);
}


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
    else
         {
             // Log fatal. This causes execution to stop.
             LOG(ERROR) << implementation << ": Undefined implementation for block";
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
  else if (implementation.compare("Galileo_E1_TCP_CONNECTOR_Tracking") == 0)
      {
          std::unique_ptr<TrackingInterface> block_(new GalileoE1TcpConnectorTracking(configuration.get(), role, in_streams,
                  out_streams, queue));
          block = std::move(block_);
      }
  else
         {
             // Log fatal. This causes execution to stop.
             LOG(ERROR) << implementation << ": Undefined implementation for block";
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
else
       {
           // Log fatal. This causes execution to stop.
           LOG(ERROR) << implementation << ": Undefined implementation for block";
       }
   return std::move(block);
}
