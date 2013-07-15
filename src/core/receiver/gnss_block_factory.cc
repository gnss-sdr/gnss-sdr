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

#include "gnss_block_factory.h"
#include <string>
#include <sstream>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <glog/log_severity.h>
#include <glog/logging.h>
#include "configuration_interface.h"
#include "gnss_block_interface.h"
#include "pass_through.h"
#include "file_signal_source.h"
#include "null_sink_output_filter.h"
#include "file_output_filter.h"
#include "channel.h"
#include "uhd_signal_source.h"
#include "signal_conditioner.h"
#include "ishort_to_complex.h"
#include "direct_resampler_conditioner.h"
#include "fir_filter.h"
#include "freq_xlating_fir_filter.h"
#include "gps_l1_ca_pcps_acquisition.h"
#include "gps_l1_ca_pcps_assisted_acquisition.h"
#include "galileo_e1_pcps_ambiguous_acquisition.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "gps_l1_ca_dll_pll_optim_tracking.h"
#include "gps_l1_ca_dll_fll_pll_tracking.h"
#include "gps_l1_ca_tcp_connector_tracking.h"
#include "galileo_e1_dll_pll_veml_tracking.h"
#include "galileo_e1_tcp_connector_tracking.h"
#include "gps_l1_ca_telemetry_decoder.h"
#include "galileo_e1b_telemetry_decoder.h"
#include "gps_l1_ca_observables.h"
#include "gps_l1_ca_pvt.h"

#if GN3S_DRIVER
	#include "gn3s_signal_source.h"
#endif

#if RTLSDR_DRIVER
	#include "rtlsdr_signal_source.h"
#endif

using google::LogMessage;


GNSSBlockFactory::GNSSBlockFactory()
{}


GNSSBlockFactory::~GNSSBlockFactory()
{}


GNSSBlockInterface* GNSSBlockFactory::GetSignalSource(
        ConfigurationInterface *configuration, boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "File_Signal_Source";
    std::string implementation = configuration->property(
            "SignalSource.implementation", default_implementation);
    DLOG(INFO) << "Getting SignalSource with implementation "
            << implementation;
    return GetBlock(configuration, "SignalSource", implementation, 0, 1,
            queue);
}



GNSSBlockInterface* GNSSBlockFactory::GetSignalConditioner(
        ConfigurationInterface *configuration, boost::shared_ptr<gr::msg_queue> queue)
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

    DLOG(INFO) << "Getting SignalConditioner with DataTypeAdapter implementation: "
            << data_type_adapter << ", InputFilter implementation: "
            << input_filter << ", and Resampler implementation: "
            << resampler;

    return new SignalConditioner(configuration, GetBlock(configuration,
    		"DataTypeAdapter", data_type_adapter, 1, 1, queue), GetBlock(
    		configuration,"InputFilter", input_filter, 1, 1, queue),
    		GetBlock(configuration,"Resampler", resampler, 1, 1, queue),
    		"SignalConditioner", "Signal_Conditioner", queue);
}



GNSSBlockInterface* GNSSBlockFactory::GetObservables(
        ConfigurationInterface *configuration, boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "GPS_L1_CA_Observables";
    std::string implementation = configuration->property(
            "Observables.implementation", default_implementation);
    DLOG(INFO) << "Getting Observables with implementation "
            << implementation;
    unsigned int channel_count =
            configuration->property("Channels.count", 12);
    return GetBlock(configuration, "Observables", implementation,
            channel_count, channel_count, queue);
}



GNSSBlockInterface* GNSSBlockFactory::GetPVT(
        ConfigurationInterface *configuration, boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "Pass_Through";
    std::string implementation = configuration->property(
            "PVT.implementation", default_implementation);
    DLOG(INFO) << "Getting PVT with implementation " << implementation;
    unsigned int channel_count = configuration->property("Channels.count", 12);
    return GetBlock(configuration, "PVT", implementation, channel_count, 1,
            queue);
}



GNSSBlockInterface* GNSSBlockFactory::GetOutputFilter(
        ConfigurationInterface *configuration, boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "Null_Sink_Output_Filter";
    std::string implementation = configuration->property(
            "OutputFilter.implementation", default_implementation);
    DLOG(INFO) << "Getting OutputFilter with implementation " << implementation;
    return GetBlock(configuration, "OutputFilter", implementation, 1, 0,
            queue);
}


GNSSBlockInterface* GNSSBlockFactory::GetChannel(
        ConfigurationInterface *configuration, std::string acq,
        std::string trk, std::string tlm, int channel,
        boost::shared_ptr<gr::msg_queue> queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    DLOG(INFO) << "Instantiating channel " << id;
    DLOG(INFO) << "Getting Channel " << id << " with" << std::endl << " Acquisition Implementation: "
    		<< acq << std::endl << " Tracking Implementation: " << trk << std::endl << " Telemetry "
    		"Decoder implementation: " << tlm;

    return new Channel(configuration, channel, GetBlock(configuration,
            "Channel", "Pass_Through", 1, 1, queue),
            (AcquisitionInterface*)GetBlock(configuration, "Acquisition",
                    acq, 1, 1, queue), (TrackingInterface*)GetBlock(
                            configuration, "Tracking", trk, 1, 1, queue),
                            (TelemetryDecoderInterface*)GetBlock(configuration,
                                    "TelemetryDecoder", tlm, 1, 1, queue), "Channel",
                                    "Channel", queue);
}



std::vector<GNSSBlockInterface*>* GNSSBlockFactory::GetChannels(
        ConfigurationInterface *configuration, boost::shared_ptr<gr::msg_queue> queue)
{
    std::string default_implementation = "Pass_Through";
    unsigned int channel_count =
            configuration->property("Channels.count", 12);
    DLOG(INFO) << "Getting " << channel_count << " channels";
    std::vector<GNSSBlockInterface*>* channels = new std::vector<
            GNSSBlockInterface*>();
    std::string tracking = configuration->property("Tracking.implementation",
            default_implementation);
    std::string telemetry_decoder = configuration->property(
            "TelemetryDecoder.implementation", default_implementation);
    std::string acquisition_implementation = configuration->property(
    		"Acquisition.implementation", default_implementation);

    for (unsigned int i = 0; i < channel_count; i++)
        {
            std::string acquisition_implementation_specific = configuration->property(
            		"Acquisition"+ boost::lexical_cast<std::string>(i) + ".implementation",
            		default_implementation);
            if(acquisition_implementation_specific.compare(default_implementation) != 0)
            {
            	acquisition_implementation = acquisition_implementation_specific;
            }
            channels->push_back(GetChannel(configuration,
                    acquisition_implementation, tracking, telemetry_decoder, i,
                    queue));
        }
    return channels;
}


    /*
     * Returns the block with the required configuration and implementation
     *
     * PLEASE ADD YOUR NEW BLOCK HERE!!
     */
GNSSBlockInterface* GNSSBlockFactory::GetBlock(
        ConfigurationInterface *configuration, std::string role,
        std::string implementation, unsigned int in_streams,
        unsigned int out_streams, boost::shared_ptr<gr::msg_queue> queue)
{
    GNSSBlockInterface* block = NULL; //Change to nullptr when available in compilers (C++11)

    //PASS THROUGH ----------------------------------------------------------------
    if (implementation.compare("Pass_Through") == 0)
        {
            block = new Pass_Through(configuration, role, in_streams, out_streams);
        }

    // SIGNAL SOURCES -------------------------------------------------------------
    else if (implementation.compare("File_Signal_Source") == 0)
        {
            try
            {
                    block = new FileSignalSource(configuration, role, in_streams,
                            out_streams, queue);
            }
            catch (const std::exception &e)
            {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    LOG_AT_LEVEL(INFO) << implementation
                            << ": Source file not found";
                    exit(1);
            }
        }
    else if (implementation.compare("UHD_Signal_Source") == 0)
        {
            block = new UhdSignalSource(configuration, role, in_streams,
                    out_streams, queue);
        }
#if GN3S_DRIVER
    else if (implementation.compare("GN3S_Signal_Source") == 0)
        {
            block = new Gn3sSignalSource(configuration, role, in_streams,
                    out_streams, queue);
        }
#endif

#if RTLSDR_DRIVER
    else if (implementation.compare("Rtlsdr_Signal_Source") == 0)
        {
            block = new RtlsdrSignalSource(configuration, role, in_streams,
                    out_streams, queue);
        }
#endif

    // DATA TYPE ADAPTER -----------------------------------------------------------
    else if (implementation.compare("Ishort_To_Complex") == 0)
        {
            block = new IshortToComplex(configuration, role, in_streams,
                    out_streams, queue);
        }

    // INPUT FILTER ----------------------------------------------------------------
    else if (implementation.compare("Fir_Filter") == 0)
        {
            block = new FirFilter(configuration, role, in_streams,
                    out_streams, queue);
        }
    else if (implementation.compare("Freq_Xlating_Fir_Filter") == 0)
        {
            block = new FreqXlatingFirFilter(configuration, role, in_streams,
                    out_streams, queue);
        }

    // RESAMPLER -------------------------------------------------------------------
    else if (implementation.compare("Direct_Resampler") == 0)
        {
            block = new DirectResamplerConditioner(configuration, role,
                    in_streams, out_streams);
        }

    // ACQUISITION BLOCKS ---------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_PCPS_Acquisition") == 0)
        {
            block = new GpsL1CaPcpsAcquisition(configuration, role, in_streams,
                    out_streams, queue);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_Assisted_Acquisition") == 0)
        {
            block = new GpsL1CaPcpsAssistedAcquisition(configuration, role, in_streams,
                    out_streams, queue);
        }
    else if (implementation.compare("Galileo_E1_PCPS_Ambiguous_Acquisition") == 0)
        {
            block = new GalileoE1PcpsAmbiguousAcquisition(configuration, role, in_streams,
                    out_streams, queue);
        }

    // TRACKING BLOCKS -------------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking") == 0)
        {
            block = new GpsL1CaDllPllTracking(configuration, role, in_streams,
                    out_streams, queue);
        }
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_Optim_Tracking") == 0)
        {
            block = new GpsL1CaDllPllOptimTracking(configuration, role, in_streams,
                    out_streams, queue);
        }
    else if (implementation.compare("GPS_L1_CA_DLL_FLL_PLL_Tracking") == 0)
        {
            block = new GpsL1CaDllFllPllTracking(configuration, role, in_streams,
                    out_streams, queue);
        }
    else if (implementation.compare("GPS_L1_CA_TCP_CONNECTOR_Tracking") == 0)
        {
            block = new GpsL1CaTcpConnectorTracking(configuration, role, in_streams,
                    out_streams, queue);
        }
    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking") == 0)
        {
            block = new GalileoE1DllPllVemlTracking(configuration, role, in_streams,
                    out_streams, queue);
        }
    else if (implementation.compare("Galileo_E1_TCP_CONNECTOR_Tracking") == 0)
        {
            block = new GalileoE1TcpConnectorTracking(configuration, role, in_streams,
                    out_streams, queue);
        }

    // TELEMETRY DECODERS ----------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_Telemetry_Decoder") == 0)
        {
            block = new GpsL1CaTelemetryDecoder(configuration, role, in_streams,
                    out_streams, queue);
        }
    else if (implementation.compare("Galileo_E1B_Telemetry_Decoder") == 0)
        {
            block = new GalileoE1BTelemetryDecoder(configuration, role, in_streams,
                    out_streams, queue);
        }
    // OBSERVABLES -----------------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_Observables") == 0)
        {
            block = new GpsL1CaObservables(configuration, role, in_streams,
                    out_streams, queue);
        }

    // PVT -------------------------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_PVT") == 0)
        {
            block = new GpsL1CaPvt(configuration, role, in_streams,
                    out_streams, queue);
        }

    // OUTPUT FILTERS --------------------------------------------------------------
    else if (implementation.compare("Null_Sink_Output_Filter") == 0)
        {
            block = new NullSinkOutputFilter(configuration, role, in_streams,
                    out_streams);
        }
    else if (implementation.compare("File_Output_Filter") == 0)
        {
            block = new FileOutputFilter(configuration, role, in_streams,
                    out_streams);
        }
    else
        {
            // Log fatal. This causes execution to stop.
            LOG_AT_LEVEL(ERROR) << implementation
                    << ": Undefined implementation for block";
        }
    return block;
}
