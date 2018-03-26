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
#include "configuration_interface.h"
#include "in_memory_configuration.h"
#include "gnss_block_interface.h"
#include "pass_through.h"
#include "file_signal_source.h"
#include "nsr_file_signal_source.h"
#include "two_bit_cpx_file_signal_source.h"
#include "spir_file_signal_source.h"
#include "spir_gss6450_file_signal_source.h"
#include "rtl_tcp_signal_source.h"
#include "two_bit_packed_file_signal_source.h"
#include "labsat_signal_source.h"
#include "channel.h"
#include "signal_conditioner.h"
#include "array_signal_conditioner.h"
#include "byte_to_short.h"
#include "ibyte_to_cbyte.h"
#include "ibyte_to_cshort.h"
#include "ibyte_to_complex.h"
#include "ishort_to_cshort.h"
#include "ishort_to_complex.h"
#include "direct_resampler_conditioner.h"
#include "mmse_resampler_conditioner.h"
#include "fir_filter.h"
#include "freq_xlating_fir_filter.h"
#include "beamformer_filter.h"
#include "pulse_blanking_filter.h"
#include "notch_filter.h"
#include "notch_filter_lite.h"
#include "gps_l1_ca_pcps_acquisition.h"
#include "gps_l2_m_pcps_acquisition.h"
#include "gps_l5i_pcps_acquisition.h"
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
#include "galileo_e5a_pcps_acquisition.h"
#include "glonass_l1_ca_pcps_acquisition.h"
#include "glonass_l2_ca_pcps_acquisition.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "gps_l1_ca_dll_pll_c_aid_tracking.h"
#include "gps_l1_ca_tcp_connector_tracking.h"
#include "galileo_e1_dll_pll_veml_tracking.h"
#include "galileo_e1_tcp_connector_tracking.h"
#include "galileo_e5a_dll_pll_tracking.h"
#include "gps_l2_m_dll_pll_tracking.h"
#include "glonass_l1_ca_dll_pll_tracking.h"
#include "glonass_l1_ca_dll_pll_c_aid_tracking.h"
#include "glonass_l2_ca_dll_pll_tracking.h"
#include "glonass_l2_ca_dll_pll_c_aid_tracking.h"
#include "gps_l5i_dll_pll_tracking.h"
#include "gps_l1_ca_telemetry_decoder.h"
#include "gps_l2c_telemetry_decoder.h"
#include "gps_l5_telemetry_decoder.h"
#include "galileo_e1b_telemetry_decoder.h"
#include "galileo_e5a_telemetry_decoder.h"
#include "glonass_l1_ca_telemetry_decoder.h"
#include "glonass_l2_ca_telemetry_decoder.h"
#include "sbas_l1_telemetry_decoder.h"
#include "hybrid_observables.h"
#include "rtklib_pvt.h"

#if ENABLE_FPGA
#include "gps_l1_ca_dll_pll_c_aid_tracking_fpga.h"
#include "gps_l1_ca_pcps_acquisition_fpga.h"
#endif

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

#if PLUTOSDR_DRIVER
#include "plutosdr_signal_source.h"
#endif

#if FMCOMMS2_DRIVER
#include "fmcomms2_signal_source.h"
#endif

#if FLEXIBAND_DRIVER
#include "flexiband_signal_source.h"
#endif

#if CUDA_GPU_ACCEL
#include "gps_l1_ca_dll_pll_tracking_gpu.h"
#endif

#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <string>
#include <sstream>
#include <iostream>


using google::LogMessage;


GNSSBlockFactory::GNSSBlockFactory() {}


GNSSBlockFactory::~GNSSBlockFactory() {}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetSignalSource(
    std::shared_ptr<ConfigurationInterface> configuration, gr::msg_queue::sptr queue, int ID)
{
    std::string default_implementation = "File_Signal_Source";
    std::string role = "SignalSource";  //backwards compatibility for old conf files
    if (ID != -1)
        {
            role = "SignalSource" + boost::lexical_cast<std::string>(ID);
        }
    std::string implementation = configuration->property(role + ".implementation", default_implementation);
    LOG(INFO) << "Getting SignalSource with implementation " << implementation;
    return GetBlock(configuration, role, implementation, 0, 1, queue);
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetSignalConditioner(
    std::shared_ptr<ConfigurationInterface> configuration, int ID)
{
    std::string default_implementation = "Pass_Through";
    //backwards compatibility for old conf files
    std::string role_conditioner = "SignalConditioner";
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

    std::string signal_conditioner = configuration->property(role_conditioner + ".implementation", default_implementation);

    std::string data_type_adapter;
    std::string input_filter;
    std::string resampler;
    if (signal_conditioner.compare("Pass_Through") == 0)
        {
            data_type_adapter = "Pass_Through";
            input_filter = "Pass_Through";
            resampler = "Pass_Through";
        }
    else
        {
            data_type_adapter = configuration->property(role_datatypeadapter + ".implementation", default_implementation);
            input_filter = configuration->property(role_inputfilter + ".implementation", default_implementation);
            resampler = configuration->property(role_resampler + ".implementation", default_implementation);
        }

    LOG(INFO) << "Getting SignalConditioner with DataTypeAdapter implementation: "
              << data_type_adapter << ", InputFilter implementation: "
              << input_filter << ", and Resampler implementation: "
              << resampler;

    if (signal_conditioner.compare("Array_Signal_Conditioner") == 0)
        {
            //instantiate the array version
            std::unique_ptr<GNSSBlockInterface> conditioner_(new ArraySignalConditioner(configuration.get(),
                std::move(GetBlock(configuration, role_datatypeadapter, data_type_adapter, 1, 1)),
                std::move(GetBlock(configuration, role_inputfilter, input_filter, 1, 1)),
                std::move(GetBlock(configuration, role_resampler, resampler, 1, 1)),
                role_conditioner, "Signal_Conditioner"));
            return conditioner_;
        }
    else
        {
            //single-antenna version
            std::unique_ptr<GNSSBlockInterface> conditioner_(new SignalConditioner(configuration.get(),
                std::move(GetBlock(configuration, role_datatypeadapter, data_type_adapter, 1, 1)),
                std::move(GetBlock(configuration, role_inputfilter, input_filter, 1, 1)),
                std::move(GetBlock(configuration, role_resampler, resampler, 1, 1)),
                role_conditioner, "Signal_Conditioner"));
            return conditioner_;
        }
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetObservables(std::shared_ptr<ConfigurationInterface> configuration)
{
    std::string default_implementation = "Hybrid_Observables";
    std::string implementation = configuration->property("Observables.implementation", default_implementation);
    LOG(INFO) << "Getting Observables with implementation " << implementation;
    unsigned int Galileo_channels = configuration->property("Channels_1B.count", 0);
    Galileo_channels += configuration->property("Channels_5X.count", 0);
    unsigned int GPS_channels = configuration->property("Channels_1C.count", 0);
    GPS_channels += configuration->property("Channels_2S.count", 0);
    GPS_channels += configuration->property("Channels_L5.count", 0);
    unsigned int Glonass_channels = configuration->property("Channels_1G.count", 0);
    Glonass_channels += configuration->property("Channels_2G.count", 0);
    return GetBlock(configuration, "Observables", implementation, Galileo_channels + GPS_channels + Glonass_channels, Galileo_channels + GPS_channels + Glonass_channels);
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetPVT(std::shared_ptr<ConfigurationInterface> configuration)
{
    std::string default_implementation = "RTKLIB_PVT";
    std::string implementation = configuration->property("PVT.implementation", default_implementation);
    LOG(INFO) << "Getting PVT with implementation " << implementation;
    unsigned int Galileo_channels = configuration->property("Channels_1B.count", 0);
    Galileo_channels += configuration->property("Channels_5X.count", 0);
    unsigned int GPS_channels = configuration->property("Channels_1C.count", 0);
    GPS_channels += configuration->property("Channels_2S.count", 0);
    GPS_channels += configuration->property("Channels_L5.count", 0);
    unsigned int Glonass_channels = configuration->property("Channels_1G.count", 0);
    Glonass_channels += configuration->property("Channels_2G.count", 0);
    return GetBlock(configuration, "PVT", implementation, Galileo_channels + GPS_channels + Glonass_channels, 0);
}


//********* GPS L1 C/A CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_1C(
    std::shared_ptr<ConfigurationInterface> configuration,
    std::string acq, std::string trk, std::string tlm, int channel,
    gr::msg_queue::sptr queue)
{
    //"appendix" is added to the "role" with the aim of Acquisition, Tracking and Telemetry Decoder adapters
    //can find their specific configurations when they read the config
    //TODO: REMOVE APPENDIX!! AND CHECK ALTERNATIVE MECHANISM TO GET PARTICULARIZED PARAMETERS
    LOG(INFO) << "Instantiating Channel " << channel << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;

    std::string aux = configuration->property("Acquisition_1C" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux.compare("W") != 0)
        {
            appendix1 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_1C" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux.compare("W") != 0)
        {
            appendix2 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_1C" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux.compare("W") != 0)
        {
            appendix3 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::shared_ptr<InMemoryConfiguration> config;
    config = std::make_shared<InMemoryConfiguration>();
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_1C" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_1C" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type.compare(trk_item_type))
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }
    config->set_property("Channel.item_type", acq_item_type);

    std::unique_ptr<GNSSBlockInterface> pass_through_ = GetBlock(config, "Channel", "Pass_Through", 1, 1, queue);
    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_1C" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_1C" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_1C" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_(new Channel(configuration.get(), channel, std::move(pass_through_),
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "1C", queue));

    return channel_;
}


//********* GPS L2C (M) CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_2S(
    std::shared_ptr<ConfigurationInterface> configuration,
    std::string acq, std::string trk, std::string tlm, int channel,
    gr::msg_queue::sptr queue)
{
    LOG(INFO) << "Instantiating Channel " << channel << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;
    std::string aux = configuration->property("Acquisition_2S" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux.compare("W") != 0)
        {
            appendix1 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_2S" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux.compare("W") != 0)
        {
            appendix2 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_2S" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux.compare("W") != 0)
        {
            appendix3 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::shared_ptr<InMemoryConfiguration> config;
    config = std::make_shared<InMemoryConfiguration>();
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_2S" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_2S" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type.compare(trk_item_type))
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }
    config->set_property("Channel.item_type", acq_item_type);

    std::unique_ptr<GNSSBlockInterface> pass_through_ = GetBlock(configuration, "Channel", "Pass_Through", 1, 1, queue);
    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_2S" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_2S" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_2S" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_(new Channel(configuration.get(), channel, std::move(pass_through_),
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "2S", queue));

    return channel_;
}


//********* GALILEO E1 B CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_1B(
    std::shared_ptr<ConfigurationInterface> configuration,
    std::string acq, std::string trk, std::string tlm, int channel,
    gr::msg_queue::sptr queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << id << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;
    std::string aux = configuration->property("Acquisition_1B" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux.compare("W") != 0)
        {
            appendix1 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_1B" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux.compare("W") != 0)
        {
            appendix2 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_1B" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux.compare("W") != 0)
        {
            appendix3 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::shared_ptr<InMemoryConfiguration> config;
    config = std::make_shared<InMemoryConfiguration>();
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_1B" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_1B" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type.compare(trk_item_type))
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }
    config->set_property("Channel.item_type", acq_item_type);

    std::unique_ptr<GNSSBlockInterface> pass_through_ = GetBlock(configuration, "Channel", "Pass_Through", 1, 1, queue);
    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_1B" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_1B" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_1B" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_(new Channel(configuration.get(), channel, std::move(pass_through_),
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "1B", queue));

    return channel_;
}


//********* GALILEO E5a  CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_5X(
    std::shared_ptr<ConfigurationInterface> configuration,
    std::string acq, std::string trk, std::string tlm, int channel,
    gr::msg_queue::sptr queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << id << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;
    std::string aux = configuration->property("Acquisition_5X" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux.compare("W") != 0)
        {
            appendix1 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_5X" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux.compare("W") != 0)
        {
            appendix2 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_5X" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux.compare("W") != 0)
        {
            appendix3 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::shared_ptr<InMemoryConfiguration> config;
    config = std::make_shared<InMemoryConfiguration>();
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_5X" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_5X" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type.compare(trk_item_type))
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }
    config->set_property("Channel.item_type", acq_item_type);

    std::unique_ptr<GNSSBlockInterface> pass_through_ = GetBlock(configuration, "Channel", "Pass_Through", 1, 1, queue);
    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_5X" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_5X" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_5X" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_(new Channel(configuration.get(), channel, std::move(pass_through_),
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "5X", queue));

    return channel_;
}


//********* GLONASS L1 C/A CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_1G(
    std::shared_ptr<ConfigurationInterface> configuration,
    std::string acq, std::string trk, std::string tlm, int channel,
    boost::shared_ptr<gr::msg_queue> queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << channel << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder Implementation: " << tlm;

    std::string aux = configuration->property("Acquisition_1G" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux.compare("W") != 0)
        {
            appendix1 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_1G" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux.compare("W") != 0)
        {
            appendix2 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_1G" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux.compare("W") != 0)
        {
            appendix3 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::shared_ptr<InMemoryConfiguration> config;
    config = std::make_shared<InMemoryConfiguration>();
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_1G" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_1G" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type.compare(trk_item_type))
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }
    config->set_property("Channel.item_type", acq_item_type);

    std::unique_ptr<GNSSBlockInterface> pass_through_ = GetBlock(config, "Channel", "Pass_Through", 1, 1, queue);
    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_1G" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_1G" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_1G" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_(new Channel(configuration.get(), channel, std::move(pass_through_),
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "1G", queue));

    return channel_;
}



//********* GLONASS L2 C/A CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_2G(
    std::shared_ptr<ConfigurationInterface> configuration,
    std::string acq, std::string trk, std::string tlm, int channel,
    boost::shared_ptr<gr::msg_queue> queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << channel << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder Implementation: " << tlm;

    std::string aux = configuration->property("Acquisition_2G" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux.compare("W") != 0)
        {
            appendix1 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_2G" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux.compare("W") != 0)
        {
            appendix2 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_2G" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux.compare("W") != 0)
        {
            appendix3 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::shared_ptr<InMemoryConfiguration> config;
    config = std::make_shared<InMemoryConfiguration>();
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_2G" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_2G" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type.compare(trk_item_type))
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }
    config->set_property("Channel.item_type", acq_item_type);

    std::unique_ptr<GNSSBlockInterface> pass_through_ = GetBlock(config, "Channel", "Pass_Through", 1, 1, queue);
    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_2G" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_2G" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_2G" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_(new Channel(configuration.get(), channel, std::move(pass_through_),
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "2G", queue));

    return channel_;
}



//********* GPS L5  CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_L5(
    std::shared_ptr<ConfigurationInterface> configuration,
    std::string acq, std::string trk, std::string tlm, int channel,
    gr::msg_queue::sptr queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << id << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;
    std::string aux = configuration->property("Acquisition_L5" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux.compare("W") != 0)
        {
            appendix1 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_L5" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux.compare("W") != 0)
        {
            appendix2 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_L5" + boost::lexical_cast<std::string>(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux.compare("W") != 0)
        {
            appendix3 = boost::lexical_cast<std::string>(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::shared_ptr<InMemoryConfiguration> config;
    config = std::make_shared<InMemoryConfiguration>();
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_L5" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_L5" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type.compare(trk_item_type))
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }
    config->set_property("Channel.item_type", acq_item_type);

    std::unique_ptr<GNSSBlockInterface> pass_through_ = GetBlock(configuration, "Channel", "Pass_Through", 1, 1, queue);
    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_L5" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_L5" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_L5" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_(new Channel(configuration.get(), channel, std::move(pass_through_),
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "L5", queue));

    return channel_;
}


std::unique_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> GNSSBlockFactory::GetChannels(
    std::shared_ptr<ConfigurationInterface> configuration, gr::msg_queue::sptr queue)
{
    std::string default_implementation = "Pass_Through";
    std::string tracking_implementation;
    std::string telemetry_decoder_implementation;
    std::string acquisition_implementation;

    unsigned int channel_absolute_id = 0;

    unsigned int Channels_1C_count = configuration->property("Channels_1C.count", 0);
    unsigned int Channels_2S_count = configuration->property("Channels_2S.count", 0);
    unsigned int Channels_1B_count = configuration->property("Channels_1B.count", 0);
    unsigned int Channels_5X_count = configuration->property("Channels_5X.count", 0);
    unsigned int Channels_1G_count = configuration->property("Channels_1G.count", 0);
    unsigned int Channels_2G_count = configuration->property("Channels_2G.count", 0);
    unsigned int Channels_L5_count = configuration->property("Channels_L5.count", 0);

    unsigned int total_channels = Channels_1C_count +
                                  Channels_2S_count +
                                  Channels_1B_count +
                                  Channels_5X_count +
                                  Channels_1G_count +
								  Channels_2G_count +
                                  Channels_L5_count;

    std::unique_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> channels(new std::vector<std::unique_ptr<GNSSBlockInterface>>(total_channels));

    //**************** GPS L1 C/A  CHANNELS **********************
    LOG(INFO) << "Getting " << Channels_1C_count << " GPS L1 C/A channels";
    acquisition_implementation = configuration->property("Acquisition_1C.implementation", default_implementation);
    tracking_implementation = configuration->property("Tracking_1C.implementation", default_implementation);
    telemetry_decoder_implementation = configuration->property("TelemetryDecoder_1C.implementation", default_implementation);

    for (unsigned int i = 0; i < Channels_1C_count; i++)
        {
            //(i.e. Acquisition_1C0.implementation=xxxx)
            std::string acquisition_implementation_specific = configuration->property(
                "Acquisition_1C" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                acquisition_implementation);
            //(i.e. Tracking_1C0.implementation=xxxx)
            std::string tracking_implementation_specific = configuration->property(
                "Tracking_1C" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                tracking_implementation);
            std::string telemetry_decoder_implementation_specific = configuration->property(
                "TelemetryDecoder_1C" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                telemetry_decoder_implementation);

            // Push back the channel to the vector of channels
            channels->at(channel_absolute_id) = std::move(GetChannel_1C(configuration,
                acquisition_implementation_specific,
                tracking_implementation_specific,
                telemetry_decoder_implementation_specific,
                channel_absolute_id,
                queue));
            channel_absolute_id++;
        }

    //**************** GPS L2C (M)  CHANNELS **********************
    LOG(INFO) << "Getting " << Channels_2S_count << " GPS L2C (M) channels";
    tracking_implementation = configuration->property("Tracking_2S.implementation", default_implementation);
    telemetry_decoder_implementation = configuration->property("TelemetryDecoder_2S.implementation", default_implementation);
    acquisition_implementation = configuration->property("Acquisition_2S.implementation", default_implementation);
    for (unsigned int i = 0; i < Channels_2S_count; i++)
        {
            //(i.e. Acquisition_1C0.implementation=xxxx)
            std::string acquisition_implementation_specific = configuration->property(
                "Acquisition_2S" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                acquisition_implementation);
            //(i.e. Tracking_1C0.implementation=xxxx)
            std::string tracking_implementation_specific = configuration->property(
                "Tracking_2S" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                tracking_implementation);
            std::string telemetry_decoder_implementation_specific = configuration->property(
                "TelemetryDecoder_2S" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                telemetry_decoder_implementation);

            // Push back the channel to the vector of channels
            channels->at(channel_absolute_id) = std::move(GetChannel_2S(configuration,
                acquisition_implementation_specific,
                tracking_implementation_specific,
                telemetry_decoder_implementation_specific,
                channel_absolute_id,
                queue));
            channel_absolute_id++;
        }

    //**************** GPS L5  CHANNELS **********************
    LOG(INFO) << "Getting " << Channels_L5_count << " GPS L5 channels";
    tracking_implementation = configuration->property("Tracking_L5.implementation", default_implementation);
    telemetry_decoder_implementation = configuration->property("TelemetryDecoder_L5.implementation", default_implementation);
    acquisition_implementation = configuration->property("Acquisition_L5.implementation", default_implementation);
    for (unsigned int i = 0; i < Channels_L5_count; i++)
        {
            //(i.e. Acquisition_1C0.implementation=xxxx)
            std::string acquisition_implementation_specific = configuration->property(
                "Acquisition_L5" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                acquisition_implementation);
            //(i.e. Tracking_1C0.implementation=xxxx)
            std::string tracking_implementation_specific = configuration->property(
                "Tracking_L5" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                tracking_implementation);
            std::string telemetry_decoder_implementation_specific = configuration->property(
                "TelemetryDecoder_L5" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                telemetry_decoder_implementation);

            // Push back the channel to the vector of channels
            channels->at(channel_absolute_id) = std::move(GetChannel_L5(configuration,
                acquisition_implementation_specific,
                tracking_implementation_specific,
                telemetry_decoder_implementation_specific,
                channel_absolute_id,
                queue));
            channel_absolute_id++;
        }

    //**************** GALILEO E1 B (I/NAV OS) CHANNELS **********************
    LOG(INFO) << "Getting " << Channels_1B_count << " GALILEO E1 B (I/NAV OS) channels";
    tracking_implementation = configuration->property("Tracking_1B.implementation", default_implementation);
    telemetry_decoder_implementation = configuration->property("TelemetryDecoder_1B.implementation", default_implementation);
    acquisition_implementation = configuration->property("Acquisition_1B.implementation", default_implementation);
    for (unsigned int i = 0; i < Channels_1B_count; i++)
        {
            //(i.e. Acquisition_1C0.implementation=xxxx)
            std::string acquisition_implementation_specific = configuration->property(
                "Acquisition_1B" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                acquisition_implementation);
            //(i.e. Tracking_1C0.implementation=xxxx)
            std::string tracking_implementation_specific = configuration->property(
                "Tracking_1B" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                tracking_implementation);
            std::string telemetry_decoder_implementation_specific = configuration->property(
                "TelemetryDecoder_1B" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                telemetry_decoder_implementation);

            // Push back the channel to the vector of channels
            channels->at(channel_absolute_id) = std::move(GetChannel_1B(configuration,
                acquisition_implementation_specific,
                tracking_implementation_specific,
                telemetry_decoder_implementation_specific,
                channel_absolute_id,
                queue));
            channel_absolute_id++;
        }

    //**************** GALILEO E5a I (F/NAV OS)  CHANNELS **********************
    LOG(INFO) << "Getting " << Channels_5X_count << " GALILEO E5a I (F/NAV OS) channels";
    tracking_implementation = configuration->property("Tracking_5X.implementation", default_implementation);
    telemetry_decoder_implementation = configuration->property("TelemetryDecoder_5X.implementation", default_implementation);
    acquisition_implementation = configuration->property("Acquisition_5X.implementation", default_implementation);
    for (unsigned int i = 0; i < Channels_5X_count; i++)
        {
            //(i.e. Acquisition_1C0.implementation=xxxx)
            std::string acquisition_implementation_specific = configuration->property(
                "Acquisition_5X" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                acquisition_implementation);
            //(i.e. Tracking_1C0.implementation=xxxx)
            std::string tracking_implementation_specific = configuration->property(
                "Tracking_5X" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                tracking_implementation);
            std::string telemetry_decoder_implementation_specific = configuration->property(
                "TelemetryDecoder_5X" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                telemetry_decoder_implementation);

            // Push back the channel to the vector of channels
            channels->at(channel_absolute_id) = std::move(GetChannel_5X(configuration,
                acquisition_implementation_specific,
                tracking_implementation_specific,
                telemetry_decoder_implementation_specific,
                channel_absolute_id,
                queue));
            channel_absolute_id++;
        }

    //**************** GLONASS L1 C/A  CHANNELS **********************
    LOG(INFO) << "Getting " << Channels_1G_count << " GLONASS L1 C/A channels";
    acquisition_implementation = configuration->property("Acquisition_1G.implementation", default_implementation);
    tracking_implementation = configuration->property("Tracking_1G.implementation", default_implementation);
    telemetry_decoder_implementation = configuration->property("TelemetryDecoder_1G.implementation", default_implementation);

    for (unsigned int i = 0; i < Channels_1G_count; i++)
        {
            //(i.e. Acquisition_1G0.implementation=xxxx)
            std::string acquisition_implementation_specific = configuration->property(
                "Acquisition_1G" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                acquisition_implementation);
            //(i.e. Tracking_1G0.implementation=xxxx)
            std::string tracking_implementation_specific = configuration->property(
                "Tracking_1G" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                tracking_implementation);
            std::string telemetry_decoder_implementation_specific = configuration->property(
                "TelemetryDecoder_1G" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                telemetry_decoder_implementation);

            // Push back the channel to the vector of channels
            channels->at(channel_absolute_id) = std::move(GetChannel_1G(configuration,
                acquisition_implementation_specific,
                tracking_implementation_specific,
                telemetry_decoder_implementation_specific,
                channel_absolute_id,
                queue));
            channel_absolute_id++;
        }

    //**************** GLONASS L2 C/A  CHANNELS **********************
    LOG(INFO) << "Getting " << Channels_2G_count << " GLONASS L2 C/A channels";
    acquisition_implementation = configuration->property("Acquisition_2G.implementation", default_implementation);
    tracking_implementation = configuration->property("Tracking_2G.implementation", default_implementation);
    telemetry_decoder_implementation = configuration->property("TelemetryDecoder_2G.implementation", default_implementation);

    for (unsigned int i = 0; i < Channels_2G_count; i++)
        {
            //(i.e. Acquisition_2G0.implementation=xxxx)
            std::string acquisition_implementation_specific = configuration->property(
                "Acquisition_2G" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                acquisition_implementation);
            //(i.e. Tracking_2G0.implementation=xxxx)
            std::string tracking_implementation_specific = configuration->property(
                "Tracking_2G" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                tracking_implementation);
            std::string telemetry_decoder_implementation_specific = configuration->property(
                "TelemetryDecoder_2G" + boost::lexical_cast<std::string>(channel_absolute_id) + ".implementation",
                telemetry_decoder_implementation);

            // Push back the channel to the vector of channels
            channels->at(channel_absolute_id) = std::move(GetChannel_2G(configuration,
                acquisition_implementation_specific,
                tracking_implementation_specific,
                telemetry_decoder_implementation_specific,
                channel_absolute_id,
                queue));
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
    unsigned int out_streams, gr::msg_queue::sptr queue)
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
#if MODERN_GNURADIO
    else if (implementation.compare("Two_Bit_Cpx_File_Signal_Source") == 0)
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_(new TwoBitCpxFileSignalSource(configuration.get(), role, in_streams,
                        out_streams, queue));
                    block = std::move(block_);
                }
            catch (const std::exception &e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
    else if (implementation.compare("Two_Bit_Packed_File_Signal_Source") == 0)
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_(new TwoBitPackedFileSignalSource(configuration.get(), role, in_streams,
                        out_streams, queue));
                    block = std::move(block_);
                }
            catch (const std::exception &e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
#endif
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
    else if (implementation.compare("Spir_GSS6450_File_Signal_Source") == 0)
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_(new SpirGSS6450FileSignalSource(configuration.get(), role, in_streams,
                        out_streams, queue));
                    block = std::move(block_);
                }
            catch (const std::exception &e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
    else if (implementation.compare("RtlTcp_Signal_Source") == 0)
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_(new RtlTcpSignalSource(configuration.get(), role, in_streams,
                        out_streams, queue));
                    block = std::move(block_);
                }
            catch (const std::exception &e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
    else if (implementation.compare("Labsat_Signal_Source") == 0)
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_(new LabsatSignalSource(configuration.get(), role, in_streams,
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

#if PLUTOSDR_DRIVER
    else if (implementation.compare("Plutosdr_Signal_Source") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new PlutosdrSignalSource(configuration.get(), role, in_streams,
                out_streams, queue));
            block = std::move(block_);
        }
#endif

#if FMCOMMS2_DRIVER
    else if (implementation.compare("Fmcomms2_Signal_Source") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new Fmcomms2SignalSource(configuration.get(), role, in_streams,
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
            std::unique_ptr<GNSSBlockInterface> block_(new ByteToShort(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Ibyte_To_Cbyte") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new IbyteToCbyte(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Ibyte_To_Cshort") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new IbyteToCshort(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Ibyte_To_Complex") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new IbyteToComplex(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Ishort_To_Cshort") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new IshortToCshort(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Ishort_To_Complex") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new IshortToComplex(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }

    // INPUT FILTER ----------------------------------------------------------------
    else if (implementation.compare("Fir_Filter") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new FirFilter(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Freq_Xlating_Fir_Filter") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new FreqXlatingFirFilter(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Beamformer_Filter") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new BeamformerFilter(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Pulse_Blanking_Filter") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new PulseBlankingFilter(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Notch_Filter") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new NotchFilter(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Notch_Filter_Lite") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new NotchFilterLite(configuration.get(), role, in_streams,
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

    else if ((implementation.compare("Fractional_Resampler") == 0) || (implementation.compare("Mmse_Resampler") == 0))
        {
            std::unique_ptr<GNSSBlockInterface> block_(new MmseResamplerConditioner(configuration.get(), role,
                in_streams, out_streams));
            block = std::move(block_);
        }

    // ACQUISITION BLOCKS ---------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }

#if ENABLE_FPGA
    else if (implementation.compare("GPS_L1_CA_PCPS_Acquisition_Fpga") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsAcquisitionFpga(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#endif
    else if (implementation.compare("GPS_L1_CA_PCPS_Assisted_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsAssistedAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_Tong_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsTongAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }

#if OPENCL_BLOCKS
    else if (implementation.compare("GPS_L1_CA_PCPS_OpenCl_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsOpenClAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#endif

    else if (implementation.compare("GPS_L1_CA_PCPS_Acquisition_Fine_Doppler") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsAcquisitionFineDoppler(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_QuickSync_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaPcpsQuickSyncAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L2_M_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL2MPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L5i_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL5iPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1PcpsAmbiguousAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_8ms_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1Pcps8msAmbiguousAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_Tong_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1PcpsTongAmbiguousAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_CCCWSR_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1PcpsCccwsrAmbiguousAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_Noncoherent_IQ_Acquisition_CAF") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE5aNoncoherentIQAcquisitionCaf(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_Pcps_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE5aPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1PcpsQuickSyncAmbiguousAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L1_CA_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GlonassL1CaPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L2_CA_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GlonassL2CaPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    // TRACKING BLOCKS -------------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_C_Aid_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL1CaDllPllCAidTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_C_Aid_Tracking_Fpga") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL1CaDllPllCAidTrackingFpga(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#endif
    else if (implementation.compare("GPS_L1_CA_TCP_CONNECTOR_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaTcpConnectorTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L2_M_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL2MDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L5i_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL5iDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#if CUDA_GPU_ACCEL
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_GPU") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaDllPllTrackingGPU(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#endif
    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1DllPllVemlTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_TCP_CONNECTOR_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1TcpConnectorTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE5aDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L1_CA_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GlonassL1CaDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L1_CA_DLL_PLL_C_Aid_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GlonassL1CaDllPllCAidTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L2_CA_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GlonassL2CaDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L2_CA_DLL_PLL_C_Aid_Tracking") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GlonassL2CaDllPllCAidTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    // TELEMETRY DECODERS ----------------------------------------------------------
    else if (implementation.compare("GPS_L1_CA_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL1CaTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L2C_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL2CTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L5_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GpsL5TelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1B_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE1BTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("SBAS_L1_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new SbasL1TelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GalileoE5aTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L1_CA_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GlonassL1CaTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L2_CA_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<GNSSBlockInterface> block_(new GlonassL2CaTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    // OBSERVABLES -----------------------------------------------------------------
    else if ((implementation.compare("Hybrid_Observables") == 0) || (implementation.compare("GPS_L1_CA_Observables") == 0) || (implementation.compare("GPS_L2C_Observables") == 0) ||
             (implementation.compare("Galileo_E5A_Observables") == 0))
        {
            std::unique_ptr<GNSSBlockInterface> block_(new HybridObservables(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }

    // PVT -------------------------------------------------------------------------
    else if ((implementation.compare("RTKLIB_PVT") == 0) || (implementation.compare("GPS_L1_CA_PVT") == 0) || (implementation.compare("Galileo_E1_PVT") == 0) || (implementation.compare("Hybrid_PVT") == 0))
        {
            std::unique_ptr<GNSSBlockInterface> block_(new RtklibPvt(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role << "." << implementation << ": Undefined implementation for block";
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
    unsigned int out_streams)
{
    std::unique_ptr<AcquisitionInterface> block;
    // ACQUISITION BLOCKS ---------------------------------------------------------
    if (implementation.compare("GPS_L1_CA_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation.compare("GPS_L1_CA_PCPS_Acquisition_Fpga") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsAcquisitionFpga(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#endif
    else if (implementation.compare("GPS_L1_CA_PCPS_Assisted_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsAssistedAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_Tong_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsTongAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }

#if OPENCL_BLOCKS
    else if (implementation.compare("GPS_L1_CA_PCPS_OpenCl_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsOpenClAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#endif

    else if (implementation.compare("GPS_L1_CA_PCPS_Acquisition_Fine_Doppler") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsAcquisitionFineDoppler(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_PCPS_QuickSync_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL1CaPcpsQuickSyncAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L2_M_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL2MPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L5i_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GpsL5iPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE1PcpsAmbiguousAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_8ms_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE1Pcps8msAmbiguousAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_Tong_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE1PcpsTongAmbiguousAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_PCPS_CCCWSR_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE1PcpsCccwsrAmbiguousAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }

    else if (implementation.compare("Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE1PcpsQuickSyncAmbiguousAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_Noncoherent_IQ_Acquisition_CAF") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE5aNoncoherentIQAcquisitionCaf(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_Pcps_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GalileoE5aPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L1_CA_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GlonassL1CaPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L2_CA_PCPS_Acquisition") == 0)
        {
            std::unique_ptr<AcquisitionInterface> block_(new GlonassL2CaPcpsAcquisition(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role << "." << implementation << ": Undefined implementation for block";
        }
    return std::move(block);
}


std::unique_ptr<TrackingInterface> GNSSBlockFactory::GetTrkBlock(
    std::shared_ptr<ConfigurationInterface> configuration,
    std::string role,
    std::string implementation, unsigned int in_streams,
    unsigned int out_streams)
{
    std::unique_ptr<TrackingInterface> block;

    // TRACKING BLOCKS -------------------------------------------------------------
    if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL1CaDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_C_Aid_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL1CaDllPllCAidTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_C_Aid_Tracking_Fpga") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL1CaDllPllCAidTrackingFpga(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#endif
    else if (implementation.compare("GPS_L1_CA_TCP_CONNECTOR_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL1CaTcpConnectorTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_DLL_PLL_VEML_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GalileoE1DllPllVemlTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1_TCP_CONNECTOR_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GalileoE1TcpConnectorTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GalileoE5aDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L2_M_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL2MDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L5i_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL5iDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#if CUDA_GPU_ACCEL
    else if (implementation.compare("GPS_L1_CA_DLL_PLL_Tracking_GPU") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GpsL1CaDllPllTrackingGPU(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
#endif
    else if (implementation.compare("GLONASS_L1_CA_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GlonassL1CaDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L1_CA_DLL_PLL_C_Aid_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GlonassL1CaDllPllCAidTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L2_CA_DLL_PLL_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GlonassL2CaDllPllTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L2_CA_DLL_PLL_C_Aid_Tracking") == 0)
        {
            std::unique_ptr<TrackingInterface> block_(new GlonassL2CaDllPllCAidTracking(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role << "." << implementation << ": Undefined implementation for block";
        }
    return std::move(block);
}


std::unique_ptr<TelemetryDecoderInterface> GNSSBlockFactory::GetTlmBlock(
    std::shared_ptr<ConfigurationInterface> configuration,
    std::string role,
    std::string implementation, unsigned int in_streams,
    unsigned int out_streams)
{
    std::unique_ptr<TelemetryDecoderInterface> block;

    // TELEMETRY DECODERS ----------------------------------------------------------
    if (implementation.compare("GPS_L1_CA_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new GpsL1CaTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E1B_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new GalileoE1BTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("SBAS_L1_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new SbasL1TelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("Galileo_E5a_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new GalileoE5aTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L2C_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new GpsL2CTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L1_CA_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new GlonassL1CaTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GLONASS_L2_CA_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new GlonassL2CaTelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else if (implementation.compare("GPS_L5_Telemetry_Decoder") == 0)
        {
            std::unique_ptr<TelemetryDecoderInterface> block_(new GpsL5TelemetryDecoder(configuration.get(), role, in_streams,
                out_streams));
            block = std::move(block_);
        }
    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role << "." << implementation << ": Undefined implementation for block";
        }
    return std::move(block);
}
