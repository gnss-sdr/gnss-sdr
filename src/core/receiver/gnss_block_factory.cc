/*!
 * \file gnss_block_factory.cc
 * \brief  This class implements a factory that returns instances of GNSS blocks.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *         Marc Majoral, 2018. mmajoral(at)cttc.es
 *         Carles Fernandez-Prades, 2014-2020. cfernandez(at)cttc.es
 *
 * This class encapsulates the complexity behind the instantiation
 * of GNSS blocks.
 *
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "gnss_block_factory.h"
#include "acquisition_interface.h"  // for AcquisitionInterface
#include "array_signal_conditioner.h"
#include "beamformer_filter.h"
#include "beidou_b1i_dll_pll_tracking.h"
#include "beidou_b1i_pcps_acquisition.h"
#include "beidou_b1i_telemetry_decoder.h"
#include "beidou_b3i_dll_pll_tracking.h"
#include "beidou_b3i_pcps_acquisition.h"
#include "beidou_b3i_telemetry_decoder.h"
#include "byte_to_short.h"
#include "channel.h"
#include "configuration_interface.h"
#include "direct_resampler_conditioner.h"
#include "file_signal_source.h"
#include "fir_filter.h"
#include "freq_xlating_fir_filter.h"
#include "galileo_e1_dll_pll_veml_tracking.h"
#include "galileo_e1_pcps_8ms_ambiguous_acquisition.h"
#include "galileo_e1_pcps_ambiguous_acquisition.h"
#include "galileo_e1_pcps_cccwsr_ambiguous_acquisition.h"
#include "galileo_e1_pcps_quicksync_ambiguous_acquisition.h"
#include "galileo_e1_pcps_tong_ambiguous_acquisition.h"
#include "galileo_e1_tcp_connector_tracking.h"
#include "galileo_e1b_telemetry_decoder.h"
#include "galileo_e5a_dll_pll_tracking.h"
#include "galileo_e5a_noncoherent_iq_acquisition_caf.h"
#include "galileo_e5a_pcps_acquisition.h"
#include "galileo_e5a_telemetry_decoder.h"
#include "glonass_l1_ca_dll_pll_c_aid_tracking.h"
#include "glonass_l1_ca_dll_pll_tracking.h"
#include "glonass_l1_ca_pcps_acquisition.h"
#include "glonass_l1_ca_telemetry_decoder.h"
#include "glonass_l2_ca_dll_pll_c_aid_tracking.h"
#include "glonass_l2_ca_dll_pll_tracking.h"
#include "glonass_l2_ca_pcps_acquisition.h"
#include "glonass_l2_ca_telemetry_decoder.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_make_unique.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "gps_l1_ca_kf_tracking.h"
#include "gps_l1_ca_pcps_acquisition.h"
#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "gps_l1_ca_pcps_assisted_acquisition.h"
#include "gps_l1_ca_pcps_quicksync_acquisition.h"
#include "gps_l1_ca_pcps_tong_acquisition.h"
#include "gps_l1_ca_tcp_connector_tracking.h"
#include "gps_l1_ca_telemetry_decoder.h"
#include "gps_l2_m_dll_pll_tracking.h"
#include "gps_l2_m_pcps_acquisition.h"
#include "gps_l2c_telemetry_decoder.h"
#include "gps_l5_dll_pll_tracking.h"
#include "gps_l5_telemetry_decoder.h"
#include "gps_l5i_pcps_acquisition.h"
#include "hybrid_observables.h"
#include "ibyte_to_cbyte.h"
#include "ibyte_to_complex.h"
#include "ibyte_to_cshort.h"
#include "in_memory_configuration.h"
#include "ishort_to_complex.h"
#include "ishort_to_cshort.h"
#include "labsat_signal_source.h"
#include "mmse_resampler_conditioner.h"
#include "multichannel_file_signal_source.h"
#include "notch_filter.h"
#include "notch_filter_lite.h"
#include "nsr_file_signal_source.h"
#include "pass_through.h"
#include "pulse_blanking_filter.h"
#include "rtklib_pvt.h"
#include "rtl_tcp_signal_source.h"
#include "sbas_l1_telemetry_decoder.h"
#include "signal_conditioner.h"
#include "spir_file_signal_source.h"
#include "spir_gss6450_file_signal_source.h"
#include "telemetry_decoder_interface.h"
#include "tracking_interface.h"
#include "two_bit_cpx_file_signal_source.h"
#include "two_bit_packed_file_signal_source.h"
#include <glog/logging.h>
#include <exception>  // for exception
#include <utility>    // for move

#if RAW_UDP
#include "custom_udp_signal_source.h"
#endif

#if ENABLE_FPGA
#include "galileo_e1_dll_pll_veml_tracking_fpga.h"
#include "galileo_e1_pcps_ambiguous_acquisition_fpga.h"
#include "galileo_e5a_dll_pll_tracking_fpga.h"
#include "galileo_e5a_pcps_acquisition_fpga.h"
#include "gps_l1_ca_dll_pll_tracking_fpga.h"
#include "gps_l1_ca_pcps_acquisition_fpga.h"
#include "gps_l2_m_dll_pll_tracking_fpga.h"
#include "gps_l2_m_pcps_acquisition_fpga.h"
#include "gps_l5_dll_pll_tracking_fpga.h"
#include "gps_l5i_pcps_acquisition_fpga.h"
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

#if AD9361_DRIVER
#include "ad9361_fpga_signal_source.h"
#endif

#if FLEXIBAND_DRIVER
#include "flexiband_signal_source.h"
#endif

#if CUDA_GPU_ACCEL
#include "gps_l1_ca_dll_pll_tracking_gpu.h"
#endif


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetSignalSource(
    const ConfigurationInterface* configuration, Concurrent_Queue<pmt::pmt_t>* queue, int ID)  // NOLINT(performance-unnecessary-value-param)
{
    std::string default_implementation = "File_Signal_Source";
    std::string role = "SignalSource";  // backwards compatibility for old conf files
    try
        {
            if (ID != -1)
                {
                    role = "SignalSource" + std::to_string(ID);
                }
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << e.what();
        }
    std::string implementation = configuration->property(role + ".implementation", default_implementation);
    LOG(INFO) << "Getting SignalSource with implementation " << implementation;
    return GetBlock(configuration, role, implementation, 0, 1, queue);
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetSignalConditioner(
    const ConfigurationInterface* configuration, int ID)
{
    std::string default_implementation = "Pass_Through";
    // backwards compatibility for old conf files
    std::string role_conditioner = "SignalConditioner";
    std::string role_datatypeadapter = "DataTypeAdapter";
    std::string role_inputfilter = "InputFilter";
    std::string role_resampler = "Resampler";
    try
        {
            if (ID != -1)
                {
                    role_conditioner = "SignalConditioner" + std::to_string(ID);
                    role_datatypeadapter = "DataTypeAdapter" + std::to_string(ID);
                    role_inputfilter = "InputFilter" + std::to_string(ID);
                    role_resampler = "Resampler" + std::to_string(ID);
                }
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << e.what();
        }
    std::string signal_conditioner = configuration->property(role_conditioner + ".implementation", default_implementation);

    std::string data_type_adapter;
    std::string input_filter;
    std::string resampler;
    if (signal_conditioner == "Pass_Through")
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

    if (signal_conditioner == "Array_Signal_Conditioner")
        {
            // instantiate the array version
            std::unique_ptr<GNSSBlockInterface> conditioner_ = std::make_unique<ArraySignalConditioner>(configuration,
                GetBlock(configuration, role_datatypeadapter, data_type_adapter, 1, 1),
                GetBlock(configuration, role_inputfilter, input_filter, 1, 1),
                GetBlock(configuration, role_resampler, resampler, 1, 1),
                role_conditioner, "Signal_Conditioner");
            return conditioner_;
        }

    // single-antenna version
    std::unique_ptr<GNSSBlockInterface> conditioner_ = std::make_unique<SignalConditioner>(configuration,
        GetBlock(configuration, role_datatypeadapter, data_type_adapter, 1, 1),
        GetBlock(configuration, role_inputfilter, input_filter, 1, 1),
        GetBlock(configuration, role_resampler, resampler, 1, 1),
        role_conditioner, "Signal_Conditioner");
    return conditioner_;
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetObservables(const ConfigurationInterface* configuration)
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
    unsigned int Beidou_channels = configuration->property("Channels_B1.count", 0);
    Beidou_channels += configuration->property("Channels_B3.count", 0);
    unsigned int extra_channels = 1;  // For monitor channel sample counter
    return GetBlock(configuration, "Observables", implementation,
        Galileo_channels +
            GPS_channels +
            Glonass_channels +
            Beidou_channels +
            extra_channels,
        Galileo_channels +
            GPS_channels +
            Glonass_channels +
            Beidou_channels);
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetPVT(const ConfigurationInterface* configuration)
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
    unsigned int Beidou_channels = configuration->property("Channels_B1.count", 0);
    Beidou_channels += configuration->property("Channels_B3.count", 0);
    return GetBlock(configuration, "PVT", implementation,
        Galileo_channels + GPS_channels + Glonass_channels + Beidou_channels, 0);
}


// ********* GPS L1 C/A CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_1C(
    const ConfigurationInterface* configuration,
    const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    // "appendix" is added to the "role" with the aim of Acquisition, Tracking and Telemetry Decoder adapters
    // can find their specific configurations when they read the config
    // TODO: REMOVE APPENDIX!! AND CHECK ALTERNATIVE MECHANISM TO GET PARTICULARIZED PARAMETERS
    LOG(INFO) << "Instantiating Channel " << channel << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;

    std::string aux = configuration->property("Acquisition_1C" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux != "W")
        {
            appendix1 = std::to_string(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_1C" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux != "W")
        {
            appendix2 = std::to_string(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_1C" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux != "W")
        {
            appendix3 = std::to_string(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_1C" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_1C" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type != trk_item_type)
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }

    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_1C" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_1C" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_1C" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_ = std::make_unique<Channel>(configuration, channel,
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "1C", queue);

    return channel_;
}


// ********* GPS L2C (M) CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_2S(
    const ConfigurationInterface* configuration,
    const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    LOG(INFO) << "Instantiating Channel " << channel << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;
    std::string aux = configuration->property("Acquisition_2S" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux != "W")
        {
            appendix1 = std::to_string(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_2S" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux != "W")
        {
            appendix2 = std::to_string(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_2S" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux != "W")
        {
            appendix3 = std::to_string(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_2S" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_2S" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type != trk_item_type)
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }

    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_2S" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_2S" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_2S" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_ = std::make_unique<Channel>(configuration, channel,
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "2S", queue);

    return channel_;
}


// ********* GALILEO E1 B CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_1B(
    const ConfigurationInterface* configuration,
    const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << id << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;
    std::string aux = configuration->property("Acquisition_1B" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux != "W")
        {
            appendix1 = std::to_string(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_1B" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux != "W")
        {
            appendix2 = std::to_string(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_1B" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux != "W")
        {
            appendix3 = std::to_string(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_1B" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_1B" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type != trk_item_type)
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }

    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_1B" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_1B" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_1B" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_ = std::make_unique<Channel>(configuration, channel,
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "1B", queue);

    return channel_;
}


// ********* GALILEO E5a  CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_5X(
    const ConfigurationInterface* configuration,
    const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << id << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;
    std::string aux = configuration->property("Acquisition_5X" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux != "W")
        {
            appendix1 = std::to_string(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_5X" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux != "W")
        {
            appendix2 = std::to_string(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_5X" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux != "W")
        {
            appendix3 = std::to_string(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_5X" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_5X" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type != trk_item_type)
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }

    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_5X" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_5X" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_5X" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_ = std::make_unique<Channel>(configuration, channel,
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "5X", queue);

    return channel_;
}


// ********* GLONASS L1 C/A CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_1G(
    const ConfigurationInterface* configuration,
    const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << channel << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder Implementation: " << tlm;

    std::string aux = configuration->property("Acquisition_1G" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux != "W")
        {
            appendix1 = std::to_string(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_1G" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux != "W")
        {
            appendix2 = std::to_string(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_1G" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux != "W")
        {
            appendix3 = std::to_string(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_1G" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_1G" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type != trk_item_type)
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }

    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_1G" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_1G" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_1G" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_ = std::make_unique<Channel>(configuration, channel,
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "1G", queue);

    return channel_;
}


// ********* GLONASS L2 C/A CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_2G(
    const ConfigurationInterface* configuration,
    const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << channel << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder Implementation: " << tlm;

    std::string aux = configuration->property("Acquisition_2G" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux != "W")
        {
            appendix1 = std::to_string(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_2G" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux != "W")
        {
            appendix2 = std::to_string(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_2G" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux != "W")
        {
            appendix3 = std::to_string(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_2G" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_2G" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type != trk_item_type)
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }

    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_2G" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_2G" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_2G" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_ = std::make_unique<Channel>(configuration, channel,
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "2G", queue);

    return channel_;
}


// ********* GPS L5  CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_L5(
    const ConfigurationInterface* configuration,
    const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << id << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;
    std::string aux = configuration->property("Acquisition_L5" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux != "W")
        {
            appendix1 = std::to_string(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_L5" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux != "W")
        {
            appendix2 = std::to_string(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_L5" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux != "W")
        {
            appendix3 = std::to_string(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_L5" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_L5" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type != trk_item_type)
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }

    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_L5" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_L5" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_L5" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_ = std::make_unique<Channel>(configuration, channel,
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "L5", queue);

    return channel_;
}


// ********* BeiDou B1I  CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_B1(
    const ConfigurationInterface* configuration,
    const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << id << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;
    std::string aux = configuration->property("Acquisition_B1" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux != "W")
        {
            appendix1 = std::to_string(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_B1" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux != "W")
        {
            appendix2 = std::to_string(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_B1" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux != "W")
        {
            appendix3 = std::to_string(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_B1" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_B1" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type != trk_item_type)
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }

    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_B1" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_B1" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_B1" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_ = std::make_unique<Channel>(configuration, channel,
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "B1", queue);

    return channel_;
}


// ********* BeiDou B3I  CHANNEL *****************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel_B3(
    const ConfigurationInterface* configuration,
    const std::string& acq, const std::string& trk, const std::string& tlm, int channel,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    std::stringstream stream;
    stream << channel;
    std::string id = stream.str();
    LOG(INFO) << "Instantiating Channel " << id << " with Acquisition Implementation: "
              << acq << ", Tracking Implementation: " << trk << ", Telemetry Decoder implementation: " << tlm;
    std::string aux = configuration->property("Acquisition_B3" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix1;
    if (aux != "W")
        {
            appendix1 = std::to_string(channel);
        }
    else
        {
            appendix1 = "";
        }
    aux = configuration->property("Tracking_B3" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix2;
    if (aux != "W")
        {
            appendix2 = std::to_string(channel);
        }
    else
        {
            appendix2 = "";
        }
    aux = configuration->property("TelemetryDecoder_B3" + std::to_string(channel) + ".implementation", std::string("W"));
    std::string appendix3;
    if (aux != "W")
        {
            appendix3 = std::to_string(channel);
        }
    else
        {
            appendix3 = "";
        }
    // Automatically detect input data type
    std::string default_item_type = "gr_complex";
    std::string acq_item_type = configuration->property("Acquisition_B3" + appendix1 + ".item_type", default_item_type);
    std::string trk_item_type = configuration->property("Tracking_B3" + appendix2 + ".item_type", default_item_type);
    if (acq_item_type != trk_item_type)
        {
            LOG(ERROR) << "Acquisition and Tracking blocks must have the same input data type!";
        }

    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_B3" + appendix1, acq, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_B3" + appendix2, trk, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_B3" + appendix3, tlm, 1, 1);

    std::unique_ptr<GNSSBlockInterface> channel_ = std::make_unique<Channel>(configuration, channel,
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", "B3", queue);

    return channel_;
}


std::unique_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> GNSSBlockFactory::GetChannels(
    const ConfigurationInterface* configuration, Concurrent_Queue<pmt::pmt_t>* queue)  // NOLINT(performance-unnecessary-value-param)
{
    std::string default_implementation = "Pass_Through";
    std::string tracking_implementation;
    std::string telemetry_decoder_implementation;
    std::string acquisition_implementation;

    unsigned int channel_absolute_id = 0;

    unsigned int Channels_1C_count = configuration->property("Channels_1C.count", 0);
    unsigned int Channels_1B_count = configuration->property("Channels_1B.count", 0);
    unsigned int Channels_1G_count = configuration->property("Channels_1G.count", 0);
    unsigned int Channels_2G_count = configuration->property("Channels_2G.count", 0);
    unsigned int Channels_2S_count = configuration->property("Channels_2S.count", 0);
    unsigned int Channels_5X_count = configuration->property("Channels_5X.count", 0);
    unsigned int Channels_L5_count = configuration->property("Channels_L5.count", 0);
    unsigned int Channels_B1_count = configuration->property("Channels_B1.count", 0);
    unsigned int Channels_B3_count = configuration->property("Channels_B3.count", 0);

    unsigned int total_channels = Channels_1C_count +
                                  Channels_1B_count +
                                  Channels_1G_count +
                                  Channels_2S_count +
                                  Channels_2G_count +
                                  Channels_5X_count +
                                  Channels_L5_count +
                                  Channels_B1_count +
                                  Channels_B3_count;

    auto channels = std::make_unique<std::vector<std::unique_ptr<GNSSBlockInterface>>>(total_channels);
    try
        {
            // **************** GPS L1 C/A  CHANNELS **********************
            LOG(INFO) << "Getting " << Channels_1C_count << " GPS L1 C/A channels";
            acquisition_implementation = configuration->property("Acquisition_1C.implementation", default_implementation);
            tracking_implementation = configuration->property("Tracking_1C.implementation", default_implementation);
            telemetry_decoder_implementation = configuration->property("TelemetryDecoder_1C.implementation", default_implementation);

            for (unsigned int i = 0; i < Channels_1C_count; i++)
                {
                    // (i.e. Acquisition_1C0.implementation=xxxx)
                    std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition_1C" + std::to_string(channel_absolute_id) + ".implementation",
                        acquisition_implementation);
                    // (i.e. Tracking_1C0.implementation=xxxx)
                    std::string tracking_implementation_specific = configuration->property(
                        "Tracking_1C" + std::to_string(channel_absolute_id) + ".implementation",
                        tracking_implementation);
                    std::string telemetry_decoder_implementation_specific = configuration->property(
                        "TelemetryDecoder_1C" + std::to_string(channel_absolute_id) + ".implementation",
                        telemetry_decoder_implementation);

                    // Push back the channel to the vector of channels
                    channels->at(channel_absolute_id) = GetChannel_1C(configuration,
                        acquisition_implementation_specific,
                        tracking_implementation_specific,
                        telemetry_decoder_implementation_specific,
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GPS L2C (M)  CHANNELS **********************
            LOG(INFO) << "Getting " << Channels_2S_count << " GPS L2C (M) channels";
            tracking_implementation = configuration->property("Tracking_2S.implementation", default_implementation);
            telemetry_decoder_implementation = configuration->property("TelemetryDecoder_2S.implementation", default_implementation);
            acquisition_implementation = configuration->property("Acquisition_2S.implementation", default_implementation);
            for (unsigned int i = 0; i < Channels_2S_count; i++)
                {
                    // (i.e. Acquisition_1C0.implementation=xxxx)
                    std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition_2S" + std::to_string(channel_absolute_id) + ".implementation",
                        acquisition_implementation);
                    // (i.e. Tracking_1C0.implementation=xxxx)
                    std::string tracking_implementation_specific = configuration->property(
                        "Tracking_2S" + std::to_string(channel_absolute_id) + ".implementation",
                        tracking_implementation);
                    std::string telemetry_decoder_implementation_specific = configuration->property(
                        "TelemetryDecoder_2S" + std::to_string(channel_absolute_id) + ".implementation",
                        telemetry_decoder_implementation);

                    // Push back the channel to the vector of channels
                    channels->at(channel_absolute_id) = GetChannel_2S(configuration,
                        acquisition_implementation_specific,
                        tracking_implementation_specific,
                        telemetry_decoder_implementation_specific,
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GPS L5  CHANNELS **********************
            LOG(INFO) << "Getting " << Channels_L5_count << " GPS L5 channels";
            tracking_implementation = configuration->property("Tracking_L5.implementation", default_implementation);
            telemetry_decoder_implementation = configuration->property("TelemetryDecoder_L5.implementation", default_implementation);
            acquisition_implementation = configuration->property("Acquisition_L5.implementation", default_implementation);
            for (unsigned int i = 0; i < Channels_L5_count; i++)
                {
                    // (i.e. Acquisition_1C0.implementation=xxxx)
                    std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition_L5" + std::to_string(channel_absolute_id) + ".implementation",
                        acquisition_implementation);
                    // (i.e. Tracking_1C0.implementation=xxxx)
                    std::string tracking_implementation_specific = configuration->property(
                        "Tracking_L5" + std::to_string(channel_absolute_id) + ".implementation",
                        tracking_implementation);
                    std::string telemetry_decoder_implementation_specific = configuration->property(
                        "TelemetryDecoder_L5" + std::to_string(channel_absolute_id) + ".implementation",
                        telemetry_decoder_implementation);

                    // Push back the channel to the vector of channels
                    channels->at(channel_absolute_id) = GetChannel_L5(configuration,
                        acquisition_implementation_specific,
                        tracking_implementation_specific,
                        telemetry_decoder_implementation_specific,
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GALILEO E1 B (I/NAV OS) CHANNELS **********************
            LOG(INFO) << "Getting " << Channels_1B_count << " GALILEO E1 B (I/NAV OS) channels";
            tracking_implementation = configuration->property("Tracking_1B.implementation", default_implementation);
            telemetry_decoder_implementation = configuration->property("TelemetryDecoder_1B.implementation", default_implementation);
            acquisition_implementation = configuration->property("Acquisition_1B.implementation", default_implementation);
            for (unsigned int i = 0; i < Channels_1B_count; i++)
                {
                    // (i.e. Acquisition_1C0.implementation=xxxx)
                    std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition_1B" + std::to_string(channel_absolute_id) + ".implementation",
                        acquisition_implementation);
                    // (i.e. Tracking_1C0.implementation=xxxx)
                    std::string tracking_implementation_specific = configuration->property(
                        "Tracking_1B" + std::to_string(channel_absolute_id) + ".implementation",
                        tracking_implementation);
                    std::string telemetry_decoder_implementation_specific = configuration->property(
                        "TelemetryDecoder_1B" + std::to_string(channel_absolute_id) + ".implementation",
                        telemetry_decoder_implementation);

                    // Push back the channel to the vector of channels
                    channels->at(channel_absolute_id) = GetChannel_1B(configuration,
                        acquisition_implementation_specific,
                        tracking_implementation_specific,
                        telemetry_decoder_implementation_specific,
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GALILEO E5a I (F/NAV OS)  CHANNELS **********************
            LOG(INFO) << "Getting " << Channels_5X_count << " GALILEO E5a I (F/NAV OS) channels";
            tracking_implementation = configuration->property("Tracking_5X.implementation", default_implementation);
            telemetry_decoder_implementation = configuration->property("TelemetryDecoder_5X.implementation", default_implementation);
            acquisition_implementation = configuration->property("Acquisition_5X.implementation", default_implementation);
            for (unsigned int i = 0; i < Channels_5X_count; i++)
                {
                    // (i.e. Acquisition_1C0.implementation=xxxx)
                    std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition_5X" + std::to_string(channel_absolute_id) + ".implementation",
                        acquisition_implementation);
                    // (i.e. Tracking_1C0.implementation=xxxx)
                    std::string tracking_implementation_specific = configuration->property(
                        "Tracking_5X" + std::to_string(channel_absolute_id) + ".implementation",
                        tracking_implementation);
                    std::string telemetry_decoder_implementation_specific = configuration->property(
                        "TelemetryDecoder_5X" + std::to_string(channel_absolute_id) + ".implementation",
                        telemetry_decoder_implementation);

                    // Push back the channel to the vector of channels
                    channels->at(channel_absolute_id) = GetChannel_5X(configuration,
                        acquisition_implementation_specific,
                        tracking_implementation_specific,
                        telemetry_decoder_implementation_specific,
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GLONASS L1 C/A  CHANNELS **********************
            LOG(INFO) << "Getting " << Channels_1G_count << " GLONASS L1 C/A channels";
            acquisition_implementation = configuration->property("Acquisition_1G.implementation", default_implementation);
            tracking_implementation = configuration->property("Tracking_1G.implementation", default_implementation);
            telemetry_decoder_implementation = configuration->property("TelemetryDecoder_1G.implementation", default_implementation);

            for (unsigned int i = 0; i < Channels_1G_count; i++)
                {
                    // (i.e. Acquisition_1G0.implementation=xxxx)
                    std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition_1G" + std::to_string(channel_absolute_id) + ".implementation",
                        acquisition_implementation);
                    // (i.e. Tracking_1G0.implementation=xxxx)
                    std::string tracking_implementation_specific = configuration->property(
                        "Tracking_1G" + std::to_string(channel_absolute_id) + ".implementation",
                        tracking_implementation);
                    std::string telemetry_decoder_implementation_specific = configuration->property(
                        "TelemetryDecoder_1G" + std::to_string(channel_absolute_id) + ".implementation",
                        telemetry_decoder_implementation);

                    // Push back the channel to the vector of channels
                    channels->at(channel_absolute_id) = GetChannel_1G(configuration,
                        acquisition_implementation_specific,
                        tracking_implementation_specific,
                        telemetry_decoder_implementation_specific,
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GLONASS L2 C/A  CHANNELS **********************
            LOG(INFO) << "Getting " << Channels_2G_count << " GLONASS L2 C/A channels";
            acquisition_implementation = configuration->property("Acquisition_2G.implementation", default_implementation);
            tracking_implementation = configuration->property("Tracking_2G.implementation", default_implementation);
            telemetry_decoder_implementation = configuration->property("TelemetryDecoder_2G.implementation", default_implementation);

            for (unsigned int i = 0; i < Channels_2G_count; i++)
                {
                    // (i.e. Acquisition_2G0.implementation=xxxx)
                    std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition_2G" + std::to_string(channel_absolute_id) + ".implementation",
                        acquisition_implementation);
                    // (i.e. Tracking_2G0.implementation=xxxx)
                    std::string tracking_implementation_specific = configuration->property(
                        "Tracking_2G" + std::to_string(channel_absolute_id) + ".implementation",
                        tracking_implementation);
                    std::string telemetry_decoder_implementation_specific = configuration->property(
                        "TelemetryDecoder_2G" + std::to_string(channel_absolute_id) + ".implementation",
                        telemetry_decoder_implementation);

                    // Push back the channel to the vector of channels
                    channels->at(channel_absolute_id) = GetChannel_2G(configuration,
                        acquisition_implementation_specific,
                        tracking_implementation_specific,
                        telemetry_decoder_implementation_specific,
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** BEIDOU B1I  CHANNELS **********************
            LOG(INFO) << "Getting " << Channels_B1_count << " BEIDOU B1I channels";
            acquisition_implementation = configuration->property("Acquisition_B1.implementation", default_implementation);
            tracking_implementation = configuration->property("Tracking_B1.implementation", default_implementation);
            telemetry_decoder_implementation = configuration->property("TelemetryDecoder_B1.implementation", default_implementation);

            for (unsigned int i = 0; i < Channels_B1_count; i++)
                {
                    // (i.e. Acquisition_2G0.implementation=xxxx)
                    std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition_B1" + std::to_string(channel_absolute_id) + ".implementation",
                        acquisition_implementation);
                    // (i.e. Tracking_2G0.implementation=xxxx)
                    std::string tracking_implementation_specific = configuration->property(
                        "Tracking_B1" + std::to_string(channel_absolute_id) + ".implementation",
                        tracking_implementation);
                    std::string telemetry_decoder_implementation_specific = configuration->property(
                        "TelemetryDecoder_B1" + std::to_string(channel_absolute_id) + ".implementation",
                        telemetry_decoder_implementation);

                    // Push back the channel to the vector of channels
                    channels->at(channel_absolute_id) = GetChannel_B1(configuration,
                        acquisition_implementation_specific,
                        tracking_implementation_specific,
                        telemetry_decoder_implementation_specific,
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** BEIDOU B3I  CHANNELS **********************
            LOG(INFO) << "Getting " << Channels_B3_count << " BEIDOU B3I channels";
            acquisition_implementation = configuration->property("Acquisition_B3.implementation", default_implementation);
            tracking_implementation = configuration->property("Tracking_B3.implementation", default_implementation);
            telemetry_decoder_implementation = configuration->property("TelemetryDecoder_B3.implementation", default_implementation);

            for (unsigned int i = 0; i < Channels_B3_count; i++)
                {
                    // (i.e. Acquisition_2G0.implementation=xxxx)
                    std::string acquisition_implementation_specific = configuration->property(
                        "Acquisition_B3" + std::to_string(channel_absolute_id) + ".implementation",
                        acquisition_implementation);
                    // (i.e. Tracking_2G0.implementation=xxxx)
                    std::string tracking_implementation_specific = configuration->property(
                        "Tracking_B3" + std::to_string(channel_absolute_id) + ".implementation",
                        tracking_implementation);
                    std::string telemetry_decoder_implementation_specific = configuration->property(
                        "TelemetryDecoder_B3" + std::to_string(channel_absolute_id) + ".implementation",
                        telemetry_decoder_implementation);

                    // Push back the channel to the vector of channels
                    channels->at(channel_absolute_id) = GetChannel_B3(configuration,
                        acquisition_implementation_specific,
                        tracking_implementation_specific,
                        telemetry_decoder_implementation_specific,
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << e.what();
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
    const ConfigurationInterface* configuration,
    const std::string& role,
    const std::string& implementation, unsigned int in_streams,
    unsigned int out_streams, Concurrent_Queue<pmt::pmt_t>* queue)  // NOLINT(performance-unnecessary-value-param)
{
    std::unique_ptr<GNSSBlockInterface> block;

    // PASS THROUGH ------------------------------------------------------------
    if (implementation == "Pass_Through")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<Pass_Through>(configuration, role, in_streams, out_streams);
            block = std::move(block_);
        }

    // SIGNAL SOURCES ----------------------------------------------------------
    else if (implementation == "File_Signal_Source")
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<FileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }

            catch (const std::exception& e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
    else if (implementation == "Multichannel_File_Signal_Source")
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<MultichannelFileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }

            catch (const std::exception& e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
#if RAW_UDP
    else if (implementation == "Custom_UDP_Signal_Source")
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<CustomUDPSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }

            catch (const std::exception& e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
#endif
    else if (implementation == "Nsr_File_Signal_Source")
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<NsrFileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            catch (const std::exception& e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
    else if (implementation == "Two_Bit_Cpx_File_Signal_Source")
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<TwoBitCpxFileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            catch (const std::exception& e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
    else if (implementation == "Two_Bit_Packed_File_Signal_Source")
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<TwoBitPackedFileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            catch (const std::exception& e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
    else if (implementation == "Spir_File_Signal_Source")
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<SpirFileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            catch (const std::exception& e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
    else if (implementation == "Spir_GSS6450_File_Signal_Source")
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<SpirGSS6450FileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            catch (const std::exception& e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
    else if (implementation == "RtlTcp_Signal_Source")
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<RtlTcpSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            catch (const std::exception& e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
    else if (implementation == "Labsat_Signal_Source")
        {
            try
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<LabsatSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }

            catch (const std::exception& e)
                {
                    std::cout << "GNSS-SDR program ended." << std::endl;
                    exit(1);
                }
        }
#if UHD_DRIVER
    else if (implementation == "UHD_Signal_Source")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<UhdSignalSource>(configuration, role, in_streams,
                out_streams, queue);
            block = std::move(block_);
        }
#endif
#if GN3S_DRIVER
    else if (implementation == "GN3S_Signal_Source")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<Gn3sSignalSource>(configuration, role, in_streams,
                out_streams, queue);
            block = std::move(block_);
        }
#endif

#if RAW_ARRAY_DRIVER
    else if (implementation == "Raw_Array_Signal_Source")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<RawArraySignalSource>(configuration, role, in_streams,
                out_streams, queue);
            block = std::move(block_);
        }
#endif

#if OSMOSDR_DRIVER
    else if (implementation == "Osmosdr_Signal_Source")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<OsmosdrSignalSource>(configuration, role, in_streams,
                out_streams, queue);
            block = std::move(block_);
        }
#endif

#if PLUTOSDR_DRIVER
    else if (implementation == "Plutosdr_Signal_Source")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<PlutosdrSignalSource>(configuration, role, in_streams,
                out_streams, queue);
            block = std::move(block_);
        }
#endif

#if FMCOMMS2_DRIVER
    else if (implementation == "Fmcomms2_Signal_Source")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<Fmcomms2SignalSource>(configuration, role, in_streams,
                out_streams, queue);
            block = std::move(block_);
        }
#endif

#if FLEXIBAND_DRIVER
    else if (implementation == "Flexiband_Signal_Source")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<FlexibandSignalSource>(configuration, role, in_streams,
                out_streams, queue);
            block = std::move(block_);
        }
#endif

    // DATA TYPE ADAPTER -----------------------------------------------------------
    else if (implementation == "Byte_To_Short")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<ByteToShort>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Ibyte_To_Cbyte")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<IbyteToCbyte>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Ibyte_To_Cshort")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<IbyteToCshort>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Ibyte_To_Complex")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<IbyteToComplex>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Ishort_To_Cshort")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<IshortToCshort>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Ishort_To_Complex")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<IshortToComplex>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

    // INPUT FILTER ----------------------------------------------------------------
    else if (implementation == "Fir_Filter")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<FirFilter>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Freq_Xlating_Fir_Filter")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<FreqXlatingFirFilter>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Beamformer_Filter")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<BeamformerFilter>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Pulse_Blanking_Filter")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<PulseBlankingFilter>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Notch_Filter")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<NotchFilter>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Notch_Filter_Lite")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<NotchFilterLite>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

    // RESAMPLER -------------------------------------------------------------------
    else if (implementation == "Direct_Resampler")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<DirectResamplerConditioner>(configuration, role,
                in_streams, out_streams);
            block = std::move(block_);
        }

    else if ((implementation == "Fractional_Resampler") || (implementation == "Mmse_Resampler"))
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<MmseResamplerConditioner>(configuration, role,
                in_streams, out_streams);
            block = std::move(block_);
        }

    // ACQUISITION BLOCKS ---------------------------------------------------------
    else if (implementation == "GPS_L1_CA_PCPS_Acquisition")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

#if ENABLE_FPGA
    else if (implementation == "GPS_L1_CA_PCPS_Acquisition_Fpga")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "GPS_L1_CA_PCPS_Assisted_Acquisition")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaPcpsAssistedAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L1_CA_PCPS_Tong_Acquisition")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaPcpsTongAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

#if OPENCL_BLOCKS
    else if (implementation == "GPS_L1_CA_PCPS_OpenCl_Acquisition")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaPcpsOpenClAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif

    else if (implementation == "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaPcpsAcquisitionFineDoppler>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L1_CA_PCPS_QuickSync_Acquisition")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaPcpsQuickSyncAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L2_M_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL2MPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "GPS_L2_M_PCPS_Acquisition_Fpga")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL2MPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "GPS_L5i_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL5iPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "GPS_L5i_PCPS_Acquisition_Fpga")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL5iPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1PcpsAmbiguousAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition_Fpga")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1PcpsAmbiguousAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "Galileo_E1_PCPS_8ms_Ambiguous_Acquisition")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1Pcps8msAmbiguousAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E1_PCPS_Tong_Ambiguous_Acquisition")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1PcpsTongAmbiguousAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E1_PCPS_CCCWSR_Ambiguous_Acquisition")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1PcpsCccwsrAmbiguousAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5aNoncoherentIQAcquisitionCaf>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E5a_Pcps_Acquisition")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5aPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "Galileo_E5a_Pcps_Acquisition_Fpga")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5aPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1PcpsQuickSyncAmbiguousAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L1_CA_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GlonassL1CaPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L2_CA_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GlonassL2CaPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B1I_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<BeidouB1iPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B3I_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<BeidouB3iPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

    // TRACKING BLOCKS -------------------------------------------------------------
    else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L1_CA_KF_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaKfTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_Fpga")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaDllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "GPS_L1_CA_TCP_CONNECTOR_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaTcpConnectorTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L2_M_DLL_PLL_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL2MDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "GPS_L2_M_DLL_PLL_Tracking_Fpga")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL2MDllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if ((implementation == "GPS_L5i_DLL_PLL_Tracking") or (implementation == "GPS_L5_DLL_PLL_Tracking"))
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL5DllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if ((implementation == "GPS_L5i_DLL_PLL_Tracking_Fpga") or (implementation == "GPS_L5_DLL_PLL_Tracking_Fpga"))
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL5DllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
#if CUDA_GPU_ACCEL
    else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_GPU")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaDllPllTrackingGPU>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1DllPllVemlTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GalileoE1DllPllVemlTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "Galileo_E1_TCP_CONNECTOR_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1TcpConnectorTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5aDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_Fpga")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5aDllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "GLONASS_L1_CA_DLL_PLL_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GlonassL1CaDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L1_CA_DLL_PLL_C_Aid_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GlonassL1CaDllPllCAidTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L2_CA_DLL_PLL_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GlonassL2CaDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L2_CA_DLL_PLL_C_Aid_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GlonassL2CaDllPllCAidTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B1I_DLL_PLL_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<BeidouB1iDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B3I_DLL_PLL_Tracking")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<BeidouB3iDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    // TELEMETRY DECODERS ----------------------------------------------------------
    else if (implementation == "GPS_L1_CA_Telemetry_Decoder")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L2C_Telemetry_Decoder")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL2CTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L5_Telemetry_Decoder")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL5TelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E1B_Telemetry_Decoder")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1BTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "SBAS_L1_Telemetry_Decoder")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<SbasL1TelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E5a_Telemetry_Decoder")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5aTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L1_CA_Telemetry_Decoder")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GlonassL1CaTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L2_CA_Telemetry_Decoder")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GlonassL2CaTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B1I_Telemetry_Decoder")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<BeidouB1iTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B3I_Telemetry_Decoder")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<BeidouB3iTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

    // OBSERVABLES -----------------------------------------------------------------
    else if ((implementation == "Hybrid_Observables") || (implementation == "GPS_L1_CA_Observables") || (implementation == "GPS_L2C_Observables") ||
             (implementation == "Galileo_E5A_Observables"))
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<HybridObservables>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

    // PVT -------------------------------------------------------------------------
    else if ((implementation == "RTKLIB_PVT") || (implementation == "GPS_L1_CA_PVT") || (implementation == "Galileo_E1_PVT") || (implementation == "Hybrid_PVT"))
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<Rtklib_Pvt>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

#if AD9361_DRIVER
    // The AD9361_DRIVER Driver is instantiated last. In this way, when using the FPGA, and when using the GNSS receiver
    // in post-processing mode, the receiver is configured and ready when the DMA starts sending samples to the receiver.
    else if (implementation == "Ad9361_Fpga_Signal_Source")
        {
            std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<Ad9361FpgaSignalSource>(configuration, role, in_streams,
                out_streams, queue);
            block = std::move(block_);
        }
#endif

    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role << "." << implementation << ": Undefined implementation for block";
        }
    return block;
}


/*
 *
 * PLEASE ADD YOUR NEW BLOCK HERE!!
 *
 * Not very elegant, Acq, Trk and Tlm blocks must be added here, too.
 * To be improved!
 */

std::unique_ptr<AcquisitionInterface> GNSSBlockFactory::GetAcqBlock(
    const ConfigurationInterface* configuration,
    const std::string& role,
    const std::string& implementation, unsigned int in_streams,
    unsigned int out_streams)
{
    std::unique_ptr<AcquisitionInterface> block;
    // ACQUISITION BLOCKS ---------------------------------------------------------
    if (implementation == "GPS_L1_CA_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL1CaPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "GPS_L1_CA_PCPS_Acquisition_Fpga")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL1CaPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "GPS_L1_CA_PCPS_Assisted_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL1CaPcpsAssistedAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L1_CA_PCPS_Tong_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL1CaPcpsTongAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

#if OPENCL_BLOCKS
    else if (implementation == "GPS_L1_CA_PCPS_OpenCl_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL1CaPcpsOpenClAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif

    else if (implementation == "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL1CaPcpsAcquisitionFineDoppler>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L1_CA_PCPS_QuickSync_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL1CaPcpsQuickSyncAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L2_M_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL2MPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "GPS_L2_M_PCPS_Acquisition_Fpga")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL2MPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "GPS_L5i_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL5iPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "GPS_L5i_PCPS_Acquisition_Fpga")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL5iPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE1PcpsAmbiguousAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition_Fpga")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE1PcpsAmbiguousAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "Galileo_E1_PCPS_8ms_Ambiguous_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE1Pcps8msAmbiguousAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E1_PCPS_Tong_Ambiguous_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE1PcpsTongAmbiguousAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E1_PCPS_CCCWSR_Ambiguous_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE1PcpsCccwsrAmbiguousAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

    else if (implementation == "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE1PcpsQuickSyncAmbiguousAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE5aNoncoherentIQAcquisitionCaf>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E5a_Pcps_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE5aPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "Galileo_E5a_Pcps_Acquisition_Fpga")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE5aPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "GLONASS_L1_CA_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GlonassL1CaPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L2_CA_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GlonassL2CaPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B1I_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<BeidouB1iPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B3I_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<BeidouB3iPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role << "." << implementation << ": Undefined implementation for block";
        }
    return block;
}


std::unique_ptr<TrackingInterface> GNSSBlockFactory::GetTrkBlock(
    const ConfigurationInterface* configuration,
    const std::string& role,
    const std::string& implementation, unsigned int in_streams,
    unsigned int out_streams)
{
    std::unique_ptr<TrackingInterface> block;

    // TRACKING BLOCKS -------------------------------------------------------------
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L1_CA_KF_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaKfTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_Fpga")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaDllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "GPS_L1_CA_TCP_CONNECTOR_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaTcpConnectorTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GalileoE1DllPllVemlTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_Fpga")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GalileoE1DllPllVemlTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "Galileo_E1_TCP_CONNECTOR_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GalileoE1TcpConnectorTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GalileoE5aDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_Fpga")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GalileoE5aDllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "GPS_L2_M_DLL_PLL_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL2MDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if (implementation == "GPS_L2_M_DLL_PLL_Tracking_Fpga")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL2MDllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if ((implementation == "GPS_L5i_DLL_PLL_Tracking") or (implementation == "GPS_L5_DLL_PLL_Tracking"))
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL5DllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#if ENABLE_FPGA
    else if ((implementation == "GPS_L5i_DLL_PLL_Tracking_Fpga") or (implementation == "GPS_L5_DLL_PLL_Tracking_Fpga"))
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL5DllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
#if CUDA_GPU_ACCEL
    else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_GPU")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaDllPllTrackingGPU>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else if (implementation == "GLONASS_L1_CA_DLL_PLL_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GlonassL1CaDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L1_CA_DLL_PLL_C_Aid_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GlonassL1CaDllPllCAidTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L2_CA_DLL_PLL_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GlonassL2CaDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L2_CA_DLL_PLL_C_Aid_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GlonassL2CaDllPllCAidTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B1I_DLL_PLL_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<BeidouB1iDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B3I_DLL_PLL_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<BeidouB3iDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role << "." << implementation << ": Undefined implementation for block";
        }
    return block;
}


std::unique_ptr<TelemetryDecoderInterface> GNSSBlockFactory::GetTlmBlock(
    const ConfigurationInterface* configuration,
    const std::string& role,
    const std::string& implementation, unsigned int in_streams,
    unsigned int out_streams)
{
    std::unique_ptr<TelemetryDecoderInterface> block;

    // TELEMETRY DECODERS ----------------------------------------------------------
    if (implementation == "GPS_L1_CA_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<GpsL1CaTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E1B_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<GalileoE1BTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "SBAS_L1_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<SbasL1TelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E5a_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<GalileoE5aTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L2C_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<GpsL2CTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L1_CA_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<GlonassL1CaTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GLONASS_L2_CA_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<GlonassL2CaTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L5_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<GpsL5TelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B1I_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<BeidouB1iTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "BEIDOU_B3I_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<BeidouB3iTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }

    else
        {
            // Log fatal. This causes execution to stop.
            LOG(ERROR) << role << "." << implementation << ": Undefined implementation for block";
        }
    return block;
}
