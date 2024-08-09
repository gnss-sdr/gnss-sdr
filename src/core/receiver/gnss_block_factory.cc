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
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "gnss_block_factory.h"
#include "acquisition_interface.h"
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
#include "fifo_signal_source.h"
#include "file_signal_source.h"
#include "file_timestamp_signal_source.h"
#include "fir_filter.h"
#include "four_bit_cpx_file_signal_source.h"
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
#include "galileo_e5b_dll_pll_tracking.h"
#include "galileo_e5b_pcps_acquisition.h"
#include "galileo_e5b_telemetry_decoder.h"
#include "galileo_e6_dll_pll_tracking.h"
#include "galileo_e6_pcps_acquisition.h"
#include "galileo_e6_telemetry_decoder.h"
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
#include "gnss_sdr_string_literals.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "gps_l1_ca_gaussian_tracking.h"
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
#include <cstdlib>    // for exit
#include <exception>  // for exception
#include <iostream>   // for cerr
#include <utility>    // for move

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#if RAW_UDP
#include "custom_udp_signal_source.h"
#endif

#if ENABLE_FPGA
#include "galileo_e1_dll_pll_veml_tracking_fpga.h"
#include "galileo_e1_pcps_ambiguous_acquisition_fpga.h"
#include "galileo_e5a_dll_pll_tracking_fpga.h"
#include "galileo_e5a_pcps_acquisition_fpga.h"
#include "galileo_e5b_pcps_acquisition_fpga.h"
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
#include "ad936x_custom_signal_source.h"
#include "plutosdr_signal_source.h"
#endif

#if FMCOMMS2_DRIVER
#include "fmcomms2_signal_source.h"
#endif

#if ENABLE_FPGA and AD9361_DRIVER
#include "adrv9361_z7035_signal_source_fpga.h"
#include "fmcomms5_signal_source_fpga.h"
#endif

#if MAX2771_DRIVER
#include "max2771_evkit_signal_source_fpga.h"
#endif

#if DMA_PROXY_DRIVER
#include "dma_signal_source_fpga.h"
#endif

#if LIMESDR_DRIVER
#include "limesdr_signal_source.h"
#endif

#if FLEXIBAND_DRIVER
#include "flexiband_signal_source.h"
#endif

#if ZEROMQ_DRIVER
#include "zmq_signal_source.h"
#endif

#if CUDA_GPU_ACCEL
#include "gps_l1_ca_dll_pll_tracking_gpu.h"
#endif

using namespace std::string_literals;

namespace
{
auto const impl_prop = ".implementation"s;  // "implementation" property; used nearly universally
auto const item_prop = ".item_type"s;       // "item_type" property

// unique_ptr dynamic cast from https://stackoverflow.com/a/26377517/9220132
template <typename To, typename From>
std::unique_ptr<To> dynamic_unique_cast(std::unique_ptr<From>&& p)
{
    if (To* cast = dynamic_cast<To*>(p.get()))
        {
            std::unique_ptr<To> result(cast);
            p.release();  // NOLINT(bugprone-unused-return-value)
            return result;
        }
    return std::unique_ptr<To>(nullptr);
}


auto findRole(ConfigurationInterface const* configuration, std::string const& base, int ID) -> std::string
{
    auto role = base + std::to_string(ID);

    // Legacy behavior: pass -1 for unadorned property.
    // Current behavior: if there is no "Tag0" use "Tag" instead
    if (ID < 1)
        {
            auto stub = configuration->property(role + impl_prop, ""s);
            if (stub.empty()) role = base;  // NOLINT  -- legacy format
        }
    return role;
};
}  // namespace


std::unique_ptr<SignalSourceInterface> GNSSBlockFactory::GetSignalSource(
    const ConfigurationInterface* configuration, Concurrent_Queue<pmt::pmt_t>* queue, int ID)
{
    auto role = findRole(configuration, "SignalSource"s, ID);
    auto implementation = configuration->property(role + impl_prop, ""s);
    LOG(INFO) << "Getting SignalSource " << role << " with implementation " << implementation;

    return dynamic_unique_cast<SignalSourceInterface>(GetBlock(configuration, role, 0, 1, queue));
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetSignalConditioner(
    const ConfigurationInterface* configuration, int ID)
{
    const std::string empty_implementation;

    auto role_conditioner = findRole(configuration, "SignalConditioner"s, ID);
    auto role_datatypeadapter = findRole(configuration, "DataTypeAdapter"s, ID);
    auto role_inputfilter = findRole(configuration, "InputFilter"s, ID);
    auto role_resampler = findRole(configuration, "Resampler"s, ID);

    DLOG(INFO) << "role: " << role_conditioner << " (ID=" << ID << ")";

    const std::string signal_conditioner = configuration->property(role_conditioner + impl_prop, ""s);

    const std::string data_type_adapter = configuration->property(role_datatypeadapter + impl_prop, ""s);
    const std::string input_filter = configuration->property(role_inputfilter + impl_prop, ""s);
    const std::string resampler = configuration->property(role_resampler + impl_prop, ""s);

    if (signal_conditioner == "Pass_Through")
        {
            if (!data_type_adapter.empty() and (data_type_adapter != "Pass_Through"))
                {
                    LOG(WARNING) << "Configuration warning: if " << role_conditioner << impl_prop << "\n"
                                 << "is set to Pass_Through, then the " << role_datatypeadapter << impl_prop << "\n"
                                 << "parameter should be either not set or set to Pass_Through.\n"
                                 << role_datatypeadapter << " configuration parameters will be ignored.";
                }
            if (!input_filter.empty() and (input_filter != "Pass_Through"))
                {
                    LOG(WARNING) << "Configuration warning: if " << role_conditioner << impl_prop << "\n"
                                 << "is set to Pass_Through, then the " << role_inputfilter << impl_prop << "\n"
                                 << "parameter should be either not set or set to Pass_Through.\n"
                                 << role_inputfilter << " configuration parameters will be ignored.";
                }
            if (!resampler.empty() and (resampler != "Pass_Through"))
                {
                    LOG(WARNING) << "Configuration warning: if " << role_conditioner << impl_prop << "\n"
                                 << "is set to Pass_Through, then the " << role_resampler << impl_prop << "\n"
                                 << "parameter should be either not set or set to Pass_Through.\n"
                                 << role_resampler << " configuration parameters will be ignored.";
                }
            LOG(INFO) << "Getting " << role_conditioner << " with Pass_Through implementation";

            std::unique_ptr<GNSSBlockInterface> conditioner_ = std::make_unique<Pass_Through>(configuration, role_conditioner, 1, 1);

            return conditioner_;
        }

    LOG(INFO) << "Getting " << role_conditioner << " with " << role_datatypeadapter << " implementation: "
              << data_type_adapter << ", " << role_inputfilter << " implementation: "
              << input_filter << ", and " << role_resampler << " implementation: "
              << resampler;

    if (signal_conditioner == "Array_Signal_Conditioner")
        {
            // instantiate the array version
            std::unique_ptr<GNSSBlockInterface> conditioner_ = std::make_unique<ArraySignalConditioner>(
                GetBlock(configuration, role_datatypeadapter, 1, 1),
                GetBlock(configuration, role_inputfilter, 1, 1),
                GetBlock(configuration, role_resampler, 1, 1),
                role_conditioner);
            return conditioner_;
        }

    if (signal_conditioner != "Signal_Conditioner")
        {
            std::cerr << "Error in configuration file: SignalConditioner.implementation=" << signal_conditioner << " is not a valid value.\n";
            return nullptr;
        }

    // single-antenna version
    std::unique_ptr<GNSSBlockInterface> conditioner_ = std::make_unique<SignalConditioner>(
        GetBlock(configuration, role_datatypeadapter, 1, 1),
        GetBlock(configuration, role_inputfilter, 1, 1),
        GetBlock(configuration, role_resampler, 1, 1),
        role_conditioner);
    return conditioner_;
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetObservables(const ConfigurationInterface* configuration)
{
    const std::string empty_implementation;
    std::string implementation = configuration->property("Observables.implementation", empty_implementation);
    LOG(INFO) << "Getting Observables with implementation " << implementation;
    if (implementation.find("_Observables") == std::string::npos)
        {
            std::cerr << "Error in configuration file: please set Observables.implementation=Hybrid_Observables\n";
            return nullptr;
        }
    unsigned int Galileo_channels = configuration->property("Channels_1B.count", 0);
    Galileo_channels += configuration->property("Channels_5X.count", 0);
    Galileo_channels += configuration->property("Channels_7X.count", 0);
    Galileo_channels += configuration->property("Channels_E6.count", 0);
    unsigned int GPS_channels = configuration->property("Channels_1C.count", 0);
    GPS_channels += configuration->property("Channels_2S.count", 0);
    GPS_channels += configuration->property("Channels_L5.count", 0);
    unsigned int Glonass_channels = configuration->property("Channels_1G.count", 0);
    Glonass_channels += configuration->property("Channels_2G.count", 0);
    unsigned int Beidou_channels = configuration->property("Channels_B1.count", 0);
    Beidou_channels += configuration->property("Channels_B3.count", 0);
    unsigned int extra_channels = 1;  // For monitor channel sample counter
    return GetBlock(configuration, "Observables",
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
    const std::string empty_implementation;
    std::string implementation = configuration->property("PVT.implementation", empty_implementation);
    LOG(INFO) << "Getting PVT with implementation " << implementation;
    if (implementation.find("_PVT") == std::string::npos)
        {
            std::cerr << "Error in configuration file: please set PVT.implementation=RTKLIB_PVT\n";
            return nullptr;
        }
    unsigned int Galileo_channels = configuration->property("Channels_1B.count", 0);
    Galileo_channels += configuration->property("Channels_5X.count", 0);
    Galileo_channels += configuration->property("Channels_7X.count", 0);
    Galileo_channels += configuration->property("Channels_E6.count", 0);
    unsigned int GPS_channels = configuration->property("Channels_1C.count", 0);
    GPS_channels += configuration->property("Channels_2S.count", 0);
    GPS_channels += configuration->property("Channels_L5.count", 0);
    unsigned int Glonass_channels = configuration->property("Channels_1G.count", 0);
    Glonass_channels += configuration->property("Channels_2G.count", 0);
    unsigned int Beidou_channels = configuration->property("Channels_B1.count", 0);
    Beidou_channels += configuration->property("Channels_B3.count", 0);
    return GetBlock(configuration, "PVT",
        Galileo_channels + GPS_channels + Glonass_channels + Beidou_channels, 0);
}


// ************************** GNSS CHANNEL *************************************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel(
    const ConfigurationInterface* configuration,
    const std::string& signal,
    int channel,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    // "appendix" is added to the "role" with the aim of Acquisition, Tracking and Telemetry Decoder adapters
    // can find their specific configurations for channels
    std::string aux = configuration->property("Acquisition_" + signal + std::to_string(channel) + impl_prop, std::string("W"));
    std::string appendix1;
    if (aux != "W")
        {
            appendix1 = std::to_string(channel);
        }

    aux = configuration->property("Tracking_" + signal + std::to_string(channel) + impl_prop, std::string("W"));
    std::string appendix2;
    if (aux != "W")
        {
            appendix2 = std::to_string(channel);
        }

    aux = configuration->property("TelemetryDecoder_" + signal + std::to_string(channel) + impl_prop, std::string("W"));
    std::string appendix3;
    if (aux != "W")
        {
            appendix3 = std::to_string(channel);
        }

    // Automatically detect input data type
    const std::string default_item_type("gr_complex");
    std::string acq_item_type = configuration->property("Acquisition_" + signal + appendix1 + item_prop, default_item_type);
    std::string trk_item_type = configuration->property("Tracking_" + signal + appendix2 + item_prop, default_item_type);
    if (acq_item_type != trk_item_type)
        {
            std::cerr << "Configuration error: Acquisition and Tracking blocks must have the same input data type!\n";
            return nullptr;
        }

    LOG(INFO) << "Instantiating Channel " << channel
              << " with Acquisition Implementation: "
              << configuration->property("Acquisition_" + signal + appendix1 + impl_prop, std::string("W"))
              << ", Tracking Implementation: "
              << configuration->property("Tracking_" + signal + appendix2 + impl_prop, std::string("W"))
              << ", Telemetry Decoder implementation: "
              << configuration->property("TelemetryDecoder_" + signal + appendix3 + impl_prop, std::string("W"));

    std::unique_ptr<AcquisitionInterface> acq_ = GetAcqBlock(configuration, "Acquisition_" + signal + appendix1, 1, 0);
    std::unique_ptr<TrackingInterface> trk_ = GetTrkBlock(configuration, "Tracking_" + signal + appendix2, 1, 1);
    std::unique_ptr<TelemetryDecoderInterface> tlm_ = GetTlmBlock(configuration, "TelemetryDecoder_" + signal + appendix3, 1, 1);

    if (acq_ == nullptr or trk_ == nullptr or tlm_ == nullptr)
        {
            return nullptr;
        }
    if (trk_->item_size() == 0)
        {
            std::cerr << "Configuration error: " << trk_->role() << item_prop << "=" << acq_item_type << " is not defined for implementation " << trk_->implementation() << '\n';
            return nullptr;
        }

    std::unique_ptr<GNSSBlockInterface> channel_ = std::make_unique<Channel>(configuration, channel,
        std::move(acq_),
        std::move(trk_),
        std::move(tlm_),
        "Channel", signal, queue);

    return channel_;
}


std::unique_ptr<std::vector<std::unique_ptr<GNSSBlockInterface>>> GNSSBlockFactory::GetChannels(
    const ConfigurationInterface* configuration,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    int channel_absolute_id = 0;

    const unsigned int Channels_1C_count = configuration->property("Channels_1C.count", 0);
    const unsigned int Channels_1B_count = configuration->property("Channels_1B.count", 0);
    const unsigned int Channels_1G_count = configuration->property("Channels_1G.count", 0);
    const unsigned int Channels_2G_count = configuration->property("Channels_2G.count", 0);
    const unsigned int Channels_2S_count = configuration->property("Channels_2S.count", 0);
    const unsigned int Channels_5X_count = configuration->property("Channels_5X.count", 0);
    const unsigned int Channels_L5_count = configuration->property("Channels_L5.count", 0);
    const unsigned int Channels_B1_count = configuration->property("Channels_B1.count", 0);
    const unsigned int Channels_B3_count = configuration->property("Channels_B3.count", 0);
    const unsigned int Channels_7X_count = configuration->property("Channels_7X.count", 0);
    const unsigned int Channels_E6_count = configuration->property("Channels_E6.count", 0);

    const unsigned int total_channels = Channels_1C_count +
                                        Channels_1B_count +
                                        Channels_1G_count +
                                        Channels_2S_count +
                                        Channels_2G_count +
                                        Channels_5X_count +
                                        Channels_L5_count +
                                        Channels_B1_count +
                                        Channels_B3_count +
                                        Channels_7X_count +
                                        Channels_E6_count;

    auto channels = std::make_unique<std::vector<std::unique_ptr<GNSSBlockInterface>>>(total_channels);
    try
        {
            // **************** GPS L1 C/A CHANNELS ****************************
            LOG(INFO) << "Getting " << Channels_1C_count << " GPS L1 C/A channels";

            for (unsigned int i = 0; i < Channels_1C_count; i++)
                {
                    // Store the channel into the vector of channels
                    channels->at(channel_absolute_id) = GetChannel(configuration,
                        std::string("1C"),
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GPS L2C (M) CHANNELS ***************************
            LOG(INFO) << "Getting " << Channels_2S_count << " GPS L2C (M) channels";

            for (unsigned int i = 0; i < Channels_2S_count; i++)
                {
                    // Store the channel into the vector of channels
                    channels->at(channel_absolute_id) = GetChannel(configuration,
                        std::string("2S"),
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GPS L5 CHANNELS ********************************
            LOG(INFO) << "Getting " << Channels_L5_count << " GPS L5 channels";

            for (unsigned int i = 0; i < Channels_L5_count; i++)
                {
                    // Store the channel into the vector of channels
                    channels->at(channel_absolute_id) = GetChannel(configuration,
                        std::string("L5"),
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GALILEO E1 B (I/NAV OS) CHANNELS ***************
            LOG(INFO) << "Getting " << Channels_1B_count << " GALILEO E1 B (I/NAV OS) channels";

            for (unsigned int i = 0; i < Channels_1B_count; i++)
                {
                    // Store the channel into the vector of channels
                    channels->at(channel_absolute_id) = GetChannel(configuration,
                        std::string("1B"),
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GALILEO E5a I (F/NAV OS) CHANNELS **************
            LOG(INFO) << "Getting " << Channels_5X_count << " GALILEO E5a I (F/NAV OS) channels";

            for (unsigned int i = 0; i < Channels_5X_count; i++)
                {
                    // Store the channel into the vector of channels
                    channels->at(channel_absolute_id) = GetChannel(configuration,
                        std::string("5X"),
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GALILEO E6 (B/C HAS) CHANNELS **************
            LOG(INFO) << "Getting " << Channels_E6_count << " GALILEO E6 (B/C HAS) channels";

            for (unsigned int i = 0; i < Channels_E6_count; i++)
                {
                    // Store the channel into the vector of channels
                    channels->at(channel_absolute_id) = GetChannel(configuration,
                        std::string("E6"),
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GLONASS L1 C/A CHANNELS ************************
            LOG(INFO) << "Getting " << Channels_1G_count << " GLONASS L1 C/A channels";

            for (unsigned int i = 0; i < Channels_1G_count; i++)
                {
                    // Store the channel into the vector of channels
                    channels->at(channel_absolute_id) = GetChannel(configuration,
                        std::string("1G"),
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GLONASS L2 C/A CHANNELS ************************
            LOG(INFO) << "Getting " << Channels_2G_count << " GLONASS L2 C/A channels";

            for (unsigned int i = 0; i < Channels_2G_count; i++)
                {
                    // Store the channel into the vector of channels
                    channels->at(channel_absolute_id) = GetChannel(configuration,
                        std::string("2G"),
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** BEIDOU B1I CHANNELS ****************************
            LOG(INFO) << "Getting " << Channels_B1_count << " BEIDOU B1I channels";

            for (unsigned int i = 0; i < Channels_B1_count; i++)
                {
                    // Store the channel into the vector of channels
                    channels->at(channel_absolute_id) = GetChannel(configuration,
                        std::string("B1"),
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** BEIDOU B3I CHANNELS ****************************
            LOG(INFO) << "Getting " << Channels_B3_count << " BEIDOU B3I channels";

            for (unsigned int i = 0; i < Channels_B3_count; i++)
                {
                    // Store the channel into the vector of channels
                    channels->at(channel_absolute_id) = GetChannel(configuration,
                        std::string("B3"),
                        channel_absolute_id,
                        queue);
                    channel_absolute_id++;
                }

            // **************** GALILEO E5b I (I/NAV OS) CHANNELS **************
            LOG(INFO) << "Getting " << Channels_7X_count << " GALILEO E5b I (I/NAV OS) channels";

            for (unsigned int i = 0; i < Channels_7X_count; i++)
                {
                    // Store the channel into the vector of channels
                    channels->at(channel_absolute_id) = GetChannel(configuration,
                        std::string("7X"),
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
    unsigned int in_streams,
    unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    std::unique_ptr<GNSSBlockInterface> block;
    const std::string implementation = configuration->property(role + impl_prop, "Pass_Through"s);

    try
        {
            // PASS THROUGH ------------------------------------------------------------
            if (implementation == "Pass_Through")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<Pass_Through>(configuration, role, in_streams, out_streams);
                    block = std::move(block_);
                }

            // SIGNAL SOURCES ----------------------------------------------------------
            else if (implementation == "Fifo_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<FifoSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            else if (implementation == "File_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<FileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            else if (implementation == "File_Timestamp_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<FileTimestampSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            else if (implementation == "Multichannel_File_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<MultichannelFileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
#if RAW_UDP
            else if (implementation == "Custom_UDP_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<CustomUDPSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
#endif
            else if (implementation == "Nsr_File_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<NsrFileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            else if (implementation == "Two_Bit_Cpx_File_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<TwoBitCpxFileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            else if (implementation == "Four_Bit_Cpx_File_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<FourBitCpxFileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            else if (implementation == "Two_Bit_Packed_File_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<TwoBitPackedFileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            else if (implementation == "Spir_File_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<SpirFileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            else if (implementation == "Spir_GSS6450_File_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<SpirGSS6450FileSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            else if (implementation == "RtlTcp_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<RtlTcpSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            else if (implementation == "Labsat_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<LabsatSignalSource>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
#if UHD_DRIVER
            else if (implementation == "UHD_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<UhdSignalSource>(configuration, role, in_streams,
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

#if LIMESDR_DRIVER
            else if (implementation == "Limesdr_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface>
                        block_ = std::make_unique<LimesdrSignalSource>(configuration, role, in_streams,
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
            else if (implementation == "Ad936x_Custom_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<Ad936xCustomSignalSource>(configuration, role, in_streams,
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

#if ENABLE_FPGA and AD9361_DRIVER
            else if (implementation == "ADRV9361_Z7035_Signal_Source_FPGA")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<Adrv9361z7035SignalSourceFPGA>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
            else if (implementation == "FMCOMMS5_Signal_Source_FPGA")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<Fmcomms5SignalSourceFPGA>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
#endif

#if ENABLE_FPGA and MAX2771_DRIVER
            else if (implementation == "MAX2771_EVKIT_Signal_Source_FPGA")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<MAX2771EVKITSignalSourceFPGA>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
#endif

#if ENABLE_FPGA and DMA_PROXY_DRIVER
            else if (implementation == "DMA_Signal_Source_FPGA")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<DMASignalSourceFPGA>(configuration, role, in_streams,
                        out_streams, queue);
                    block = std::move(block_);
                }
#endif

#if ZEROMQ_DRIVER
            else if (implementation == "ZMQ_Signal_Source")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<ZmqSignalSource>(configuration, role, in_streams,
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

            // INPUT FILTER ------------------------------------------------------------
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

            // RESAMPLER ---------------------------------------------------------------
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

            // ACQUISITION BLOCKS ------------------------------------------------------
            else if (implementation == "GPS_L1_CA_PCPS_Acquisition")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaPcpsAcquisition>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
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
            else if (implementation == "GPS_L5i_PCPS_Acquisition")
                {
                    std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL5iPcpsAcquisition>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1PcpsAmbiguousAcquisition>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
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
            else if (implementation == "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1PcpsQuickSyncAmbiguousAcquisition>(configuration, role, in_streams,
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
            else if (implementation == "Galileo_E5b_PCPS_Acquisition")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5bPcpsAcquisition>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E6_PCPS_Acquisition")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE6PcpsAcquisition>(configuration, role, in_streams,
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
#if OPENCL_BLOCKS
            else if (implementation == "GPS_L1_CA_PCPS_OpenCl_Acquisition")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaPcpsOpenClAcquisition>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
#endif
#if ENABLE_FPGA
            else if (implementation == "GPS_L1_CA_PCPS_Acquisition_FPGA")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaPcpsAcquisitionFpga>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition_FPGA")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1PcpsAmbiguousAcquisitionFpga>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "GPS_L2_M_PCPS_Acquisition_FPGA")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL2MPcpsAcquisitionFpga>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "GPS_L5i_PCPS_Acquisition_FPGA")
                {
                    std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL5iPcpsAcquisitionFpga>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E5a_Pcps_Acquisition_FPGA")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5aPcpsAcquisitionFpga>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E5b_PCPS_Acquisition_FPGA")
                {
                    std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE5bPcpsAcquisitionFpga>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
#endif

            // TRACKING BLOCKS ---------------------------------------------------------
            else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaDllPllTracking>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "GPS_L1_CA_Gaussian_Tracking")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaGaussianTracking>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "GPS_L1_CA_KF_Tracking")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaKfTracking>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
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
            else if ((implementation == "GPS_L5i_DLL_PLL_Tracking") or (implementation == "GPS_L5_DLL_PLL_Tracking"))
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL5DllPllTracking>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1DllPllVemlTracking>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
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
            else if (implementation == "Galileo_E5b_DLL_PLL_Tracking")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5bDllPllTracking>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E6_DLL_PLL_Tracking")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE6DllPllTracking>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
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
#if CUDA_GPU_ACCEL
            else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_GPU")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL1CaDllPllTrackingGPU>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
#endif
#if ENABLE_FPGA
            else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_FPGA")
                {
                    std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaDllPllTrackingFpga>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_FPGA")
                {
                    std::unique_ptr<TrackingInterface> block_ = std::make_unique<GalileoE1DllPllVemlTrackingFpga>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "GPS_L2_M_DLL_PLL_Tracking_FPGA")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL2MDllPllTrackingFpga>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if ((implementation == "GPS_L5i_DLL_PLL_Tracking_FPGA") or (implementation == "GPS_L5_DLL_PLL_Tracking_FPGA"))
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GpsL5DllPllTrackingFpga>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_FPGA")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5aDllPllTrackingFpga>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
#endif

            // TELEMETRY DECODERS ------------------------------------------------------
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
            else if (implementation == "SBAS_L1_Telemetry_Decoder")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<SbasL1TelemetryDecoder>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E1B_Telemetry_Decoder")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE1BTelemetryDecoder>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E5a_Telemetry_Decoder")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5aTelemetryDecoder>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E5b_Telemetry_Decoder")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE5bTelemetryDecoder>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }
            else if (implementation == "Galileo_E6_Telemetry_Decoder")
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<GalileoE6TelemetryDecoder>(configuration, role, in_streams,
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

            // OBSERVABLES -------------------------------------------------------------
            else if ((implementation == "Hybrid_Observables") || (implementation == "GPS_L1_CA_Observables") || (implementation == "GPS_L2C_Observables") ||
                     (implementation == "Galileo_E5A_Observables"))
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<HybridObservables>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }

            // PVT ---------------------------------------------------------------------
            else if ((implementation == "RTKLIB_PVT") || (implementation == "GPS_L1_CA_PVT") || (implementation == "Galileo_E1_PVT") || (implementation == "Hybrid_PVT"))
                {
                    std::unique_ptr<GNSSBlockInterface> block_ = std::make_unique<Rtklib_Pvt>(configuration, role, in_streams,
                        out_streams);
                    block = std::move(block_);
                }

            else
                {
                    std::cerr << "Configuration error in " << role << " block: implementation " + implementation + " is not available.\n"s;
                    block = nullptr;
                }
        }
    catch (const std::exception& e)
        {
            LOG(INFO) << "Exception raised while instantiating the block: " << e.what();
            std::cout << "Configuration error in " << role << " block, implementation " << (implementation == "Wrong"s ? "not defined."s : implementation) << ". The error was:\n"
                      << e.what() << '\n';
            std::cout << "GNSS-SDR program ended.\n";
            exit(1);
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
    unsigned int in_streams,
    unsigned int out_streams)
{
    std::unique_ptr<AcquisitionInterface> block;
    const std::string implementation = configuration->property(role + impl_prop, "Wrong"s);

    // ACQUISITION BLOCKS ------------------------------------------------------
    if (implementation == "GPS_L1_CA_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL1CaPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
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
    else if (implementation == "GPS_L5i_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL5iPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE1PcpsAmbiguousAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
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
    else if (implementation == "Galileo_E5b_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE5bPcpsAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E6_PCPS_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE6PcpsAcquisition>(configuration, role, in_streams,
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
#if OPENCL_BLOCKS
    else if (implementation == "GPS_L1_CA_PCPS_OpenCl_Acquisition")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL1CaPcpsOpenClAcquisition>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
#if ENABLE_FPGA
    else if (implementation == "GPS_L1_CA_PCPS_Acquisition_FPGA")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL1CaPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition_FPGA")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE1PcpsAmbiguousAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L2_M_PCPS_Acquisition_FPGA")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL2MPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L5i_PCPS_Acquisition_FPGA")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GpsL5iPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E5a_Pcps_Acquisition_FPGA")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE5aPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E5b_PCPS_Acquisition_FPGA")
        {
            std::unique_ptr<AcquisitionInterface> block_ = std::make_unique<GalileoE5bPcpsAcquisitionFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif

    else
        {
            std::cerr << "Configuration error in " << role << " block: implementation " << (implementation == "Wrong"s ? "not defined."s : implementation + " not available."s) << '\n';
            block = nullptr;
        }
    return block;
}


std::unique_ptr<TrackingInterface> GNSSBlockFactory::GetTrkBlock(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
{
    std::unique_ptr<TrackingInterface> block;
    const std::string implementation = configuration->property(role + impl_prop, "Wrong"s);

    // TRACKING BLOCKS ---------------------------------------------------------
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L1_CA_Gaussian_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaGaussianTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L1_CA_KF_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaKfTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
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
    else if (implementation == "Galileo_E5b_DLL_PLL_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GalileoE5bDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E6_DLL_PLL_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GalileoE6DllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L2_M_DLL_PLL_Tracking")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL2MDllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if ((implementation == "GPS_L5i_DLL_PLL_Tracking") or (implementation == "GPS_L5_DLL_PLL_Tracking"))
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL5DllPllTracking>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
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
#if CUDA_GPU_ACCEL
    else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_GPU")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaDllPllTrackingGPU>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
#if ENABLE_FPGA
    else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_FPGA")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL1CaDllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_FPGA")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GalileoE1DllPllVemlTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "GPS_L2_M_DLL_PLL_Tracking_FPGA")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL2MDllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if ((implementation == "GPS_L5i_DLL_PLL_Tracking_FPGA") or (implementation == "GPS_L5_DLL_PLL_Tracking_FPGA"))
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GpsL5DllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_FPGA")
        {
            std::unique_ptr<TrackingInterface> block_ = std::make_unique<GalileoE5aDllPllTrackingFpga>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
#endif
    else
        {
            std::cerr << "Configuration error in " << role << " block: implementation " << (implementation == "Wrong"s ? "not defined."s : implementation + " not available."s) << '\n';
            block = nullptr;
        }
    return block;
}


std::unique_ptr<TelemetryDecoderInterface> GNSSBlockFactory::GetTlmBlock(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
{
    std::unique_ptr<TelemetryDecoderInterface> block;
    const std::string implementation = configuration->property(role + impl_prop, "Wrong"s);

    // TELEMETRY DECODERS ------------------------------------------------------
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
    else if (implementation == "Galileo_E5b_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<GalileoE5bTelemetryDecoder>(configuration, role, in_streams,
                out_streams);
            block = std::move(block_);
        }
    else if (implementation == "Galileo_E6_Telemetry_Decoder")
        {
            std::unique_ptr<TelemetryDecoderInterface> block_ = std::make_unique<GalileoE6TelemetryDecoder>(configuration, role, in_streams,
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
            std::cerr << "Configuration error in " << role << " block: implementation " << (implementation == "Wrong"s ? "not defined."s : implementation + " not available."s) << '\n';
            block = nullptr;
        }

    return block;
}
