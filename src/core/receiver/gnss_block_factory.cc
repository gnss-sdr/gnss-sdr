/*!
 * \file gnss_block_factory.cc
 * \brief  This class implements a factory that returns instances of GNSS blocks.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *         Marc Majoral, 2018. mmajoral(at)cttc.es
 *         Carles Fernandez-Prades, 2014-2020. cfernandez(at)cttc.es
 *         Mathieu Favreau, 2025-2026. favreau.mathieu(at)hotmail.com
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
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "gnss_block_factory.h"
#include "acquisition_interface.h"
#include "array_signal_conditioner.h"
#include "beamformer_filter.h"
#include "beidou_b1i_telemetry_decoder_gs.h"
#include "beidou_b3i_telemetry_decoder_gs.h"
#include "byte_to_short.h"
#include "channel.h"
#include "configuration_interface.h"
#include "cshort_to_grcomplex.h"
#include "direct_resampler_conditioner.h"
#include "dll_pll_tracking_adapter.h"
#include "fifo_signal_source.h"
#include "file_signal_source.h"
#include "file_timestamp_signal_source.h"
#include "fir_filter.h"
#include "four_bit_cpx_file_signal_source.h"
#include "freq_xlating_fir_filter.h"
#include "galileo_e1_pcps_8ms_ambiguous_acquisition.h"
#include "galileo_e1_pcps_cccwsr_ambiguous_acquisition.h"
#include "galileo_e1_pcps_quicksync_ambiguous_acquisition.h"
#include "galileo_e1_pcps_tong_ambiguous_acquisition.h"
#include "galileo_e1_tcp_connector_tracking.h"
#include "galileo_e5a_noncoherent_iq_acquisition_caf.h"
#include "galileo_telemetry_decoder_gs.h"
#include "glonass_l1_ca_telemetry_decoder_gs.h"
#include "glonass_l2_ca_telemetry_decoder_gs.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_make_unique.h"
#include "gnss_sdr_string_literals.h"
#include "gps_l1_ca_gaussian_tracking.h"
#include "gps_l1_ca_kf_tracking.h"
#include "gps_l1_ca_pcps_acquisition_fine_doppler.h"
#include "gps_l1_ca_pcps_assisted_acquisition.h"
#include "gps_l1_ca_pcps_quicksync_acquisition.h"
#include "gps_l1_ca_pcps_tong_acquisition.h"
#include "gps_l1_ca_tcp_connector_tracking.h"
#include "gps_l1_ca_telemetry_decoder_gs.h"
#include "gps_l2c_telemetry_decoder_gs.h"
#include "gps_l5_telemetry_decoder_gs.h"
#include "hybrid_observables.h"
#include "ibyte_to_cbyte.h"
#include "ibyte_to_complex.h"
#include "ibyte_to_cshort.h"
#include "ishort_to_complex.h"
#include "ishort_to_cshort.h"
#include "labsat_signal_source.h"
#include "mmse_resampler_conditioner.h"
#include "multichannel_file_signal_source.h"
#include "notch_filter.h"
#include "notch_filter_lite.h"
#include "nsr_file_signal_source.h"
#include "ntlab_file_signal_source.h"
#include "pass_through.h"
#include "pcps_acquisition_adapter.h"
#include "pulse_blanking_filter.h"
#include "rtklib_pvt.h"
#include "rtl_tcp_signal_source.h"
#include "sbas_l1_telemetry_decoder_gs.h"
#include "signal_conditioner.h"
#include "signal_flag.h"
#include "spir_file_signal_source.h"
#include "spir_gss6450_file_signal_source.h"
#include "telemetry_decoder_adapter.h"
#include "telemetry_decoder_interface.h"
#include "tlm_conf.h"
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
#include "dll_pll_tracking_adapter_fpga.h"
#include "pcps_acquisition_adapter_fpga.h"
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

#if AD936X_SDR_DRIVER
#include "ad936x_custom_signal_source.h"
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

#if ENABLE_ION_SOURCE
#undef Owner
#include "ion_gsms_signal_source.h"
#endif

using namespace std::string_literals;

namespace
{
auto const impl_prop = ".implementation"s;  // "implementation" property; used nearly universally
auto const item_prop = ".item_type"s;       // "item_type" property

auto findRole(const ConfigurationInterface* configuration, const std::string& base, int ID)
{
    const auto role = base + std::to_string(ID);

    // Legacy behavior: pass -1 for unadorned property.
    // Current behavior: if there is no "Tag0" use "Tag" instead
    if (ID < 1 && !configuration->is_present(role + impl_prop))
        {
            return base;  //  legacy format
        }

    return role;
};

std::string get_role_name(const ConfigurationInterface* configuration, const std::string& role_prefix, const std::string& signal, int channel)
{
    const auto role_name = role_prefix + signal + std::to_string(channel);

    if (configuration->is_present(role_name + impl_prop))
        {
            return role_name;
        }

    return role_prefix + signal;
}

const auto signal_mapping = std::vector<std::pair<std::string, std::string>>{
    {"1C", "GPS L1 C/A"},
    {"2S", "GPS L2C (M)"},
    {"L5", "GPS L5"},
    {"1B", "GALILEO E1 B (I/NAV OS)"},
    {"5X", "GALILEO E5a I (F/NAV OS)"},
    {"E6", "GALILEO E6 (B/C HAS)"},
    {"1G", "GLONASS L1 C/A"},
    {"2G", "GLONASS L2 C/A"},
    {"B1", "BEIDOU B1I"},
    {"B3", "BEIDOU B3I"},
    {"7X", "GALILEO E5b I (I/NAV OS)"},
    {"J1", "QZSS L1 C/A"},
    {"J5", "QZSS L5"},
};

unsigned int get_channel_count(const ConfigurationInterface* configuration)
{
    unsigned int channel_count = 0;

    for (const auto& entry : signal_mapping)
        {
            const auto& signal_str = entry.first;
            channel_count += configuration->property("Channels_" + signal_str + ".count", 0);
        }

    return channel_count;
}

template <typename F>
auto get_block(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    F get_block_func)
{
    const std::string implementation = configuration->property(role + impl_prop, "Wrong"s);
    auto block = get_block_func(implementation, configuration, role, in_streams, out_streams);

    if (!block)
        {
            std::cerr << "Configuration error in " << role << " block: implementation " << (implementation == "Wrong"s ? "not defined."s : implementation + " not available."s) << '\n';
        }

    return block;
}


std::unique_ptr<SignalSourceInterface> get_signal_source_block(
    const std::string& implementation,
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue)
{
    if (implementation == "Fifo_Signal_Source")
        {
            return std::make_unique<FifoSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "File_Signal_Source")
        {
            return std::make_unique<FileSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "File_Timestamp_Signal_Source")
        {
            return std::make_unique<FileTimestampSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "Multichannel_File_Signal_Source")
        {
            return std::make_unique<MultichannelFileSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#if RAW_UDP
    else if (implementation == "Custom_UDP_Signal_Source")
        {
            return std::make_unique<CustomUDPSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#endif
    else if (implementation == "Nsr_File_Signal_Source")
        {
            return std::make_unique<NsrFileSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "Two_Bit_Cpx_File_Signal_Source")
        {
            return std::make_unique<TwoBitCpxFileSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "Four_Bit_Cpx_File_Signal_Source")
        {
            return std::make_unique<FourBitCpxFileSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "Two_Bit_Packed_File_Signal_Source")
        {
            return std::make_unique<TwoBitPackedFileSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "NTLab_File_Signal_Source")
        {
            return std::make_unique<NTLabFileSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "Spir_File_Signal_Source")
        {
            return std::make_unique<SpirFileSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "Spir_GSS6450_File_Signal_Source")
        {
            return std::make_unique<SpirGSS6450FileSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "RtlTcp_Signal_Source")
        {
            return std::make_unique<RtlTcpSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "Labsat_Signal_Source")
        {
            return std::make_unique<LabsatSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#if UHD_DRIVER
    else if (implementation == "UHD_Signal_Source")
        {
            return std::make_unique<UhdSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if ENABLE_ION_SOURCE
    else if (implementation == "ION_GSMS_Signal_Source")
        {
            return std::make_unique<IONGSMSSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if RAW_ARRAY_DRIVER
    else if (implementation == "Raw_Array_Signal_Source")
        {
            return std::make_unique<RawArraySignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if OSMOSDR_DRIVER
    else if (implementation == "Osmosdr_Signal_Source")
        {
            return std::make_unique<OsmosdrSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if LIMESDR_DRIVER
    else if (implementation == "Limesdr_Signal_Source")
        {
            return std::make_unique<LimesdrSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if PLUTOSDR_DRIVER
    else if (implementation == "Plutosdr_Signal_Source")
        {
            return std::make_unique<PlutosdrSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if PLUTOSDR_DRIVER || AD936X_SDR_DRIVER
    else if (implementation == "Ad936x_Custom_Signal_Source")
        {
            return std::make_unique<Ad936xCustomSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if FMCOMMS2_DRIVER
    else if (implementation == "Fmcomms2_Signal_Source")
        {
            return std::make_unique<Fmcomms2SignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if FLEXIBAND_DRIVER
    else if (implementation == "Flexiband_Signal_Source")
        {
            return std::make_unique<FlexibandSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if ENABLE_FPGA and AD9361_DRIVER
    else if (implementation == "ADRV9361_Z7035_Signal_Source_FPGA")
        {
            return std::make_unique<Adrv9361z7035SignalSourceFPGA>(configuration, role, in_streams, out_streams, queue);
        }
    else if (implementation == "FMCOMMS5_Signal_Source_FPGA")
        {
            return std::make_unique<Fmcomms5SignalSourceFPGA>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if ENABLE_FPGA and MAX2771_DRIVER
    else if (implementation == "MAX2771_EVKIT_Signal_Source_FPGA")
        {
            return std::make_unique<MAX2771EVKITSignalSourceFPGA>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if ENABLE_FPGA and DMA_PROXY_DRIVER
    else if (implementation == "DMA_Signal_Source_FPGA")
        {
            return std::make_unique<DMASignalSourceFPGA>(configuration, role, in_streams, out_streams, queue);
        }
#endif
#if ZEROMQ_DRIVER
    else if (implementation == "ZMQ_Signal_Source")
        {
            return std::make_unique<ZmqSignalSource>(configuration, role, in_streams, out_streams, queue);
        }
#endif

    return nullptr;
}


std::unique_ptr<AcquisitionInterface> get_acq_block(
    const std::string& implementation,
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
{
    // ACQUISITION BLOCKS ------------------------------------------------------
    if (implementation == "GPS_L1_CA_PCPS_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, GPS_1C);
        }
    else if (implementation == "GPS_L1_CA_PCPS_Assisted_Acquisition")
        {
            return std::make_unique<GpsL1CaPcpsAssistedAcquisition>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "GPS_L1_CA_PCPS_Tong_Acquisition")
        {
            return std::make_unique<GpsL1CaPcpsTongAcquisition>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler")
        {
            return std::make_unique<GpsL1CaPcpsAcquisitionFineDoppler>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "GPS_L1_CA_PCPS_QuickSync_Acquisition")
        {
            return std::make_unique<GpsL1CaPcpsQuickSyncAcquisition>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "GPS_L2_M_PCPS_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, GPS_2S);
        }
    else if (implementation == "GPS_L5i_PCPS_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, GPS_L5);
        }
    else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, GAL_1B);
        }
    else if (implementation == "Galileo_E1_PCPS_8ms_Ambiguous_Acquisition")
        {
            return std::make_unique<GalileoE1Pcps8msAmbiguousAcquisition>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "Galileo_E1_PCPS_Tong_Ambiguous_Acquisition")
        {
            return std::make_unique<GalileoE1PcpsTongAmbiguousAcquisition>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "Galileo_E1_PCPS_CCCWSR_Ambiguous_Acquisition")
        {
            return std::make_unique<GalileoE1PcpsCccwsrAmbiguousAcquisition>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "Galileo_E1_PCPS_QuickSync_Ambiguous_Acquisition")
        {
            return std::make_unique<GalileoE1PcpsQuickSyncAmbiguousAcquisition>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "Galileo_E5a_Noncoherent_IQ_Acquisition_CAF")
        {
            return std::make_unique<GalileoE5aNoncoherentIQAcquisitionCaf>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "Galileo_E5a_Pcps_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, GAL_E5a);
        }
    else if (implementation == "Galileo_E5b_PCPS_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, GAL_E5b);
        }
    else if (implementation == "Galileo_E6_PCPS_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, GAL_E6);
        }
    else if (implementation == "GLONASS_L1_CA_PCPS_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, GLO_1G);
        }
    else if (implementation == "GLONASS_L2_CA_PCPS_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, GLO_2G);
        }
    else if (implementation == "BEIDOU_B1I_PCPS_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, BDS_B1);
        }
    else if (implementation == "BEIDOU_B3I_PCPS_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, BDS_B3);
        }
    else if (implementation == "QZSS_L1_PCPS_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, QZS_J1);
        }
    else if (implementation == "QZSS_L5i_PCPS_Acquisition")
        {
            return std::make_unique<PcpsAcquisitionAdapter>(configuration, role, implementation, in_streams, out_streams, QZS_J5);
        }
#if OPENCL_BLOCKS
    else if (implementation == "GPS_L1_CA_PCPS_OpenCl_Acquisition")
        {
            return std::make_unique<GpsL1CaPcpsOpenClAcquisition>(configuration, role, in_streams, out_streams);
        }
#endif
#if ENABLE_FPGA
    else if (implementation == "GPS_L1_CA_PCPS_Acquisition_FPGA")
        {
            return std::make_unique<PcpsAcquisitionAdapterFpga>(configuration, role, implementation, in_streams, out_streams, GPS_1C);
        }
    else if (implementation == "Galileo_E1_PCPS_Ambiguous_Acquisition_FPGA")
        {
            return std::make_unique<PcpsAcquisitionAdapterFpga>(configuration, role, implementation, in_streams, out_streams, GAL_1B);
        }
    else if (implementation == "GPS_L2_M_PCPS_Acquisition_FPGA")
        {
            return std::make_unique<PcpsAcquisitionAdapterFpga>(configuration, role, implementation, in_streams, out_streams, GPS_2S);
        }
    else if (implementation == "GPS_L5i_PCPS_Acquisition_FPGA")
        {
            return std::make_unique<PcpsAcquisitionAdapterFpga>(configuration, role, implementation, in_streams, out_streams, GPS_L5);
        }
    else if (implementation == "Galileo_E5a_Pcps_Acquisition_FPGA")
        {
            return std::make_unique<PcpsAcquisitionAdapterFpga>(configuration, role, implementation, in_streams, out_streams, GAL_E5a);
        }
    else if (implementation == "Galileo_E5b_PCPS_Acquisition_FPGA")
        {
            return std::make_unique<PcpsAcquisitionAdapterFpga>(configuration, role, implementation, in_streams, out_streams, GAL_E5b);
        }
#endif

    return nullptr;
}


std::unique_ptr<TrackingInterface> get_trk_block(
    const std::string& implementation,
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
{
    if (implementation == "GPS_L1_CA_DLL_PLL_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, GPS_1C);
        }
    else if (implementation == "GPS_L1_CA_Gaussian_Tracking")
        {
            return std::make_unique<GpsL1CaGaussianTracking>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "GPS_L1_CA_KF_Tracking")
        {
            return std::make_unique<GpsL1CaKfTracking>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "GPS_L1_CA_TCP_CONNECTOR_Tracking")
        {
            return std::make_unique<GpsL1CaTcpConnectorTracking>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, GAL_1B);
        }
    else if (implementation == "Galileo_E1_TCP_CONNECTOR_Tracking")
        {
            return std::make_unique<GalileoE1TcpConnectorTracking>(configuration, role, in_streams, out_streams);
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, GAL_E5a);
        }
    else if (implementation == "Galileo_E5b_DLL_PLL_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, GAL_E5b);
        }
    else if (implementation == "Galileo_E6_DLL_PLL_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, GAL_E6);
        }
    else if (implementation == "GPS_L2_M_DLL_PLL_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, GPS_2S);
        }
    else if ((implementation == "GPS_L5i_DLL_PLL_Tracking") or (implementation == "GPS_L5_DLL_PLL_Tracking"))
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, GPS_L5);
        }
    else if (implementation == "GLONASS_L1_CA_DLL_PLL_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, GLO_1G);
        }
    else if (implementation == "GLONASS_L2_CA_DLL_PLL_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, GLO_2G);
        }
    else if (implementation == "BEIDOU_B1I_DLL_PLL_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, BDS_B1);
        }
    else if (implementation == "BEIDOU_B3I_DLL_PLL_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, BDS_B3);
        }
    else if (implementation == "QZSS_L1_CA_DLL_PLL_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, QZS_J1);
        }
    else if (implementation == "QZSS_L5_DLL_PLL_Tracking")
        {
            return std::make_unique<DllPllTrackingAdapter>(configuration, role, implementation, in_streams, out_streams, QZS_J5);
        }
#if CUDA_GPU_ACCEL
    else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_GPU")
        {
            return std::make_unique<GpsL1CaDllPllTrackingGPU>(configuration, role, in_streams, out_streams);
        }
#endif
#if ENABLE_FPGA
    else if (implementation == "GPS_L1_CA_DLL_PLL_Tracking_FPGA")
        {
            return std::make_unique<DllPllTrackingAdapterFpga>(configuration, role, implementation, in_streams, out_streams, GPS_1C);
        }
    else if (implementation == "Galileo_E1_DLL_PLL_VEML_Tracking_FPGA")
        {
            return std::make_unique<DllPllTrackingAdapterFpga>(configuration, role, implementation, in_streams, out_streams, GAL_1B);
        }
    else if (implementation == "GPS_L2_M_DLL_PLL_Tracking_FPGA")
        {
            return std::make_unique<DllPllTrackingAdapterFpga>(configuration, role, implementation, in_streams, out_streams, GPS_2S);
        }
    else if ((implementation == "GPS_L5i_DLL_PLL_Tracking_FPGA") or (implementation == "GPS_L5_DLL_PLL_Tracking_FPGA"))
        {
            return std::make_unique<DllPllTrackingAdapterFpga>(configuration, role, implementation, in_streams, out_streams, GPS_L5);
        }
    else if (implementation == "Galileo_E5a_DLL_PLL_Tracking_FPGA")
        {
            return std::make_unique<DllPllTrackingAdapterFpga>(configuration, role, implementation, in_streams, out_streams, GAL_E5a);
        }
#endif

    return nullptr;
}


Tlm_Conf get_tlm_conf(const ConfigurationInterface* configuration, const std::string& role)
{
    Tlm_Conf conf;

    if (configuration != nullptr)
        {
            conf.SetFromConfiguration(configuration, role);
        }

    return conf;
}


Tlm_Conf get_e1_tlm_conf(const ConfigurationInterface* configuration, const std::string& role)
{
    auto tlm_parameters = get_tlm_conf(configuration, role);

    if (configuration != nullptr)
        {
            tlm_parameters.enable_reed_solomon = configuration->property(role + ".enable_reed_solomon", false);
            tlm_parameters.use_ced = configuration->property(role + ".use_reduced_ced", false);
        }

    return tlm_parameters;
}


std::unique_ptr<TelemetryDecoderInterface> get_tlm_block(
    const std::string& implementation,
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
{
    telemetry_impl_interface_sptr telemetry;

    if (implementation == "GPS_L1_CA_Telemetry_Decoder")
        {
            telemetry = gps_l1_ca_make_telemetry_decoder_gs(get_tlm_conf(configuration, role));
        }
    else if (implementation == "Galileo_E1B_Telemetry_Decoder")
        {
            telemetry = galileo_make_telemetry_decoder_gs(get_e1_tlm_conf(configuration, role), 1);
        }
    else if (implementation == "SBAS_L1_Telemetry_Decoder")
        {
            telemetry = sbas_l1_make_telemetry_decoder_gs(configuration != nullptr ? configuration->property(role + ".dump", false) : false);
        }
    else if (implementation == "Galileo_E5a_Telemetry_Decoder")
        {
            telemetry = galileo_make_telemetry_decoder_gs(get_tlm_conf(configuration, role), 2);
        }
    else if (implementation == "Galileo_E5b_Telemetry_Decoder")
        {
            telemetry = galileo_make_telemetry_decoder_gs(get_tlm_conf(configuration, role), 1);
        }
    else if (implementation == "Galileo_E6_Telemetry_Decoder")
        {
            telemetry = galileo_make_telemetry_decoder_gs(get_tlm_conf(configuration, role), 3);
        }
    else if (implementation == "GPS_L2C_Telemetry_Decoder")
        {
            telemetry = gps_l2c_make_telemetry_decoder_gs(get_tlm_conf(configuration, role));
        }
    else if (implementation == "GLONASS_L1_CA_Telemetry_Decoder")
        {
            telemetry = glonass_l1_ca_make_telemetry_decoder_gs(get_tlm_conf(configuration, role));
        }
    else if (implementation == "GLONASS_L2_CA_Telemetry_Decoder")
        {
            telemetry = glonass_l2_ca_make_telemetry_decoder_gs(get_tlm_conf(configuration, role));
        }
    else if (implementation == "GPS_L5_Telemetry_Decoder")
        {
            telemetry = gps_l5_make_telemetry_decoder_gs(get_tlm_conf(configuration, role));
        }
    else if (implementation == "BEIDOU_B1I_Telemetry_Decoder")
        {
            telemetry = beidou_b1i_make_telemetry_decoder_gs(get_tlm_conf(configuration, role));
        }
    else if (implementation == "BEIDOU_B3I_Telemetry_Decoder")
        {
            telemetry = beidou_b3i_make_telemetry_decoder_gs(get_tlm_conf(configuration, role));
        }
    else if (implementation == "QZSS_L1_Telemetry_Decoder")
        {
            telemetry = gps_l1_ca_make_telemetry_decoder_gs(get_tlm_conf(configuration, role), L1LnavSystem::QZSS);
        }
    else if (implementation == "QZSS_L5_Telemetry_Decoder")
        {
            telemetry = gps_l5_make_telemetry_decoder_gs(get_tlm_conf(configuration, role), CnavSystem::QZSS);
        }

    if (telemetry)
        {
            return std::make_unique<TelemetryDecoderAdapter>(role, implementation, in_streams, out_streams, std::move(telemetry));
        }

    return nullptr;
}


/*
 * Returns the block with the required configuration and implementation
 *
 * PLEASE ADD YOUR NEW BLOCK HERE!!
 *
 * IMPORTANT NOTE: Acquisition, Tracking and telemetry blocks are only included here for testing purposes.
 */
std::unique_ptr<GNSSBlockInterface> get_block(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue = nullptr)
{
    std::unique_ptr<GNSSBlockInterface> block;
    const std::string implementation = configuration->property(role + impl_prop, "Pass_Through"s);

    try
        {
            // PASS THROUGH ------------------------------------------------------------
            if (implementation == "Pass_Through")
                {
                    block = std::make_unique<Pass_Through>(configuration, role, in_streams, out_streams);
                }

            // SIGNAL SOURCES ----------------------------------------------------------
            else if ((block = get_signal_source_block(implementation, configuration, role, in_streams, out_streams, queue)))
                {
                }

            // DATA TYPE ADAPTER -----------------------------------------------------------
            else if (implementation == "Byte_To_Short")
                {
                    block = std::make_unique<ByteToShort>(configuration, role, in_streams, out_streams);
                }
            else if (implementation == "Ibyte_To_Cbyte")
                {
                    block = std::make_unique<IbyteToCbyte>(configuration, role, in_streams, out_streams);
                }
            else if (implementation == "Ibyte_To_Cshort")
                {
                    block = std::make_unique<IbyteToCshort>(configuration, role, in_streams, out_streams);
                }
            else if (implementation == "Ibyte_To_Complex")
                {
                    block = std::make_unique<IbyteToComplex>(configuration, role, in_streams, out_streams);
                }
            else if (implementation == "Ishort_To_Cshort")
                {
                    block = std::make_unique<IshortToCshort>(configuration, role, in_streams, out_streams);
                }
            else if (implementation == "Ishort_To_Complex")
                {
                    block = std::make_unique<IshortToComplex>(configuration, role, in_streams, out_streams);
                }
            else if (implementation == "Cshort_To_Gr_Complex")
                {
                    block = std::make_unique<CshortToGrComplex>(configuration, role, in_streams, out_streams);
                }

            // INPUT FILTER ------------------------------------------------------------
            else if (implementation == "Fir_Filter")
                {
                    block = std::make_unique<FirFilter>(configuration, role, in_streams, out_streams);
                }
            else if (implementation == "Freq_Xlating_Fir_Filter")
                {
                    block = std::make_unique<FreqXlatingFirFilter>(configuration, role, in_streams, out_streams);
                }
            else if (implementation == "Beamformer_Filter")
                {
                    block = std::make_unique<BeamformerFilter>(configuration, role, in_streams, out_streams);
                }
            else if (implementation == "Pulse_Blanking_Filter")
                {
                    block = std::make_unique<PulseBlankingFilter>(configuration, role, in_streams, out_streams);
                }
            else if (implementation == "Notch_Filter")
                {
                    block = std::make_unique<NotchFilter>(configuration, role, in_streams, out_streams);
                }
            else if (implementation == "Notch_Filter_Lite")
                {
                    block = std::make_unique<NotchFilterLite>(configuration, role, in_streams, out_streams);
                }

            // RESAMPLER ---------------------------------------------------------------
            else if (implementation == "Direct_Resampler")
                {
                    block = std::make_unique<DirectResamplerConditioner>(configuration, role, in_streams, out_streams);
                }

            else if ((implementation == "Fractional_Resampler") || (implementation == "Mmse_Resampler"))
                {
                    block = std::make_unique<MmseResamplerConditioner>(configuration, role, in_streams, out_streams);
                }

            // ACQUISITION BLOCKS ------------------------------------------------------
            else if ((block = get_acq_block(implementation, configuration, role, in_streams, out_streams)))
                {
                }

            // TRACKING BLOCKS ---------------------------------------------------------
            else if ((block = get_trk_block(implementation, configuration, role, in_streams, out_streams)))
                {
                }

            // TELEMETRY DECODERS ------------------------------------------------------
            else if ((block = get_tlm_block(implementation, configuration, role, in_streams, out_streams)))
                {
                }

            // OBSERVABLES -------------------------------------------------------------
            else if ((implementation == "Hybrid_Observables") || (implementation == "GPS_L1_CA_Observables") || (implementation == "GPS_L2C_Observables") ||
                     (implementation == "Galileo_E5A_Observables"))
                {
                    block = std::make_unique<HybridObservables>(configuration, role, in_streams, out_streams);
                }

            // PVT ---------------------------------------------------------------------
            else if ((implementation == "RTKLIB_PVT") || (implementation == "GPS_L1_CA_PVT") || (implementation == "Galileo_E1_PVT") || (implementation == "Hybrid_PVT"))
                {
                    block = std::make_unique<Rtklib_Pvt>(configuration, role, in_streams, out_streams);
                }
            else
                {
                    std::cerr << "Configuration error in " << role << " block: implementation " + implementation + " is not available.\n"s;
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


std::unique_ptr<GNSSBlockInterface> get_block_force_impl(
    const ConfigurationInterface* configuration, const std::string& role, const std::string& forced_impl, unsigned int in_streams, unsigned int out_streams)
{
    const std::string implementation = configuration->property(role + impl_prop, ""s);
    LOG(INFO) << "Getting " << role << " with implementation " << implementation;

    if (implementation.find(forced_impl) == std::string::npos)
        {
            std::cerr << "Error in configuration file: please set " << role << impl_prop << "=" << forced_impl << "\n";
            return nullptr;
        }

    return get_block(configuration, role, in_streams, out_streams);
}

}  // namespace


std::unique_ptr<SignalSourceInterface> GNSSBlockFactory::GetSignalSource(
    const ConfigurationInterface* configuration, Concurrent_Queue<pmt::pmt_t>* queue, int ID) const
{
    const auto role = findRole(configuration, "SignalSource"s, ID);
    const auto implementation = configuration->property(role + impl_prop, ""s);
    LOG(INFO) << "Getting SignalSource " << role << " with implementation " << implementation;
    return get_signal_source_block(implementation, configuration, role, 0, 1, queue);
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetSignalConditioner(
    const ConfigurationInterface* configuration, int ID) const
{
    const auto role_conditioner = findRole(configuration, "SignalConditioner"s, ID);
    const auto role_datatypeadapter = findRole(configuration, "DataTypeAdapter"s, ID);
    const auto role_inputfilter = findRole(configuration, "InputFilter"s, ID);
    const auto role_resampler = findRole(configuration, "Resampler"s, ID);

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

            return std::make_unique<Pass_Through>(configuration, role_conditioner, 1, 1);
        }

    LOG(INFO) << "Getting " << role_conditioner << " with " << role_datatypeadapter << " implementation: "
              << data_type_adapter << ", " << role_inputfilter << " implementation: "
              << input_filter << ", and " << role_resampler << " implementation: "
              << resampler;

    if (signal_conditioner == "Array_Signal_Conditioner")
        {
            // instantiate the array version
            return std::make_unique<ArraySignalConditioner>(
                GetBlock(configuration, role_datatypeadapter, 1, 1),
                GetBlock(configuration, role_inputfilter, 1, 1),
                GetBlock(configuration, role_resampler, 1, 1),
                role_conditioner);
        }

    if (signal_conditioner != "Signal_Conditioner")
        {
            std::cerr << "Error in configuration file: SignalConditioner.implementation=" << signal_conditioner << " is not a valid value.\n";
            return nullptr;
        }

    // single-antenna version
    return std::make_unique<SignalConditioner>(
        GetBlock(configuration, role_datatypeadapter, 1, 1),
        GetBlock(configuration, role_inputfilter, 1, 1),
        GetBlock(configuration, role_resampler, 1, 1),
        role_conditioner);
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetObservables(const ConfigurationInterface* configuration) const
{
    const auto channel_count = get_channel_count(configuration);
    return get_block_force_impl(configuration, "Observables", "Hybrid_Observables", channel_count + 1, channel_count);  // 1 for monitor channel sample counter
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetPVT(const ConfigurationInterface* configuration) const
{
    return get_block_force_impl(configuration, "PVT", "RTKLIB_PVT", get_channel_count(configuration), 0);
}


// ************************** GNSS CHANNEL *************************************
std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetChannel(
    const ConfigurationInterface* configuration,
    const std::string& signal,
    int channel,
    Concurrent_Queue<pmt::pmt_t>* queue) const
{
    const auto acq_role_name = get_role_name(configuration, "Acquisition_", signal, channel);
    const auto trk_role_name = get_role_name(configuration, "Tracking_", signal, channel);
    const auto tlm_role_name = get_role_name(configuration, "TelemetryDecoder_", signal, channel);

    // Automatically detect input data type
    const std::string default_item_type("gr_complex");
    const std::string acq_item_type = configuration->property(acq_role_name + item_prop, default_item_type);
    const std::string trk_item_type = configuration->property(trk_role_name + item_prop, default_item_type);

    if (acq_item_type != trk_item_type)
        {
            std::cerr << "Configuration error: Acquisition and Tracking blocks must have the same input data type!\n";
            return nullptr;
        }

    LOG(INFO) << "Instantiating Channel " << channel
              << " with Acquisition Implementation: " << configuration->property(acq_role_name + impl_prop, "Invalid"s)
              << ", Tracking Implementation: " << configuration->property(trk_role_name + impl_prop, "Invalid"s)
              << ", Telemetry Decoder implementation: " << configuration->property(tlm_role_name + impl_prop, "Invalid"s);

    auto acq_ = GetAcqBlock(configuration, acq_role_name, 1, 0);
    auto trk_ = GetTrkBlock(configuration, trk_role_name, 1, 1);
    auto tlm_ = GetTlmBlock(configuration, tlm_role_name, 1, 1);

    if (acq_ == nullptr or trk_ == nullptr or tlm_ == nullptr)
        {
            return nullptr;
        }
    if (trk_->item_size() == 0)
        {
            std::cerr << "Configuration error: " << trk_->role() << item_prop << "=" << acq_item_type << " is not defined for implementation " << trk_->implementation() << '\n';
            return nullptr;
        }

    return std::make_unique<Channel>(configuration, channel, std::move(acq_), std::move(trk_), std::move(tlm_), "Channel", signal, queue);
}


std::vector<std::unique_ptr<GNSSBlockInterface>> GNSSBlockFactory::GetChannels(
    const ConfigurationInterface* configuration,
    Concurrent_Queue<pmt::pmt_t>* queue) const
{
    int channel_absolute_id = 0;
    std::vector<std::unique_ptr<GNSSBlockInterface>> channels(get_channel_count(configuration));

    try
        {
            for (const auto& entry : signal_mapping)
                {
                    const auto& signal_str = entry.first;
                    const auto& signal_pretty_str = entry.second;
                    const auto channel_count = static_cast<unsigned int>(configuration->property("Channels_" + signal_str + ".count", 0));
                    LOG(INFO) << "Getting " << channel_count << " " << signal_pretty_str << " channels";

                    for (unsigned int i = 0; i < channel_count; i++)
                        {
                            // Store the channel into the vector of channels
                            channels.at(channel_absolute_id) = GetChannel(configuration, signal_str, channel_absolute_id, queue);
                            ++channel_absolute_id;
                        }
                }
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << e.what();
        }

    return channels;
}


std::unique_ptr<GNSSBlockInterface> GNSSBlockFactory::GetBlock(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue) const
{
    return get_block(configuration, role, in_streams, out_streams, queue);
}


std::unique_ptr<AcquisitionInterface> GNSSBlockFactory::GetAcqBlock(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) const
{
    return get_block(configuration, role, in_streams, out_streams, get_acq_block);
}


std::unique_ptr<TrackingInterface> GNSSBlockFactory::GetTrkBlock(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) const
{
    return get_block(configuration, role, in_streams, out_streams, get_trk_block);
}


std::unique_ptr<TelemetryDecoderInterface> GNSSBlockFactory::GetTlmBlock(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) const
{
    return get_block(configuration, role, in_streams, out_streams, get_tlm_block);
}
