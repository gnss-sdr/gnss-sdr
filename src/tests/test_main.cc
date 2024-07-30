/*!
 * \file test_main.cc
 * \brief This file implements all system tests.
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
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

#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "gps_acq_assist.h"
#include <gtest/gtest.h>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>

#if USE_GLOG_AND_GFLAGS
#include <gflags/gflags.h>
#include <glog/logging.h>
#if GFLAGS_OLD_NAMESPACE
namespace gflags
{
using namespace google;
}
DECLARE_string(log_dir);
#endif
#else
#include "gnss_sdr_flags.h"
#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <absl/log/flags.h>
#include <absl/log/globals.h>
#include <absl/log/initialize.h>
#include <absl/log/log.h>
#include <absl/log/log_sink.h>
#include <absl/log/log_sink_registry.h>

class TestingLogSink : public absl::LogSink
{
public:
    TestingLogSink()
    {
        if (!absl::GetFlag(FLAGS_log_dir).empty())
            {
                filename = std::string(absl::GetFlag(FLAGS_log_dir) + "/run_tests.log");
            }
        else
            {
                filename = std::string(GetTempDir() + "/run_tests.log");
            }
        logfile.open(filename);
    }
    void Send(const absl::LogEntry &entry) override
    {
        logfile << entry.text_message_with_prefix_and_newline() << std::flush;
    }

private:
    std::ofstream logfile;
    std::string filename;
};
#endif


#if UNIT_TESTING_MINIMAL
#include "unit-tests/arithmetic/matio_test.cc"
#if EXTRA_TESTS
#include "unit-tests/signal-processing-blocks/acquisition/acq_performance_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/tracking_pull-in_test.cc"
#if FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/tracking/tracking_pull-in_test_fpga.cc"
#endif  // FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/observables/hybrid_observables_test.cc"
#endif  // EXTRA_TESTS

#else  // UNIT_TESTING_MINIMAL

#include "unit-tests/arithmetic/matio_test.cc"
#include "unit-tests/arithmetic/code_generation_test.cc"
#include "unit-tests/arithmetic/complex_carrier_test.cc"
#include "unit-tests/arithmetic/conjugate_test.cc"
#include "unit-tests/arithmetic/fft_length_test.cc"
#include "unit-tests/arithmetic/fft_speed_test.cc"
#include "unit-tests/arithmetic/magnitude_squared_test.cc"
#include "unit-tests/arithmetic/multiply_test.cc"
#include "unit-tests/arithmetic/preamble_correlator_test.cc"
#include "unit-tests/control-plane/in_memory_configuration_test.cc"
#include "unit-tests/control-plane/protobuf_test.cc"
#include "unit-tests/control-plane/string_converter_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_8ms_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_cccwsr_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_quicksync_ambiguous_acquisition_gsoc2014_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_tong_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e5a_pcps_acquisition_gsoc2014_gensource_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e5b_pcps_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e6_pcps_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/glonass_l1_ca_pcps_acquisition_gsoc2017_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_quicksync_acquisition_gsoc2014_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_tong_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/adapter/adapter_test.cc"
#include "unit-tests/signal-processing-blocks/adapter/pass_through_test.cc"
#include "unit-tests/signal-processing-blocks/libs/item_type_helpers_test.cc"
#include "unit-tests/signal-processing-blocks/osnma/gnss_crypto_test.cc"
#include "unit-tests/signal-processing-blocks/osnma/osnma_msg_receiver_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/geohash_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/nmea_printer_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rinex_printer_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rtcm_printer_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rtcm_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/serdes_monitor_pvt_test.cc"
#include "unit-tests/signal-processing-blocks/resampler/direct_resampler_conditioner_cc_test.cc"
#include "unit-tests/signal-processing-blocks/resampler/mmse_resampler_test.cc"
#include "unit-tests/signal-processing-blocks/sources/gnss_sdr_valve_test.cc"
#include "unit-tests/signal-processing-blocks/sources/unpack_2bit_samples_test.cc"
#include "unit-tests/signal-processing-blocks/telemetry_decoder/galileo_fnav_inav_decoder_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/cpu_multicorrelator_real_codes_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/cpu_multicorrelator_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/discriminator_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/galileo_e5a_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/galileo_e5b_dll_pll_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/tracking_loop_filter_test.cc"
#include "unit-tests/system-parameters/galileo_e1b_reed_solomon_test.cc"
#include "unit-tests/system-parameters/galileo_e6b_reed_solomon_test.cc"
#include "unit-tests/system-parameters/glonass_gnav_crc_test.cc"
#include "unit-tests/system-parameters/glonass_gnav_ephemeris_test.cc"
#include "unit-tests/system-parameters/glonass_gnav_nav_message_test.cc"
#include "unit-tests/system-parameters/has_decoding_test.cc"

#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES
#include "unit-tests/control-plane/control_thread_test.cc"
#include "unit-tests/control-plane/file_configuration_test.cc"
#include "unit-tests/control-plane/gnss_block_factory_test.cc"
#include "unit-tests/control-plane/gnss_flowgraph_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_ambiguous_acquisition_gsoc_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_ambiguous_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/filter/fir_filter_test.cc"
#include "unit-tests/signal-processing-blocks/filter/notch_filter_lite_test.cc"
#include "unit-tests/signal-processing-blocks/filter/notch_filter_test.cc"
#include "unit-tests/signal-processing-blocks/filter/pulse_blanking_filter_test.cc"
#include "unit-tests/signal-processing-blocks/sources/file_signal_source_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/galileo_e1_dll_pll_veml_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/glonass_l1_ca_dll_pll_c_aid_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/glonass_l1_ca_dll_pll_tracking_test.cc"
#endif

#if OPENCL_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_opencl_acquisition_gsoc2013_test.cc"
#endif

#include "unit-tests/signal-processing-blocks/tracking/bayesian_estimation_test.cc"
#if ARMADILLO_HAVE_MVNRND
#include "unit-tests/signal-processing-blocks/tracking/cubature_filter_test.cc"
// #include "unit-tests/signal-processing-blocks/tracking/unscented_filter_test.cc"
#endif

#if CUDA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/tracking/gpu_multicorrelator_test.cc"
#endif

#if FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_ambiguous_acquisition_test_fpga.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_test_fpga.cc"
#include "unit-tests/signal-processing-blocks/tracking/gps_l1_ca_dll_pll_tracking_test_fpga.cc"
#endif

#if EXTRA_TESTS
#include "unit-tests/signal-processing-blocks/acquisition/acq_performance_test.cc"
// #include "unit-tests/signal-processing-blocks/acquisition/beidou_b1i_pcps_acquisition_test.cc"
// #include "unit-tests/signal-processing-blocks/acquisition/beidou_b3i_pcps_acquisition_test.cc"
#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES
#include "unit-tests/signal-processing-blocks/acquisition/glonass_l1_ca_pcps_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l2_m_pcps_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/osnma/osnma_test_vectors.cc"
#include "unit-tests/signal-processing-blocks/tracking/gps_l2_m_dll_pll_tracking_test.cc"
#endif
// #include "unit-tests/signal-processing-blocks/pvt/rtklib_solver_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/gps_l1_ca_dll_pll_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/gps_l1_ca_gaussian_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/tracking_pull-in_test.cc"
#if FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/tracking/tracking_pull-in_test_fpga.cc"
#endif  // FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/observables/hybrid_observables_test.cc"
#if FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/observables/hybrid_observables_test_fpga.cc"
#endif  // FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/telemetry_decoder/gps_l1_ca_telemetry_decoder_test.cc"
#endif  // EXTRA_TESTS

#endif  // UNIT_TESTING_MINIMAL

// For GPS NAVIGATION (L1)
Concurrent_Queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
Concurrent_Map<Gps_Acq_Assist> global_gps_acq_assist_map;

int main(int argc, char **argv)
{
    std::cout << "Running GNSS-SDR Tests...\n";
    int res = 0;
    try
        {
            testing::InitGoogleTest(&argc, argv);
        }
    catch (...)
        {
        }  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
#if USE_GLOG_AND_GFLAGS
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
#else
    absl::ParseCommandLine(argc, argv);
    absl::LogSink *testLogSink = new TestingLogSink;
    absl::AddLogSink(testLogSink);
    absl::InitializeLog();
#endif
    try
        {
            res = RUN_ALL_TESTS();
        }
    catch (...)
        {
            LOG(WARNING) << "Unexpected catch";
        }
#if USE_GLOG_AND_GFLAGS
    gflags::ShutDownCommandLineFlags();
#else
    absl::FlushLogSinks();
#endif
    return res;
}
