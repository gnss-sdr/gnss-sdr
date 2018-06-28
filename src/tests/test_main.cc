/*!
* \file test_main.cc
* \brief This file implements all system tests.
* \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
*
*
* -------------------------------------------------------------------------
*
* Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
*
* GNSS-SDR is a software defined Global Navigation
* Satellite Systems receiver
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
*
* -------------------------------------------------------------------------
*/

#include "concurrent_queue.h"
#include "concurrent_map.h"
#include "control_thread.h"
#include "gps_navigation_message.h"

#include "gps_ephemeris.h"
#include "gps_cnav_ephemeris.h"
#include "gps_almanac.h"
#include "gps_iono.h"
#include "gps_cnav_iono.h"
#include "gps_utc_model.h"

#include "galileo_ephemeris.h"
#include "galileo_almanac.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"

#include "sbas_ephemeris.h"

#include "glonass_gnav_ephemeris.h"
#include "glonass_gnav_almanac.h"
#include "glonass_gnav_utc_model.h"

#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gnuradio/msg_queue.h>
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <queue>
#include <memory>


using google::LogMessage;

DECLARE_string(log_dir);

#include "unit-tests/arithmetic/complex_carrier_test.cc"
#include "unit-tests/arithmetic/conjugate_test.cc"
#include "unit-tests/arithmetic/magnitude_squared_test.cc"
#include "unit-tests/arithmetic/multiply_test.cc"
#include "unit-tests/arithmetic/code_generation_test.cc"
#include "unit-tests/arithmetic/fft_length_test.cc"
#include "unit-tests/arithmetic/fft_speed_test.cc"
#include "unit-tests/arithmetic/matio_test.cc"

#include "unit-tests/control-plane/file_configuration_test.cc"
#include "unit-tests/control-plane/in_memory_configuration_test.cc"
#include "unit-tests/control-plane/control_message_factory_test.cc"
#include "unit-tests/control-plane/control_thread_test.cc"
#include "unit-tests/control-plane/gnss_flowgraph_test.cc"
#include "unit-tests/control-plane/string_converter_test.cc"
#include "unit-tests/control-plane/gnss_block_factory_test.cc"

#include "unit-tests/signal-processing-blocks/sources/file_signal_source_test.cc"
#include "unit-tests/signal-processing-blocks/sources/gnss_sdr_valve_test.cc"
#include "unit-tests/signal-processing-blocks/sources/unpack_2bit_samples_test.cc"

#include "unit-tests/signal-processing-blocks/adapter/pass_through_test.cc"
#include "unit-tests/signal-processing-blocks/adapter/adapter_test.cc"

#include "unit-tests/signal-processing-blocks/filter/fir_filter_test.cc"
#include "unit-tests/signal-processing-blocks/filter/pulse_blanking_filter_test.cc"
#include "unit-tests/signal-processing-blocks/filter/notch_filter_test.cc"
#include "unit-tests/signal-processing-blocks/filter/notch_filter_lite_test.cc"

#include "unit-tests/signal-processing-blocks/resampler/direct_resampler_conditioner_cc_test.cc"
#include "unit-tests/signal-processing-blocks/resampler/mmse_resampler_test.cc"

#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_quicksync_acquisition_gsoc2014_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_tong_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_ambiguous_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_ambiguous_acquisition_gsoc_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_8ms_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_tong_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_cccwsr_ambiguous_acquisition_gsoc2013_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e1_pcps_quicksync_ambiguous_acquisition_gsoc2014_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/galileo_e5a_pcps_acquisition_gsoc2014_gensource_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/glonass_l1_ca_pcps_acquisition_gsoc2017_test.cc"
// #include "unit-tests/signal-processing-blocks/acquisition/glonass_l2_ca_pcps_acquisition_test.cc"

#if OPENCL_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_opencl_acquisition_gsoc2013_test.cc"
#endif

#include "unit-tests/signal-processing-blocks/tracking/galileo_e1_dll_pll_veml_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/galileo_e5a_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/glonass_l1_ca_dll_pll_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/glonass_l1_ca_dll_pll_c_aid_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/tracking_loop_filter_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/cpu_multicorrelator_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/cpu_multicorrelator_real_codes_test.cc"

#if CUDA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/tracking/gpu_multicorrelator_test.cc"
#endif

#if FPGA_BLOCKS_TEST
#include "unit-tests/signal-processing-blocks/tracking/gps_l1_ca_dll_pll_tracking_test_fpga.cc"
#include "unit-tests/signal-processing-blocks/acquisition/gps_l1_ca_pcps_acquisition_test_fpga.cc"
#endif

#include "unit-tests/signal-processing-blocks/pvt/rtcm_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rtcm_printer_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/rinex_printer_test.cc"
#include "unit-tests/signal-processing-blocks/pvt/nmea_printer_test.cc"

#if EXTRA_TESTS
#include "unit-tests/signal-processing-blocks/acquisition/gps_l2_m_pcps_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/acquisition/glonass_l1_ca_pcps_acquisition_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/gps_l2_m_dll_pll_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/gps_l1_ca_dll_pll_tracking_test.cc"
#include "unit-tests/signal-processing-blocks/tracking/gps_l1_ca_dll_pll_tracking_pull-in_test.cc"
#include "unit-tests/signal-processing-blocks/telemetry_decoder/gps_l1_ca_telemetry_decoder_test.cc"
#include "unit-tests/signal-processing-blocks/observables/hybrid_observables_test.cc"
#endif

#include "unit-tests/system-parameters/glonass_gnav_ephemeris_test.cc"
#include "unit-tests/system-parameters/glonass_gnav_nav_message_test.cc"

// For GPS NAVIGATION (L1)
concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

int main(int argc, char **argv)
{
    std::cout << "Running GNSS-SDR Tests..." << std::endl;
    int res = 0;
    try
        {
            testing::InitGoogleTest(&argc, argv);
        }
    catch (...)
        {
        }  // catch the "testing::internal::<unnamed>::ClassUniqueToAlwaysTrue" from gtest
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    try
        {
            res = RUN_ALL_TESTS();
        }
    catch (...)
        {
            LOG(WARNING) << "Unexpected catch";
        }
    google::ShutDownCommandLineFlags();
    return res;
}
