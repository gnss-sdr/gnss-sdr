/*!
 * \file unit_tests_core.cc
 * \brief Unit tests for arithmetic, control plane, adapters, libs, resamplers, sources, filters and telemetry decoders.
 * \author Carles Fernandez-Prades, 2026 cfernandez(at)cttc.es
 *
 * This translation unit #includes the test sources of its category so that
 * they are compiled together into the run_tests executable. Splitting the
 * tests into several aggregate units (instead of a single one) keeps the
 * per-unit size, and hence the compiler's memory and time, bounded.
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

#include "unit_tests_common.h"
#include "unit-tests/arithmetic/matio_test.cc"

#if !UNIT_TESTING_MINIMAL

#include "unit-tests/arithmetic/code_generation_test.cc"
#include "unit-tests/arithmetic/complex_carrier_test.cc"
#include "unit-tests/arithmetic/conjugate_test.cc"
#include "unit-tests/arithmetic/fft_speed_test.cc"
#include "unit-tests/arithmetic/magnitude_squared_test.cc"
#include "unit-tests/arithmetic/multiply_test.cc"
#include "unit-tests/arithmetic/preamble_correlator_test.cc"
#include "unit-tests/control-plane/gnss_synchro_monitor_test.cc"
#include "unit-tests/control-plane/in_memory_configuration_test.cc"
#include "unit-tests/control-plane/protobuf_test.cc"
#include "unit-tests/control-plane/satellite_visibility_test.cc"
#include "unit-tests/control-plane/string_converter_test.cc"
#include "unit-tests/signal-processing-blocks/adapter/adapter_test.cc"
#include "unit-tests/signal-processing-blocks/adapter/pass_through_test.cc"
#include "unit-tests/signal-processing-blocks/libs/beidou_b1c_signal_replica_test.cc"
#include "unit-tests/signal-processing-blocks/libs/beidou_b2a_signal_replica_test.cc"
#include "unit-tests/signal-processing-blocks/libs/item_type_helpers_test.cc"
#include "unit-tests/signal-processing-blocks/libs/rtklib_lli_test.cc"
#include "unit-tests/signal-processing-blocks/resampler/direct_resampler_conditioner_cc_test.cc"
#include "unit-tests/signal-processing-blocks/resampler/mmse_resampler_test.cc"
#include "unit-tests/signal-processing-blocks/sources/gnss_sdr_valve_test.cc"
#include "unit-tests/signal-processing-blocks/sources/unpack_2bit_samples_test.cc"
#include "unit-tests/signal-processing-blocks/telemetry_decoder/galileo_fnav_inav_decoder_test.cc"
#include "unit-tests/signal-processing-blocks/telemetry_decoder/sbas_l1_telemetry_decoder_test.cc"

#ifndef EXCLUDE_TESTS_REQUIRING_BINARIES
#include "unit-tests/control-plane/control_thread_test.cc"
#include "unit-tests/control-plane/file_configuration_test.cc"
#include "unit-tests/control-plane/gnss_block_factory_test.cc"
#include "unit-tests/control-plane/gnss_flowgraph_test.cc"
#include "unit-tests/signal-processing-blocks/filter/fir_filter_test.cc"
#include "unit-tests/signal-processing-blocks/filter/notch_filter_lite_test.cc"
#include "unit-tests/signal-processing-blocks/filter/notch_filter_test.cc"
#include "unit-tests/signal-processing-blocks/filter/pulse_blanking_filter_test.cc"
#include "unit-tests/signal-processing-blocks/sources/file_signal_source_test.cc"
#include "unit-tests/signal-processing-blocks/sources/labsat23_source_test.cc"
#endif

#endif  // !UNIT_TESTING_MINIMAL
