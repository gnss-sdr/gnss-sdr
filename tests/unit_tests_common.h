/*!
 * \file unit_tests_common.h
 * \brief Common includes for the translation units that aggregate the unit
 * tests linked into the run_tests executable.
 * \author Carles Fernandez-Prades, 2026 cfernandez(at)cttc.es
 *
 * The unit tests are written as .cc files that are #included from a small set
 * of aggregate translation units (unit_tests_*.cc) rather than compiled
 * separately, so this header provides the headers those files rely on.
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

#ifndef GNSS_SDR_UNIT_TESTS_COMMON_H
#define GNSS_SDR_UNIT_TESTS_COMMON_H

#include "concurrent_map.h"
#include "concurrent_queue.h"
#include "gnss_sdr_flags.h"
#include "gps_acq_assist.h"
#include <gtest/gtest.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
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
#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <absl/log/flags.h>
#include <absl/log/globals.h>
#include <absl/log/initialize.h>
#include <absl/log/log.h>
#include <absl/log/log_sink.h>
#include <absl/log/log_sink_registry.h>
#endif

#endif  // GNSS_SDR_UNIT_TESTS_COMMON_H
