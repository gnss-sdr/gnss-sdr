/*!
 * \file galileo_e1_pcps_cccwsr_ambiguous_acquisition.cc
 * \brief Adapts a PCPS CCCWSR acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#include "galileo_e1_pcps_cccwsr_ambiguous_acquisition.h"
#include "Galileo_E1.h"
#include "configuration_interface.h"
#include "galileo_e1_signal_replica.h"
#include "pcps_cccwsr_acquisition_cc.h"
#include <boost/math/distributions/exponential.hpp>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GalileoE1PcpsCccwsrAmbiguousAcquisition::GalileoE1PcpsCccwsrAmbiguousAcquisition(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisitionCustom(
          configuration,
          role,
          in_streams,
          out_streams,
          GALILEO_E1_CODE_CHIP_RATE_CPS,
          GALILEO_E1_B_CODE_LENGTH_CHIPS,
          GALILEO_E1_CODE_PERIOD_MS,
          true,
          false),
      code_data_(vector_length_),
      code_pilot_(vector_length_),
      cboc_(configuration->property(role + ".cboc", false))
{
    if (is_type_gr_complex())
        {
            const auto samples_per_ms = static_cast<int>(code_length_) / 4;

            acquisition_cc_ = pcps_cccwsr_make_acquisition_cc(acq_parameters_.sampled_ms, acq_parameters_.max_dwells,
                acq_parameters_.doppler_max, acq_parameters_.doppler_step, acq_parameters_.fs_in, samples_per_ms, code_length_,
                acq_parameters_.dump, acq_parameters_.dump_filename, acq_parameters_.enable_monitor_output);

            DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }
}


void GalileoE1PcpsCccwsrAmbiguousAcquisition::set_local_code()
{
    if (is_type_gr_complex())
        {
            std::array<char, 3> signal = {{'1', 'B', '\0'}};
            galileo_e1_code_gen_complex_sampled(code_data_, signal, cboc_, gnss_synchro_->PRN, acq_parameters_.fs_in, 0, false);

            std::array<char, 3> signal_C = {{'1', 'C', '\0'}};
            galileo_e1_code_gen_complex_sampled(code_pilot_, signal_C, cboc_, gnss_synchro_->PRN, acq_parameters_.fs_in, 0, false);

            acquisition_cc_->set_local_code(code_data_.data(), code_pilot_.data());
        }
}
