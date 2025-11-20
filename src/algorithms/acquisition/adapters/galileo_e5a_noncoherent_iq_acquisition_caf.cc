/*!
 * \file galileo_e5a_noncoherent_iq_acquisition_caf.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          </ul>
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

#include "galileo_e5a_noncoherent_iq_acquisition_caf.h"
#include "Galileo_E5a.h"
#include "configuration_interface.h"
#include "galileo_e5_signal_replica.h"
#include "galileo_e5a_noncoherent_iq_acquisition_caf_cc.h"
#include <boost/math/distributions/exponential.hpp>
#include <algorithm>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl-lite/gsl-lite.hpp>
namespace own = gsl_lite;
#endif

namespace
{
int get_zero_padding(const ConfigurationInterface* configuration, const std::string& role)
{
    return configuration->property(role + ".Zero_padding", 0);
}

uint32_t get_max_sampled_ms(const ConfigurationInterface* configuration, const std::string& role)
{
    const auto zero_padding = get_zero_padding(configuration, role);

    if (zero_padding > 0)
        {
            DLOG(INFO) << "Zero padding activated. Changing to 1ms code + 1ms zero padding ";
            std::cout << "Zero padding activated. Changing to 1ms code + 1ms zero padding\n";
            return 2;
        }

    return 3;
}
}  // namespace

GalileoE5aNoncoherentIQAcquisitionCaf::GalileoE5aNoncoherentIQAcquisitionCaf(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BasePcpsAcquisitionCustom(
          configuration,
          role,
          in_streams,
          out_streams,
          GALILEO_E5A_CODE_CHIP_RATE_CPS,
          GALILEO_E5A_CODE_LENGTH_CHIPS,
          GALILEO_E5A_CODE_PERIOD_MS,
          false,
          ThresholdComputeDoppler(),
          get_max_sampled_ms(configuration, role)),
      zero_padding_(get_zero_padding(configuration, role)),
      caf_window_hz_(configuration->property(role + ".CAF_window_hz", 0)),
      codeQ_(acq_parameters_.vector_length)
{
    if (is_type_gr_complex())
        {
            const auto sig = configuration->property("Channel.signal", std::string("5X"));
            const auto both_signal_components = (sig.at(0) == '5' && sig.at(1) == 'X');

            acquisition_cc_ = galileo_e5a_noncoherentIQ_make_acquisition_caf_cc(
                acq_parameters_, both_signal_components, caf_window_hz_, zero_padding_);
        }
}


void GalileoE5aNoncoherentIQAcquisitionCaf::set_local_code()
{
    if (is_type_gr_complex())
        {
            const auto code_length = acq_parameters_.code_length;
            const auto vector_length = acq_parameters_.vector_length;

            auto& codeI_ = code_;
            std::vector<std::complex<float>> codeI(code_length);
            std::vector<std::complex<float>> codeQ(code_length);

            if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                {
                    std::array<char, 3> a = {{'5', 'I', '\0'}};
                    galileo_e5_a_code_gen_complex_sampled(codeI, gnss_synchro_->PRN, a, acq_parameters_.fs_in, 0);

                    std::array<char, 3> b = {{'5', 'Q', '\0'}};
                    galileo_e5_a_code_gen_complex_sampled(codeQ, gnss_synchro_->PRN, b, acq_parameters_.fs_in, 0);
                }
            else
                {
                    std::array<char, 3> signal_type_ = {{'5', 'X', '\0'}};
                    galileo_e5_a_code_gen_complex_sampled(codeI, gnss_synchro_->PRN, signal_type_, acq_parameters_.fs_in, 0);
                }
            // WARNING: 3ms are coherently integrated. Secondary sequence (1,1,1)
            // is generated, and modulated in the 'block'.
            own::span<gr_complex> codeI_span(codeI_.data(), vector_length);
            own::span<gr_complex> codeQ_span(codeQ_.data(), vector_length);
            if (zero_padding_ == 0)  // if no zero_padding
                {
                    for (unsigned int i = 0; i < acq_parameters_.sampled_ms; i++)
                        {
                            std::copy_n(codeI.data(), code_length, codeI_span.subspan(i * code_length, code_length).data());
                            if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                                {
                                    std::copy_n(codeQ.data(), code_length, codeQ_span.subspan(i * code_length, code_length).data());
                                }
                        }
                }
            else
                {
                    // 1ms code + 1ms zero padding
                    std::copy_n(codeI.data(), code_length, codeI_.data());
                    if (gnss_synchro_->Signal[0] == '5' && gnss_synchro_->Signal[1] == 'X')
                        {
                            std::copy_n(codeQ.data(), code_length, codeQ_.data());
                        }
                }

            acquisition_cc_->set_local_code(codeI_.data(), codeQ_.data());
        }
}
