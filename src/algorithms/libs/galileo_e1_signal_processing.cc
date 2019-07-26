/*!
 * \file galileo_e1_signal_processing.cc
 * \brief This library implements various functions for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "galileo_e1_signal_processing.h"
#include "Galileo_E1.h"
#include "gnss_signal_processing.h"
#include <memory>
#include <string>
#include <vector>


void galileo_e1_code_gen_int(gsl::span<int> _dest, const std::array<char, 3>& _Signal, int32_t _prn)
{
    std::string _galileo_signal = _Signal.data();
    int32_t prn = _prn - 1;
    int32_t index = 0;

    // A simple error check
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }

    if (_galileo_signal.rfind("1B") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (char i : GALILEO_E1_B_PRIMARY_CODE[prn])
                {
                    hex_to_binary_converter(_dest.subspan(index, 4), i);
                    index += 4;
                }
        }
    else if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (char i : GALILEO_E1_C_PRIMARY_CODE[prn])
                {
                    hex_to_binary_converter(_dest.subspan(index, 4), i);
                    index += 4;
                }
        }
}


void galileo_e1_sinboc_11_gen_int(gsl::span<int> _dest, gsl::span<const int> _prn)
{
    const uint32_t _length_in = GALILEO_E1_B_CODE_LENGTH_CHIPS;
    auto _period = static_cast<uint32_t>(_dest.size() / _length_in);
    for (uint32_t i = 0; i < _length_in; i++)
        {
            for (uint32_t j = 0; j < (_period / 2); j++)
                {
                    _dest[i * _period + j] = _prn[i];
                }
            for (uint32_t j = (_period / 2); j < _period; j++)
                {
                    _dest[i * _period + j] = -_prn[i];
                }
        }
}


void galileo_e1_sinboc_61_gen_int(gsl::span<int> _dest, gsl::span<const int> _prn)
{
    const uint32_t _length_in = GALILEO_E1_B_CODE_LENGTH_CHIPS;
    auto _period = static_cast<uint32_t>(_dest.size() / _length_in);

    for (uint32_t i = 0; i < _length_in; i++)
        {
            for (uint32_t j = 0; j < _period; j += 2)
                {
                    _dest[i * _period + j] = _prn[i];
                }
            for (uint32_t j = 1; j < _period; j += 2)
                {
                    _dest[i * _period + j] = -_prn[i];
                }
        }
}


void galileo_e1_code_gen_sinboc11_float(gsl::span<float> _dest, const std::array<char, 3>& _Signal, uint32_t _prn)
{
    std::string _galileo_signal = _Signal.data();
    const auto _codeLength = static_cast<uint32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS);
    std::array<int32_t, 4092> primary_code_E1_chips{};
    galileo_e1_code_gen_int(primary_code_E1_chips, _Signal, _prn);  // generate Galileo E1 code, 1 sample per chip
    for (uint32_t i = 0; i < _codeLength; i++)
        {
            _dest[2 * i] = static_cast<float>(primary_code_E1_chips[i]);
            _dest[2 * i + 1] = -_dest[2 * i];
        }
}


void galileo_e1_gen_float(gsl::span<float> _dest, gsl::span<int> _prn, const std::array<char, 3>& _Signal)
{
    std::string _galileo_signal = _Signal.data();
    const uint32_t _codeLength = 12 * GALILEO_E1_B_CODE_LENGTH_CHIPS;
    const float alpha = sqrt(10.0 / 11.0);
    const float beta = sqrt(1.0 / 11.0);

    std::array<int32_t, 12 * 4092> sinboc_11{};
    std::array<int32_t, 12 * 4092> sinboc_61{};
    gsl::span<int32_t> sinboc_11_(sinboc_11.data(), _codeLength);
    gsl::span<int32_t> sinboc_61_(sinboc_61.data(), _codeLength);

    galileo_e1_sinboc_11_gen_int(sinboc_11_, _prn);  // generate sinboc(1,1) 12 samples per chip
    galileo_e1_sinboc_61_gen_int(sinboc_61_, _prn);  // generate sinboc(6,1) 12 samples per chip

    if (_galileo_signal.rfind("1B") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (uint32_t i = 0; i < _codeLength; i++)
                {
                    _dest[i] = alpha * static_cast<float>(sinboc_11[i]) +
                               beta * static_cast<float>(sinboc_61[i]);
                }
        }
    else if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (uint32_t i = 0; i < _codeLength; i++)
                {
                    _dest[i] = alpha * static_cast<float>(sinboc_11[i]) -
                               beta * static_cast<float>(sinboc_61[i]);
                }
        }
}


void galileo_e1_code_gen_float_sampled(gsl::span<float> _dest, const std::array<char, 3>& _Signal,
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift,
    bool _secondary_flag)
{
    // This function is based on the GNU software GPS for MATLAB in Kay Borre's book
    std::string _galileo_signal = _Signal.data();
    uint32_t _samplesPerCode;
    const int32_t _codeFreqBasis = GALILEO_E1_CODE_CHIP_RATE_HZ;  // Hz
    std::vector<int32_t> primary_code_E1_chips(static_cast<int32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS));
    _samplesPerCode = static_cast<uint32_t>(static_cast<double>(_fs) / (static_cast<double>(_codeFreqBasis) / GALILEO_E1_B_CODE_LENGTH_CHIPS));
    const int32_t _samplesPerChip = (_cboc == true) ? 12 : 2;

    const uint32_t delay = ((static_cast<int32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS) - _chip_shift) % static_cast<int32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS)) * _samplesPerCode / GALILEO_E1_B_CODE_LENGTH_CHIPS;

    galileo_e1_code_gen_int(primary_code_E1_chips, _Signal, _prn);  // generate Galileo E1 code, 1 sample per chip

    const uint32_t _codeLength = _samplesPerChip * GALILEO_E1_B_CODE_LENGTH_CHIPS;
    std::unique_ptr<float> _signal_E1{new float[_codeLength]};
    gsl::span<float> _signal_E1_span(_signal_E1, _codeLength);

    if (_cboc == true)
        {
            galileo_e1_gen_float(_signal_E1_span, primary_code_E1_chips, _Signal);  // generate cboc 12 samples per chip
        }
    else
        {
            std::vector<int32_t> _signal_E1_int(static_cast<int32_t>(_codeLength));
            galileo_e1_sinboc_11_gen_int(_signal_E1_int, primary_code_E1_chips);  // generate sinboc(1,1) 2 samples per chip

            for (uint32_t ii = 0; ii < _codeLength; ++ii)
                {
                    _signal_E1_span[ii] = static_cast<float>(_signal_E1_int[ii]);
                }
        }

    if (_fs != _samplesPerChip * _codeFreqBasis)
        {
            std::unique_ptr<float> _resampled_signal{new float[_samplesPerCode]};

            resampler(gsl::span<float>(_signal_E1, _codeLength), gsl::span<float>(_resampled_signal, _samplesPerCode), _samplesPerChip * _codeFreqBasis, _fs);  // resamples code to fs

            _signal_E1 = std::move(_resampled_signal);
        }
    uint32_t size_signal_E1 = _codeLength;
    if (_fs != _samplesPerChip * _codeFreqBasis)
        {
            size_signal_E1 = _samplesPerCode;
        }
    gsl::span<float> _signal_E1_span_aux(_signal_E1, size_signal_E1);
    if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2 && _secondary_flag)
        {
            std::unique_ptr<float> _signal_E1C_secondary{new float[static_cast<int32_t>(GALILEO_E1_C_SECONDARY_CODE_LENGTH) * _samplesPerCode]};
            gsl::span<float> _signal_E1C_secondary_span(_signal_E1C_secondary, static_cast<int32_t>(GALILEO_E1_C_SECONDARY_CODE_LENGTH) * _samplesPerCode);
            for (uint32_t i = 0; i < static_cast<uint32_t>(GALILEO_E1_C_SECONDARY_CODE_LENGTH); i++)
                {
                    for (unsigned k = 0; k < _samplesPerCode; k++)
                        {
                            _signal_E1C_secondary_span[i * _samplesPerCode + k] = _signal_E1_span_aux[k] * (GALILEO_E1_C_SECONDARY_CODE.at(i) == '0' ? 1.0F : -1.0F);
                        }
                }

            _samplesPerCode *= static_cast<int32_t>(GALILEO_E1_C_SECONDARY_CODE_LENGTH);

            _signal_E1 = std::move(_signal_E1C_secondary);
        }
    if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2 && _secondary_flag)
        {
            size_signal_E1 = static_cast<int32_t>(GALILEO_E1_C_SECONDARY_CODE_LENGTH) * _samplesPerCode;
        }
    gsl::span<float> _signal_E1_span_aux2(_signal_E1, size_signal_E1);
    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            _dest[(i + delay) % _samplesPerCode] = _signal_E1_span_aux2[i];
        }
}


void galileo_e1_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, const std::array<char, 3>& _Signal,
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift,
    bool _secondary_flag)
{
    std::string _galileo_signal = _Signal.data();
    const int32_t _codeFreqBasis = GALILEO_E1_CODE_CHIP_RATE_HZ;  // Hz
    auto _samplesPerCode = static_cast<uint32_t>(static_cast<double>(_fs) /
                                                 (static_cast<double>(_codeFreqBasis) / GALILEO_E1_B_CODE_LENGTH_CHIPS));

    if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2 && _secondary_flag)
        {
            _samplesPerCode *= static_cast<int32_t>(GALILEO_E1_C_SECONDARY_CODE_LENGTH);
        }

    std::vector<float> real_code(_samplesPerCode);
    galileo_e1_code_gen_float_sampled(real_code, _Signal, _cboc, _prn, _fs, _chip_shift, _secondary_flag);

    for (uint32_t ii = 0; ii < _samplesPerCode; ++ii)
        {
            _dest[ii] = std::complex<float>(real_code[ii], 0.0F);
        }
}


void galileo_e1_code_gen_float_sampled(gsl::span<float> _dest, const std::array<char, 3>& _Signal,
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift)
{
    galileo_e1_code_gen_float_sampled(_dest, _Signal, _cboc, _prn, _fs, _chip_shift, false);
}


void galileo_e1_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, const std::array<char, 3>& _Signal,
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift)
{
    galileo_e1_code_gen_complex_sampled(_dest, _Signal, _cboc, _prn, _fs, _chip_shift, false);
}
