/*!
 * \file evk1029_source.cc
 * \brief Unpacks the SAPHYRION EVK1029 raw capture files (continuous stream
 * of OBA-encoded 4-bit samples, no periodic block header).
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

#include "evk1029_source.h"
#include "command_event.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/math.h>
#include <volk/volk.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
// One file read at a time; large enough to amortize I/O overhead without
// requiring a huge allocation.
constexpr std::size_t kReadChunkBytes = 1 << 20;  // 1 MiB

// Empirically tuned output-multiple quantum (~0.7 ms); measurably reduces
// CPU here and downstream versus a smaller one. Derived from the actual
// sampling frequency so the same target holds at other rates.
constexpr double kOutputMultipleTargetSeconds = 0.0007;
constexpr int kOutputMultipleFallback = 131072;  // used if sampling_frequency is unavailable/invalid

int output_multiple_from_sampling_frequency(double sampling_frequency)
{
    if (!(sampling_frequency > 0.0))
        {
            return kOutputMultipleFallback;
        }
    const double target_samples = sampling_frequency * kOutputMultipleTargetSeconds;
    // Round to the *nearest* power of two, not just the next one up: compare
    // the target against both the power-of-two bracket it falls between.
    const int lower_exp = static_cast<int>(std::floor(std::log2(std::max(target_samples, 1.0))));
    const double lower = std::exp2(static_cast<double>(lower_exp));
    const double upper = std::exp2(static_cast<double>(lower_exp + 1));
    return static_cast<int>((target_samples - lower <= upper - target_samples) ? lower : upper);
}

std::size_t max_history(const std::vector<Evk1029FreqXlatingBand>& bands)
{
    std::size_t m = 0;
    for (const auto& b : bands)
        {
            m = std::max(m, b.taps.size() > 0 ? b.taps.size() - 1 : 0);
        }
    return m;
}

// lut[byte] packs this byte's two decoded OBA samples (low nibble first) so
// a single uint16_t store writes both at once on this little-endian target,
// instead of two separate int8_t stores plus the shift/mask/scale per one.
std::array<uint16_t, 256> make_oba_decode_lut()
{
    std::array<uint16_t, 256> lut{};
    for (int byte = 0; byte < 256; byte++)
        {
            const auto low = static_cast<uint8_t>(static_cast<int8_t>((byte & 0x0F) * 2 - 15));
            const auto high = static_cast<uint8_t>(static_cast<int8_t>(((byte >> 4) & 0x0F) * 2 - 15));
            lut[byte] = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
        }
    return lut;
}
const std::array<uint16_t, 256> kObaDecodeLut = make_oba_decode_lut();
}  // namespace


Evk1029Source_sptr evk1029_make_source(const std::string& filename, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency, uint64_t bytes_to_skip,
    const std::vector<Evk1029FreqXlatingBand>& freq_xlating_bands)
{
    return Evk1029Source_sptr(new Evk1029Source(filename, queue, sampling_frequency, bytes_to_skip, freq_xlating_bands));
}


Evk1029Source::Evk1029Source(const std::string& filename, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency, uint64_t bytes_to_skip,
    const std::vector<Evk1029FreqXlatingBand>& freq_xlating_bands)
    : gr::sync_decimator("evk1029_source",
          gr::io_signature::make(0, 0, 0),
          freq_xlating_bands.empty() ? gr::io_signature::make(1, 1, sizeof(int8_t)) : gr::io_signature::make(static_cast<int>(freq_xlating_bands.size()), static_cast<int>(freq_xlating_bands.size()), sizeof(gr_complex)),
          freq_xlating_bands.empty() ? 1 : freq_xlating_bands.front().decimation_factor),
      queue_(queue),
      buffer_(kReadChunkBytes),
      buffer_valid_(0),
      buffer_pos_(0),
      eof_(false),
      history_len_(max_history(freq_xlating_bands))
{
    // Keep the scheduler from calling work() with a tiny noutput_items.
    // In freq_xlating mode, raising output_multiple instead deadlocks with
    // the many direct channel readers (confirmed via gdb); use a bigger
    // output buffer instead, which carries no such risk.
    if (freq_xlating_bands.empty())
        {
            set_output_multiple(output_multiple_from_sampling_frequency(sampling_frequency));
        }
    else
        {
            for (int port = 0; port < static_cast<int>(freq_xlating_bands.size()); port++)
                {
                    set_min_output_buffer(port, 4000000);
                }
        }

    for (const auto& band : freq_xlating_bands)
        {
            Band b;
            b.composite_fir.resize(band.taps.size());
            const float fwT0 = static_cast<float>(2.0 * GR_M_PI * band.center_freq / sampling_frequency);
            for (std::size_t i = 0; i < band.taps.size(); i++)
                {
                    b.composite_fir[i] = band.taps[i] * std::exp(gr_complex(0, static_cast<float>(i) * fwT0));
                }
            std::reverse(b.composite_fir.begin(), b.composite_fir.end());  // match gr::filter::kernel::fir_filter::set_taps()'s internal reversal
            b.phase_incr = std::exp(gr_complex(0, -fwT0 * static_cast<float>(band.decimation_factor)));
            const float group_delay_phase = -fwT0 * static_cast<float>(band.taps.size() - 1) / 2.0F;
            b.phase = std::exp(gr_complex(0, group_delay_phase));
            b.samples_since_phase_renorm = 0;
            bands_.push_back(std::move(b));
        }
    decoded_.resize(history_len_);  // leading history_len_ entries start at 0; harmless until the first real samples overwrite them via decode

    binary_input_file_.open(filename.c_str(), std::ios::in | std::ios::binary);
    if (binary_input_file_.is_open())
        {
            std::cout << "EVK1029_Source: file '" << filename << "' successfully opened.\n";
        }
    else
        {
            std::cerr << "EVK1029_Source: failed to open file: " << filename << '\n';
            exit(1);
        }

    if (bytes_to_skip > 0)
        {
            // Round up to the next 64-bit (8-byte) word boundary -- see this
            // parameter's doc comment in the header for why.
            constexpr uint64_t kWordBytes = 8;
            const uint64_t aligned_offset = ((bytes_to_skip + kWordBytes - 1) / kWordBytes) * kWordBytes;

            binary_input_file_.seekg(0, std::ios::end);
            const auto file_size = static_cast<uint64_t>(binary_input_file_.tellg());
            if (aligned_offset >= file_size)
                {
                    std::cerr << "EVK1029_Source: seconds_to_skip lands at byte " << aligned_offset
                              << " (rounded up to a 64-bit word), at or beyond the end of '" << filename
                              << "' (" << file_size << " bytes) -- nothing to read.\n";
                    exit(1);
                }

            binary_input_file_.seekg(static_cast<std::streamoff>(aligned_offset), std::ios::beg);
            std::cout << "EVK1029_Source: skipping the first " << aligned_offset << " bytes of the capture (requested "
                      << bytes_to_skip << ", rounded up to a 64-bit word boundary)" << std::endl;
        }
}


Evk1029Source::~Evk1029Source()
{
    try
        {
            if (binary_input_file_.is_open())
                {
                    binary_input_file_.close();
                }
        }
    catch (const std::ifstream::failure& e)
        {
            std::cerr << "EVK1029_Source: problem closing input file.\n";
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
}


std::size_t Evk1029Source::decode_into_buffer(std::size_t decoded_needed)
{
    std::size_t produced = 0;
    while (produced + 1 < decoded_needed)
        {
            if (buffer_pos_ >= buffer_valid_)
                {
                    binary_input_file_.read(reinterpret_cast<char*>(buffer_.data()), static_cast<std::streamsize>(buffer_.size()));
                    buffer_valid_ = static_cast<std::size_t>(binary_input_file_.gcount());
                    buffer_pos_ = 0;
                    if (buffer_valid_ == 0)
                        {
                            eof_ = true;
                            return produced;
                        }
                }

            const uint8_t byte = buffer_[buffer_pos_++];
            std::memcpy(&decoded_[history_len_ + produced], &kObaDecodeLut[byte], sizeof(uint16_t));
            produced += 2;
        }
    return produced;
}


int Evk1029Source::work(int noutput_items,
    gr_vector_const_void_star& input_items __attribute__((unused)),
    gr_vector_void_star& output_items)
{
    if (bands_.empty())
        {
            auto* out = reinterpret_cast<int8_t*>(output_items[0]);
            int produced = 0;
            bool eof = false;

            // Each byte unpacks to two samples; only a zero-byte read is EOF.
            while (produced + 1 < noutput_items)
                {
                    if (buffer_pos_ >= buffer_valid_)
                        {
                            binary_input_file_.read(reinterpret_cast<char*>(buffer_.data()), static_cast<std::streamsize>(buffer_.size()));
                            buffer_valid_ = static_cast<std::size_t>(binary_input_file_.gcount());
                            buffer_pos_ = 0;
                            if (buffer_valid_ == 0)
                                {
                                    eof = true;
                                    break;
                                }
                        }

                    const uint8_t byte = buffer_[buffer_pos_++];
                    std::memcpy(&out[produced], &kObaDecodeLut[byte], sizeof(uint16_t));
                    produced += 2;
                }

            if (eof && produced == 0)
                {
                    std::cout << "EVK1029_Source: EOF\n";
                    queue_->push(pmt::make_any(command_event_make(200, 0)));
                    return this->WORK_DONE;
                }

            return produced;
        }

    // Freq-xlating mode: noutput_items is post-decimation complex samples,
    // identical across every band (they share one declared decimation).
    const int decimation = this->decimation();
    const std::size_t decoded_needed = static_cast<std::size_t>(noutput_items) * static_cast<std::size_t>(decimation);
    if (decoded_.size() < history_len_ + decoded_needed)
        {
            decoded_.resize(history_len_ + decoded_needed);  // only grows; stays at its high-water mark otherwise
        }
    const std::size_t decoded_available = decode_into_buffer(decoded_needed);  // may be < decoded_needed on EOF
    const int usable_noutput_items = static_cast<int>(decoded_available / static_cast<std::size_t>(decimation));

    if (usable_noutput_items == 0)
        {
            if (eof_)
                {
                    std::cout << "EVK1029_Source: EOF\n";
                    queue_->push(pmt::make_any(command_event_make(200, 0)));
                    return this->WORK_DONE;
                }
            return 0;
        }

    if (converted_.size() < history_len_ + decoded_available)
        {
            converted_.resize(history_len_ + decoded_available);
        }
    volk_8i_s32f_convert_32f(converted_.data(), decoded_.data(), 1.0F / 256.0F, history_len_ + decoded_available);

    for (std::size_t b = 0; b < bands_.size(); b++)
        {
            Band& band = bands_[b];
            auto* out = reinterpret_cast<gr_complex*>(output_items[b]);
            const std::size_t ntaps = band.composite_fir.size();
            const std::size_t band_offset = history_len_ - (ntaps - 1);  // aligns this band's own (possibly shorter) history window
            for (int i = 0; i < usable_noutput_items; i++)
                {
                    volk_32fc_32f_dot_prod_32fc(&out[i], band.composite_fir.data(), converted_.data() + band_offset + static_cast<std::size_t>(i) * decimation, ntaps);
                }
            volk_32fc_s32fc_x2_rotator2_32fc(out, out, &band.phase_incr, &band.phase, usable_noutput_items);
            band.samples_since_phase_renorm += usable_noutput_items;
            if (band.samples_since_phase_renorm > 4096)
                {
                    band.phase /= std::abs(band.phase);
                    band.samples_since_phase_renorm = 0;
                }
        }

    // Carry the last history_len_ decoded samples forward for the next call.
    if (history_len_ > 0)
        {
            std::copy(decoded_.begin() + static_cast<std::ptrdiff_t>(decoded_available), decoded_.begin() + static_cast<std::ptrdiff_t>(decoded_available + history_len_), decoded_.begin());
        }

    return usable_noutput_items;
}
