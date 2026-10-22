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
#include "gnss_sdr_make_unique.h"  // for std::make_unique in C++11
#include <gnuradio/io_signature.h>
#include <gnuradio/math.h>
#include <volk/volk.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>

#if defined(__linux__) || defined(__APPLE__)
#include <pthread.h>
#endif

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

// Linux and macOS's pthread_setname_np() take different arguments (the
// macOS one can only target the calling thread), so this only ever names
// the thread it's called from -- each BandWorker's own thread calls this
// on itself at the top of worker_loop().
void name_current_thread(const std::string& name)
{
#if defined(__linux__)
    pthread_setname_np(pthread_self(), name.c_str());
#elif defined(__APPLE__)
    pthread_setname_np(name.c_str());
#endif
}
}  // namespace


Evk1029Source_sptr evk1029_make_source(std::unique_ptr<RawStreamReader> reader, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency,
    const std::vector<Evk1029FreqXlatingBand>& freq_xlating_bands)
{
    return Evk1029Source_sptr(new Evk1029Source(std::move(reader), queue, sampling_frequency, freq_xlating_bands));
}


Evk1029Source::Evk1029Source(std::unique_ptr<RawStreamReader> reader, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency,
    const std::vector<Evk1029FreqXlatingBand>& freq_xlating_bands)
    : gr::sync_decimator("evk1029_source",
          gr::io_signature::make(0, 0, 0),
          freq_xlating_bands.empty() ? gr::io_signature::make(1, 1, sizeof(int8_t)) : gr::io_signature::make(static_cast<int>(freq_xlating_bands.size()), static_cast<int>(freq_xlating_bands.size()), sizeof(gr_complex)),
          freq_xlating_bands.empty() ? 1 : freq_xlating_bands.front().decimation_factor),
      reader_(std::move(reader)),
      queue_(queue),
      buffer_(kReadChunkBytes),
      buffer_valid_(0),
      buffer_pos_(0),
      eof_(false),
      history_len_(max_history(freq_xlating_bands))
{
    for (const auto& band : freq_xlating_bands)
        {
            if (band.decimation_factor != freq_xlating_bands.front().decimation_factor || band.taps.empty())
                {
                    throw std::invalid_argument("EVK1029_Source: all bands need taps and the same decimation factor");
                }
        }

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
            // Center every filter on the longest one's window, so all bands
            // get the same group delay (history_len_ / 2 input samples).
            // Aligning their newest samples instead would make a shorter
            // filter lead the others by half the length difference, which
            // biases pseudoranges between bands.
            const std::size_t slack = history_len_ - (b.composite_fir.size() - 1);
            b.history_offset = slack / 2;
            if (slack % 2 != 0)
                {
                    LOG(WARNING) << "EVK1029_Source: a band filter is shorter than the longest one by an odd number of taps; "
                                 << "its output lags the other bands by half an input sample (" << 0.5e9 / sampling_frequency << " ns)";
                }
#if CUDA_GPU_ACCEL
            if (band.use_cuda)
                {
                    auto engine = std::make_unique<CudaDdcEngine>();
                    if (engine->is_valid() && engine->set_taps(reinterpret_cast<const std::complex<float>*>(b.composite_fir.data()), static_cast<int>(b.composite_fir.size())))
                        {
                            LOG(INFO) << "EVK1029_Source: band " << bands_.size() << " filtered on CUDA device " << engine->device_name();
                            b.cuda_engine = std::move(engine);
                        }
                    else
                        {
                            LOG(WARNING) << "EVK1029_Source: CUDA DDC engine unavailable for band " << bands_.size() << " (" << engine->last_error() << "), falling back to CPU";
                        }
                }
#endif
            b.phase_incr = std::exp(gr_complex(0, -fwT0 * static_cast<float>(band.decimation_factor)));
            const float group_delay_phase = -fwT0 * static_cast<float>(band.taps.size() - 1) / 2.0F;
            b.phase = std::exp(gr_complex(0, group_delay_phase));
            b.samples_since_phase_renorm = 0;
            bands_.push_back(std::move(b));
        }
    decoded_.resize(history_len_);    // leading history_len_ entries start at 0; harmless until the first real samples overwrite them via decode
    converted_.resize(history_len_);  // same; carried-forward float history starts at 0.0F to match
    decoded_b_.resize(history_len_);
    converted_b_.resize(history_len_);

    // One persistent FIR worker per band: work()'s own calling thread only
    // dispatches and waits.
    for (std::size_t b = 0; b < bands_.size(); b++)
        {
            workers_.push_back(std::make_unique<BandWorker>());
            BandWorker* w = workers_.back().get();
            w->thread = std::thread(&Evk1029Source::worker_loop, this, b, w);
        }

    // Decode-ahead worker (see DecodeWorker's doc comment).
    if (!bands_.empty())
        {
            decode_worker_ = std::make_unique<DecodeWorker>();
            decode_worker_->thread = std::thread(&Evk1029Source::decode_worker_loop, this);
        }
}


Evk1029Source::~Evk1029Source()
{
    for (auto& w : workers_)
        {
            {
                std::lock_guard<std::mutex> lock(w->mutex);
                w->stop = true;
            }
            w->cv.notify_all();
            w->thread.join();
        }
    if (decode_worker_)
        {
            // reader_->read() can block indefinitely on a live stream;
            // wake it first so the join() below can actually complete.
            if (reader_)
                {
                    reader_->request_stop();
                }
            {
                std::lock_guard<std::mutex> lock(decode_worker_->mutex);
                decode_worker_->stop = true;
            }
            decode_worker_->cv.notify_all();
            decode_worker_->thread.join();
        }
}


std::size_t Evk1029Source::decode_into_buffer(std::size_t decoded_needed, std::vector<int8_t>& target, std::size_t start_offset)
{
    std::size_t produced = start_offset;
    while (produced + 1 < decoded_needed)
        {
            if (buffer_pos_ >= buffer_valid_)
                {
                    buffer_valid_ = reader_->read(buffer_.data(), buffer_.size());
                    buffer_pos_ = 0;
                    if (buffer_valid_ == 0)
                        {
                            eof_ = true;
                            return produced;
                        }
                }

            // Decode as many bytes as fit both what's left in buffer_ and what's
            // still needed, in one tight pass -- avoids re-checking the buffer
            // bound on every single byte like a naive one-byte-at-a-time loop would.
            const std::size_t bytes_wanted = (decoded_needed - produced) / 2;
            const std::size_t bytes_available = buffer_valid_ - buffer_pos_;
            const std::size_t bytes_to_decode = std::min(bytes_wanted, bytes_available);
            int8_t* dst = &target[history_len_ + produced];
            const uint8_t* src = &buffer_[buffer_pos_];
            for (std::size_t i = 0; i < bytes_to_decode; i++)
                {
                    std::memcpy(dst + i * 2, &kObaDecodeLut[src[i]], sizeof(uint16_t));
                }
            buffer_pos_ += bytes_to_decode;
            produced += bytes_to_decode * 2;
        }
    return produced;
}


void Evk1029Source::process_band(std::size_t b, int usable_noutput_items, gr_complex* out)
{
    Band& band = bands_[b];
    const std::size_t ntaps = band.composite_fir.size();
    const int decimation = this->decimation();

    bool computed_on_gpu = false;
#if CUDA_GPU_ACCEL
    if (band.cuda_engine)
        {
            const int n_data = (usable_noutput_items - 1) * decimation + static_cast<int>(ntaps);
            computed_on_gpu = band.cuda_engine->compute(converted_.data() + band.history_offset, n_data, decimation,
                reinterpret_cast<std::complex<float>*>(out), usable_noutput_items);
            if (!computed_on_gpu)
                {
                    LOG(WARNING) << "EVK1029_Source: CUDA DDC compute failed for band " << b << " (" << band.cuda_engine->last_error() << "), falling back to CPU";
                    band.cuda_engine.reset();  // stay on CPU for every subsequent call too
                }
        }
#endif
    if (!computed_on_gpu)
        {
            for (int i = 0; i < usable_noutput_items; i++)
                {
                    volk_32fc_32f_dot_prod_32fc(&out[i], band.composite_fir.data(), converted_.data() + band.history_offset + static_cast<std::size_t>(i) * decimation, ntaps);
                }
        }

#if VOLK_LACKS_ROTATOR2
    volk_32fc_s32fc_x2_rotator_32fc(out, out, band.phase_incr, &band.phase, usable_noutput_items);
#else
    volk_32fc_s32fc_x2_rotator2_32fc(out, out, &band.phase_incr, &band.phase, usable_noutput_items);
#endif
    band.samples_since_phase_renorm += usable_noutput_items;
    if (band.samples_since_phase_renorm > 4096)
        {
            band.phase /= std::abs(band.phase);
            band.samples_since_phase_renorm = 0;
        }
}


void Evk1029Source::worker_loop(std::size_t band_index, BandWorker* w)
{
    name_current_thread("evk1029_wrk" + std::to_string(band_index));
    while (true)
        {
            std::unique_lock<std::mutex> lock(w->mutex);
            w->cv.wait(lock, [w] { return w->has_work || w->stop; });
            if (w->stop)
                {
                    return;
                }
            const int items = w->usable_noutput_items;
            gr_complex* out = w->out;
            lock.unlock();

            process_band(band_index, items, out);

            lock.lock();
            w->has_work = false;
            w->finished = true;
            lock.unlock();
            w->cv.notify_all();
        }
}


void Evk1029Source::decode_worker_loop()
{
    name_current_thread("evk1029_dec");
    while (true)
        {
            std::unique_lock<std::mutex> lock(decode_worker_->mutex);
            decode_worker_->cv.wait(lock, [this] { return decode_worker_->has_work || decode_worker_->stop; });
            if (decode_worker_->stop)
                {
                    return;
                }
            const std::size_t decoded_needed = decode_worker_->decoded_needed;
            const std::size_t start_offset = decode_worker_->start_offset;
            std::vector<int8_t>* target_decoded = decode_worker_->target_decoded;
            volk_gnsssdr::vector<float>* target_converted = decode_worker_->target_converted;
            lock.unlock();

            if (target_decoded->size() < history_len_ + decoded_needed)
                {
                    target_decoded->resize(history_len_ + decoded_needed);
                }
            const std::size_t produced = decode_into_buffer(decoded_needed, *target_decoded, start_offset);
            if (target_converted->size() < history_len_ + produced)
                {
                    target_converted->resize(history_len_ + produced);
                }
            // Only the portion beyond start_offset is newly decoded -- the
            // rest (carried-forward history + leftover) already has valid,
            // previously-converted floats.
            if (produced > start_offset)
                {
                    volk_8i_s32f_convert_32f(target_converted->data() + history_len_ + start_offset, target_decoded->data() + history_len_ + start_offset, 1.0F / 256.0F, produced - start_offset);
                }

            lock.lock();
            decode_worker_->produced = produced;
            decode_worker_->has_work = false;
            decode_worker_->finished = true;
            lock.unlock();
            decode_worker_->cv.notify_all();
        }
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
                            buffer_valid_ = reader_->read(buffer_.data(), buffer_.size());
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
    //
    // decoded_/converted_ are always "this call's" buffers; decoded_b_/
    // converted_b_ are their ping-pong partner, prefetched by decode_worker_
    // during the PREVIOUS call's FIR work (kicked off below) so this call
    // usually just waits (near-instant) and swaps buffer identities instead
    // of decoding+converting on its own critical path. The prefetch predicts
    // "same size as this call"; noutput_items jitters slightly in practice,
    // so a mismatch is never just discarded -- undershoot is topped up
    // synchronously, overshoot's unused tail is carried forward as
    // "leftover" into the next buffer, same as FIR history already is.
    const int decimation = this->decimation();
    const std::size_t decoded_needed = static_cast<std::size_t>(noutput_items) * static_cast<std::size_t>(decimation);
    std::size_t decoded_available;
    std::size_t leftover = 0;  // samples already decoded/converted beyond decoded_available, carried forward below

    if (first_call_)
        {
            if (decoded_.size() < history_len_ + decoded_needed)
                {
                    decoded_.resize(history_len_ + decoded_needed);
                }
            decoded_available = decode_into_buffer(decoded_needed, decoded_);
            if (converted_.size() < history_len_ + decoded_available)
                {
                    converted_.resize(history_len_ + decoded_available);
                }
            volk_8i_s32f_convert_32f(converted_.data() + history_len_, decoded_.data() + history_len_, 1.0F / 256.0F, decoded_available);
            first_call_ = false;
        }
    else if (!decode_worker_has_pending_result_)
        {
            // No prefetch was started last time -- that means EOF was already
            // hit then (see below), so there's nothing more to give.
            decoded_available = 0;
        }
    else
        {
            std::unique_lock<std::mutex> lock(decode_worker_->mutex);
            decode_worker_->cv.wait(lock, [this] { return decode_worker_->finished; });
            std::size_t produced = decode_worker_->produced;
            lock.unlock();
            decode_worker_has_pending_result_ = false;

            if (produced < decoded_needed && !eof_)
                {
                    // Prediction (same size as the previous call) undershot --
                    // should be rare given stable noutput_items; top up
                    // synchronously rather than returning a short batch.
                    if (decoded_b_.size() < history_len_ + decoded_needed)
                        {
                            decoded_b_.resize(history_len_ + decoded_needed);
                        }
                    const std::size_t new_produced = decode_into_buffer(decoded_needed, decoded_b_, produced);
                    if (converted_b_.size() < history_len_ + new_produced)
                        {
                            converted_b_.resize(history_len_ + new_produced);
                        }
                    volk_8i_s32f_convert_32f(converted_b_.data() + history_len_ + produced, decoded_b_.data() + history_len_ + produced, 1.0F / 256.0F, new_produced - produced);
                    produced = new_produced;
                }
            decoded_available = std::min(produced, decoded_needed);
            if (produced > decoded_needed)
                {
                    leftover = produced - decoded_needed;  // carried forward below, not discarded
                }

            std::swap(decoded_, decoded_b_);
            std::swap(converted_, converted_b_);
        }

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

    // Kick off decode-ahead for the NEXT call into decoded_b_/converted_b_
    // (the now-free buffer), to overlap with this call's FIR work below.
    // Carries forward history_len_ + leftover samples: the usual FIR-history
    // tail, immediately followed by any samples decoded_ had beyond what
    // this call actually consumed (unused overshoot from the prediction),
    // so nothing decoded is ever thrown away.
    if (!eof_)
        {
            const std::size_t carry_len = history_len_ + leftover;
            if (carry_len > 0)
                {
                    if (decoded_b_.size() < carry_len)
                        {
                            decoded_b_.resize(carry_len);
                        }
                    if (converted_b_.size() < carry_len)
                        {
                            converted_b_.resize(carry_len);
                        }
                    std::copy(decoded_.begin() + static_cast<std::ptrdiff_t>(decoded_available), decoded_.begin() + static_cast<std::ptrdiff_t>(decoded_available + carry_len), decoded_b_.begin());
                    std::copy(converted_.begin() + static_cast<std::ptrdiff_t>(decoded_available), converted_.begin() + static_cast<std::ptrdiff_t>(decoded_available + carry_len), converted_b_.begin());
                }
            {
                std::lock_guard<std::mutex> lock(decode_worker_->mutex);
                decode_worker_->decoded_needed = std::max(decoded_needed, leftover);  // predict same size as this call
                decode_worker_->start_offset = leftover;
                decode_worker_->target_decoded = &decoded_b_;
                decode_worker_->target_converted = &converted_b_;
                decode_worker_->finished = false;
                decode_worker_->has_work = true;
            }
            decode_worker_->cv.notify_all();
            decode_worker_has_pending_result_ = true;
        }

    // Hand every band to its persistent worker, then wait for all of them.
    for (std::size_t b = 0; b < bands_.size(); b++)
        {
            BandWorker* w = workers_[b].get();
            {
                std::lock_guard<std::mutex> lock(w->mutex);
                w->usable_noutput_items = usable_noutput_items;
                w->out = reinterpret_cast<gr_complex*>(output_items[b]);
                w->finished = false;
                w->has_work = true;
            }
            w->cv.notify_all();
        }
    for (std::size_t b = 0; b < bands_.size(); b++)
        {
            BandWorker* w = workers_[b].get();
            std::unique_lock<std::mutex> lock(w->mutex);
            w->cv.wait(lock, [w] { return w->finished; });
        }

    return usable_noutput_items;
}
