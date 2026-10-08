/*!
 * \file cuda_ddc_engine.h
 * \brief GPU (CUDA) implementation of a complex-tap FIR digital down-converter
 *
 * Computes out[i] = sum_k taps[k] * data[i*decimation + k] for a batch of i,
 * where taps is complex (an NCO baked into a real low-pass prototype, see
 * Evk1029Source) and data is real -- the same math as
 * Evk1029Source::process_band()'s inner loop, offloaded to an NVIDIA GPU.
 * The public interface deliberately contains no CUDA types so that it can be
 * consumed from plain C++ translation units; the implementation lives in
 * cuda_ddc_engine.cu.
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

#ifndef GNSS_SDR_CUDA_DDC_ENGINE_H
#define GNSS_SDR_CUDA_DDC_ENGINE_H

#include <complex>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup signal_source_libs
 * \{ */

/*!
 * \brief Batched complex-tap FIR/DDC evaluation on a CUDA device.
 *
 * Typical usage (mirrors Evk1029Source::process_band()):
 *   1. Construct once per band.
 *   2. set_taps() whenever the composite FIR (NCO + low-pass) changes.
 *   3. compute() once per work() call; synchronous from the caller's
 *      perspective (host/device transfer and kernel execution all complete
 *      before it returns), but uses this instance's own CUDA stream so
 *      several bands' calls can be issued back to back without serializing
 *      on a single default stream.
 *
 * Host-side buffers are pinned (page-locked) internally for faster transfer;
 * the caller's data/out pointers are ordinary host memory.
 */
class CudaDdcEngine
{
public:
    //! \param device CUDA device ordinal, or -1 for the current/default device.
    explicit CudaDdcEngine(int device = -1);
    ~CudaDdcEngine();

    CudaDdcEngine(const CudaDdcEngine&) = delete;
    CudaDdcEngine& operator=(const CudaDdcEngine&) = delete;
    CudaDdcEngine(CudaDdcEngine&&) = delete;
    CudaDdcEngine& operator=(CudaDdcEngine&&) = delete;

    //! True when construction succeeded and the engine can be used.
    bool is_valid() const;

    //! Human-readable description of the last error (empty if none).
    const std::string& last_error() const;

    //! Name of the CUDA device in use (empty if not valid).
    const std::string& device_name() const;

    //! Uploads the composite FIR taps (already NCO-shifted and reversed to
    //! match Evk1029Source's own convention). Returns false on failure (see
    //! last_error()); the engine remains valid but compute() will fail too
    //! until a successful set_taps() call.
    bool set_taps(const std::complex<float>* taps, int ntaps);

    //! data must have at least (n_out - 1) * decimation + ntaps valid
    //! elements (ntaps from the last successful set_taps() call); out
    //! receives n_out complex samples. Returns false on failure (see
    //! last_error()) -- the caller should fall back to the CPU path for
    //! this call and may retry compute() on later calls.
    bool compute(const float* data, int n_data, int decimation, std::complex<float>* out, int n_out);

private:
    struct Impl;
    Impl* d_impl;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_CUDA_DDC_ENGINE_H
