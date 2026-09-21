/*!
 * \file cuda_pcps_engine.h
 * \brief GPU (CUDA + cuFFT) implementation of the Parallel Code Phase Search grid
 * \author Phillip Vu, 2026. phillipvu(at)users.noreply.github.com
 *
 * Computes, for a batch of Doppler bins, the squared magnitude of the circular
 * cross-correlation between the carrier-wiped input and the local code, using
 * batched FFTs on an NVIDIA GPU. It is the GPU counterpart of the inner loop of
 * pcps_acquisition::doppler_grid(). The public interface deliberately contains
 * no CUDA types so that it can be consumed from plain C++ translation units;
 * the implementation lives in cuda_pcps_engine.cu.
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

#ifndef GNSS_SDR_CUDA_PCPS_ENGINE_H
#define GNSS_SDR_CUDA_PCPS_ENGINE_H

#include <complex>
#include <cstdint>
#include <memory>
#include <string>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup acquisition_libs
 * \{ */


/*!
 * \brief Batched PCPS grid evaluation on a CUDA device.
 *
 * Typical usage (mirrors pcps_acquisition):
 *   1. Construct with the FFT geometry of the acquisition block.
 *   2. set_doppler_wipeoffs() whenever the Doppler grid changes.
 *   3. set_fft_codes() whenever the local code (PRN) changes.
 *   4. compute_grid() once per dwell; the magnitude grid is written to the
 *      host rows supplied by the caller, so the existing CPU peak-search and
 *      statistics code can be reused unchanged.
 *
 * Each instance owns a CUDA stream, so several channels acquiring at the same
 * time run their grids concurrently on the device.
 */
class CudaPcpsEngine
{
public:
    //! Identifier of the Doppler grid uploaded with set_doppler_wipeoffs()
    enum GridId : int
    {
        MAIN_GRID = 0,   //!< Coarse grid (doppler_min .. doppler_max, doppler_step)
        STEP2_GRID = 1,  //!< Fine grid used when make_two_steps=true
        NUM_GRIDS = 2
    };

    /*!
     * \param fft_size            Length of the FFTs (input vector length).
     * \param effective_fft_size  Number of code-phase cells written per Doppler bin
     *                            (fft_size, or fft_size/2 when bit_transition_flag is set).
     * \param max_doppler_bins    Upper bound on the number of bins in any grid.
     * \param device              CUDA device ordinal, or -1 for the current/default device.
     */
    CudaPcpsEngine(uint32_t fft_size, uint32_t effective_fft_size, uint32_t max_doppler_bins, int device = -1);
    ~CudaPcpsEngine();

    CudaPcpsEngine(const CudaPcpsEngine&) = delete;
    CudaPcpsEngine& operator=(const CudaPcpsEngine&) = delete;
    CudaPcpsEngine(CudaPcpsEngine&&) = delete;
    CudaPcpsEngine& operator=(CudaPcpsEngine&&) = delete;

    //! True when construction succeeded and the engine can be used.
    bool is_valid() const;

    //! Human-readable description of the last error (empty if none).
    const std::string& last_error() const;

    //! Name of the CUDA device in use (empty if not valid).
    const std::string& device_name() const;

    /*!
     * \brief Upload conj(FFT(local code)), fft_size elements.
     */
    bool set_fft_codes(const std::complex<float>* fft_codes);

    /*!
     * \brief Upload the Doppler wipe-off carriers for one grid.
     * \param grid      MAIN_GRID or STEP2_GRID.
     * \param wipeoffs  Array of `bins` host pointers, each pointing at fft_size complex samples.
     * \param bins      Number of Doppler bins (<= max_doppler_bins).
     */
    bool set_doppler_wipeoffs(GridId grid, const std::complex<float>* const* wipeoffs, uint32_t bins);

    /*!
     * \brief Evaluate the PCPS grid for `bins` Doppler bins.
     *
     * For every bin k:  mag_k[i] (+)= | IFFT( FFT(in .* w_k) .* C )[offset + i] |^2,
     * with C = conj(FFT(code)), i in [0, effective_fft_size).
     *
     * \param in            Input samples (fft_size complex values, host memory).
     * \param grid          Which wipe-off grid to use.
     * \param bins          Number of Doppler bins to compute (<= bins uploaded for `grid`).
     * \param offset        First sample of the output window (fft_size/2 when bit_transition_flag).
     * \param accumulate    false: overwrite the output rows; true: add to their current contents
     *                      (non-coherent integration across dwells). Accumulation is kept on
     *                      the device, so the host rows are always the full running sum.
     * \param magnitude_out Array of `bins` host pointers, each with room for effective_fft_size floats.
     */
    bool compute_grid(const std::complex<float>* in, GridId grid, uint32_t bins, uint32_t offset,
        bool accumulate, float* const* magnitude_out);

    //! Number of bytes allocated on the device by this instance.
    size_t device_bytes() const;

private:
    struct Impl;
    std::unique_ptr<Impl> p;
    bool run_pipeline(int grid, uint32_t bins, uint32_t offset, bool accumulate);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CUDA_PCPS_ENGINE_H
