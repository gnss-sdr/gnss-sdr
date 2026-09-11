/*!
 * \file pcps_acquisition.h
 * \brief This class implements a Parallel Code Phase Search Acquisition
 *
 *  Acquisition strategy (Kay Borre book + CFAR threshold).
 *  <ol>
 *  <li> Compute the input signal power estimation
 *  <li> Doppler serial search loop
 *  <li> Perform the FFT-based circular convolution (parallel time search)
 *  <li> Record the maximum peak and the associated synchronization parameters
 *  <li> Compute the test statistics and compare to the threshold
 *  <li> Declare positive or negative acquisition using a message queue
 *  </ol>
 *
 * Kay Borre book: K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * "A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach", Birkhauser, 2007. pp 81-84
 *
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          <li> Cillian O'Driscoll, 2017. cillian(at)ieee.org
 *          <li> Antonio Ramos, 2017. antonio.ramos@cttc.es
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

#ifndef GNSS_SDR_PCPS_ACQUISITION_H
#define GNSS_SDR_PCPS_ACQUISITION_H

#include "acquisition_impl_interface.h"
#if ARMA_NO_BOUND_CHECKING
#define ARMA_NO_DEBUG 1
#endif

#include "acq_conf.h"
#include "channel_fsm.h"
#include "gnss_sdr_fft.h"
#include <armadillo>
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>              // for gr_complex
#include <gnuradio/thread/thread.h>           // for scoped_lock
#include <gnuradio/types.h>                   // for gr_vector_const_void_star
#include <volk/volk_complex.h>                // for lv_16sc_t
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <complex>
#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <utility>


#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl-lite/gsl-lite.hpp>
namespace own = gsl_lite;
#endif

/** \addtogroup Acquisition
 * Classes for GNSS signal acquisition
 * \{ */
/** \addtogroup Acq_gnuradio_blocks acquisition_gr_blocks
 * GNU Radio processing blocks for GNSS signal acquisition
 * \{ */


class Gnss_Synchro;
class pcps_acquisition;

using pcps_acquisition_sptr = gnss_shared_ptr<pcps_acquisition>;

pcps_acquisition_sptr pcps_make_acquisition(const Acq_Conf& conf_);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 */
class pcps_acquisition : public acquisition_impl_interface
{
public:
    ~pcps_acquisition() noexcept override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to exchange synchronization data between acquisition and tracking blocks.
     * \param p_gnss_synchro Satellite information shared by the processing blocks.
     */
    inline void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override
    {
        gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
        d_gnss_synchro = p_gnss_synchro;
    }

    /*!
     * \brief Sets local code for PCPS acquisition algorithm.
     * \param code - Pointer to the PRN code.
     */
    void set_local_code(std::complex<float>* code) override;

    void set_resampler_latency(uint32_t latency_samples);

    /*!
     * \brief Returns the maximum peak of grid search.
     */
    inline uint32_t mag() const override
    {
        return 0;  // Not implemented
    }

    /*!
     * \brief Starts acquisition algorithm, turning from standby mode to
     * active mode
     * \param active - bool that activates/deactivates the block.
     */
    void set_active(bool active) override;

    /*!
     * \brief Set acquisition channel unique ID
     * \param channel - receiver channel.
     */
    inline void set_channel(uint32_t channel) override
    {
        d_channel = channel;
    }

    /*!
     * \brief Set channel fsm associated to this acquisition instance
     */
    inline void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) override
    {
        d_channel_fsm = std::move(channel_fsm);
    }

    /*!
     * \brief Set Doppler center frequency for the grid search. It will refresh the Doppler grid.
     * \param doppler_center - Frequency center of the search grid [Hz].
     */
    void set_doppler_center(int32_t doppler_center);

    /*!
     * \brief Sets how many Doppler bins to search, centered on set_doppler_center().
     * One unified mechanism for every degree of Doppler uncertainty: a regular,
     * unassisted search and a primary-frequency-assisted single-bin search (the
     * Doppler is exactly known, from an already-tracked primary frequency or a
     * visibility-aware prediction) go through this same call, differing only in
     * what num_doppler_bins they pass -- there is no separate "narrowed" code
     * path. Refreshes the Doppler grid and, since the detection threshold is
     * itself a function of how many bins are being searched (see
     * compute_threshold()), recalculates it for the new bin count.
     * \param num_doppler_bins - number of candidate Doppler bins to search, or 0
     * to (re)search the full configured Doppler range (computed from
     * doppler_max/doppler_step) -- the only case a caller can't just supply the
     * bin count directly, since that full-grid count is otherwise private to
     * this class. Any other value is the literal candidate bin count: 1 for an
     * exactly-known Doppler, or any N in between to accommodate a search with
     * partial uncertainty.
     */
    void set_doppler_num_bins(uint32_t num_doppler_bins);

    /*!
     * \brief Parallel Code Phase Search Acquisition signal processing.
     */
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items) override;

private:
    friend pcps_acquisition_sptr pcps_make_acquisition(const Acq_Conf& conf_);
    explicit pcps_acquisition(const Acq_Conf& conf_);

    struct AcquisitionResult
    {
        int32_t doppler{0};
        uint32_t index_time{0};
        uint64_t sample_count{0};
        float test_statistics{0};
        bool positive_acq{false};
    };

    // Whether a search of candidate_bins Doppler bins needs a dedicated
    // noise-only reference row. Two cases: a narrowed/assisted search
    // (candidate_bins < d_num_doppler_bins) always needs one, unconditionally
    // -- same as today's narrowed mode assumed, and regardless of which
    // statistic is active, so a narrowed dump stays self-describing either
    // way; a plain full grid (candidate_bins == d_num_doppler_bins) needs one
    // only when CFAR is active and the in-grid wraparound distance
    // max_to_input_power_statistic() would otherwise use (candidate_bins/2
    // bins away) can't clear d_min_reference_separation_hz at this candidate
    // count and d_doppler_step -- a large enough full grid may already clear
    // it via wraparound and need no extra row at all, and peak-ratio has no
    // such reference concept to begin with. Reads
    // d_use_CFAR_algorithm_flag, so -- unlike d_full_grid_reference_needs_extra_row's
    // own copy of this same formula, evaluated inline in the member-initializer
    // list against the conf_ constructor parameter directly -- this is only
    // safe to call once construction has actually finished (d_use_CFAR_algorithm_flag
    // is declared, so initialized, after d_full_grid_reference_needs_extra_row):
    // only set_doppler_num_bins() calls this, never the constructor.
    bool needs_extra_reference_row(uint32_t candidate_bins) const;

    void update_local_carrier(own::span<gr_complex> carrier_vector, float freq) const;
    void update_grid_doppler_wipeoffs();
    void update_grid_doppler_wipeoffs_step2();
    void doppler_grid(const gr_complex* in);
    AcquisitionResult compute_statistics();
    void update_synchro(const AcquisitionResult& result);
    void handle_threshold_reached(AcquisitionResult& result);
    void handle_integration_done(const AcquisitionResult& result);
    void acquisition_core(uint64_t sample_count);
    void log_acquisition(const AcquisitionResult& result) const;
    void send_negative_acquisition(const AcquisitionResult& result);
    void send_positive_acquisition(const AcquisitionResult& result);
    void dump_results(const AcquisitionResult& result);
    void ensure_dump_grid_allocated();
    void copy_magnitude_grid_to_dump_grid();
    bool should_dump_channel() const;
    std::complex<float>* doppler_wipeoff_data(uint32_t doppler_index);
    std::complex<float>* doppler_wipeoff_step_two_data(uint32_t doppler_index);
    float* magnitude_grid_data(uint32_t doppler_index);
    const float* magnitude_grid_data(uint32_t doppler_index) const;
    bool is_fdma();
    float get_threshold() const;
    // candidate_count: number of computed grid rows eligible to be selected as the
    // acquisition result -- narrowed mode, and a plain full grid whose natural
    // reference distance can't clear reference_bin_min_sidelobes, both compute
    // d_num_reference_rows_active (1 or 2) extra trailing rows (see
    // d_full_grid_reference_needs_extra_row) that are never real candidates, so
    // candidate_count excludes them. max_to_input_power_statistic additionally
    // takes num_doppler_bins (>= candidate_count), the full computed row count,
    // for its CFAR reference-bin lookup -- first_vs_second_peak_statistic has no
    // such reference concept, so it only needs candidate_count.
    AcquisitionResult first_vs_second_peak_statistic(uint32_t candidate_count, int32_t doppler_max, int32_t doppler_step);
    AcquisitionResult max_to_input_power_statistic(uint32_t num_doppler_bins, uint32_t candidate_count, int32_t doppler_max, int32_t doppler_step);
    void wait_if_active();

    const Acq_Conf d_acq_parameters;
    const std::string d_dump_filename;
    const float d_doppler_max;
    const uint32_t d_samplesPerChip;
    const uint32_t d_doppler_step;
    const uint32_t d_consumed_samples;
    const uint32_t d_fft_size;
    const uint32_t d_effective_fft_size;
    const uint32_t d_magnitude_grid_stride;
    const uint32_t d_doppler_wipeoffs_stride;
    const uint32_t d_num_doppler_bins;
    const uint32_t d_num_doppler_bins_step2;
    const uint32_t d_dump_channel;
    // (reference_bin_min_sidelobes + 0.5) / coherent_integration_time_seconds: the
    // Doppler-domain target a CFAR noise-reference bin should clear from the
    // search grid's candidate span, in Hz -- used to decide, for whatever
    // candidate bin count is currently active, WHETHER it needs a dedicated
    // extra reference row (see needs_extra_reference_row() below); never to
    // place that row beyond d_doppler_max. See reference_bin_min_sidelobes'
    // doc comment in acq_conf.h for why exceeding d_doppler_max is
    // deliberately never done.
    const float d_min_reference_separation_hz;
    // True when a plain full grid's natural wraparound reference distance
    // (d_num_doppler_bins/2 bins, i.e. what max_to_input_power_statistic's
    // "opposite bin" formula would reach) can't clear d_min_reference_separation_hz
    // -- computed once at construction from d_num_doppler_bins/d_doppler_step, which
    // never change afterward. This is exactly needs_extra_reference_row(d_num_doppler_bins)
    // (see below), kept as its own const member only because it's needed before
    // construction finishes, to size d_num_doppler_bins_full_grid_active and
    // therefore d_magnitude_grid/d_grid_doppler_wipeoffs -- every other caller,
    // at any candidate bin count, goes through needs_extra_reference_row(). Only
    // says whether a dedicated reference row is needed at all -- see
    // d_num_reference_rows_active for how many (1 or 2).
    const bool d_full_grid_reference_needs_extra_row;
    // d_num_doppler_bins, plus reference rows (see d_num_reference_rows_active --
    // 2 here, since a full grid worth searching a dedicated reference row for is
    // always > 1 candidate bin in practice) when d_full_grid_reference_needs_extra_row
    // is true. This (not d_num_doppler_bins) is the full grid's row count -- both
    // the constructed value of d_num_doppler_bins_active and what
    // set_doppler_num_bins(0) (the "full range" sentinel) restores it to -- and
    // what d_magnitude_grid/d_grid_doppler_wipeoffs are sized to accommodate: the
    // ceiling every set_doppler_num_bins() call must stay within.
    const uint32_t d_num_doppler_bins_full_grid_active;
    const float d_threshold_step_two;
    const bool d_cshort;
    const bool d_use_CFAR_algorithm_flag;
    const bool d_dump;

    // Need lock to access these
    std::weak_ptr<ChannelFsm> d_channel_fsm;
    std::unique_ptr<gr::thread::thread> d_worker;
    Gnss_Synchro* d_gnss_synchro;
    std::queue<Gnss_Synchro> d_monitor_queue;
    int32_t d_state;
    int32_t d_doppler_center;
    int32_t d_doppler_bias;
    // Number of step-1 Doppler grid rows actually searched right now: the
    // candidate bin count last passed to set_doppler_num_bins() (or the full
    // grid's candidate count, if it was called with the 0 sentinel), plus
    // d_num_reference_rows_active. Always <= d_num_doppler_bins_full_grid_active
    // -- set_doppler_num_bins() clamps to that ceiling -- so it never exceeds
    // what d_magnitude_grid/d_grid_doppler_wipeoffs were allocated for.
    uint32_t d_num_doppler_bins_active;
    // How many of d_num_doppler_bins_active's trailing rows are dedicated
    // noise-only reference rows (see needs_extra_reference_row()) rather than
    // real Doppler candidates -- set directly by set_doppler_num_bins() for
    // whatever candidate count is currently active. 0 when no reference row
    // is needed (plain full grid, wraparound clears reference_bin_min_sidelobes
    // on its own). 1 when needed but there's only a single real candidate (an
    // exactly-known Doppler, always at offset 0 from center -- no sign for a
    // second, opposite-side row to usefully mirror), same fixed placement at
    // doppler_center + doppler_max as always. 2 when needed and there's more
    // than one real candidate: a row at doppler_center + doppler_max (index
    // candidate_count) AND one at doppler_center - doppler_max (index
    // candidate_count + 1) -- max_to_input_power_statistic() picks whichever
    // of the two sits on the opposite side of center from the winning
    // candidate, so the reference is never on the same side as (and closer to)
    // a signal that happens to be riding near the edge of the search range.
    uint32_t d_num_reference_rows_active;
    // Detection threshold for the currently active candidate bin count (see
    // d_num_doppler_bins_active) -- compute_threshold() folds the number of
    // bins being searched into the false-alarm probability (more bins tested
    // means more chances to false-alarm at a fixed per-bin PFA), so this must
    // be recalculated by set_doppler_num_bins() every time that count
    // changes; reusing a threshold calibrated for a different bin count would
    // over- or under-detect. Excludes the reference row itself from the count
    // passed to compute_threshold() -- it's never a candidate result, so it
    // isn't one of the hypotheses being tested for a false alarm either.
    float d_threshold_active;
    uint32_t d_buffer_count;
    uint32_t d_channel;
    uint32_t d_resampler_latency_samples;
    uint64_t d_sample_count;
    bool d_step_two;
    bool d_active;
    bool d_worker_active;

    // Only access these in acquisition_core and functions strictly called from acquisition_core
    uint32_t d_num_noncoherent_integrations_counter;
    int64_t d_dump_number;
    float d_input_power;
    float d_doppler_center_step_two;
    volk_gnsssdr::vector<float> d_magnitude_grid;
    volk_gnsssdr::vector<float> d_tmp_buffer;
    volk_gnsssdr::vector<std::complex<float>> d_input_signal;
    volk_gnsssdr::vector<std::complex<float>> d_grid_doppler_wipeoffs_step_two;
    std::unique_ptr<gnss_fft_complex_rev> d_ifft;
    arma::fmat d_grid;
    arma::fmat d_narrow_grid;

    // These are never accessed outside acquisition_core while acquisition is active
    volk_gnsssdr::vector<std::complex<float>> d_grid_doppler_wipeoffs;
    volk_gnsssdr::vector<std::complex<float>> d_fft_codes;
    volk_gnsssdr::vector<std::complex<float>> d_data_buffer;
    volk_gnsssdr::vector<lv_16sc_t> d_data_buffer_sc;
    std::unique_ptr<gnss_fft_complex_fwd> d_fft_if;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PCPS_ACQUISITION_H
