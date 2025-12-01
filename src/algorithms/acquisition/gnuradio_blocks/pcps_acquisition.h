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
    ~pcps_acquisition() override;

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
    bool is_fdma();
    float get_threshold() const;
    AcquisitionResult first_vs_second_peak_statistic(uint32_t num_doppler_bins, int32_t doppler_max, int32_t doppler_step);
    AcquisitionResult max_to_input_power_statistic(uint32_t num_doppler_bins, int32_t doppler_max, int32_t doppler_step);
    void wait_if_active();

    const Acq_Conf d_acq_parameters;
    const std::string d_dump_filename;
    const float d_doppler_max;
    const uint32_t d_samplesPerChip;
    const uint32_t d_doppler_step;
    const uint32_t d_consumed_samples;
    const uint32_t d_fft_size;
    const uint32_t d_effective_fft_size;
    const uint32_t d_num_doppler_bins;
    const uint32_t d_num_doppler_bins_step2;
    const uint32_t d_dump_channel;
    const float d_threshold;
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
    volk_gnsssdr::vector<volk_gnsssdr::vector<float>> d_magnitude_grid;
    volk_gnsssdr::vector<float> d_tmp_buffer;
    volk_gnsssdr::vector<std::complex<float>> d_input_signal;
    volk_gnsssdr::vector<volk_gnsssdr::vector<std::complex<float>>> d_grid_doppler_wipeoffs_step_two;
    std::unique_ptr<gnss_fft_complex_rev> d_ifft;
    arma::fmat d_grid;
    arma::fmat d_narrow_grid;

    // These are never accessed outside acquisition_core while acquisition is active
    volk_gnsssdr::vector<volk_gnsssdr::vector<std::complex<float>>> d_grid_doppler_wipeoffs;
    volk_gnsssdr::vector<std::complex<float>> d_fft_codes;
    volk_gnsssdr::vector<std::complex<float>> d_data_buffer;
    volk_gnsssdr::vector<lv_16sc_t> d_data_buffer_sc;
    std::unique_ptr<gnss_fft_complex_fwd> d_fft_if;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PCPS_ACQUISITION_H
