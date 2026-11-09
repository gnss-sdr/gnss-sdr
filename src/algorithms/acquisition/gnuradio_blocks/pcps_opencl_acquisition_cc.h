/*!
 * \file pcps_opencl_acquisition_cc.h
 * \brief This class implements a Parallel Code Phase Search Acquisition
 * using OpenCL to offload some functions to the GPU.
 *
 *  Acquisition strategy (Kay Borre book + CFAR threshold).
 *  <ol>
 *  <li> Compute the input signal power estimation
 *  <li> Doppler serial search loop
 *  <li> Perform the FFT-based circular convolution (parallel time search)
 *  <li> Record the maximum peak and the associated synchronization parameters
 *  <li> Compute the test statistics and compare to the threshold
 *  <li> Declare positive or negative acquisition using a message port
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

#ifndef GNSS_SDR_PCPS_OPENCL_ACQUISITION_CC_H
#define GNSS_SDR_PCPS_OPENCL_ACQUISITION_CC_H

#include "acq_conf.h"
#include "acquisition_impl_interface.h"
#include "channel_fsm.h"
#include "deterministic_acquisition.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_fft.h"
#include "gnss_synchro.h"
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>  // for unique_ptr, weak_ptr
#include <mutex>
#include <thread>
#include <vector>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_gnuradio_blocks
 * \{ */


class pcps_opencl_acquisition_cc;

using pcps_opencl_acquisition_cc_sptr = gnss_shared_ptr<pcps_opencl_acquisition_cc>;

pcps_opencl_acquisition_cc_sptr pcps_make_opencl_acquisition_cc(const Acq_Conf& conf, uint32_t max_dwells);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 *
 * The search runs on the first OpenCL GPU device found. Without one, or after
 * an OpenCL error, it runs on the CPU.
 */
class pcps_opencl_acquisition_cc : public acquisition_impl_interface
{
public:
    /*!
     * \brief Default destructor.
     */
    ~pcps_opencl_acquisition_cc();

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to exchange synchronization data between acquisition and tracking blocks.
     * \param p_gnss_synchro Satellite information shared by the processing blocks.
     */
    inline void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override
    {
        d_gnss_synchro = p_gnss_synchro;
    }

    /*!
     * \brief Returns the maximum peak of grid search.
     */
    inline uint32_t mag() const override
    {
        return d_mag;
    }

    /*!
     * \brief Sets local code for PCPS acquisition algorithm.
     * \param code - Pointer to the PRN code.
     */
    void set_local_code(std::complex<float>* code) override;

    /*!
     * \brief Starts acquisition algorithm, turning from standby mode to
     * active mode
     * \param active - bool that activates/deactivates the block.
     */
    inline void set_active(bool active) override
    {
        std::lock_guard<std::mutex> lock(d_state_mutex);
        if (!active)
            {
                d_state = 0;
            }

        if (active)
            {
                // GNSS-SDR.deterministic=true: a lookahead after the event that
                // caused it. Before d_active, so that general_work() never sees
                // an active search without its stamped start
                d_det.activate(d_sample_counter, d_channel);
            }
        d_active = active;
    }

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
        d_channel_fsm = channel_fsm;
    }

    /*!
     * \brief True if the search runs on an OpenCL GPU device, false if it
     * runs on the CPU.
     */
    inline bool opencl_ready() const
    {
        return d_use_opencl;
    }

    /*!
     * \brief Stops the search and waits for the acquisition core.
     */
    bool stop() override;

    /*!
     * \brief Parallel Code Phase Search Acquisition signal processing.
     */
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items) override;

private:
    friend pcps_opencl_acquisition_cc_sptr
    pcps_make_opencl_acquisition_cc(const Acq_Conf& conf, uint32_t max_dwells);

    explicit pcps_opencl_acquisition_cc(const Acq_Conf& conf, uint32_t max_dwells);

    // OpenCL objects, defined in the .cc file so that the OpenCL headers do
    // not reach the users of this block
    struct OpenClState;

    void init_opencl();
    int process_input(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);
    void acquisition_core();
    void search_opencl(const gr_complex* in);
    void search_volk(const gr_complex* in);
    void correlate_volk(const gr_complex* in, uint32_t doppler_index);
    void dump_correlation(int doppler);
    // Test statistic of the dwell, once all its Doppler bins are searched
    void compute_test_statistics(uint64_t samplestamp);

    // Doppler bin whose correlation floor is the CFAR noise reference
    uint32_t reference_doppler_index(uint32_t peak_doppler_index) const;

    void conclude_dwell(uint64_t samplestamp);
    void run_acquisition_cores();
    bool next_dwell_ready();
    void join_core();

    const Acq_Conf d_acq_params;

    std::ofstream d_dump_file;

    Gnss_Synchro* d_gnss_synchro;

    std::atomic<uint64_t> d_sample_counter;  // also read by the acquisition cores

    float d_mag;
    float d_input_power;
    float d_test_statistics;

    std::atomic<int> d_state;  // set by the acquisition cores

    uint32_t d_max_dwells;
    std::atomic<uint32_t> d_well_count;  // dwells processed by the acquisition cores
    const uint32_t d_fft_size;
    const uint32_t d_fft_size_pow2;  // OpenCL FFT size: a power of two, at least 2 * d_fft_size
    uint32_t d_num_doppler_bins;
    uint32_t d_channel;
    std::atomic<uint32_t> d_in_dwell_count;  // dwells in d_in_buffer

    std::atomic<bool> d_active;
    std::atomic<bool> d_core_working;
    std::atomic<bool> d_use_opencl;  // false without a GPU device, or after an OpenCL error
    std::mutex d_state_mutex;        // a result and a deactivation do not cross
    std::mutex d_core_mutex;         // the FFT engines and the local code, shared by set_local_code() and the acquisition core
    std::thread d_core_thread;
    DeterministicAcquisition d_det;  // GNSS-SDR.deterministic=true

    std::weak_ptr<ChannelFsm> d_channel_fsm;

    std::unique_ptr<OpenClState> d_cl;  // nullptr without a GPU device

    // CPU implementation, also the fallback after an OpenCL error
    std::unique_ptr<gnss_fft_complex_fwd> d_fft_if;
    std::unique_ptr<gnss_fft_complex_rev> d_ifft;

    std::vector<std::vector<gr_complex>> d_grid_doppler_wipeoffs;
    std::vector<std::vector<gr_complex>> d_in_buffer;
    std::vector<gr_complex> d_fft_codes;
    std::vector<uint64_t> d_sample_counter_buffer;
    std::vector<float> d_magnitude;
    std::vector<float> d_bin_peak;      // per Doppler bin: maximum |correlation|^2
    std::vector<uint32_t> d_bin_index;  // per Doppler bin: its lag
    std::vector<float> d_bin_power;     // per Doppler bin: sum of |correlation|^2

    // Mean |correlation|^2 / d_fft_size^2 of each Doppler bin of the dwell
    std::vector<float> d_doppler_bin_power;
    // Peak of the dwell (its value is d_mag)
    uint32_t d_peak_code_phase{0};
    uint32_t d_peak_doppler_index{0};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PCPS_OPENCL_ACQUISITION_CC_H
