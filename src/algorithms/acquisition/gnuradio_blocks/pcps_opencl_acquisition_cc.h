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

#define CL_SILENCE_DEPRECATION
#include "channel_fsm.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_fft.h"
#include "gnss_synchro.h"
#include "opencl/fft_internal.h"
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include "opencl/cl.hpp"
#include <cstdint>
#include <fstream>
#include <memory>  // for weak_ptr
#include <string>
#include <vector>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_gnuradio_blocks
 * \{ */


class pcps_opencl_acquisition_cc;

using pcps_opencl_acquisition_cc_sptr = gnss_shared_ptr<pcps_opencl_acquisition_cc>;

pcps_opencl_acquisition_cc_sptr pcps_make_opencl_acquisition_cc(
    uint32_t sampled_ms,
    uint32_t max_dwells,
    uint32_t doppler_max,
    int64_t fs_in,
    int samples_per_ms,
    int samples_per_code,
    bool bit_transition_flag,
    bool dump,
    const std::string& dump_filename,
    bool enable_monitor_output);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 */
class pcps_opencl_acquisition_cc : public gr::block
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
    inline void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
    {
        d_gnss_synchro = p_gnss_synchro;
    }

    /*!
     * \brief Returns the maximum peak of grid search.
     */
    inline uint32_t mag() const
    {
        return d_mag;
    }

    /*!
     * \brief Initializes acquisition algorithm.
     */
    void init();

    /*!
     * \brief Sets local code for PCPS acquisition algorithm.
     * \param code - Pointer to the PRN code.
     */
    void set_local_code(std::complex<float>* code);

    /*!
     * \brief Starts acquisition algorithm, turning from standby mode to
     * active mode
     * \param active - bool that activates/deactivates the block.
     */
    inline void set_active(bool active)
    {
        d_active = active;
    }

    /*!
     * \brief If set to 1, ensures that acquisition starts at the
     * first available sample.
     * \param state - int=1 forces start of acquisition
     */
    void set_state(int state);

    /*!
     * \brief Set acquisition channel unique ID
     * \param channel - receiver channel.
     */
    inline void set_channel(uint32_t channel)
    {
        d_channel = channel;
    }

    /*!
     * \brief Set channel fsm associated to this acquisition instance
     */
    inline void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm)
    {
        d_channel_fsm = channel_fsm;
    }

    /*!
     * \brief Set statistics threshold of PCPS algorithm.
     * \param threshold - Threshold for signal detection (check \ref Navitec2012,
     * Algorithm 1, for a definition of this threshold).
     */
    inline void set_threshold(float threshold)
    {
        d_threshold = threshold;
    }

    /*!
     * \brief Set maximum Doppler grid search
     * \param doppler_max - Maximum Doppler shift considered in the grid search [Hz].
     */
    inline void set_doppler_max(uint32_t doppler_max)
    {
        d_doppler_max = doppler_max;
    }

    /*!
     * \brief Set Doppler steps for the grid search
     * \param doppler_step - Frequency bin of the search grid [Hz].
     */
    inline void set_doppler_step(uint32_t doppler_step)
    {
        d_doppler_step = doppler_step;
    }

    inline bool opencl_ready() const
    {
        bool ready = false;
        if (d_opencl == 0)
            {
                ready = true;
            }
        return ready;
    }

    void acquisition_core_volk();

    void acquisition_core_opencl();

    /*!
     * \brief Parallel Code Phase Search Acquisition signal processing.
     */
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);

private:
    friend pcps_opencl_acquisition_cc_sptr
    pcps_make_opencl_acquisition_cc(uint32_t sampled_ms, uint32_t max_dwells,
        uint32_t doppler_max, int64_t fs_in,
        int samples_per_ms, int samples_per_code,
        bool bit_transition_flag,
        bool dump,
        const std::string& dump_filename,
        bool enable_monitor_output);

    pcps_opencl_acquisition_cc(uint32_t sampled_ms, uint32_t max_dwells,
        uint32_t doppler_max, int64_t fs_in,
        int samples_per_ms, int samples_per_code,
        bool bit_transition_flag,
        bool dump,
        const std::string& dump_filename,
        bool enable_monitor_output);

    void calculate_magnitudes(gr_complex* fft_begin, int doppler_shift,
        int doppler_offset);

    int init_opencl_environment(const std::string& kernel_filename);

    cl::Platform d_cl_platform;
    cl::Device d_cl_device;
    cl::Context d_cl_context;
    cl::Program d_cl_program;
    cl::Buffer* d_cl_buffer_in;
    cl::Buffer* d_cl_buffer_fft_codes;
    cl::Buffer* d_cl_buffer_1;
    cl::Buffer* d_cl_buffer_2;
    cl::Buffer* d_cl_buffer_magnitude;
    cl::Buffer** d_cl_buffer_grid_doppler_wipeoffs;
    cl::CommandQueue* d_cl_queue;
    clFFT_Plan d_cl_fft_plan;
    cl_int d_cl_fft_batch_size;

    std::weak_ptr<ChannelFsm> d_channel_fsm;

    std::unique_ptr<gnss_fft_complex_fwd> d_fft_if;
    std::unique_ptr<gnss_fft_complex_rev> d_ifft;

    std::vector<std::vector<gr_complex>> d_grid_doppler_wipeoffs;
    std::vector<std::vector<gr_complex>> d_in_buffer;
    std::vector<gr_complex> d_fft_codes;
    std::vector<gr_complex> d_zero_vector;
    std::vector<uint64_t> d_sample_counter_buffer;
    std::vector<float> d_magnitude;

    std::string d_dump_filename;
    std::string d_satellite_str;

    std::ofstream d_dump_file;

    Gnss_Synchro* d_gnss_synchro;

    int64_t d_fs_in;
    uint64_t d_sample_counter;

    int* d_max_doppler_indexs;

    float d_threshold;
    float d_doppler_freq;
    float d_mag;
    float d_input_power;
    float d_test_statistics;

    int d_samples_per_ms;
    int d_samples_per_code;
    int d_state;
    int d_opencl;

    uint32_t d_doppler_resolution;
    uint32_t d_doppler_max;
    uint32_t d_doppler_step;
    uint32_t d_sampled_ms;
    uint32_t d_max_dwells;
    uint32_t d_well_count;
    uint32_t d_fft_size;
    uint32_t d_fft_size_pow2;
    uint32_t d_num_doppler_bins;
    uint32_t d_code_phase;
    uint32_t d_channel;
    uint32_t d_in_dwell_count;

    bool d_bit_transition_flag;
    bool d_active;
    bool d_core_working;
    bool d_dump;
    bool d_enable_monitor_output;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PCPS_OPENCL_ACQUISITION_CC_H
