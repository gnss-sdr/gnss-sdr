/*!
 * \file galileo_pcps_8ms_acquisition_cc.h
 * \brief This class implements a Parallel Code Phase Search Acquisition for
 * Galileo E1 signals with coherent integration time = 8 ms (two codes)
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#ifndef GNSS_SDR_PCPS_8MS_ACQUISITION_CC_H
#define GNSS_SDR_PCPS_8MS_ACQUISITION_CC_H

#include "channel_fsm.h"
#include "gnss_synchro.h"
#include <gnuradio/block.h>
#include <gnuradio/fft/fft.h>
#include <gnuradio/gr_complex.h>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_gnuradio_blocks
 * \{ */


class galileo_pcps_8ms_acquisition_cc;

using galileo_pcps_8ms_acquisition_cc_sptr = gnss_shared_ptr<galileo_pcps_8ms_acquisition_cc>;

galileo_pcps_8ms_acquisition_cc_sptr
galileo_pcps_8ms_make_acquisition_cc(uint32_t sampled_ms,
    uint32_t max_dwells,
    uint32_t doppler_max,
    int64_t fs_in,
    int32_t samples_per_ms,
    int32_t samples_per_code,
    bool dump,
    const std::string& dump_filename,
    bool enable_monitor_output);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition for
 * Galileo E1 signals with coherent integration time = 8 ms (two codes)
 */
class galileo_pcps_8ms_acquisition_cc : public gr::block
{
public:
    /*!
     * \brief Default destructor.
     */
    ~galileo_pcps_8ms_acquisition_cc();

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
    void set_state(int32_t state);

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
        d_channel_fsm = std::move(channel_fsm);
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

    /*!
     * \brief Parallel Code Phase Search Acquisition signal processing.
     */
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);

private:
    friend galileo_pcps_8ms_acquisition_cc_sptr
    galileo_pcps_8ms_make_acquisition_cc(
        uint32_t sampled_ms,
        uint32_t max_dwells,
        uint32_t doppler_max,
        int64_t fs_in,
        int32_t samples_per_ms,
        int32_t samples_per_code,
        bool dump,
        const std::string& dump_filename,
        bool enable_monitor_output);

    galileo_pcps_8ms_acquisition_cc(
        uint32_t sampled_ms,
        uint32_t max_dwells,
        uint32_t doppler_max,
        int64_t fs_in,
        int32_t samples_per_ms,
        int32_t samples_per_code,
        bool dump,
        const std::string& dump_filename,
        bool enable_monitor_output);

    void calculate_magnitudes(
        gr_complex* fft_begin,
        int32_t doppler_shift,
        int32_t doppler_offset);

    std::weak_ptr<ChannelFsm> d_channel_fsm;
#if GNURADIO_FFT_USES_TEMPLATES
    std::unique_ptr<gr::fft::fft_complex_fwd> d_fft_if;
    std::unique_ptr<gr::fft::fft_complex_rev> d_ifft;
#else
    std::unique_ptr<gr::fft::fft_complex> d_fft_if;
    std::unique_ptr<gr::fft::fft_complex> d_ifft;
#endif
    std::vector<std::vector<gr_complex>> d_grid_doppler_wipeoffs;
    std::vector<gr_complex> d_fft_code_A;
    std::vector<gr_complex> d_fft_code_B;
    std::vector<float> d_magnitude;

    std::string d_satellite_str;
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    Gnss_Synchro* d_gnss_synchro;

    int64_t d_fs_in;
    uint64_t d_sample_counter;

    float d_threshold;
    float d_doppler_freq;
    float d_mag;
    float d_input_power;
    float d_test_statistics;
    int32_t d_state;
    int32_t d_samples_per_ms;
    int32_t d_samples_per_code;
    uint32_t d_channel;
    uint32_t d_doppler_resolution;
    uint32_t d_doppler_max;
    uint32_t d_doppler_step;
    uint32_t d_sampled_ms;
    uint32_t d_max_dwells;
    uint32_t d_well_count;
    uint32_t d_fft_size;
    uint32_t d_num_doppler_bins;
    uint32_t d_code_phase;

    bool d_active;
    bool d_dump;
    bool d_enable_monitor_output;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PCPS_8MS_ACQUISITION_CC_H
