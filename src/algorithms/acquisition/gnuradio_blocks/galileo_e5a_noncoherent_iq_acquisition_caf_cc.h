/*!
 * \file galileo_e5a_noncoherent_iq_acquisition_caf_cc.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
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

#ifndef GNSS_SDR_GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_CC_H
#define GNSS_SDR_GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_CC_H

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


class galileo_e5a_noncoherentIQ_acquisition_caf_cc;

using galileo_e5a_noncoherentIQ_acquisition_caf_cc_sptr = gnss_shared_ptr<galileo_e5a_noncoherentIQ_acquisition_caf_cc>;

galileo_e5a_noncoherentIQ_acquisition_caf_cc_sptr galileo_e5a_noncoherentIQ_make_acquisition_caf_cc(
    unsigned int sampled_ms,
    unsigned int max_dwells,
    unsigned int doppler_max, int64_t fs_in,
    int samples_per_ms, int samples_per_code,
    bool bit_transition_flag,
    bool dump,
    const std::string& dump_filename,
    bool both_signal_components_,
    int CAF_window_hz_,
    int Zero_padding_,
    bool enable_monitor_output);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 */
class galileo_e5a_noncoherentIQ_acquisition_caf_cc : public gr::block
{
public:
    /*!
     * \brief Default destructor.
     */
    ~galileo_e5a_noncoherentIQ_acquisition_caf_cc();

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
    inline unsigned int mag() const
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
    void set_local_code(std::complex<float>* code, std::complex<float>* codeQ);

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
    inline void set_channel(unsigned int channel)
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
    inline void set_doppler_max(unsigned int doppler_max)
    {
        d_doppler_max = doppler_max;
    }

    /*!
     * \brief Set Doppler steps for the grid search
     * \param doppler_step - Frequency bin of the search grid [Hz].
     */
    inline void set_doppler_step(unsigned int doppler_step)
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
    friend galileo_e5a_noncoherentIQ_acquisition_caf_cc_sptr
    galileo_e5a_noncoherentIQ_make_acquisition_caf_cc(
        unsigned int sampled_ms,
        unsigned int max_dwells,
        unsigned int doppler_max, int64_t fs_in,
        int samples_per_ms, int samples_per_code,
        bool bit_transition_flag,
        bool dump,
        const std::string& dump_filename,
        bool both_signal_components_,
        int CAF_window_hz_,
        int Zero_padding_,
        bool enable_monitor_output);

    galileo_e5a_noncoherentIQ_acquisition_caf_cc(
        unsigned int sampled_ms,
        unsigned int max_dwells,
        unsigned int doppler_max, int64_t fs_in,
        int samples_per_ms, int samples_per_code,
        bool bit_transition_flag,
        bool dump,
        const std::string& dump_filename,
        bool both_signal_components_,
        int CAF_window_hz_,
        int Zero_padding_,
        bool enable_monitor_output);

    void calculate_magnitudes(gr_complex* fft_begin, int doppler_shift,
        int doppler_offset);

    float estimate_input_power(gr_complex* in);

    std::weak_ptr<ChannelFsm> d_channel_fsm;
#if GNURADIO_FFT_USES_TEMPLATES
    std::unique_ptr<gr::fft::fft_complex_fwd> d_fft_if;
    std::unique_ptr<gr::fft::fft_complex_rev> d_ifft;
#else
    std::unique_ptr<gr::fft::fft_complex> d_fft_if;
    std::unique_ptr<gr::fft::fft_complex> d_ifft;
#endif
    std::vector<std::vector<gr_complex>> d_grid_doppler_wipeoffs;
    std::vector<gr_complex> d_fft_code_I_A;
    std::vector<gr_complex> d_fft_code_I_B;
    std::vector<gr_complex> d_fft_code_Q_A;
    std::vector<gr_complex> d_fft_code_Q_B;
    std::vector<gr_complex> d_inbuffer;
    std::vector<float> d_magnitudeIA;
    std::vector<float> d_magnitudeIB;
    std::vector<float> d_magnitudeQA;
    std::vector<float> d_magnitudeQB;
    std::vector<float> d_CAF_vector;
    std::vector<float> d_CAF_vector_I;
    std::vector<float> d_CAF_vector_Q;

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

    int d_state;
    int d_samples_per_ms;
    int d_samples_per_code;
    int d_CAF_window_hz;
    int d_buffer_count;
    int d_doppler_resolution;
    int d_doppler_max;
    int d_doppler_step;
    int d_fft_size;
    int d_num_doppler_bins;
    unsigned int d_gr_stream_buffer;
    unsigned int d_channel;
    unsigned int d_max_dwells;
    unsigned int d_well_count;
    unsigned int d_sampled_ms;
    unsigned int d_code_phase;

    bool d_bit_transition_flag;
    bool d_active;
    bool d_dump;
    bool d_both_signal_components;
    bool d_enable_monitor_output;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_CC_H
