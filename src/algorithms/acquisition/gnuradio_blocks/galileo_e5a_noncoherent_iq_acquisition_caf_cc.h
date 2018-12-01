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
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_CC_H_
#define GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_CC_H_

#include <fstream>
#include <string>
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/fft/fft.h>
#include "gnss_synchro.h"

class galileo_e5a_noncoherentIQ_acquisition_caf_cc;

typedef boost::shared_ptr<galileo_e5a_noncoherentIQ_acquisition_caf_cc> galileo_e5a_noncoherentIQ_acquisition_caf_cc_sptr;

galileo_e5a_noncoherentIQ_acquisition_caf_cc_sptr
galileo_e5a_noncoherentIQ_make_acquisition_caf_cc(unsigned int sampled_ms,
    unsigned int max_dwells,
    unsigned int doppler_max, int64_t fs_in,
    int samples_per_ms, int samples_per_code,
    bool bit_transition_flag,
    bool dump,
    std::string dump_filename,
    bool both_signal_components_,
    int CAF_window_hz_,
    int Zero_padding_);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 */
class galileo_e5a_noncoherentIQ_acquisition_caf_cc : public gr::block
{
private:
    friend galileo_e5a_noncoherentIQ_acquisition_caf_cc_sptr
    galileo_e5a_noncoherentIQ_make_acquisition_caf_cc(
        unsigned int sampled_ms,
        unsigned int max_dwells,
        unsigned int doppler_max, int64_t fs_in,
        int samples_per_ms, int samples_per_code,
        bool bit_transition_flag,
        bool dump,
        std::string dump_filename,
        bool both_signal_components_,
        int CAF_window_hz_,
        int Zero_padding_);

    galileo_e5a_noncoherentIQ_acquisition_caf_cc(
        unsigned int sampled_ms,
        unsigned int max_dwells,
        unsigned int doppler_max, int64_t fs_in,
        int samples_per_ms, int samples_per_code,
        bool bit_transition_flag,
        bool dump,
        std::string dump_filename,
        bool both_signal_components_,
        int CAF_window_hz_,
        int Zero_padding_);

    void calculate_magnitudes(gr_complex* fft_begin, int doppler_shift,
        int doppler_offset);
    float estimate_input_power(gr_complex* in);

    int64_t d_fs_in;
    int d_samples_per_ms;
    int d_sampled_ms;
    int d_samples_per_code;
    unsigned int d_doppler_resolution;
    float d_threshold;
    std::string d_satellite_str;
    unsigned int d_doppler_max;
    unsigned int d_doppler_step;
    unsigned int d_max_dwells;
    unsigned int d_well_count;
    unsigned int d_fft_size;
    uint64_t d_sample_counter;
    gr_complex** d_grid_doppler_wipeoffs;
    unsigned int d_num_doppler_bins;
    gr_complex* d_fft_code_I_A;
    gr_complex* d_fft_code_I_B;
    gr_complex* d_fft_code_Q_A;
    gr_complex* d_fft_code_Q_B;
    gr_complex* d_inbuffer;
    gr::fft::fft_complex* d_fft_if;
    gr::fft::fft_complex* d_ifft;
    Gnss_Synchro* d_gnss_synchro;
    unsigned int d_code_phase;
    float d_doppler_freq;
    float d_mag;
    float* d_magnitudeIA;
    float* d_magnitudeIB;
    float* d_magnitudeQA;
    float* d_magnitudeQB;
    float d_input_power;
    float d_test_statistics;
    bool d_bit_transition_flag;
    std::ofstream d_dump_file;
    bool d_active;
    int d_state;
    bool d_dump;
    bool d_both_signal_components;
    //    bool d_CAF_filter;
    int d_CAF_window_hz;
    float* d_CAF_vector;
    float* d_CAF_vector_I;
    float* d_CAF_vector_Q;
    //    double* d_CAF_vector;
    //    double* d_CAF_vector_I;
    //    double* d_CAF_vector_Q;
    unsigned int d_channel;
    std::string d_dump_filename;
    unsigned int d_buffer_count;
    unsigned int d_gr_stream_buffer;

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
};
#endif /* GALILEO_E5A_NONCOHERENT_IQ_ACQUISITION_CAF_CC_H_ */
