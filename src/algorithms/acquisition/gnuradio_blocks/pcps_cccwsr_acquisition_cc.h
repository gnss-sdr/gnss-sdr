/*!
 * \file pcps_cccwsr_acquisition_cc.h
 * \brief This class implements a Parallel Code Phase Search acquisition
 *  with Coherent Channel Combining With Sign Recovery scheme.
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
 *
 * D.Borio, C.O'Driscoll, G.Lachapelle, "Coherent, Noncoherent and
 * Differentially Coherent Combining Techniques for Acquisition of
 * New Composite GNSS Signals", IEEE Transactions On Aerospace and
 * Electronic Systems vol. 45 no. 3, July 2009, section IV
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_PCPS_CCCWSR_ACQUISITION_CC_H_
#define GNSS_SDR_PCPS_CCCWSR_ACQUISITION_CC_H_

#include <fstream>
#include <string>
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/fft/fft.h>
#include "gnss_synchro.h"


class pcps_cccwsr_acquisition_cc;

typedef boost::shared_ptr<pcps_cccwsr_acquisition_cc> pcps_cccwsr_acquisition_cc_sptr;

pcps_cccwsr_acquisition_cc_sptr
pcps_cccwsr_make_acquisition_cc(unsigned int sampled_ms, unsigned int max_dwells,
    unsigned int doppler_max, long freq, long fs_in,
    int samples_per_ms, int samples_per_code,
    bool dump, std::string dump_filename);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition with
 * Coherent Channel Combining With Sign Recovery scheme.
 */
class pcps_cccwsr_acquisition_cc : public gr::block
{
private:
    friend pcps_cccwsr_acquisition_cc_sptr
    pcps_cccwsr_make_acquisition_cc(unsigned int sampled_ms, unsigned int max_dwells,
        unsigned int doppler_max, long freq, long fs_in,
        int samples_per_ms, int samples_per_code,
        bool dump, std::string dump_filename);

    pcps_cccwsr_acquisition_cc(unsigned int sampled_ms, unsigned int max_dwells,
        unsigned int doppler_max, long freq, long fs_in,
        int samples_per_ms, int samples_per_code,
        bool dump, std::string dump_filename);

    void calculate_magnitudes(gr_complex* fft_begin, int doppler_shift,
        int doppler_offset);

    long d_fs_in;
    long d_freq;
    int d_samples_per_ms;
    int d_samples_per_code;
    unsigned int d_doppler_resolution;
    float d_threshold;
    std::string d_satellite_str;
    unsigned int d_doppler_max;
    unsigned int d_doppler_step;
    unsigned int d_sampled_ms;
    unsigned int d_max_dwells;
    unsigned int d_well_count;
    unsigned int d_fft_size;
    unsigned long int d_sample_counter;
    gr_complex** d_grid_doppler_wipeoffs;
    unsigned int d_num_doppler_bins;
    gr_complex* d_fft_code_data;
    gr_complex* d_fft_code_pilot;
    gr::fft::fft_complex* d_fft_if;
    gr::fft::fft_complex* d_ifft;
    Gnss_Synchro* d_gnss_synchro;
    unsigned int d_code_phase;
    float d_doppler_freq;
    float d_mag;
    float* d_magnitude;
    gr_complex* d_data_correlation;
    gr_complex* d_pilot_correlation;
    gr_complex* d_correlation_plus;
    gr_complex* d_correlation_minus;
    float d_input_power;
    float d_test_statistics;
    std::ofstream d_dump_file;
    bool d_active;
    int d_state;
    bool d_dump;
    unsigned int d_channel;
    std::string d_dump_filename;

public:
    /*!
     * \brief Default destructor.
     */
    ~pcps_cccwsr_acquisition_cc();

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
      * \brief Sets local code for CCCWSR acquisition algorithm.
      * \param data_code - Pointer to the data PRN code.
      * \param pilot_code - Pointer to the pilot PRN code.
      */
    void set_local_code(std::complex<float>* code_data, std::complex<float>* code_pilot);

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
      * \brief Set statistics threshold of CCCWSR algorithm.
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
      * \brief Coherent Channel Combining With Sign Recovery Acquisition signal processing.
      */
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);
};

#endif /* GNSS_SDR_PCPS_CCCWSR_ACQUISITION_CC_H_*/
