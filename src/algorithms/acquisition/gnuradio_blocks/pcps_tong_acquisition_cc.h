/*!
 * \file pcps_tong_acquisition_cc.h
 * \brief This class implements a Parallel Code Phase Search Acquisition with
 * Tong algorithm.
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
 *
 *  Acquisition strategy (Kaplan book + CFAR threshold).
 *  <ol>
 *  <li> Compute the input signal power estimation.
 *  <li> Doppler serial search loop.
 *  <li> Perform the FFT-based circular convolution (parallel time search).
 *  <li> Compute the tests statistics for all the cells.
 *  <li> Accumulate the grid of tests statistics with the previous grids.
 *  <li> Record the maximum peak and the associated synchronization parameters.
 *  <li> Compare the maximum averaged test statistics with a threshold.
 *  <li> If the test statistics exceeds the threshold, increment the Tong counter.
 *  <li>   Otherwise, decrement the Tong counter.
 *  <li> If the Tong counter is equal to a given maximum value, declare positive
 *  <li>   acquisition. If the Tong counter is equa to zero, declare negative
 *  <li>   acquisition. Otherwise, process the next block.
 *  </ol>
 *
 * Kaplan book: D.Kaplan, J.Hegarty, "Understanding GPS. Principles
 * and Applications", Artech House, 2006, pp 223-227
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

#ifndef GNSS_SDR_PCPS_TONG_ACQUISITION_CC_H_
#define GNSS_SDR_PCPS_TONG_ACQUISITION_CC_H_

#include "gnss_synchro.h"
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/fft/fft.h>
#include <fstream>
#include <string>


class pcps_tong_acquisition_cc;

typedef boost::shared_ptr<pcps_tong_acquisition_cc> pcps_tong_acquisition_cc_sptr;

pcps_tong_acquisition_cc_sptr
pcps_tong_make_acquisition_cc(unsigned int sampled_ms, unsigned int doppler_max,
    long freq, long fs_in, int samples_per_ms,
    int samples_per_code, unsigned int tong_init_val,
    unsigned int tong_max_val, unsigned int tong_max_dwells,
    bool dump, std::string dump_filename);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition with
 * Tong algorithm.
 */
class pcps_tong_acquisition_cc : public gr::block
{
private:
    friend pcps_tong_acquisition_cc_sptr
    pcps_tong_make_acquisition_cc(unsigned int sampled_ms, unsigned int doppler_max,
        long freq, long fs_in, int samples_per_ms,
        int samples_per_code, unsigned int tong_init_val,
        unsigned int tong_max_val, unsigned int tong_max_dwells,
        bool dump, std::string dump_filename);

    pcps_tong_acquisition_cc(unsigned int sampled_ms, unsigned int doppler_max,
        long freq, long fs_in, int samples_per_ms,
        int samples_per_code, unsigned int tong_init_val,
        unsigned int tong_max_val, unsigned int tong_max_dwells,
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
    unsigned int d_dwell_count;
    unsigned int d_tong_count;
    unsigned int d_tong_init_val;
    unsigned int d_tong_max_val;
    unsigned int d_tong_max_dwells;
    unsigned int d_fft_size;
    unsigned long int d_sample_counter;
    gr_complex** d_grid_doppler_wipeoffs;
    unsigned int d_num_doppler_bins;
    gr_complex* d_fft_codes;
    float** d_grid_data;
    gr::fft::fft_complex* d_fft_if;
    gr::fft::fft_complex* d_ifft;
    Gnss_Synchro* d_gnss_synchro;
    unsigned int d_code_phase;
    float d_doppler_freq;
    float d_mag;
    float* d_magnitude;
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
    ~pcps_tong_acquisition_cc();

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
      * \brief Sets local code for TONG acquisition algorithm.
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
    inline void set_channel(unsigned int channel)
    {
        d_channel = channel;
    }

    /*!
      * \brief Set statistics threshold of TONG algorithm.
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

#endif /* GNSS_SDR_PCPS_TONG_ACQUISITION_CC_H_ */
