/*!
 * \file pcps_assisted_acquisition_cc.h
 * \brief This class implements a Parallel Code Phase Search Acquisition with assistance and multi-dwells
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
 *          <li> Javier Arribas, 2013. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_PCPS_ASSISTED_ACQUISITION_CC_H_
#define GNSS_SDR_PCPS_ASSISTED_ACQUISITION_CC_H_

#include <fstream>
#include <string>
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/fft/fft.h>
#include "gnss_synchro.h"

class pcps_assisted_acquisition_cc;

typedef boost::shared_ptr<pcps_assisted_acquisition_cc>
    pcps_assisted_acquisition_cc_sptr;

pcps_assisted_acquisition_cc_sptr
pcps_make_assisted_acquisition_cc(int max_dwells, unsigned int sampled_ms,
    int doppler_max, int doppler_min, long freq, long fs_in, int samples_per_ms,
    bool dump, std::string dump_filename);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 */
class pcps_assisted_acquisition_cc : public gr::block
{
private:
    friend pcps_assisted_acquisition_cc_sptr
    pcps_make_assisted_acquisition_cc(int max_dwells, unsigned int sampled_ms,
        int doppler_max, int doppler_min, long freq, long fs_in,
        int samples_per_ms, bool dump,
        std::string dump_filename);

    pcps_assisted_acquisition_cc(int max_dwells, unsigned int sampled_ms,
        int doppler_max, int doppler_min, long freq, long fs_in,
        int samples_per_ms, bool dump,
        std::string dump_filename);

    void calculate_magnitudes(gr_complex* fft_begin, int doppler_shift,
        int doppler_offset);

    int compute_and_accumulate_grid(gr_vector_const_void_star& input_items);
    float estimate_input_power(gr_vector_const_void_star& input_items);
    double search_maximum();
    void get_assistance();
    void reset_grid();
    void redefine_grid();
    void free_grid_memory();

    long d_fs_in;
    long d_freq;
    int d_samples_per_ms;
    int d_max_dwells;
    unsigned int d_doppler_resolution;
    int d_gnuradio_forecast_samples;
    float d_threshold;
    std::string d_satellite_str;
    int d_doppler_max;
    int d_doppler_min;
    int d_config_doppler_max;
    int d_config_doppler_min;

    int d_num_doppler_points;
    int d_doppler_step;
    unsigned int d_sampled_ms;
    unsigned int d_fft_size;
    unsigned long int d_sample_counter;
    gr_complex* d_carrier;
    gr_complex* d_fft_codes;

    float** d_grid_data;
    gr_complex** d_grid_doppler_wipeoffs;

    gr::fft::fft_complex* d_fft_if;
    gr::fft::fft_complex* d_ifft;
    Gnss_Synchro* d_gnss_synchro;
    unsigned int d_code_phase;
    float d_doppler_freq;
    float d_input_power;
    float d_test_statistics;
    std::ofstream d_dump_file;
    int d_state;
    bool d_active;
    bool d_disable_assist;
    int d_well_count;
    bool d_dump;
    unsigned int d_channel;

    std::string d_dump_filename;

public:
    /*!
     * \brief Default destructor.
     */
    ~pcps_assisted_acquisition_cc();

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
        return d_test_statistics;
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
    void set_doppler_step(unsigned int doppler_step);

    /*!
     * \brief Parallel Code Phase Search Acquisition signal processing.
     */
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);

    void forecast(int noutput_items, gr_vector_int& ninput_items_required);
};

#endif /* GNSS_SDR_PCPS_assisted_acquisition_cc_H_*/
