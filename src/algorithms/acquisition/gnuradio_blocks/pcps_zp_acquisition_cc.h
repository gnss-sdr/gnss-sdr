/*!
 * \file pcps_zp_acquisition_cc.h
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
 *          <li> Antonio Ramos, 2017. antonio.ramos(at)cttc.es
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_PCPS_ZP_ACQUISITION_CC_H_
#define GNSS_SDR_PCPS_ZP_ACQUISITION_CC_H_

#include <fstream>
#include <string>
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/fft/fft.h>
#include "gnss_synchro.h"

class pcps_zp_acquisition_cc;

typedef boost::shared_ptr<pcps_zp_acquisition_cc> pcps_zp_acquisition_cc_sptr;

pcps_zp_acquisition_cc_sptr
pcps_zp_make_acquisition_cc(unsigned int sampled_ms, unsigned int max_dwells,
                         unsigned int doppler_max, long freq, long fs_in,
                         int samples_per_ms, int samples_per_code,
                         bool bit_transition_flag, bool use_CFAR_algorithm_flag,
                         bool dump, bool blocking,
                         std::string dump_filename);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 */
class pcps_zp_acquisition_cc: public gr::block
{
private:
    friend pcps_zp_acquisition_cc_sptr
    pcps_zp_make_acquisition_cc(unsigned int sampled_ms, unsigned int max_dwells,
            unsigned int doppler_max, long freq, long fs_in,
            int samples_per_ms, int samples_per_code,
            bool bit_transition_flag, bool use_CFAR_algorithm_flag,
            bool dump, bool blocking,
            std::string dump_filename);

    pcps_zp_acquisition_cc(unsigned int sampled_ms, unsigned int max_dwells,
            unsigned int doppler_max, long freq, long fs_in,
            int samples_per_ms, int samples_per_code,
            bool bit_transition_flag, bool use_CFAR_algorithm_flag,
            bool dump, bool blocking, 
            std::string dump_filename);

    void update_local_carrier(gr_complex* carrier_vector, int correlator_length_samples, float freq);

    void acquisition_core(const gr_complex* data_in);

    void send_negative_acquisition();

    void send_positive_acquisition();

    long d_fs_in;
    long d_freq;
    int d_samples_per_ms;
    int d_samples_per_code;
    int d_state;
    unsigned int d_doppler_max;
    unsigned int d_doppler_step;
    unsigned int d_sampled_ms;
    unsigned int d_max_dwells;
    unsigned int d_well_count;
    unsigned int d_fft_size;
    unsigned int d_segment_size;
    unsigned int d_num_doppler_bins;
    unsigned int d_channel;
    unsigned long int d_sample_counter;
    float d_threshold;
    float d_doppler_freq;
    float d_mag;
    float d_input_power;
    float d_test_statistics;
    float* d_magnitude;
    bool d_bit_transition_flag;
    bool d_use_CFAR_algorithm_flag;
    bool d_dump;
    bool d_blocking;
    bool d_active;
    bool d_testing;
    std::ofstream d_dump_file;
    std::string d_dump_filename;
    std::string d_satellite_str;
    gr_complex** d_grid_doppler_wipeoffs;
    gr_complex* d_fft_codes;
    gr_complex* d_data_buffer;
    gr::fft::fft_complex* d_fft_if;
    gr::fft::fft_complex* d_ifft;
    Gnss_Synchro* d_gnss_synchro;

public:

     ~pcps_zp_acquisition_cc();

     /*!
      * \brief Set acquisition/tracking common Gnss_Synchro object pointer
      * to exchange synchronization data between acquisition and tracking blocks.
      * \param p_gnss_synchro Satellite information shared by the processing blocks.
      */
     inline void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
     {
         gr::thread::scoped_lock lock(d_setlock); // require mutex with work function called by the scheduler
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
     void set_local_code(std::complex<float> * code);

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
     inline void set_testing(bool testing)
     {
    	 gr::thread::scoped_lock lock(d_setlock);
    	 d_testing = testing;
     }

     inline void set_channel(unsigned int channel)
     {
         gr::thread::scoped_lock lock(d_setlock); // require mutex with work function called by the scheduler
         d_channel = channel;
     }

     /*!
      * \brief Set statistics threshold of PCPS algorithm.
      * \param threshold - Threshold for signal detection (check \ref Navitec2012,
      * Algorithm 1, for a definition of this threshold).
      */
     inline void set_threshold(float threshold)
     {
         gr::thread::scoped_lock lock(d_setlock); // require mutex with work function called by the scheduler
         d_threshold = threshold;
     }

     /*!
      * \brief Set maximum Doppler grid search
      * \param doppler_max - Maximum Doppler shift considered in the grid search [Hz].
      */
     inline void set_doppler_max(unsigned int doppler_max)
     {
         gr::thread::scoped_lock lock(d_setlock); // require mutex with work function called by the scheduler
         d_doppler_max = doppler_max;
     }

     /*!
      * \brief Set Doppler steps for the grid search
      * \param doppler_step - Frequency bin of the search grid [Hz].
      */
     inline void set_doppler_step(unsigned int doppler_step)
     {
         gr::thread::scoped_lock lock(d_setlock); // require mutex with work function called by the scheduler
         d_doppler_step = doppler_step;
     }

     inline void set_active(bool active)
     {
         gr::thread::scoped_lock lock(d_setlock); // require mutex with work function called by the scheduler
         d_active = active;
     }

     /*!
      * \brief Parallel Code Phase Search Acquisition signal processing.
      */
     int general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
};

#endif /* GNSS_SDR_PCPS_ZP_ACQUISITION_CC_H_*/
