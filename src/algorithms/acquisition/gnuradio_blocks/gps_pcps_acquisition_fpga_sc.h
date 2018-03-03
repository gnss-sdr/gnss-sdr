/*!
 * \file gps_pcps_acquisition_fpga_sc.h
 * \brief This class implements a Parallel Code Phase Search Acquisition in the FPGA.
 * This file is based on the file gps_pcps_acquisition_sc.h
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
 *          <li> Marc Majoral, 2017. mmajoral(at)cttc.cat
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

#ifndef GNSS_SDR_PCPS_ACQUISITION_FPGA_SC_H_
#define GNSS_SDR_PCPS_ACQUISITION_FPGA_SC_H_

#include <fstream>
#include <string>
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/fft/fft.h>
#include "gnss_synchro.h"
#include "gps_fpga_acquisition_8sc.h"

class gps_pcps_acquisition_fpga_sc;

typedef boost::shared_ptr<gps_pcps_acquisition_fpga_sc> gps_pcps_acquisition_fpga_sc_sptr;

gps_pcps_acquisition_fpga_sc_sptr
gps_pcps_make_acquisition_fpga_sc(unsigned int sampled_ms,
    unsigned int max_dwells, unsigned int doppler_max, long freq,
    long fs_in, int samples_per_ms, int samples_per_code,
    int vector_length_, unsigned int nsamples_total_,
    bool bit_transition_flag, bool use_CFAR_algorithm_flag,
    unsigned int select_queue_Fpga, std::string device_name, bool dump,
    std::string dump_filename);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 */
class gps_pcps_acquisition_fpga_sc : public gr::block
{
private:
    friend gps_pcps_acquisition_fpga_sc_sptr
    gps_pcps_make_acquisition_fpga_sc(unsigned int sampled_ms,
        unsigned int max_dwells, unsigned int doppler_max, long freq,
        long fs_in, int samples_per_ms, int samples_per_code,
        int vector_length, unsigned int nsamples_total,
        bool bit_transition_flag, bool use_CFAR_algorithm_flag,
        unsigned int select_queue_Fpga, std::string device_name, bool dump,
        std::string dump_filename);

    gps_pcps_acquisition_fpga_sc(unsigned int sampled_ms,
        unsigned int max_dwells, unsigned int doppler_max, long freq,
        long fs_in, int samples_per_ms, int samples_per_code,
        int vector_length, unsigned int nsamples_total,
        bool bit_transition_flag, bool use_CFAR_algorithm_flag,
        unsigned int select_queue_Fpga, std::string device_name, bool dump,
        std::string dump_filename);

    int d_samples_per_code;
    float d_threshold;
    unsigned int d_doppler_max;
    unsigned int d_doppler_step;
    unsigned int d_max_dwells;
    unsigned int d_well_count;
    unsigned int d_fft_size;
    unsigned long int d_sample_counter;
    unsigned int d_num_doppler_bins;

    Gnss_Synchro *d_gnss_synchro;
    float d_mag;
    bool d_bit_transition_flag;
    bool d_use_CFAR_algorithm_flag;
    std::ofstream d_dump_file;
    bool d_active;
    int d_state;
    bool d_dump;
    unsigned int d_channel;
    std::string d_dump_filename;

    std::shared_ptr<gps_fpga_acquisition_8sc> acquisition_fpga_8sc;

public:
    /*!
     * \brief Default destructor.
     */
    ~gps_pcps_acquisition_fpga_sc();

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to exchange synchronization data between acquisition and tracking blocks.
     * \param p_gnss_synchro Satellite information shared by the processing blocks.
     */
    inline void set_gnss_synchro(Gnss_Synchro *p_gnss_synchro)
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
    void set_local_code();

    /*!
     * \brief Starts acquisition algorithm, turning from standby mode to
     * active mode
     * \param active - bool that activates/deactivates the block.
     */
    void set_active(bool active);

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
        acquisition_fpga_8sc->set_doppler_max(doppler_max);
    }

    /*!
     * \brief Set Doppler steps for the grid search
     * \param doppler_step - Frequency bin of the search grid [Hz].
     */
    inline void set_doppler_step(unsigned int doppler_step)
    {
        d_doppler_step = doppler_step;
        acquisition_fpga_8sc->set_doppler_step(doppler_step);
    }

    /*!
     * \brief Parallel Code Phase Search Acquisition signal processing.
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif /* GNSS_SDR_PCPS_ACQUISITION_SC_H_*/
