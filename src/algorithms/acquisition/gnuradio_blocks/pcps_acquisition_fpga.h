/*!
 * \file pcps_acquisition_fpga.h
 * \brief This class implements a Parallel Code Phase Search Acquisition in the FPGA.
 *
 * Note: The CFAR algorithm is not implemented in the FPGA.
 * Note 2: The bit transition flag is not implemented in the FPGA
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
 *          <li> Marc Majoral, 2017. mmajoral(at)cttc.cat
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          <li> Cillian O'Driscoll, 2017. cillian(at)ieee.org
 *          <li> Antonio Ramos, 2017. antonio.ramos@cttc.es
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

#ifndef GNSS_SDR_PCPS_ACQUISITION_FPGA_H_
#define GNSS_SDR_PCPS_ACQUISITION_FPGA_H_


#include <gnuradio/block.h>
#include "fpga_acquisition.h"
#include "gnss_synchro.h"

typedef struct
{
    /* pcps acquisition configuration */
    unsigned int sampled_ms;
    unsigned int doppler_max;
    long freq;
    long fs_in;
    int samples_per_ms;
    int samples_per_code;
    unsigned int select_queue_Fpga;
    std::string device_name;
    lv_16sc_t *all_fft_codes; // memory that contains all the code ffts

} pcpsconf_fpga_t;

class pcps_acquisition_fpga;

typedef boost::shared_ptr<pcps_acquisition_fpga> pcps_acquisition_fpga_sptr;

pcps_acquisition_fpga_sptr
pcps_make_acquisition_fpga(pcpsconf_fpga_t conf_);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition that uses the FPGA.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 */
class pcps_acquisition_fpga : public gr::block
{
private:
    friend pcps_acquisition_fpga_sptr

    pcps_make_acquisition_fpga(pcpsconf_fpga_t conf_);

    pcps_acquisition_fpga(pcpsconf_fpga_t conf_);

    void send_negative_acquisition();

    void send_positive_acquisition();

    pcpsconf_fpga_t acq_parameters;
    bool d_active;
    float d_threshold;
    float d_mag;
    float d_input_power;
    float d_test_statistics;
    int d_state;
    unsigned int d_channel;
    unsigned int d_doppler_step;
    unsigned int d_fft_size;
    unsigned int d_num_doppler_bins;
    unsigned long int d_sample_counter;
    Gnss_Synchro* d_gnss_synchro;
    std::shared_ptr<fpga_acquisition> acquisition_fpga;

public:
    ~pcps_acquisition_fpga();

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
    void set_local_code();

    /*!
      * \brief If set to 1, ensures that acquisition starts at the
      * first available sample.
      * \param state - int=1 forces start of acquisition
      */
    void set_state(int state);

    /*!
      * \brief Starts acquisition algorithm, turning from standby mode to
      * active mode
      * \param active - bool that activates/deactivates the block.
      */
    void set_active(bool active);

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
        acq_parameters.doppler_max = doppler_max;
        acquisition_fpga->set_doppler_max(doppler_max);
    }

    /*!
      * \brief Set Doppler steps for the grid search
      * \param doppler_step - Frequency bin of the search grid [Hz].
      */
    inline void set_doppler_step(unsigned int doppler_step)
    {
        d_doppler_step = doppler_step;
        acquisition_fpga->set_doppler_step(doppler_step);
    }

    /*!
      * \brief Parallel Code Phase Search Acquisition signal processing.
      */
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);
};

#endif /* GNSS_SDR_PCPS_ACQUISITION_FPGA_H_*/
