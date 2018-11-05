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

#ifndef GNSS_SDR_PCPS_ACQUISITION_FPGA_H_
#define GNSS_SDR_PCPS_ACQUISITION_FPGA_H_


#include "fpga_acquisition.h"
#include "gnss_synchro.h"
#include <gnuradio/block.h>

typedef struct
{
    /* pcps acquisition configuration */
    uint32_t sampled_ms;
    uint32_t doppler_max;
    int64_t fs_in;
    int32_t samples_per_ms;
    int32_t samples_per_code;
    int32_t code_length;
    uint32_t select_queue_Fpga;
    std::string device_name;
    lv_16sc_t* all_fft_codes;  // memory that contains all the code ffts
    float downsampling_factor;
    uint32_t total_block_exp;
    uint32_t excludelimit;
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

    float first_vs_second_peak_statistic(uint32_t& indext, int32_t& doppler, uint32_t num_doppler_bins, int32_t doppler_max, int32_t doppler_step);

    pcpsconf_fpga_t acq_parameters;
    bool d_active;
    float d_threshold;
    float d_mag;
    float d_input_power;
    uint32_t d_doppler_index;
    float d_test_statistics;
    int32_t d_state;
    uint32_t d_channel;
    uint32_t d_doppler_step;
    uint32_t d_fft_size;
    uint32_t d_num_doppler_bins;
    uint64_t d_sample_counter;
    Gnss_Synchro* d_gnss_synchro;
    std::shared_ptr<fpga_acquisition> acquisition_fpga;

    // debug
    //float debug_d_max_absolute;
    //float debug_d_input_power_absolute;
    //int32_t debug_indext;
    //int32_t debug_doppler_index;

    float d_downsampling_factor;
    uint32_t d_select_queue_Fpga;
    bool d_single_doppler_flag;

    uint32_t d_total_block_exp;


public:
    ~pcps_acquisition_fpga();

    /*!
      * \brief Set acquisition/tracking common Gnss_Synchro object pointer
      * to exchange synchronization data between acquisition and tracking blocks.
      * \param p_gnss_synchro Satellite information shared by the processing blocks.
      */
    inline void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
    {
        //    printf("acq set gnss synchro start\n");
        d_gnss_synchro = p_gnss_synchro;
        //   printf("acq set gnss synchro end\n");
    }

    /*!
     * \brief Returns the maximum peak of grid search.
     */
    inline uint32_t mag() const
    {
        //   printf("acq dmag start\n");
        return d_mag;
        //   printf("acq dmag end\n");
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
    void set_state(int32_t state);

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
    inline void set_channel(uint32_t channel)
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
        //    printf("acq set threshold start\n");
        d_threshold = threshold;
        //   printf("acq set threshold end\n");
    }

    /*!
      * \brief Set maximum Doppler grid search
      * \param doppler_max - Maximum Doppler shift considered in the grid search [Hz].
      */
    inline void set_doppler_max(uint32_t doppler_max)
    {
        //   printf("acq set doppler max start\n");
        acq_parameters.doppler_max = doppler_max;
        acquisition_fpga->set_doppler_max(doppler_max);
        //    printf("acq set doppler max end\n");
    }

    /*!
      * \brief Set Doppler steps for the grid search
      * \param doppler_step - Frequency bin of the search grid [Hz].
      */
    inline void set_doppler_step(uint32_t doppler_step)
    {
        //   printf("acq set doppler step start\n");
        d_doppler_step = doppler_step;
        acquisition_fpga->set_doppler_step(doppler_step);
        //   printf("acq set doppler step end\n");
    }

    /*!
      * \brief Parallel Code Phase Search Acquisition signal processing.
      */
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);

    /*!
     * \brief This function is only used for the unit tests
     */
    void set_single_doppler_flag(unsigned int single_doppler_flag);

    /*!
     * \brief This funciton is only used for the unit tests
     */
    void read_acquisition_results(uint32_t *max_index,
        float *max_magnitude, float *second_magnitude, uint64_t *initial_sample, uint32_t *doppler_index, uint32_t *total_fft_scaling_factor);


    /*!
     * \brief This funciton is only used for the unit tests
     */
    void reset_acquisition(void);

    /*!
     * \brief This funciton is only used for the unit tests
     */
    void read_fpga_total_scale_factor(uint32_t *total_scale_factor, uint32_t *fw_scale_factor);
};

#endif /* GNSS_SDR_PCPS_ACQUISITION_FPGA_H_*/
