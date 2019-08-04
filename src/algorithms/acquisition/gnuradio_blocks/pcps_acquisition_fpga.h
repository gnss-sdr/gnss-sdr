/*!
 * \file pcps_acquisition_fpga.h
 * \brief This class implements a Parallel Code Phase Search Acquisition for the FPGA
 *
 *
 * Kay Borre book: K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * "A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach", Birkhauser, 2007. pp 81-84
 *
 * \authors <ul>
 *          <li> Marc Majoral, 2019. mmajoral(at)cttc.es
 *          <li> Javier Arribas, 2019. jarribas(at)cttc.es
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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


#include "channel_fsm.h"
#include "fpga_acquisition.h"
#include <boost/shared_ptr.hpp>
#include <glog/logging.h>
#include <cstdint>  // for uint32_t
#include <memory>   // for shared_ptr
#include <string>   // for string

class Gnss_Synchro;

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
    uint32_t* all_fft_codes;  // pointer to memory that contains all the code ffts
    //float downsampling_factor;
    uint32_t downsampling_factor;
    uint32_t total_block_exp;
    uint32_t excludelimit;
    bool make_2_steps;
    uint32_t num_doppler_bins_step2;
    float doppler_step2;
    bool repeat_satellite;
    uint32_t max_num_acqs;
} pcpsconf_fpga_t;

class pcps_acquisition_fpga;

using pcps_acquisition_fpga_sptr = boost::shared_ptr<pcps_acquisition_fpga>;

pcps_acquisition_fpga_sptr pcps_make_acquisition_fpga(pcpsconf_fpga_t conf_);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition that uses the FPGA.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 */
class pcps_acquisition_fpga
{
public:
    /*!
     * \brief Destructor
     */
    ~pcps_acquisition_fpga() = default;

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
     * \brief Set channel fsm associated to this acquisition instance
     */
    inline void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm)
    {
        d_channel_fsm = channel_fsm;
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
        acquisition_fpga->set_doppler_max(doppler_max);
    }

    /*!
     * \brief Set Doppler steps for the grid search
     * \param doppler_step - Frequency bin of the search grid [Hz].
     */
    inline void set_doppler_step(uint32_t doppler_step)
    {
        d_doppler_step = doppler_step;
        acquisition_fpga->set_doppler_step(doppler_step);
    }

    /*!
     * \brief Set Doppler center frequency for the grid search. It will refresh the Doppler grid.
     * \param doppler_center - Frequency center of the search grid [Hz].
     */
    inline void set_doppler_center(int32_t doppler_center)
    {
        if (doppler_center != d_doppler_center)
            {
                DLOG(INFO) << " Doppler assistance for Channel: " << d_channel << " => Doppler: " << doppler_center << "[Hz]";
                d_doppler_center = doppler_center;
            }
    }

    /*!
     * \brief This function triggers a HW reset of the FPGA PL.
     */
    void reset_acquisition(void);

private:
    friend pcps_acquisition_fpga_sptr pcps_make_acquisition_fpga(pcpsconf_fpga_t conf_);
    pcps_acquisition_fpga(pcpsconf_fpga_t conf_);
    bool d_active;
    bool d_make_2_steps;
    uint32_t d_doppler_index;
    uint32_t d_channel;
    uint32_t d_doppler_step;
    int32_t d_doppler_center;
    uint32_t d_doppler_max;
    uint32_t d_fft_size;
    uint32_t d_num_doppler_bins;
    uint32_t d_downsampling_factor;
    uint32_t d_select_queue_Fpga;
    uint32_t d_total_block_exp;
    uint32_t d_num_doppler_bins_step2;
    uint32_t d_max_num_acqs;
    int32_t d_state;
    uint64_t d_sample_counter;
    float d_threshold;
    float d_mag;
    float d_input_power;
    float d_test_statistics;
    float d_doppler_step2;
    float d_doppler_center_step_two;
    pcpsconf_fpga_t acq_parameters;
    Gnss_Synchro* d_gnss_synchro;
    std::shared_ptr<Fpga_Acquisition> acquisition_fpga;
    std::weak_ptr<ChannelFsm> d_channel_fsm;
    void send_negative_acquisition();
    void send_positive_acquisition();
    void acquisition_core(uint32_t num_doppler_bins, uint32_t doppler_step, int32_t doppler_min);
    float first_vs_second_peak_statistic(uint32_t& indext, int32_t& doppler, uint32_t num_doppler_bins, int32_t doppler_max, int32_t doppler_step);
};

#endif /* GNSS_SDR_PCPS_ACQUISITION_FPGA_H_*/
