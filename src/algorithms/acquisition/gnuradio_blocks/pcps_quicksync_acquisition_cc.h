/*!
* \file pcps_quicksync_acquisition_cc.h
* \brief This class implements a Parallel Code Phase Search Acquisition with the
* QuickSync Algorithm
*
*  Acquisition strategy (Kay Borre book CFAR + threshold).
*  <ol>
*  <li> Compute the input signal power estimation
*  <li> Doppler serial search loop
*  <li> Perform folding of the incoming signal and local generated code
*  <li> Perform the FFT-based circular convolution (parallel time search)
*  <li> Record the maximum peak and the associated synchronization parameters
*  <li> Compute the test statistics and compare to the threshold
*  <li> Declare positive or negative acquisition using a message port
*  <li> Obtain the adequate acquisition parameters by correlating the incoming
*       signal shifted by the possible folded delays
*  </ol>
*
* Kay Borre book: K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
* "A Software-Defined GPS and Galileo Receiver. A Single-Frequency
* Approach", Birkha user, 2007. pp 81-84
*
* \date Jun2 2014
* \author Damian Miralles Sanchez, dmiralles2009@gmail.com
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

#ifndef GNSS_SDR_PCPS_QUICKSYNC_ACQUISITION_CC_H_
#define GNSS_SDR_PCPS_QUICKSYNC_ACQUISITION_CC_H_

#include "channel_fsm.h"
#include "gnss_synchro.h"
#include <gnuradio/block.h>
#include <gnuradio/fft/fft.h>
#include <gnuradio/gr_complex.h>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <functional>
#include <string>
#include <utility>
#include <vector>

class pcps_quicksync_acquisition_cc;

using pcps_quicksync_acquisition_cc_sptr = boost::shared_ptr<pcps_quicksync_acquisition_cc>;

pcps_quicksync_acquisition_cc_sptr pcps_quicksync_make_acquisition_cc(
    uint32_t folding_factor,
    uint32_t sampled_ms,
    uint32_t max_dwells,
    uint32_t doppler_max,
    int64_t fs_in,
    int32_t samples_per_ms,
    int32_t samples_per_code,
    bool bit_transition_flag,
    bool dump,
    std::string dump_filename);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition with
 * the implementation of the Sparse QuickSync Algorithm.
 *
 * Check \ref Navitec2012 "Faster GPS via the Sparse Fourier Transform",
 * for details of its implementation and functionality.
 */
class pcps_quicksync_acquisition_cc : public gr::block
{
public:
    /*!
     * \brief Default destructor.
     */
    ~pcps_quicksync_acquisition_cc();

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
    void set_state(int32_t state);

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
    inline void set_doppler_max(uint32_t doppler_max)
    {
        d_doppler_max = doppler_max;
    }

    /*!
     * \brief Set Doppler steps for the grid search
     * \param doppler_step - Frequency bin of the search grid [Hz].
     */
    inline void set_doppler_step(uint32_t doppler_step)
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
    friend pcps_quicksync_acquisition_cc_sptr
    pcps_quicksync_make_acquisition_cc(uint32_t folding_factor,
        uint32_t sampled_ms, uint32_t max_dwells,
        uint32_t doppler_max, int64_t fs_in,
        int32_t samples_per_ms, int32_t samples_per_code,
        bool bit_transition_flag,
        bool dump,
        std::string dump_filename);

    pcps_quicksync_acquisition_cc(uint32_t folding_factor,
        uint32_t sampled_ms, uint32_t max_dwells,
        uint32_t doppler_max, int64_t fs_in,
        int32_t samples_per_ms, int32_t samples_per_code,
        bool bit_transition_flag,
        bool dump,
        std::string dump_filename);

    void calculate_magnitudes(gr_complex* fft_begin, int32_t doppler_shift,
        int32_t doppler_offset);

    std::vector<gr_complex> d_code;
    uint32_t d_folding_factor;  // also referred in the paper as 'p'
    std::vector<uint32_t> d_possible_delay;
    std::vector<float> d_corr_output_f;
    std::vector<float> d_magnitude_folded;
    std::vector<gr_complex> d_signal_folded;
    std::vector<gr_complex> d_code_folded;
    float d_noise_floor_power;
    int64_t d_fs_in;
    int32_t d_samples_per_ms;
    int32_t d_samples_per_code;
    uint32_t d_doppler_resolution;
    float d_threshold;
    std::string d_satellite_str;
    uint32_t d_doppler_max;
    uint32_t d_doppler_step;
    uint32_t d_sampled_ms;
    uint32_t d_max_dwells;
    uint32_t d_well_count;
    uint32_t d_fft_size;
    uint64_t d_sample_counter;
    std::vector<std::vector<gr_complex>> d_grid_doppler_wipeoffs;
    uint32_t d_num_doppler_bins;
    std::vector<gr_complex> d_fft_codes;
    std::shared_ptr<gr::fft::fft_complex> d_fft_if;
    std::shared_ptr<gr::fft::fft_complex> d_ifft;
    Gnss_Synchro* d_gnss_synchro;
    uint32_t d_code_phase;
    float d_doppler_freq;
    float d_mag;
    std::vector<float> d_magnitude;
    float d_input_power;
    float d_test_statistics;
    bool d_bit_transition_flag;
    std::ofstream d_dump_file;
    bool d_active;
    int32_t d_state;
    bool d_dump;
    uint32_t d_channel;
    std::weak_ptr<ChannelFsm> d_channel_fsm;
    std::string d_dump_filename;
};

#endif /* GNSS_SDR_PCPS_QUICKSYNC_ACQUISITION_CC_H_ */
