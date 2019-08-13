/*!
 * \file pcps_acquisition_fine_doppler_cc.h
 * \brief This class implements a Parallel Code Phase Search Acquisition with multi-dwells and fine Doppler estimation
 * for GPS L1 C/A signal
 *
 *  Acquisition strategy (Kay Borre book).
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
 *          <li> Javier Arribas, 2013. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_PCPS_ACQUISITION_FINE_DOPPLER_CC_H_
#define GNSS_SDR_PCPS_ACQUISITION_FINE_DOPPLER_CC_H_

#if ARMA_NO_BOUND_CHECKING
#define ARMA_NO_DEBUG 1
#endif

#include "acq_conf.h"
#include "channel_fsm.h"
#include "gnss_synchro.h"
#include <armadillo>
#include <gnuradio/block.h>
#include <gnuradio/fft/fft.h>
#include <gnuradio/gr_complex.h>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class pcps_acquisition_fine_doppler_cc;

using pcps_acquisition_fine_doppler_cc_sptr = boost::shared_ptr<pcps_acquisition_fine_doppler_cc>;

pcps_acquisition_fine_doppler_cc_sptr pcps_make_acquisition_fine_doppler_cc(const Acq_Conf& conf_);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition.
 *
 */
class pcps_acquisition_fine_doppler_cc : public gr::block
{
public:
    /*!
     * \brief Default destructor.
     */
    ~pcps_acquisition_fine_doppler_cc();

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
        d_dump_channel = d_channel;
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
    inline void set_doppler_max(unsigned int doppler_max)
    {
        d_config_doppler_max = doppler_max;
    }

    /*!
     * \brief Set Doppler steps for the grid search
     * \param doppler_step - Frequency bin of the search grid [Hz].
     */
    void set_doppler_step(unsigned int doppler_step);

    /*!
     * \brief If set to 1, ensures that acquisition starts at the
     * first available sample.
     * \param state - int=1 forces start of acquisition
     */
    void set_state(int state);

    /*!
     * \brief Obtains the next power of 2 greater or equal to the input parameter
     * \param n - Integer value to obtain the next power of 2.
     */
    unsigned int nextPowerOf2(unsigned int n);

    void dump_results(int effective_fft_size);

    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    /*!
     * \brief Parallel Code Phase Search Acquisition signal processing.
     */
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);

private:
    friend pcps_acquisition_fine_doppler_cc_sptr
    pcps_make_acquisition_fine_doppler_cc(const Acq_Conf& conf_);
    pcps_acquisition_fine_doppler_cc(const Acq_Conf& conf_);

    int compute_and_accumulate_grid(gr_vector_const_void_star& input_items);
    int estimate_Doppler();
    float estimate_input_power(gr_vector_const_void_star& input_items);
    double compute_CAF();
    void reset_grid();
    void update_carrier_wipeoff();
    bool start();
    Acq_Conf acq_parameters;
    int64_t d_fs_in;
    int d_samples_per_ms;
    int d_max_dwells;
    int d_gnuradio_forecast_samples;
    float d_threshold;
    std::string d_satellite_str;
    int d_config_doppler_max;
    int d_num_doppler_points;
    int d_doppler_step;
    unsigned int d_fft_size;
    uint64_t d_sample_counter;
    gr_complex* d_carrier;
    gr_complex* d_fft_codes;
    gr_complex* d_10_ms_buffer;
    float* d_magnitude;
    std::vector<std::vector<float>> d_grid_data;
    std::vector<std::vector<std::complex<float>>> d_grid_doppler_wipeoffs;
    std::shared_ptr<gr::fft::fft_complex> d_fft_if;
    std::shared_ptr<gr::fft::fft_complex> d_ifft;
    Gnss_Synchro* d_gnss_synchro;
    unsigned int d_code_phase;
    float d_doppler_freq;
    float d_test_statistics;
    int d_positive_acq;
    int d_state;
    bool d_active;
    int d_well_count;
    int d_n_samples_in_buffer;
    bool d_dump;
    unsigned int d_channel;
    std::weak_ptr<ChannelFsm> d_channel_fsm;
    std::string d_dump_filename;
    arma::fmat grid_;
    int64_t d_dump_number;
    unsigned int d_dump_channel;
};

#endif /* pcps_acquisition_fine_doppler_cc*/
