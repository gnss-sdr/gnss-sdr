/*!
 * \file galileo_pcps_8ms_acquisition_cc.h
 * \brief This class implements a Parallel Code Phase Search Acquisition for
 * Galileo E1 signals with coherent integration time = 8 ms (two codes)
 * \author Marc Molina, 2013. marc.molina.pena(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_PCPS_8MS_ACQUISITION_CC_H
#define GNSS_SDR_PCPS_8MS_ACQUISITION_CC_H

#include "acq_conf.h"
#include "acquisition_impl_interface.h"
#include "channel_fsm.h"
#include "gnss_sdr_fft.h"
#include "gnss_synchro.h"
#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup Acq_gnuradio_blocks
 * \{ */


class galileo_pcps_8ms_acquisition_cc;

using galileo_pcps_8ms_acquisition_cc_sptr = gnss_shared_ptr<galileo_pcps_8ms_acquisition_cc>;

galileo_pcps_8ms_acquisition_cc_sptr
galileo_pcps_8ms_make_acquisition_cc(const Acq_Conf& conf);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition for
 * Galileo E1 signals with coherent integration time = 8 ms (two codes)
 */
class galileo_pcps_8ms_acquisition_cc : public acquisition_impl_interface
{
public:
    /*!
     * \brief Default destructor.
     */
    ~galileo_pcps_8ms_acquisition_cc();

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to exchange synchronization data between acquisition and tracking blocks.
     * \param p_gnss_synchro Satellite information shared by the processing blocks.
     */
    inline void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override
    {
        d_gnss_synchro = p_gnss_synchro;
    }

    /*!
     * \brief Returns the maximum peak of grid search.
     */
    inline uint32_t mag() const override
    {
        return d_mag;
    }

    /*!
     * \brief Initializes acquisition algorithm.
     */
    void init() override;

    /*!
     * \brief Sets local code for PCPS acquisition algorithm.
     * \param code - Pointer to the PRN code.
     */
    void set_local_code(std::complex<float>* code) override;

    /*!
     * \brief Starts acquisition algorithm, turning from standby mode to
     * active mode
     * \param active - bool that activates/deactivates the block.
     */
    inline void set_active(bool active) override
    {
        if (!active)
            {
                d_state = 0;
            }

        d_active = active;
    }

    /*!
     * \brief Set acquisition channel unique ID
     * \param channel - receiver channel.
     */
    inline void set_channel(uint32_t channel) override
    {
        d_channel = channel;
    }

    /*!
     * \brief Set channel fsm associated to this acquisition instance
     */
    inline void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) override
    {
        d_channel_fsm = std::move(channel_fsm);
    }

    /*!
     * \brief Set statistics threshold of PCPS algorithm.
     * \param threshold - Threshold for signal detection (check \ref Navitec2012,
     * Algorithm 1, for a definition of this threshold).
     */
    inline void set_threshold(float threshold) override
    {
        d_threshold = threshold;
    }

    /*!
     * \brief Parallel Code Phase Search Acquisition signal processing.
     */
    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items) override;

private:
    friend galileo_pcps_8ms_acquisition_cc_sptr
    galileo_pcps_8ms_make_acquisition_cc(const Acq_Conf& conf);

    explicit galileo_pcps_8ms_acquisition_cc(const Acq_Conf& conf);

    void calculate_magnitudes(
        gr_complex* fft_begin,
        int32_t doppler_shift,
        int32_t doppler_offset);

    std::weak_ptr<ChannelFsm> d_channel_fsm;
    std::unique_ptr<gnss_fft_complex_fwd> d_fft_if;
    std::unique_ptr<gnss_fft_complex_rev> d_ifft;

    std::vector<std::vector<gr_complex>> d_grid_doppler_wipeoffs;
    std::vector<gr_complex> d_fft_code_A;
    std::vector<gr_complex> d_fft_code_B;
    std::vector<float> d_magnitude;

    std::string d_satellite_str;
    const Acq_Conf d_acq_params;
    std::ofstream d_dump_file;

    Gnss_Synchro* d_gnss_synchro;

    uint64_t d_sample_counter;

    float d_threshold;
    float d_mag;
    float d_input_power;
    float d_test_statistics;
    int32_t d_state;
    uint32_t d_channel;
    uint32_t d_well_count;
    const uint32_t d_fft_size;
    uint32_t d_num_doppler_bins;
    uint32_t d_code_phase;

    bool d_active;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PCPS_8MS_ACQUISITION_CC_H
