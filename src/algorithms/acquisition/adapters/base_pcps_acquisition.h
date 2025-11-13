/*!
 * \file base_ca_pcps_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface
 * \authors <ul>
 *          <li> Mathieu Favreau, 2025. favreau.mathieu(at)hotmail.com
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_BASE_PCPS_ACQUISITION_H
#define GNSS_SDR_BASE_PCPS_ACQUISITION_H

#include "acq_conf.h"
#include "channel_fsm.h"
#include "complex_byte_to_float_x2.h"
#include "gnss_synchro.h"
#include "pcps_acquisition.h"
#include <gnuradio/blocks/float_to_complex.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>

/** \addtogroup Acquisition
 * Classes for GNSS signal acquisition
 * \{ */
/** \addtogroup Acq_adapters acquisition_adapters
 * Wrap GNU Radio acquisition blocks with an AcquisitionInterface
 * \{ */


class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 */
class BasePcpsAcquisition : public AcquisitionInterface
{
public:
    BasePcpsAcquisition(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams,
        double chip_rate,
        double opt_freq,
        double code_length_chips,
        uint32_t ms_per_code);

    ~BasePcpsAcquisition() = default;

    inline std::string role() override
    {
        return role_;
    }

    inline size_t item_size() override
    {
        return acq_parameters_.it_size;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and
     *  tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    /*!
     * \brief Set acquisition channel unique ID
     */
    inline void set_channel(unsigned int channel) override
    {
        acquisition_->set_channel(channel);
    }

    /*!
     * \brief Set channel fsm associated to this acquisition instance
     */
    inline void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) override
    {
        acquisition_->set_channel_fsm(std::move(channel_fsm));
    }

    /*!
     * \brief Set statistics threshold of PCPS algorithm
     */
    void set_threshold(float threshold) override;

    /*!
     * \brief Set Doppler center for the grid search
     */
    void set_doppler_center(int doppler_center) override;

    /*!
     * \brief Initializes acquisition algorithm.
     */
    void init() override;

    /*!
     * \brief Returns the maximum peak of grid search
     */
    signed int mag() override;

    /*!
     * \brief Restart acquisition algorithm
     */
    void reset() override;

    /*!
     * \brief Stop running acquisition
     */
    void stop_acquisition() override;

    /*!
     * \brief Sets the resampler latency to account it in the acquisition code delay estimation
     */
    void set_resampler_latency(uint32_t latency_samples) override;

    /*!
     * \brief Sets local code
     */
    void set_local_code() override;

private:
    /*!
     * \brief Generate code
     */
    virtual void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) = 0;

    const Acq_Conf acq_parameters_;
    gr::blocks::float_to_complex::sptr float_to_complex_;
    complex_byte_to_float_x2_sptr cbyte_to_float_x2_;
    Gnss_Synchro* gnss_synchro_;
    const std::string role_;
    const unsigned int vector_length_;
    const unsigned int code_length_;
    volk_gnsssdr::vector<std::complex<float>> code_;
    pcps_acquisition_sptr acquisition_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BASE_PCPS_ACQUISITION_H
