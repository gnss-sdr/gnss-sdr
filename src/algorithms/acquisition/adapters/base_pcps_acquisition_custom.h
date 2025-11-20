/*!
 * \file base_ca_pcps_acquisition_custom.h
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

#ifndef GNSS_SDR_BASE_PCPS_ACQUISITION_CUSTOM_H
#define GNSS_SDR_BASE_PCPS_ACQUISITION_CUSTOM_H

#include "acquisition_impl_interface.h"
#include "channel_fsm.h"
#include "gnss_synchro.h"
#include "pcps_acquisition.h"
#include <gnuradio/blocks/stream_to_vector.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <limits>

/** \addtogroup Acquisition
 * Classes for GNSS signal acquisition
 * \{ */
/** \addtogroup Acq_adapters acquisition_adapters
 * Wrap GNU Radio acquisition blocks with an AcquisitionInterface
 * \{ */


class ConfigurationInterface;

class ThresholdComputeInterface
{
public:
    virtual float calculate_threshold(const Acq_Conf& acq_parameters) const = 0;
};

class ThresholdComputeBasic : public ThresholdComputeInterface
{
public:
    float calculate_threshold(const Acq_Conf& acq_parameters) const override;
};

class ThresholdComputeDoppler : public ThresholdComputeInterface
{
public:
    float calculate_threshold(const Acq_Conf& acq_parameters) const override;
};

class ThresholdComputeQuickSync : public ThresholdComputeInterface
{
public:
    explicit ThresholdComputeQuickSync(uint32_t folding_factor);

    float calculate_threshold(const Acq_Conf& acq_parameters) const override;

private:
    const uint32_t folding_factor_;
};

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 */
class BasePcpsAcquisitionCustom : public AcquisitionInterface
{
public:
    BasePcpsAcquisitionCustom(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams,
        double chip_rate,
        double code_length_chips,
        unsigned int ms_per_code,
        bool use_stream_to_vector,
        const ThresholdComputeInterface& threshold_compute,
        uint32_t max_sampled_ms = std::numeric_limits<uint32_t>::max());

    ~BasePcpsAcquisitionCustom() = default;

    inline std::string role() override { return role_; }

    inline size_t item_size() override { return item_size_; }

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
    void set_channel(unsigned int channel) override;

    /*!
     * \brief Set channel fsm associated to this acquisition instance
     */
    void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) override;

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

    void set_resampler_latency(uint32_t /*latency_samples*/) override {};

    /*!
     * \brief Sets local code
     */
    void set_local_code() override;


protected:
    bool is_type_gr_complex() const { return is_type_gr_complex_; }

    const Acq_Conf acq_parameters_;
    acquisition_impl_interface_sptr acquisition_cc_;
    Gnss_Synchro* gnss_synchro_;
    unsigned int channel_;
    volk_gnsssdr::vector<std::complex<float>> code_;

private:
    /*!
     * \brief Generate code
     */
    virtual void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq) = 0;

    gr::blocks::stream_to_vector::sptr stream_to_vector_;
    const std::string role_;
    const bool is_type_gr_complex_;
    const size_t item_size_;
    const bool use_stream_to_vector_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BASE_PCPS_ACQUISITION_H
