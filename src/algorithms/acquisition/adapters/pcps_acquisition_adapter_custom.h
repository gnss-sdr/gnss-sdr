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

#ifndef GNSS_SDR_PCPS_ACQUISITION_ADAPTER_CUSTOM_H
#define GNSS_SDR_PCPS_ACQUISITION_ADAPTER_CUSTOM_H

#include "acquisition_impl_interface.h"
#include "channel_fsm.h"
#include "gnss_synchro.h"
#include "pcps_acquisition.h"
#include "signal_flag.h"
#include <gnuradio/blocks/stream_to_vector.h>
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
class PcpsAcquisitionAdapterCustom : public AcquisitionInterface
{
public:
    PcpsAcquisitionAdapterCustom(const ConfigurationInterface* configuration,
        const std::string& role,
        const std::string& implementation,
        unsigned int in_streams,
        unsigned int out_streams,
        signal_flag sig_flag);

    ~PcpsAcquisitionAdapterCustom() = default;

    inline std::string role() override { return role_; }

    inline std::string implementation() override { return implementation_; }

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
     * \brief Set Doppler center for the grid search
     */
    void set_assistance(int doppler_center, int32_t assist_level) override;

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


private:
    /*!
     * \brief Generate code
     */
    void code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq);

    const Acq_Conf acq_parameters_;
    acquisition_impl_interface_sptr acquisition_cc_;
    Gnss_Synchro* gnss_synchro_;
    unsigned int channel_;
    volk_gnsssdr::vector<std::complex<float>> code_;
    gr::blocks::stream_to_vector::sptr stream_to_vector_;
    const signal_flag sig_flag_;
    const std::string role_;
    const std::string implementation_;
    const bool is_type_gr_complex_;
    const size_t item_size_;
    const bool use_stream_to_vector_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_PCPS_ACQUISITION_ADAPTER_CUSTOM_H
