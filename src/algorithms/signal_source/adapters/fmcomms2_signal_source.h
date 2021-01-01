/*!
 * \file fmcomms2_signal_source.h
 * \brief Interface to use SDR hardware based in FMCOMMS2 driver from analog
 * devices, for example FMCOMMS4 and ADALM-PLUTO (PlutoSdr)
 * \author Rodrigo Muñoz, 2017. rmunozl(at)inacap.cl, rodrigo.munoz(at)proteinlab.cl
 *
 *
 * This class represent a fmcomms2 signal source. It use the gr_iio block
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

#ifndef GNSS_SDR_FMCOMMS2_SIGNAL_SOURCE_H
#define GNSS_SDR_FMCOMMS2_SIGNAL_SOURCE_H

#include "gnss_block_interface.h"
#include <gnuradio/blocks/file_sink.h>
#if GRIIO_INCLUDE_HAS_GNURADIO
#include <gnuradio/iio/fmcomms2_source.h>
#else
#include <iio/fmcomms2_source.h>
#endif
#include "concurrent_queue.h"
#include <pmt/pmt.h>
#include <cstdint>
#include <string>


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

class Fmcomms2SignalSource : public GNSSBlockInterface
{
public:
    Fmcomms2SignalSource(const ConfigurationInterface* configuration,
        const std::string& role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t>* queue);

    ~Fmcomms2SignalSource();

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "Fmcomms2_Signal_Source"
     */
    inline std::string implementation() override
    {
        return "Fmcomms2_Signal_Source";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    gr::iio::fmcomms2_source_f32c::sptr fmcomms2_source_f32c_;
    gnss_shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;

    std::string role_;
    std::string item_type_;
    std::string dump_filename_;

    // Front-end settings
    std::string uri_;  // device direction
    std::string gain_mode_rx1_;
    std::string gain_mode_rx2_;
    std::string rf_port_select_;
    std::string filter_file_;
    std::string filter_source_;
    std::string filter_filename_;

    int64_t samples_;
    size_t item_size_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    double rf_gain_rx1_;
    double rf_gain_rx2_;
    uint64_t freq_;  // frequency of local oscilator
    uint64_t sample_rate_;
    uint64_t bandwidth_;
    uint64_t buffer_size_;  // reception buffer
    float Fpass_;
    float Fstop_;
    int RF_channels_;

    // DDS configuration for LO generation for external mixer
    double scale_dds_dbfs_;
    double phase_dds_deg_;
    double tx_attenuation_db_;
    uint64_t freq_rf_tx_hz_;
    uint64_t freq_dds_tx_hz_;
    uint64_t tx_bandwidth_;
    bool enable_dds_lo_;

    bool rx1_en_;
    bool rx2_en_;
    bool quadrature_;
    bool rf_dc_;
    bool bb_dc_;
    bool filter_auto_;
    bool rf_shutdown_;
    bool dump_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_FMCOMMS2_SIGNAL_SOURCE_H
