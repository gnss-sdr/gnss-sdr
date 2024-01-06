/*!
 * \file acq_conf_fpga.h
 * \brief Class that contains all the configuration parameters for generic
 * acquisition block based on the PCPS algorithm running in the FPGA.
 * \author Marc Majoral, 2022. mmajoral(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ACQ_CONF_FPGA_H
#define GNSS_SDR_ACQ_CONF_FPGA_H

#include "configuration_interface.h"
#include <gnuradio/gr_complex.h>
#include <cstdint>
#include <string>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup acquisition_libs acquisition_libs
 * Library with utilities for GNSS signal acquisition
 * \{ */


class Acq_Conf_Fpga
{
public:
    Acq_Conf_Fpga() = default;

    void SetFromConfiguration(const ConfigurationInterface *configuration, const std::string &role, uint32_t sel_queue_fpga, uint32_t blk_exp, uint32_t downsampling_factor_default, double chip_rate, double code_length_chips);

    /* PCPS Acquisition configuration */
    std::string device_name = "uio0";
    uint32_t *all_fft_codes = NULL;  // pointer to memory that contains all the code ffts
    int64_t fs_in{4000000LL};

    float doppler_step{250.0};
    float doppler_step2{125.0};

    uint32_t num_doppler_bins_step2{4U};

    int32_t doppler_max{5000};

    uint32_t select_queue_Fpga{0U};
    uint32_t downsampling_factor{4U};
    uint32_t total_block_exp{13U};
    uint32_t excludelimit{5U};
    uint32_t max_num_acqs{2U};
    uint32_t samples_per_code{1U};
    uint32_t code_length{16000U};
    bool make_2_steps{false};
    bool repeat_satellite{false};

private:
    const std::string acquisition_device_name = "acquisition_S00_AXI";  // UIO device name
};


/** \} */
/** \} */
#endif  // GNSS_SDR_ACQ_CONF_FPGA_H
