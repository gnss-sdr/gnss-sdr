/*!
 * \file max2771_evkit_signal_source_fpga.h
 * \brief Signal source for the MAX2771EVKIT evaluation board connected directly
 * to FPGA accelerators.
 * This source implements only the MAX2771 control. It is NOT compatible with
 * conventional SDR acquisition and tracking blocks.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_MAX2771_EVKIT_SIGNAL_SOURCE_FPGA_H
#define GNSS_SDR_MAX2771_EVKIT_SIGNAL_SOURCE_FPGA_H

#include "command_event.h"
#include "concurrent_queue.h"
#include "fpga_buffer_monitor.h"
#include "fpga_spidev.h"
#include "gnss_block_interface.h"
#include "signal_source_base.h"
#include <pmt/pmt.h>  // for pmt::pmt_t
#include <cstdint>    // for fixed-width integer types
#include <memory>     // for smart pointers
#include <mutex>      // for mutex
#include <string>     // for strings
#include <thread>     // for threads
#include <vector>     // for std::vector


/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */


class ConfigurationInterface;

class MAX2771EVKITSignalSourceFPGA : public SignalSourceBase
{
public:
    MAX2771EVKITSignalSourceFPGA(const ConfigurationInterface *configuration,
        const std::string &role, unsigned int in_stream,
        unsigned int out_stream, Concurrent_Queue<pmt::pmt_t> *queue);

    ~MAX2771EVKITSignalSourceFPGA();

    std::vector<uint32_t> setup_regs(void);


    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

private:
    const std::string default_dump_filename = std::string("FPGA_buffer_monitor_dump.dat");
    const uint64_t default_bandwidth = 2500000;
    const uint32_t default_filter_order = 5;
    const uint64_t default_sampling_rate = 4092000;
    const uint32_t default_PGA_gain_value = 0x3A;  // default PGA gain when AGC is off
    // max PGA gain value
    const uint32_t max_PGA_gain_value = 0x3F;
    // check buffer overflow and perform buffer monitoring every 1s by default
    const uint32_t buffer_monitor_period_ms = 1000;
    // buffer overflow and buffer monitoring initial delay
    const uint32_t buffer_monitoring_initial_delay_ms = 2000;
    // MAX2771 number of configuration registers
    const uint32_t MAX2771_NUM_REGS = 11;
    // MAX2771 configuration register fields
    const uint32_t NUM_FREQ_BANDS = 1;
    const uint32_t IDLE = 0x0;             // Idle mode disabled
    const uint32_t MIXPOLE = 0x0;          // set the passive filter pole at mixer output at 13 MHz.
    const uint32_t MIXERMODE = 0x0;        // L1 band enabled
    const uint32_t FCEN = 0x58;            // Center frequency not used when in low-pass filter mode. Set to default value.
    const uint32_t FCENX = 0x0;            // POlyphase filter selection set to Lowpass filter
    const uint32_t ANAIMON = 0x0;          // analog monitor disabled
    const uint32_t IQEN = 0x1;             // I and Q channels enable
    const uint32_t GAINREF = 0xAA;         // AGC Gain ref
    const uint32_t SPI_SDIO_CONFIG = 0x0;  // SPI SDIO config when tri-stated: nothing applied
    const uint32_t FORMAT = 0x1;           // sign and magnitude
    const uint32_t BITS = 0x2;             // number of bits in the ADC = 2
    const uint32_t DRVCFG = 0x0;           // output driver configuration = CMOS Logic
    const uint32_t DIEID = 0x0;            // identifies version of IC
    const uint32_t HILOADEN = 0x0;         // disable output driver for high loads
    const uint32_t FHIPEN = 0x1;           // enable highpass coupling between filter and PGA.
    const uint32_t PGAIEN = 0x1;           // I-Channel PGA Enable
    const uint32_t PGAQEN = 0x1;           // Q-Channel PGA Enable
    const uint32_t STRMEN = 0x0;           // disable DSP interface for serial streaming of data
    const uint32_t STRMSTART = 0x0;        // the rising edge of this bit enables data streaming to the output, clock, data, sync and frame sync outputs.
    const uint32_t STRMSTOP = 0x0;         // the rising edge of this bit disables data streaming to the output, clock,  data sync and frame sync outputs.
    const uint32_t STRMBITS = 0x1;         // number of bits to be streamed: I MSB, I LSB
    const uint32_t STAMPEN = 0x1;          // enable frame number insertion
    const uint32_t TIMESYNCEN = 0x1;       // enable the output of the time sync pulses at all times when streaming is enabled.
    const uint32_t DATASYNCEN = 0x0;       // disable the sync pulses at the DATASYNC output
    const uint32_t STRMRST = 0x0;          // counter reset not active
    const uint32_t LOBAND = 0x0;           // L1 band
    const uint32_t REFOUTEN = 0x1;         // Output clock buffer enable
    const uint32_t IXTAL = 0x1;            // XTAL osscillator/buffer set to normal current
    const uint32_t ICP = 0x0;              // charge pump current selection set to 0.5 mA
    const uint32_t INT_PLL = 0x1;          // PLL mode set to integer-N PLL
    const uint32_t PWRSAV = 0x0;           // PLL power save mode disabled
    const uint32_t RDIV = 0x10;            // Set the PLL reference division ratio such that the L1 band is tuned to 1575.42 Mhz
    const uint32_t FDIV = 0x80000;         // PLL fractional division ratio not used. Set to default value
    const uint32_t EXTADCCLK = 0x0;        // use internally generated clock
    const uint32_t REFCLK_L_CNT = 0x100;   // set the L counter of the reference clock configuration to its default value
    const uint32_t REFCLK_M_CNT = 0x61B;   // set the M counter of the reference clock configuration to its default value
    const uint32_t FCLKIN = 0x0;           // fractional clock divider set to default value
    const uint32_t ADCCLK = 0x0;           // ADC clock selection set to reference clock divider/multiplier
    const uint32_t MODE = 0x0;             // DSP interface mode selection
    const uint32_t ADCCLK_L_CNT = 0x100;   // set the L counter of the ADC clock configuration to its default value
    const uint32_t ADCCLK_M_CNT = 0x61B;   // set the M counter of the ADC clock configuration to its default value
    const uint32_t PRE_FRACDIV_SEL = 0x0;  // bypass fractional clock divider
    const uint32_t CLKOUT_SEL = 0x1;       // CLKOUT selection set to ADC clock
    // MAX2771 configuration register registers
    const uint32_t TEST_MODE_1_REG_VAL = 0x01E0F401;  // reserved
    const uint32_t TEST_MODE_2_REG_VAL = 0x00000002;

    bool configure(std::vector<uint32_t> register_values);
    void run_buffer_monitor_process();


    std::thread thread_buffer_monitor;

    std::shared_ptr<Fpga_buffer_monitor> buffer_monitor_fpga;
    std::shared_ptr<Fpga_spidev> spidev_fpga;

    std::mutex buffer_monitor_mutex;

    uint64_t freq_;  // frequency of local oscillator
    uint64_t sample_rate_;

    uint32_t in_stream_;
    uint32_t out_stream_;
    uint32_t bandwidth_;     // 2500000, 4200000, 8700000, 16400000, 23400000, 36000000
    uint32_t filter_order_;  // 3, 5
    uint32_t gain_in_;       // 0 to 0x3F

    size_t item_size_;  // 1

    bool chipen_;          // chip enable
    bool if_filter_gain_;  // true, false
    bool LNA_active_;      // true, false
    bool enable_agc_;      // true, false
    bool enable_ovf_check_buffer_monitor_active_;
    bool dump_;
    bool rf_shutdown_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_MAX2771_EVKIT_SIGNAL_SOURCE_FPGA_H
