/*!
 * \file ad9361_manager.cc
 * \brief An Analog Devices AD9361 front-end configuration library wrapper for configure some functions via iiod link.
 * \author Javier Arribas, jarribas(at)cttc.es
 *
 * This file contains information taken from librtlsdr:
 *  https://git.osmocom.org/rtl-sdr
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */
#include "ad9361_manager.h"
#include <glog/logging.h>
#include <ad9361.h>
#include <cmath>
#include <fstream>  // for ifstream
#include <iostream>
#include <sstream>
#include <vector>

/* check return value of attr_write function */
void errchk(int v, const char *what)
{
    if (v < 0)
        {
            LOG(WARNING) << "Error " << v << " writing to channel " << what << " value may not be supported. ";
        }
}


/* write attribute: int64_t int */
void wr_ch_lli(struct iio_channel *chn, const char *what, int64_t val)
{
    errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}


/* write attribute: string */
void wr_ch_str(struct iio_channel *chn, const char *what, const char *str)
{
    errchk(iio_channel_attr_write(chn, what, str), what);
}


/* returns ad9361 phy device */
struct iio_device *get_ad9361_phy(struct iio_context *ctx)
{
    struct iio_device *dev = iio_context_find_device(ctx, "ad9361-phy");
    return dev;
}


/* finds AD9361 streaming IIO devices */
bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
    switch (d)
        {
        case TX:
            *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
            return *dev != nullptr;
        case RX:
            *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
            return *dev != nullptr;
        default:
            return false;
        }
}


/* finds AD9361 streaming IIO channels */
bool get_ad9361_stream_ch(struct iio_context *ctx __attribute__((unused)), enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
    std::stringstream name;
    name.str("");
    name << "voltage";
    name << chid;
    *chn = iio_device_find_channel(dev, name.str().c_str(), d == TX);
    if (!*chn)
        {
            name.str("");
            name << "altvoltage";
            name << chid;
            *chn = iio_device_find_channel(dev, name.str().c_str(), d == TX);
        }
    return *chn != nullptr;
}


/* finds AD9361 phy IIO configuration channel with id chid */
bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn)
{
    std::stringstream name;
    switch (d)
        {
        case RX:
            name.str("");
            name << "voltage";
            name << chid;
            *chn = iio_device_find_channel(get_ad9361_phy(ctx), name.str().c_str(), false);
            return *chn != nullptr;
            break;
        case TX:
            name.str("");
            name << "voltage";
            name << chid;
            *chn = iio_device_find_channel(get_ad9361_phy(ctx), name.str().c_str(), true);
            return *chn != nullptr;
            break;
        default:
            return false;
        }
}


/* finds AD9361 local oscillator IIO configuration channels */
bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn)
{
    switch (d)
        {
        // LO chan is always output, i.e. true
        case RX:
            *chn = iio_device_find_channel(get_ad9361_phy(ctx), "altvoltage0", true);
            return *chn != nullptr;
        case TX:
            *chn = iio_device_find_channel(get_ad9361_phy(ctx), "altvoltage1", true);
            return *chn != nullptr;
        default:
            return false;
        }
}


/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg, enum iodev type, int chid)
{
    struct iio_channel *chn = nullptr;

    // Configure phy and lo channels
    // LOG(INFO)<<"* Acquiring AD9361 phy channel"<<chid;
    std::cout << "* Acquiring AD9361 phy channel" << chid << std::endl;
    if (!get_phy_chan(ctx, type, chid, &chn))
        {
            return false;
        }
    wr_ch_str(chn, "rf_port_select", cfg->rfport);
    wr_ch_lli(chn, "rf_bandwidth", cfg->bw_hz);
    wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

    // Configure LO channel
    // LOG(INFO)<<"* Acquiring AD9361 "<<type == TX ? "TX" : "RX";
    std::cout << "* Acquiring AD9361 " << (type == TX ? "TX" : "RX") << std::endl;
    if (!get_lo_chan(ctx, type, &chn))
        {
            return false;
        }
    wr_ch_lli(chn, "frequency", cfg->lo_hz);
    return true;
}


bool config_ad9361_rx_local(uint64_t bandwidth_,
    uint64_t sample_rate_,
    uint64_t freq_,
    const std::string &rf_port_select_,
    bool rx1_enable_,
    bool rx2_enable_,
    const std::string &gain_mode_rx1_,
    const std::string &gain_mode_rx2_,
    double rf_gain_rx1_,
    double rf_gain_rx2_,
    bool quadrature_,
    bool rfdc_,
    bool bbdc_,
    std::string filter_source_,
    std::string filter_filename_,
    float Fpass_,
    float Fstop_)

{
    // RX stream config
    std::cout << "AD9361 Acquiring IIO LOCAL context\n";
    struct iio_context *ctx;
    // Streaming devices
    struct iio_device *rx;
    struct iio_channel *rx_chan1;
    struct iio_channel *rx_chan2;
    int ret;

#ifndef LIBAD9361_VERSION_GREATER_THAN_01
    if (filter_source_ == "Design")
        {
            std::cout << "Option filter_source=Design is not available in this version. Set to filter_source=Off" << std::endl;
            filter_source_ = std::string("Off");
        }
    if (Fpass_ != 0.0 or Fstop_ != 0.0)
        {
            Fpass_ = 0.0;
            Fstop_ = 0.0;
        }
#endif

    ctx = iio_create_default_context();
    if (!ctx)
        {
            std::cout << "No context\n";
            throw std::runtime_error("AD9361 IIO No context");
        }

    if (iio_context_get_devices_count(ctx) <= 0)
        {
            std::cout << "No devices\n";
            throw std::runtime_error("AD9361 IIO No devices");
        }

    struct iio_device *ad9361_phy;
    ad9361_phy = iio_context_find_device(ctx, "ad9361-phy");

    std::cout << "* Acquiring AD9361 streaming devices\n";
    if (!get_ad9361_stream_dev(ctx, RX, &rx))
        {
            std::cout << "No rx dev found\n";
            throw std::runtime_error("AD9361 IIO No rx dev found");
        }

    std::cout << "* Initializing AD9361 IIO streaming channels\n";
    if (!get_ad9361_stream_ch(ctx, RX, rx, 0, &rx_chan1))
        {
            std::cout << "RX channel 1 not found\n";
            throw std::runtime_error("RX channel 1 not found");
        }

    if (!get_ad9361_stream_ch(ctx, RX, rx, 1, &rx_chan2))
        {
            std::cout << "RX channel 2 not found\n";
            throw std::runtime_error("RX channel 2 not found");
        }

    if (filter_source_ == "Off")
        {
            struct stream_cfg rxcfg;
            rxcfg.bw_hz = bandwidth_;
            rxcfg.fs_hz = sample_rate_;
            rxcfg.lo_hz = freq_;
            rxcfg.rfport = rf_port_select_.c_str();

            if (!cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0))
                {
                    std::cout << "RX port 0 not found\n";
                    throw std::runtime_error("AD9361 IIO RX port 0 not found");
                }
        }
    else if (filter_source_ == "Auto")
        {
            ret = ad9361_set_bb_rate(ad9361_phy, sample_rate_);
            if (ret)
                {
                    throw std::runtime_error("Unable to set BB rate");
                    // set bw
                    // params.push_back("in_voltage_rf_bandwidth=" + boost::to_string(bandwidth));
                }
            // wr_ch_str(rx_chan1, "rf_port_select", rf_port_select_.c_str());
            ret = iio_device_attr_write(ad9361_phy, "in_voltage0_rf_port_select", rf_port_select_.c_str());
            if (ret)
                {
                    throw std::runtime_error("Unable to set rf_port_select");
                }
            wr_ch_lli(rx_chan1, "rf_bandwidth", bandwidth_);
            if (!get_lo_chan(ctx, RX, &rx_chan1))
                {
                    return false;
                }
            wr_ch_lli(rx_chan1, "frequency", freq_);
        }
    else if (filter_source_ == "File")
        {
            try
                {
                    if (!load_fir_filter(filter_filename_, ad9361_phy))
                        {
                            throw std::runtime_error("Unable to load filter file");
                        }
                }
            catch (const std::runtime_error &e)
                {
                    std::cout << "Exception cached when configuring the RX FIR filter: " << e.what() << std::endl;
                }
            ret = iio_device_attr_write(ad9361_phy, "in_voltage0_rf_port_select", rf_port_select_.c_str());
            if (ret)
                {
                    throw std::runtime_error("Unable to set rf_port_select");
                }
            wr_ch_lli(rx_chan1, "rf_bandwidth", bandwidth_);
            if (!get_lo_chan(ctx, RX, &rx_chan1))
                {
                    return false;
                }
            wr_ch_lli(rx_chan1, "frequency", freq_);
        }
#if LIBAD9361_VERSION_GREATER_THAN_01
    else if (filter_source_ == "Design")
        {
            ret = ad9361_set_bb_rate_custom_filter_manual(
                ad9361_phy, sample_rate_, static_cast<uint64_t>(Fpass_), static_cast<uint64_t>(Fstop_), bandwidth_, bandwidth_);
            if (ret)
                {
                    throw std::runtime_error("Unable to set BB rate");
                }
            ret = iio_device_attr_write(ad9361_phy, "in_voltage0_rf_port_select", rf_port_select_.c_str());
            if (ret)
                {
                    throw std::runtime_error("Unable to set rf_port_select");
                }
            wr_ch_lli(rx_chan1, "rf_bandwidth", bandwidth_);
            if (!get_lo_chan(ctx, RX, &rx_chan1))
                {
                    return false;
                }
            wr_ch_lli(rx_chan1, "frequency", freq_);
        }
#endif
    else
        {
            throw std::runtime_error("Unknown filter configuration");
        }

    // Filters can only be disabled after the sample rate has been set
    if (filter_source_ == "Off")
        {
            ret = ad9361_set_trx_fir_enable(ad9361_phy, false);
            if (ret)
                {
                    throw std::runtime_error("Unable to disable filters");
                }
        }

    std::cout << "* Enabling IIO streaming channels\n";
    if (rx1_enable_)
        {
            iio_channel_enable(rx_chan1);
        }
    if (rx2_enable_)
        {
            iio_channel_enable(rx_chan2);
        }
    if (!rx1_enable_ and !rx2_enable_)
        {
            std::cout << "WARNING: No Rx channels enabled.\n";
        }

    ret = iio_device_attr_write(ad9361_phy, "trx_rate_governor", "nominal");
    if (ret < 0)
        {
            std::cout << "Failed to set trx_rate_governor: " << ret << std::endl;
        }
    ret = iio_device_attr_write(ad9361_phy, "ensm_mode", "fdd");
    if (ret < 0)
        {
            std::cout << "Failed to set ensm_mode: " << ret << std::endl;
        }
    ret = iio_device_attr_write(ad9361_phy, "calib_mode", "auto");
    if (ret < 0)
        {
            std::cout << "Failed to set calib_mode: " << ret << std::endl;
        }
    ret = iio_device_attr_write_bool(ad9361_phy, "in_voltage_quadrature_tracking_en", quadrature_);
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage_quadrature_tracking_en: " << ret << std::endl;
        }
    ret = iio_device_attr_write_bool(ad9361_phy, "in_voltage_rf_dc_offset_tracking_en", rfdc_);
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage_rf_dc_offset_tracking_en: " << ret << std::endl;
        }
    ret = iio_device_attr_write_bool(ad9361_phy, "in_voltage_bb_dc_offset_tracking_en", bbdc_);
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage_bb_dc_offset_tracking_en: " << ret << std::endl;
        }
    ret = iio_device_attr_write(ad9361_phy, "in_voltage0_gain_control_mode", gain_mode_rx1_.c_str());
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage0_gain_control_mode: " << ret << std::endl;
        }
    ret = iio_device_attr_write(ad9361_phy, "in_voltage1_gain_control_mode", gain_mode_rx2_.c_str());
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage1_gain_control_mode: " << ret << std::endl;
        }
    ret = iio_device_attr_write_double(ad9361_phy, "in_voltage0_hardwaregain", rf_gain_rx1_);
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage0_hardwaregain: " << ret << std::endl;
        }
    ret = iio_device_attr_write_double(ad9361_phy, "in_voltage1_hardwaregain", rf_gain_rx2_);
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage1_hardwaregain: " << ret << std::endl;
        }

    std::cout << "End of AD9361 RX configuration.\n";
    iio_context_destroy(ctx);
    return true;
}


bool config_ad9361_rx_remote(const std::string &remote_host,
    uint64_t bandwidth_,
    uint64_t sample_rate_,
    uint64_t freq_,
    const std::string &rf_port_select_,
    bool rx1_enable_,
    bool rx2_enable_,
    const std::string &gain_mode_rx1_,
    const std::string &gain_mode_rx2_,
    double rf_gain_rx1_,
    double rf_gain_rx2_,
    bool quadrature_,
    bool rfdc_,
    bool bbdc_,
    std::string filter_source_,
    std::string filter_filename_,
    float Fpass_,
    float Fstop_)
{
    // RX stream config
    std::cout << "AD9361 Acquiring IIO REMOTE context in host " << remote_host << std::endl;
    struct iio_context *ctx;
    // Streaming devices
    struct iio_device *rx;
    struct iio_channel *rx_chan1;
    struct iio_channel *rx_chan2;

#ifndef LIBAD9361_VERSION_GREATER_THAN_01
    if (filter_source_ == "Design")
        {
            std::cout << "Option filter_source=Design is not available in this version. Set to filter_source=Off" << std::endl;
            filter_source_ = std::string("Off");
        }
    if (Fpass_ != 0.0 or Fstop_ != 0.0)
        {
            Fpass_ = 0.0;
            Fstop_ = 0.0;
        }
#endif

    ctx = iio_create_network_context(remote_host.c_str());
    if (!ctx)
        {
            std::cout << "No context\n";
            throw std::runtime_error("AD9361 IIO No context");
        }

    if (iio_context_get_devices_count(ctx) <= 0)
        {
            std::cout << "No devices\n";
            throw std::runtime_error("AD9361 IIO No devices");
        }

    std::cout << "* Acquiring AD9361 streaming devices\n";

    if (!get_ad9361_stream_dev(ctx, RX, &rx))
        {
            std::cout << "No rx dev found\n";
            throw std::runtime_error("AD9361 IIO No rx dev found");
        }

    std::cout << "* Configuring AD9361 for streaming\n";

    struct iio_device *ad9361_phy;
    ad9361_phy = iio_context_find_device(ctx, "ad9361-phy");
    int ret;

    std::cout << "* Initializing AD9361 IIO streaming channels\n";
    if (!get_ad9361_stream_ch(ctx, RX, rx, 0, &rx_chan1))
        {
            std::cout << "RX channel 1 not found\n";
            throw std::runtime_error("RX channel 1 not found");
        }

    if (!get_ad9361_stream_ch(ctx, RX, rx, 1, &rx_chan2))
        {
            std::cout << "RX channel 2 not found\n";
            throw std::runtime_error("RX channel 2 not found");
        }

    if (filter_source_ == "Off")
        {
            struct stream_cfg rxcfg;
            rxcfg.bw_hz = bandwidth_;
            rxcfg.fs_hz = sample_rate_;
            rxcfg.lo_hz = freq_;
            rxcfg.rfport = rf_port_select_.c_str();

            if (!cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0))
                {
                    std::cout << "RX port 0 not found\n";
                    throw std::runtime_error("AD9361 IIO RX port 0 not found");
                }
        }
    else if (filter_source_ == "Auto")
        {
            ret = ad9361_set_bb_rate(ad9361_phy, sample_rate_);
            if (ret)
                {
                    throw std::runtime_error("Unable to set BB rate");
                    // set bw
                    // params.push_back("in_voltage_rf_bandwidth=" + boost::to_string(bandwidth));
                }
            // wr_ch_str(rx_chan1, "rf_port_select", rf_port_select_.c_str());
            ret = iio_device_attr_write(ad9361_phy, "in_voltage0_rf_port_select", rf_port_select_.c_str());
            if (ret)
                {
                    throw std::runtime_error("Unable to set rf_port_select");
                }
            wr_ch_lli(rx_chan1, "rf_bandwidth", bandwidth_);
            if (!get_lo_chan(ctx, RX, &rx_chan1))
                {
                    return false;
                }
            wr_ch_lli(rx_chan1, "frequency", freq_);
        }
    else if (filter_source_ == "File")
        {
            try
                {
                    if (!load_fir_filter(filter_filename_, ad9361_phy))
                        {
                            throw std::runtime_error("Unable to load filter file");
                        }
                }
            catch (const std::runtime_error &e)
                {
                    std::cout << "Exception cached when configuring the RX FIR filter: " << e.what() << std::endl;
                }
            ret = iio_device_attr_write(ad9361_phy, "in_voltage0_rf_port_select", rf_port_select_.c_str());
            if (ret)
                {
                    throw std::runtime_error("Unable to set rf_port_select");
                }
            wr_ch_lli(rx_chan1, "rf_bandwidth", bandwidth_);
            if (!get_lo_chan(ctx, RX, &rx_chan1))
                {
                    return false;
                }
            wr_ch_lli(rx_chan1, "frequency", freq_);
        }
#if LIBAD9361_VERSION_GREATER_THAN_01
    else if (filter_source_ == "Design")
        {
            ret = ad9361_set_bb_rate_custom_filter_manual(
                ad9361_phy, sample_rate_, static_cast<uint64_t>(Fpass_), static_cast<uint64_t>(Fstop_), bandwidth_, bandwidth_);
            if (ret)
                {
                    throw std::runtime_error("Unable to set BB rate");
                }
            ret = iio_device_attr_write(ad9361_phy, "in_voltage0_rf_port_select", rf_port_select_.c_str());
            if (ret)
                {
                    throw std::runtime_error("Unable to set rf_port_select");
                }
            wr_ch_lli(rx_chan1, "rf_bandwidth", bandwidth_);
            if (!get_lo_chan(ctx, RX, &rx_chan1))
                {
                    return false;
                }
            wr_ch_lli(rx_chan1, "frequency", freq_);
        }
#endif
    else
        {
            throw std::runtime_error("Unknown filter configuration");
        }

    // Filters can only be disabled after the sample rate has been set
    if (filter_source_ == "Off")
        {
            ret = ad9361_set_trx_fir_enable(ad9361_phy, false);
            if (ret)
                {
                    throw std::runtime_error("Unable to disable filters");
                }
        }

    std::cout << "* Enabling IIO streaming channels\n";
    if (rx1_enable_)
        {
            iio_channel_enable(rx_chan1);
        }
    if (rx2_enable_)
        {
            iio_channel_enable(rx_chan2);
        }
    if (!rx1_enable_ and !rx2_enable_)
        {
            std::cout << "WARNING: No Rx channels enabled.\n";
        }

    ret = iio_device_attr_write(ad9361_phy, "trx_rate_governor", "nominal");
    if (ret < 0)
        {
            std::cout << "Failed to set trx_rate_governor: " << ret << std::endl;
        }
    ret = iio_device_attr_write(ad9361_phy, "ensm_mode", "fdd");
    if (ret < 0)
        {
            std::cout << "Failed to set ensm_mode: " << ret << std::endl;
        }
    ret = iio_device_attr_write(ad9361_phy, "calib_mode", "auto");
    if (ret < 0)
        {
            std::cout << "Failed to set calib_mode: " << ret << std::endl;
        }
    ret = iio_device_attr_write_bool(ad9361_phy, "in_voltage_quadrature_tracking_en", quadrature_);
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage_quadrature_tracking_en: " << ret << std::endl;
        }
    ret = iio_device_attr_write_bool(ad9361_phy, "in_voltage_rf_dc_offset_tracking_en", rfdc_);
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage_rf_dc_offset_tracking_en: " << ret << std::endl;
        }
    ret = iio_device_attr_write_bool(ad9361_phy, "in_voltage_bb_dc_offset_tracking_en", bbdc_);
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage_bb_dc_offset_tracking_en: " << ret << std::endl;
        }
    ret = iio_device_attr_write(ad9361_phy, "in_voltage0_gain_control_mode", gain_mode_rx1_.c_str());
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage0_gain_control_mode: " << ret << std::endl;
        }
    ret = iio_device_attr_write(ad9361_phy, "in_voltage1_gain_control_mode", gain_mode_rx2_.c_str());
    if (ret < 0)
        {
            std::cout << "Failed to set in_voltage1_gain_control_mode: " << ret << std::endl;
        }
    if (gain_mode_rx1_ == "manual")
        {
            ret = iio_device_attr_write_double(ad9361_phy, "in_voltage0_hardwaregain", rf_gain_rx1_);
            if (ret < 0)
                {
                    std::cout << "Failed to set in_voltage0_hardwaregain: " << ret << std::endl;
                }
        }
    if (gain_mode_rx2_ == "manual")
        {
            ret = iio_device_attr_write_double(ad9361_phy, "in_voltage1_hardwaregain", rf_gain_rx2_);
            if (ret < 0)
                {
                    std::cout << "Failed to set in_voltage1_hardwaregain: " << ret << std::endl;
                }
        }

    std::cout << "End of AD9361 RX configuration.\n";

    iio_context_destroy(ctx);
    return true;
}


bool config_ad9361_lo_local(uint64_t bandwidth_,
    uint64_t sample_rate_,
    uint64_t freq_rf_tx_hz_,
    double tx_attenuation_db_,
    int64_t freq_dds_tx_hz_,
    double scale_dds_dbfs_,
    double phase_dds_deg_)
{
    // TX stream config
    std::cout << "Start of AD9361 TX Local Oscillator DDS configuration\n";
    struct stream_cfg txcfg;
    txcfg.bw_hz = bandwidth_;
    txcfg.fs_hz = sample_rate_;
    txcfg.lo_hz = freq_rf_tx_hz_;
    txcfg.rfport = "A";

    std::cout << "AD9361 Acquiring IIO LOCAL context\n";
    struct iio_context *ctx;
    ctx = iio_create_default_context();
    if (!ctx)
        {
            std::cout << "No context\n";
            throw std::runtime_error("AD9361 IIO No context");
        }

    // find tx device
    struct iio_device *tx;

    std::cout << "* Acquiring AD9361 TX streaming devices\n";

    if (!get_ad9361_stream_dev(ctx, TX, &tx))
        {
            std::cout << "No tx dev found\n";
            throw std::runtime_error("AD9361 IIO No tx dev found");
        }

    std::cout << "* Configuring AD9361 for streaming TX\n";
    if (!cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0))
        {
            std::cout << "TX port 0 not found\n";
            throw std::runtime_error("AD9361 IIO TX port 0 not found");
        }

    // ENABLE DDS on TX1
    struct iio_device *ad9361_phy;
    ad9361_phy = iio_context_find_device(ctx, "ad9361-phy");
    int ret;
    // set output amplifier attenuation
    ret = iio_device_attr_write_double(ad9361_phy, "out_voltage0_hardwaregain", -std::abs(tx_attenuation_db_));
    if (ret < 0)
        {
            std::cout << "Failed to set out_voltage0_hardwaregain value " << -std::abs(tx_attenuation_db_) << ". Error " << ret << std::endl;
        }
    // shut down signal in TX2
    ret = iio_device_attr_write_double(ad9361_phy, "out_voltage1_hardwaregain", -89.75);
    if (ret < 0)
        {
            std::cout << "Failed to set out_voltage1_hardwaregain value -89.75 dB. Error " << ret << std::endl;
        }

    struct iio_device *dds;
    dds = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    struct iio_channel *dds_channel0_I;
    dds_channel0_I = iio_device_find_channel(dds, "TX1_I_F1", true);

    struct iio_channel *dds_channel0_Q;
    dds_channel0_Q = iio_device_find_channel(dds, "TX1_Q_F1", true);

    ret = iio_channel_attr_write_bool(dds_channel0_I, "raw", true);
    if (ret < 0)
        {
            std::cout << "Failed to toggle DDS: " << ret << std::endl;
        }

    // set frequency, scale and phase
    ret = iio_channel_attr_write_longlong(dds_channel0_I, "frequency", static_cast<int64_t>(freq_dds_tx_hz_));
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS frequency I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_longlong(dds_channel0_Q, "frequency", static_cast<int64_t>(freq_dds_tx_hz_));
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS frequency Q: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_I, "phase", phase_dds_deg_ * 1000.0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS phase I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_Q, "phase", phase_dds_deg_ * 1000.0 + 270000.0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS phase Q: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_I, "scale", pow(10, scale_dds_dbfs_ / 20.0));
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS scale I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_Q, "scale", pow(10, scale_dds_dbfs_ / 20.0));
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS scale Q: " << ret << std::endl;
        }

    // disable TX2
    ret = iio_device_attr_write_double(ad9361_phy, "out_voltage1_hardwaregain", -89.0);
    if (ret < 0)
        {
            std::cout << "Failed to set out_voltage1_hardwaregain value " << -89.0 << " error " << ret << std::endl;
        }

    struct iio_channel *dds_channel1_I;
    dds_channel1_I = iio_device_find_channel(dds, "TX2_I_F1", true);

    struct iio_channel *dds_channel1_Q;
    dds_channel1_Q = iio_device_find_channel(dds, "TX2_Q_F1", true);

    ret = iio_channel_attr_write_double(dds_channel1_I, "scale", 0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX2 DDS scale I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel1_Q, "scale", 0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX2 DDS scale Q: " << ret << std::endl;
        }

    iio_context_destroy(ctx);
    return true;
}


bool config_ad9361_lo_remote(const std::string &remote_host,
    uint64_t bandwidth_,
    uint64_t sample_rate_,
    uint64_t freq_rf_tx_hz_,
    double tx_attenuation_db_,
    int64_t freq_dds_tx_hz_,
    double scale_dds_dbfs_,
    double phase_dds_deg_)
{
    // TX stream config
    std::cout << "Start of AD9361 TX Local Oscillator DDS configuration\n";
    struct stream_cfg txcfg;
    txcfg.bw_hz = bandwidth_;
    txcfg.fs_hz = sample_rate_;
    txcfg.lo_hz = freq_rf_tx_hz_;
    txcfg.rfport = "A";

    std::cout << "AD9361 Acquiring IIO REMOTE context in host " << remote_host << std::endl;
    struct iio_context *ctx;
    ctx = iio_create_network_context(remote_host.c_str());
    if (!ctx)
        {
            std::cout << "No context\n";
            throw std::runtime_error("AD9361 IIO No context");
        }

    // find tx device
    struct iio_device *tx;

    std::cout << "* Acquiring AD9361 TX streaming devices\n";

    if (!get_ad9361_stream_dev(ctx, TX, &tx))
        {
            std::cout << "No tx dev found\n";
            throw std::runtime_error("AD9361 IIO No tx dev found");
        }

    std::cout << "* Configuring AD9361 for streaming TX\n";
    if (!cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0))
        {
            std::cout << "TX port 0 not found\n";
            throw std::runtime_error("AD9361 IIO TX port 0 not found");
        }

    // ENABLE DDS on TX1
    struct iio_device *ad9361_phy;
    ad9361_phy = iio_context_find_device(ctx, "ad9361-phy");
    int ret;
    // set output amplifier attenuation
    ret = iio_device_attr_write_double(ad9361_phy, "out_voltage0_hardwaregain", -std::abs(tx_attenuation_db_));
    if (ret < 0)
        {
            std::cout << "Failed to set out_voltage0_hardwaregain value " << -std::abs(tx_attenuation_db_) << ". Error " << ret << std::endl;
        }

    // shut down signal in TX2
    ret = iio_device_attr_write_double(ad9361_phy, "out_voltage1_hardwaregain", -89.75);
    if (ret < 0)
        {
            std::cout << "Failed to set out_voltage1_hardwaregain value -89.75 dB. Error " << ret << std::endl;
        }
    struct iio_device *dds;
    dds = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    struct iio_channel *dds_channel0_I;
    dds_channel0_I = iio_device_find_channel(dds, "TX1_I_F1", true);

    struct iio_channel *dds_channel0_Q;
    dds_channel0_Q = iio_device_find_channel(dds, "TX1_Q_F1", true);

    ret = iio_channel_attr_write_bool(dds_channel0_I, "raw", true);
    if (ret < 0)
        {
            std::cout << "Failed to toggle DDS: " << ret << std::endl;
        }

    // set frequency, scale and phase
    ret = iio_channel_attr_write_longlong(dds_channel0_I, "frequency", static_cast<int64_t>(freq_dds_tx_hz_));
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS frequency I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_longlong(dds_channel0_Q, "frequency", static_cast<int64_t>(freq_dds_tx_hz_));
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS frequency Q: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_I, "phase", phase_dds_deg_ * 1000.0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS phase I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_Q, "phase", phase_dds_deg_ * 1000.0 + 270000.0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS phase Q: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_I, "scale", pow(10, scale_dds_dbfs_ / 20.0));
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS scale I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_Q, "scale", pow(10, scale_dds_dbfs_ / 20.0));
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS scale Q: " << ret << std::endl;
        }

    // disable TX2
    ret = iio_device_attr_write_double(ad9361_phy, "out_voltage1_hardwaregain", -89.0);
    if (ret < 0)
        {
            std::cout << "Failed to set out_voltage1_hardwaregain value " << -89.0 << " error " << ret << std::endl;
        }

    struct iio_channel *dds_channel1_I;
    dds_channel1_I = iio_device_find_channel(dds, "TX2_I_F1", true);

    struct iio_channel *dds_channel1_Q;
    dds_channel1_Q = iio_device_find_channel(dds, "TX2_Q_F1", true);

    ret = iio_channel_attr_write_double(dds_channel1_I, "scale", 0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX2 DDS scale I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel1_Q, "scale", 0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX2 DDS scale Q: " << ret << std::endl;
        }

    iio_context_destroy(ctx);
    return true;
}


bool ad9361_disable_lo_remote(const std::string &remote_host)
{
    std::cout << "AD9361 Acquiring IIO REMOTE context in host " << remote_host << std::endl;
    struct iio_context *ctx;
    ctx = iio_create_network_context(remote_host.c_str());
    if (!ctx)
        {
            std::cout << "No context\n";
            throw std::runtime_error("AD9361 IIO No context");
        }
    struct iio_device *dds;
    dds = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    struct iio_channel *dds_channel0_I;
    dds_channel0_I = iio_device_find_channel(dds, "TX1_I_F1", true);

    struct iio_channel *dds_channel0_Q;
    dds_channel0_Q = iio_device_find_channel(dds, "TX1_Q_F1", true);
    int ret;
    ret = iio_channel_attr_write_bool(dds_channel0_I, "raw", false);
    if (ret < 0)
        {
            std::cout << "Failed to toggle DDS: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_I, "scale", 0.0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS scale I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_Q, "scale", 0.0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS scale Q: " << ret << std::endl;
        }

    iio_context_destroy(ctx);

    return true;
}


bool ad9361_disable_lo_local()
{
    std::cout << "AD9361 Acquiring IIO LOCAL context" << std::endl;
    struct iio_context *ctx;
    ctx = iio_create_default_context();
    if (!ctx)
        {
            std::cout << "No context\n";
            throw std::runtime_error("AD9361 IIO No context");
        }
    struct iio_device *dds;
    dds = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    struct iio_channel *dds_channel0_I;
    dds_channel0_I = iio_device_find_channel(dds, "TX1_I_F1", true);

    struct iio_channel *dds_channel0_Q;
    dds_channel0_Q = iio_device_find_channel(dds, "TX1_Q_F1", true);
    int ret;
    ret = iio_channel_attr_write_bool(dds_channel0_I, "raw", false);
    if (ret < 0)
        {
            std::cout << "Failed to toggle DDS: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_I, "scale", 0.0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS scale I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_Q, "scale", 0.0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS scale Q: " << ret << std::endl;
        }

    iio_context_destroy(ctx);

    return true;
}


bool load_fir_filter(
    std::string &filter, struct iio_device *phy)
{
    if (filter.empty() || !iio_device_find_attr(phy, "filter_fir_config"))
        {
            return false;
        }

    std::ifstream ifs(filter.c_str(), std::ifstream::binary);
    if (!ifs)
        {
            return false;
        }

    // Here, we verify that the filter file contains data for both RX+TX.
    {
        char buf[256];
        do
            {
                ifs.getline(buf, sizeof(buf));
            }
        while (!(buf[0] == '-' || (buf[0] >= '0' && buf[0] <= '9')));

        std::string line(buf);
        if (line.find(',') == std::string::npos)
            {
                throw std::runtime_error("Incompatible filter file");
            }
    }

    ifs.seekg(0, ifs.end);
    int length = ifs.tellg();
    ifs.seekg(0, ifs.beg);

    std::vector<char> buffer(length);

    ifs.read(buffer.data(), length);
    ifs.close();

    int ret = iio_device_attr_write_raw(phy,
        "filter_fir_config", buffer.data(), length);

    return ret > 0;
}


bool disable_ad9361_rx_local()
{
    struct iio_context *ctx;
    struct iio_device *rx;
    struct iio_channel *rx_chan1;
    struct iio_channel *rx_chan2;

    ctx = iio_create_default_context();
    if (!ctx)
        {
            std::cout << "No default context available when disabling RX channels\n";
            return false;
        }

    if (iio_context_get_devices_count(ctx) <= 0)
        {
            std::cout << "No devices available when disabling RX channels\n";
            return false;
        }

    if (!get_ad9361_stream_dev(ctx, RX, &rx))
        {
            std::cout << "No rx streams found when disabling RX channels\n";
            return false;
        }

    if (!get_ad9361_stream_ch(ctx, RX, rx, 0, &rx_chan1))
        {
            std::cout << "RX channel 1 not found when disabling RX channels\n";
            return false;
        }

    if (!get_ad9361_stream_ch(ctx, RX, rx, 1, &rx_chan2))
        {
            std::cout << "RX channel 2 not found when disabling RX channels\n";
            return false;
        }

    iio_channel_disable(rx_chan1);
    iio_channel_disable(rx_chan2);
    iio_context_destroy(ctx);
    return true;
}


bool disable_ad9361_rx_remote(const std::string &remote_host)
{
    struct iio_context *ctx;
    struct iio_device *rx;
    struct iio_channel *rx_chan1;
    struct iio_channel *rx_chan2;

    ctx = iio_create_network_context(remote_host.c_str());
    if (!ctx)
        {
            std::cout << "No context available at " << remote_host << "when disabling RX channels\n";
            return false;
        }

    if (!get_ad9361_stream_dev(ctx, RX, &rx))
        {
            std::cout << "No rx streams found at " << remote_host << " when disabling RX channels\n";
            return false;
        }

    if (!get_ad9361_stream_ch(ctx, RX, rx, 0, &rx_chan1))
        {
            std::cout << "RX channel 1 not found at " << remote_host << " when disabling RX channels\n";
            return false;
        }

    if (!get_ad9361_stream_ch(ctx, RX, rx, 1, &rx_chan2))
        {
            std::cout << "RX channel 2 not found at " << remote_host << " when disabling RX channels\n";
            return false;
        }
    iio_channel_disable(rx_chan1);
    iio_channel_disable(rx_chan2);
    iio_context_destroy(ctx);
    return true;
}
