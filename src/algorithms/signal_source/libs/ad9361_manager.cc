/*!
 * \file ad9361_manager.cc
 * \brief An Analog Devices AD9361 front-end configuration library wrapper for configure some functions via iiod link.
 * \author Javier Arribas, jarribas(at)cttc.es
 *
 * This file contains information taken from librtlsdr:
 *  http://git.osmocom.org/rtl-sdr/
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */
#include "ad9361_manager.h"
#include <glog/logging.h>
#include <math.h>
#include <iostream>
#include <sstream>


/* check return value of attr_write function */
void errchk(int v, const char *what)
{
    if (v < 0)
        {
            LOG(WARNING) << "Error " << v << " writing to channel " << what << " value may not be supported. ";
        }
}


/* write attribute: long long int */
void wr_ch_lli(struct iio_channel *chn, const char *what, long long val)
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
            return *dev != NULL;
        case RX:
            *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
            return *dev != NULL;
        default:
            return false;
        }
}


/* finds AD9361 streaming IIO channels */
bool get_ad9361_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
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
    return *chn != NULL;
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
            return *chn != NULL;
            break;
        case TX:
            name.str("");
            name << "voltage";
            name << chid;
            *chn = iio_device_find_channel(get_ad9361_phy(ctx), name.str().c_str(), true);
            return *chn != NULL;
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
            return *chn != NULL;
        case TX:
            *chn = iio_device_find_channel(get_ad9361_phy(ctx), "altvoltage1", true);
            return *chn != NULL;
        default:
            return false;
        }
}


/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg, enum iodev type, int chid)
{
    struct iio_channel *chn = NULL;

    // Configure phy and lo channels
    //LOG(INFO)<<"* Acquiring AD9361 phy channel"<<chid;
    std::cout << "* Acquiring AD9361 phy channel" << chid << std::endl;
    if (!get_phy_chan(ctx, type, chid, &chn))
        {
            return false;
        }
    wr_ch_str(chn, "rf_port_select", cfg->rfport);
    wr_ch_lli(chn, "rf_bandwidth", cfg->bw_hz);
    wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

    // Configure LO channel
    //LOG(INFO)<<"* Acquiring AD9361 "<<type == TX ? "TX" : "RX";
    std::cout << "* Acquiring AD9361 " << (type == TX ? "TX" : "RX") << std::endl;
    if (!get_lo_chan(ctx, type, &chn))
        {
            return false;
        }
    wr_ch_lli(chn, "frequency", cfg->lo_hz);
    return true;
}


bool config_ad9361_rx_local(unsigned long bandwidth_,
    unsigned long sample_rate_,
    unsigned long freq_,
    std::string rf_port_select_,
    std::string gain_mode_rx1_,
    std::string gain_mode_rx2_,
    double rf_gain_rx1_,
    double rf_gain_rx2_)

{
    // RX stream config
    // Stream configurations
    struct stream_cfg rxcfg;
    rxcfg.bw_hz = bandwidth_;                // 2 MHz rf bandwidth
    rxcfg.fs_hz = sample_rate_;              // 2.5 MS/s rx sample rate
    rxcfg.lo_hz = freq_;                     // 2.5 GHz rf frequency
    rxcfg.rfport = rf_port_select_.c_str();  // port A (select for rf freq.)

    std::cout << "AD9361 Acquiring IIO LOCAL context\n";
    struct iio_context *ctx;
    // Streaming devices
    struct iio_device *rx;
    struct iio_channel *rx0_i;
    struct iio_channel *rx0_q;

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

    std::cout << "* Acquiring AD9361 streaming devices\n";

    if (!get_ad9361_stream_dev(ctx, RX, &rx))
        {
            std::cout << "No rx dev found\n";
            throw std::runtime_error("AD9361 IIO No rx dev found");
        };

    std::cout << "* Configuring AD9361 for streaming\n";
    if (!cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0))
        {
            std::cout << "RX port 0 not found\n";
            throw std::runtime_error("AD9361 IIO RX port 0 not found");
        }

    std::cout << "* Initializing AD9361 IIO streaming channels\n";
    if (!get_ad9361_stream_ch(ctx, RX, rx, 0, &rx0_i))
        {
            std::cout << "RX chan i not found\n";
            throw std::runtime_error("RX chan i not found");
        }

    if (!get_ad9361_stream_ch(ctx, RX, rx, 1, &rx0_q))
        {
            std::cout << "RX chan q not found\n";
            throw std::runtime_error("RX chan q not found");
        }

    std::cout << "* Enabling IIO streaming channels\n";
    iio_channel_enable(rx0_i);
    iio_channel_enable(rx0_q);

    struct iio_device *ad9361_phy;
    ad9361_phy = iio_context_find_device(ctx, "ad9361-phy");
    int ret;
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


bool config_ad9361_rx_remote(std::string remote_host,
    unsigned long bandwidth_,
    unsigned long sample_rate_,
    unsigned long freq_,
    std::string rf_port_select_,
    std::string gain_mode_rx1_,
    std::string gain_mode_rx2_,
    double rf_gain_rx1_,
    double rf_gain_rx2_)
{
    // RX stream config
    // Stream configurations
    struct stream_cfg rxcfg;
    rxcfg.bw_hz = bandwidth_;                // 2 MHz rf bandwidth
    rxcfg.fs_hz = sample_rate_;              // 2.5 MS/s rx sample rate
    rxcfg.lo_hz = freq_;                     // 2.5 GHz rf frequency
    rxcfg.rfport = rf_port_select_.c_str();  // port A (select for rf freq.)


    std::cout << "AD9361 Acquiring IIO REMOTE context in host " << remote_host << std::endl;
    struct iio_context *ctx;
    // Streaming devices
    struct iio_device *rx;
    struct iio_channel *rx0_i;
    struct iio_channel *rx0_q;

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
        };

    std::cout << "* Configuring AD9361 for streaming\n";
    if (!cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0))
        {
            std::cout << "RX port 0 not found\n";
            throw std::runtime_error("AD9361 IIO RX port 0 not found");
        }

    std::cout << "* Initializing AD9361 IIO streaming channels\n";
    if (!get_ad9361_stream_ch(ctx, RX, rx, 0, &rx0_i))
        {
            std::cout << "RX chan i not found\n";
            throw std::runtime_error("RX chan i not found");
        }

    if (!get_ad9361_stream_ch(ctx, RX, rx, 1, &rx0_q))
        {
            std::cout << "RX chan q not found\n";
            throw std::runtime_error("RX chan q not found");
        }

    std::cout << "* Enabling IIO streaming channels\n";
    iio_channel_enable(rx0_i);
    iio_channel_enable(rx0_q);

    struct iio_device *ad9361_phy;
    ad9361_phy = iio_context_find_device(ctx, "ad9361-phy");
    int ret;
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


bool config_ad9361_lo_local(unsigned long bandwidth_,
    unsigned long sample_rate_,
    unsigned long freq_rf_tx_hz_,
    double tx_attenuation_db_,
    long long freq_dds_tx_hz_,
    double scale_dds_dbfs_)
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

    //find tx device
    struct iio_device *tx;

    std::cout << "* Acquiring AD9361 TX streaming devices\n";

    if (!get_ad9361_stream_dev(ctx, TX, &tx))
        {
            std::cout << "No tx dev found\n";
            throw std::runtime_error("AD9361 IIO No tx dev found");
        };

    std::cout << "* Configuring AD9361 for streaming TX\n";
    if (!cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0))
        {
            std::cout << "TX port 0 not found\n";
            throw std::runtime_error("AD9361 IIO TX port 0 not found");
        }

    //ENABLE DDS on TX1
    struct iio_device *ad9361_phy;
    ad9361_phy = iio_context_find_device(ctx, "ad9361-phy");
    int ret;
    //set output amplifier attenuation
    ret = iio_device_attr_write_double(ad9361_phy, "out_voltage0_hardwaregain", -tx_attenuation_db_);
    if (ret < 0)
        {
            std::cout << "Failed to set out_voltage0_hardwaregain value " << -tx_attenuation_db_ << " error " << ret << std::endl;
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

    //set frequency, scale and phase

    ret = iio_channel_attr_write_longlong(dds_channel0_I, "frequency", (long long)freq_dds_tx_hz_);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS frequency I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_longlong(dds_channel0_Q, "frequency", (long long)freq_dds_tx_hz_);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS frequency Q: " << ret << std::endl;
        }


    ret = iio_channel_attr_write_double(dds_channel0_I, "phase", 0.0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS phase I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_Q, "phase", 270000.0);
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

    //disable TX2


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


bool config_ad9361_lo_remote(std::string remote_host,
    unsigned long bandwidth_,
    unsigned long sample_rate_,
    unsigned long freq_rf_tx_hz_,
    double tx_attenuation_db_,
    long long freq_dds_tx_hz_,
    double scale_dds_dbfs_)
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

    //find tx device
    struct iio_device *tx;

    std::cout << "* Acquiring AD9361 TX streaming devices\n";

    if (!get_ad9361_stream_dev(ctx, TX, &tx))
        {
            std::cout << "No tx dev found\n";
            throw std::runtime_error("AD9361 IIO No tx dev found");
        };

    std::cout << "* Configuring AD9361 for streaming TX\n";
    if (!cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0))
        {
            std::cout << "TX port 0 not found\n";
            throw std::runtime_error("AD9361 IIO TX port 0 not found");
        }

    //ENABLE DDS on TX1
    struct iio_device *ad9361_phy;
    ad9361_phy = iio_context_find_device(ctx, "ad9361-phy");
    int ret;
    //set output amplifier attenuation
    ret = iio_device_attr_write_double(ad9361_phy, "out_voltage0_hardwaregain", -tx_attenuation_db_);
    if (ret < 0)
        {
            std::cout << "Failed to set out_voltage0_hardwaregain value " << -tx_attenuation_db_ << " error " << ret << std::endl;
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

    //set frequency, scale and phase

    ret = iio_channel_attr_write_longlong(dds_channel0_I, "frequency", (long long)freq_dds_tx_hz_);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS frequency I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_longlong(dds_channel0_Q, "frequency", (long long)freq_dds_tx_hz_);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS frequency Q: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_I, "phase", 0.0);
    if (ret < 0)
        {
            std::cout << "Failed to set TX DDS phase I: " << ret << std::endl;
        }

    ret = iio_channel_attr_write_double(dds_channel0_Q, "phase", 270000.0);
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

    //disable TX2

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


bool ad9361_disable_lo_remote(std::string remote_host)
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
