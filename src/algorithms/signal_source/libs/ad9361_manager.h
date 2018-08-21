/*!
 * \file ad9361_manager.h
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_AD9361_MANAGER_H
#define GNSS_SDR_AD9361_MANAGER_H

#include <cstdint>
#include <string>

#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif

/* RX is input, TX is output */
enum iodev
{
    RX,
    TX
};

/* common RX and TX streaming params */
struct stream_cfg
{
    int64_t bw_hz;       // Analog banwidth in Hz
    int64_t fs_hz;       // Baseband sample rate in Hz
    int64_t lo_hz;       // Local oscillator frequency in Hz
    const char *rfport;  // Port name
};


/* check return value of attr_write function */
void errchk(int v, const char *what);

/* write attribute: int64_t int */
void wr_ch_lli(struct iio_channel *chn, const char *what, int64_t val);

/* write attribute: string */
void wr_ch_str(struct iio_channel *chn, const char *what, const char *str);

/* helper function generating channel names */
char *get_ch_name(const char *type, int id, char *tmpstr);

/* returns ad9361 phy device */
struct iio_device *get_ad9361_phy(struct iio_context *ctx);

/* finds AD9361 streaming IIO devices */
bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev);

/* finds AD9361 streaming IIO channels */
bool get_ad9361_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn);

/* finds AD9361 phy IIO configuration channel with id chid */
bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn);

/* finds AD9361 local oscillator IIO configuration channels */
bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn);

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg, enum iodev type, int chid);

bool config_ad9361_rx_local(uint64_t bandwidth_,
    uint64_t sample_rate_,
    uint64_t freq_,
    std::string rf_port_select_,
    std::string gain_mode_rx1_,
    std::string gain_mode_rx2_,
    double rf_gain_rx1_,
    double rf_gain_rx2_);

bool config_ad9361_rx_remote(std::string remote_host,
    uint64_t bandwidth_,
    uint64_t sample_rate_,
    uint64_t freq_,
    std::string rf_port_select_,
    std::string gain_mode_rx1_,
    std::string gain_mode_rx2_,
    double rf_gain_rx1_,
    double rf_gain_rx2_);

bool config_ad9361_lo_local(uint64_t bandwidth_,
    uint64_t sample_rate_,
    uint64_t freq_rf_tx_hz_,
    double tx_attenuation_db_,
    int64_t freq_dds_tx_hz_,
    double scale_dds_dbfs_);

bool config_ad9361_lo_remote(std::string remote_host,
    uint64_t bandwidth_,
    uint64_t sample_rate_,
    uint64_t freq_rf_tx_hz_,
    double tx_attenuation_db_,
    int64_t freq_dds_tx_hz_,
    double scale_dds_dbfs_);


bool ad9361_disable_lo_remote(std::string remote_host);

bool ad9361_disable_lo_local();

#endif
