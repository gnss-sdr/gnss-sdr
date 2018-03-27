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
#include <stdio.h> //only for snprintf
#include <math.h>
#include <iostream>

/* check return value of attr_write function */
static void errchk(int v, const char* what) {
     if (v < 0)
     {
         LOG(WARNING)<<"Error "<<v<<" writing to channel "<<what<<" value may not be supported. ";
     }
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
    errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
    errchk(iio_channel_attr_write(chn, what, str), what);
}


/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)
{
    snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
    return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(struct iio_context *ctx)
{
    struct iio_device *dev =  iio_context_find_device(ctx, "ad9361-phy");
    return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
    switch (d) {
        case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
        case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");  return *dev != NULL;
        default: return false;
    }
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
    *chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
    if (!*chn)
        *chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
    return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn)
{
    switch (d) {
    case RX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), false); return *chn != NULL;
    case TX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), true);  return *chn != NULL;
    default: return false;
    }
}

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn)
{
    switch (d) {
     // LO chan is always output, i.e. true
    case RX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 0), true); return *chn != NULL;
    case TX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 1), true); return *chn != NULL;
    default: return false;
    }
}


/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg, enum iodev type, int chid)
{
    struct iio_channel *chn = NULL;

    // Configure phy and lo channels
    LOG(INFO)<<"* Acquiring AD9361 phy channel"<<chid;
    if (!get_phy_chan(ctx, type, chid, &chn)) { return false; }
    wr_ch_str(chn, "rf_port_select",     cfg->rfport);
    wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
    wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

    // Configure LO channel
    LOG(INFO)<<"* Acquiring AD9361 "<<type == TX ? "TX" : "RX";
    if (!get_lo_chan(ctx, type, &chn)) { return false; }
    wr_ch_lli(chn, "frequency", cfg->lo_hz);
    return true;
}


static bool set_dds_cw_tone(struct iio_device *dac1, struct iio_channel *chn, double freq_hz, double scale_dbfs, double phase_deg)
{

    //ENABLE DDS
    int ret;
    ret = iio_channel_attr_write_bool(iio_device_find_channel(dac1, "altvoltage0", true), "raw", true);
    if (ret < 0) {
        std::cout<<"Failed to toggle DDS: "<<ret<<std::endl;
        return false;
    }

    //set frequency, scale and phase

    ret=iio_channel_attr_write_longlong(chn, "frequency",(long long) freq_hz);
    if (ret < 0) {
        std::cout<<"Failed to set TX DDS frequency: "<<ret<<std::endl;
        return false;
    }

    ret=iio_channel_attr_write_double(chn, "phase",phase_deg);
    if (ret < 0) {
        std::cout<<"Failed to set TX DDS phase: "<<ret<<std::endl;
        return false;
    }

    ret=iio_channel_attr_write_double(chn, "scale",pow(10, scale_dbfs / 20.0));
    if (ret < 0) {
        std::cout<<"Failed to set TX DDS scale: "<<ret<<std::endl;
        return false;
    }
    return true;

}
