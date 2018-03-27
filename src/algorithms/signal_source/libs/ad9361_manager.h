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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef __AD9361_MANAGER__
#define __AD9361_MANAGER__


#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif



/* RX is input, TX is output */
enum iodev { RX, TX };

/* common RX and TX streaming params */
struct stream_cfg {
    long long bw_hz; // Analog banwidth in Hz
    long long fs_hz; // Baseband sample rate in Hz
    long long lo_hz; // Local oscillator frequency in Hz
    const char* rfport; // Port name
};


enum dds_tone_type {
    TX1_T1_I,
    TX1_T2_I,
    TX1_T1_Q,
    TX1_T2_Q,
    TX2_T1_I,
    TX2_T2_I,
    TX2_T1_Q,
    TX2_T2_Q,
    TX3_T1_I,
    TX3_T2_I,
    TX3_T1_Q,
    TX3_T2_Q,
    TX4_T1_I,
    TX4_T2_I,
    TX4_T1_Q,
    TX4_T2_Q
};

enum dds_widget_type {
    WIDGET_FREQUENCY,
    WIDGET_SCALE,
    WIDGET_PHASE
};

#define DDS_DISABLED  0
#define DDS_ONE_TONE  1
#define DDS_TWO_TONE  2
#define DDS_INDEPDENT 3
#define DDS_BUFFER    4


/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;

/* check return value of attr_write function */
static void errchk(int v, const char* what);

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val);

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str);

/* helper function generating channel names */
static char* get_ch_name(const char* type, int id);

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(struct iio_context *ctx);

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev);

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn);

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn);

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn);

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg, enum iodev type, int chid);


static bool set_dds_cw_tone(struct iio_device *dac1, double freq_hz, double scale_dbfs, double phase_deg);

#endif
