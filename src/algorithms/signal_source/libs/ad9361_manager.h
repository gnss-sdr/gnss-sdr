/*!
 * \file ad9361_manager.h
 * \brief An Analog Devices AD9361 front-end configuration library wrapper for configure some functions via iiod link.
 * \author Javier Arribas, jarribas(at)cttc.es
 *
 * This file contains information taken from librtlsdr:
 *  http://git.osmocom.org/rtl-sdr/
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_AD9361_MANAGER_H_
#define GNSS_SDR_AD9361_MANAGER_H_

#include <iio.h>
#include <cstdint>
#include <string>

#define FIR_BUF_SIZE 8192

/* RX is input, TX is output */
enum iodev
{
    RX,
    TX
};

/* common RX and TX streaming params */
struct stream_cfg
{
    int64_t bw_hz;       // Analog bandwidth in Hz
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
    const std::string &rf_port_select_,
    const std::string &gain_mode_rx1_,
    const std::string &gain_mode_rx2_,
    double rf_gain_rx1_,
    double rf_gain_rx2_,
    bool quadrature_,
    bool rfdc_,
    bool bbdc_);

bool config_ad9361_rx_remote(const std::string &remote_host,
    uint64_t bandwidth_,
    uint64_t sample_rate_,
    uint64_t freq_,
    const std::string &rf_port_select_,
    const std::string &gain_mode_rx1_,
    const std::string &gain_mode_rx2_,
    double rf_gain_rx1_,
    double rf_gain_rx2_,
    bool quadrature_,
    bool rfdc_,
    bool bbdc_);

bool config_ad9361_lo_local(uint64_t bandwidth_,
    uint64_t sample_rate_,
    uint64_t freq_rf_tx_hz_,
    double tx_attenuation_db_,
    int64_t freq_dds_tx_hz_,
    double scale_dds_dbfs_,
    double phase_dds_deg_);

bool config_ad9361_lo_remote(const std::string &remote_host,
    uint64_t bandwidth_,
    uint64_t sample_rate_,
    uint64_t freq_rf_tx_hz_,
    double tx_attenuation_db_,
    int64_t freq_dds_tx_hz_,
    double scale_dds_dbfs_,
    double phase_dds_deg_);

bool ad9361_disable_lo_remote(const std::string &remote_host);

bool ad9361_disable_lo_local();

bool load_fir_filter(std::string &filter, struct iio_device *phy);

int ad9361_set_bb_rate_custom_filter_manual(struct iio_device *dev,
    uint64_t rate, uint64_t Fpass,
    uint64_t Fstop, uint64_t wnom_tx, uint64_t wnom_rx);

int ad9361_set_bb_rate_custom_filter_auto(struct iio_device *dev,
    uint64_t rate);

int apply_custom_filter(struct iio_device *dev, unsigned dec_tx,
    unsigned dec_rx, short *tapsTx,
    short *tapsRx, unsigned taps,
    uint64_t rate,
    int gain_tx, int gain_rx,
    uint64_t wnom_tx, uint64_t wnom_rx);

int ad9361_calculate_rf_clock_chain(uint64_t sample_rate,
    uint64_t rate_gov,
    uint64_t *rx_path_clks,
    uint64_t *tx_path_clks);

int determine_path_rates_with_fir(uint64_t sample_rate,
    uint64_t rate_gov,
    uint64_t *rx_path_clks,
    uint64_t *tx_path_clks,
    uint64_t tmp,
    int FIR);

bool check_rates(int FIR, const int *HB_configs, uint64_t samp_rate,
    uint64_t *rates);

int determine_pll_div(uint64_t *rates);

int check_dac_adc_config(uint64_t pll_bb, int PLL_mult,
    int dec_table_index);

double calculate_rfbw(double pll_rate, double caldiv, bool TX,
    double *rcaldiv);

int build_configuration(struct filter_design_parameters *fdpTX,
    struct filter_design_parameters *fdpRX,
    uint64_t sample_rate,
    uint64_t Fpass,
    uint64_t Fstop,
    uint64_t wnomTX,
    uint64_t wnomRX);

void set_max_taps(struct filter_design_parameters *fdpTX,
    struct filter_design_parameters *fdpRX);

#endif  // GNSS_SDR_AD9361_MANAGER_H_
