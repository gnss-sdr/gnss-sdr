/*!
 * \file ad9361_manager.cc
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
#include "ad9361_manager.h"
#include <glog/logging.h>
#include <ad9361.h>
#include <cmath>
#include <fstream>  // for ifstream
#include <iostream>
#include <sstream>
#include <vector>

#define MIN_ADC_CLK 25000000UL   //  25 MHz
#define MAX_ADC_CLK 640000000UL  // 640 MHz
#define MIN_DATA_RATE MIN_ADC_CLK / 48
#define MAX_DATA_RATE 61440000UL  // Output of FIR (RX)
#define MAX_DAC_CLK (MAX_ADC_CLK / 2)
#define MIN_DAC_CLK 25000000UL  //  25 MHz
#define MAX_RX_HB1 122880000UL
#define MAX_RX_HB2 245760000UL
#define MAX_RX_HB3 320000000UL
#define MAX_TX_HB1 122880000UL
#define MAX_TX_HB2 245760000UL
#define MAX_TX_HB3 320000000UL
#define MAX_FIR MAX_DATA_RATE * 2
#define MAX_BBPLL_DIV 64
#define MIN_BBPLL_FREQ 714928500UL   // 715 MHz - 100ppm
#define MAX_BBPLL_FREQ 1430143000UL  // 1430 MHz + 100ppm
#define check(val, min, max) ((val) <= (max) ? (val) >= (min) : false)

const uint64_t TX_MAX_PATH_RATES[] = {MAX_DAC_CLK, MAX_TX_HB3, MAX_TX_HB2, MAX_TX_HB1, MAX_FIR};
const uint64_t TX_MIN_PATH_RATES[] = {MIN_DAC_CLK, 0, 0, 0, 0};
const uint64_t RX_MAX_PATH_RATES[] = {MAX_ADC_CLK, MAX_RX_HB3, MAX_RX_HB2, MAX_RX_HB1, MAX_FIR};
const uint64_t RX_MIN_PATH_RATES[] = {MIN_ADC_CLK, 0, 0, 0, 0};

uint64_t max_rate_found;

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


void set_max_taps(struct filter_design_parameters *fdpTX,
    struct filter_design_parameters *fdpRX)
{
    // RX side
    int N, M, K;
    if (fdpRX->HB3 == 3)
        N = 16 * floor(fdpRX->converter_rate / (fdpRX->Rdata));
    else
        N = 16 * floor(fdpRX->converter_rate / (2 * fdpRX->Rdata));
    if (N > 128)
        N = 128;
    // TX side
    if (fdpTX->FIR == 1)
        M = 64;
    else
        M = 128;
    K = 16 * floor(fdpTX->converter_rate * fdpTX->DAC_div / (2 * fdpTX->Rdata));
    if (K < M)
        M = K;

    // Pick the smallest
    if (M > N)
        {
            fdpTX->maxTaps = N;
            fdpRX->maxTaps = N;
        }
    else
        {
            fdpTX->maxTaps = M;
            fdpRX->maxTaps = M;
        }
}


double calculate_rfbw(double pll_rate, double caldiv, bool TX,
    double *rcaldiv)
{
    double rfbw, min_rfbw, max_rfbw, scale;
    if (TX)
        {
            scale = 1.6;
            min_rfbw = 1250000;
            max_rfbw = 40000000;
        }
    else
        {
            scale = 1.4;
            min_rfbw = 400000;
            max_rfbw = 56000000;
        }
    rfbw =
        (double)round((pll_rate / caldiv) * (2 / (scale * (2 * M_PI) / log(2))));

    // If the RF bandwidth is outside the range of acceptable values we modify
    // the divider value until it falls into an acceptable range.
    while ((rfbw < min_rfbw) || (rfbw > max_rfbw))
        {
            if (rfbw < min_rfbw)
                caldiv = caldiv - 1;
            else
                caldiv = caldiv + 1;

            if ((caldiv < 1) || (caldiv > 511))
                {
                    fprintf(stderr, "Calibration divider out of bounds (1 - 511): %f\n", caldiv);
                    return -EINVAL;
                }
            rfbw = calculate_rfbw(pll_rate, caldiv, TX, rcaldiv);
        }
    *rcaldiv = caldiv;
    return rfbw;
}

void set_rates(uint64_t *rx_path_clks,
    uint64_t *tx_path_clks, int DAC_div, uint64_t *rates,
    int dec_table_index)
{
    int k;

    // Check if ADC will run faster in config
    if (rates[1] > max_rate_found)
        max_rate_found = rates[1];
    else
        return;

    for (k = 0; k < 6; k++)
        {
            rx_path_clks[k] = rates[k];
            tx_path_clks[k] = rates[k];

            if (k > 0)
                {  // Adjust HB's for DAC divider setting
                    if ((dec_table_index < 2) && (k < 4))
                        tx_path_clks[k] = rates[k] / DAC_div;
                    else if ((dec_table_index < 4) && (k < 3))
                        tx_path_clks[k] = rates[k] / DAC_div;
                }
        }
}


int check_dac_adc_config(uint64_t pll_bb, int PLL_mult,
    int dec_table_index)
{
    // Need to determine if DAC divider is required and if ADC and DAC rates
    // can be satisfied
    uint64_t with_dd, without_dd;
    bool a, b, c;

    with_dd = pll_bb / PLL_mult / 2;
    without_dd = pll_bb / PLL_mult / 1;

    a = check(with_dd, MIN_DAC_CLK, MAX_DAC_CLK);
    b = check(without_dd, MIN_ADC_CLK, MAX_ADC_CLK);
    c = check(without_dd, MIN_DAC_CLK, MAX_DAC_CLK);

    if (c && b)
        return 1;  // Run without dac div
    else if (a && b && (dec_table_index < 5))
        return 2;  // Run with dac div
    else
        return -1;
}


bool check_rates(int FIR __attribute__((unused)), const int *HB_configs, uint64_t samp_rate,
    uint64_t *rates)
{
    int j;
    bool c = true;

    rates[5] = samp_rate;
    for (j = 4; j > 0; j--)
        {
            rates[j] = rates[j + 1] * HB_configs[j - 1];
            if (j > 1)
                {
                    c &= check(rates[j], TX_MIN_PATH_RATES[j - 1], TX_MAX_PATH_RATES[j - 1]);
                    c &= check(rates[j], RX_MIN_PATH_RATES[j - 1], RX_MAX_PATH_RATES[j - 1]);
                }
        }
    return c;
}


int determine_pll_div(uint64_t *rates)
{
    // Determine necessary PLL multiplier
    uint64_t tmp;
    int PLL_mult = MAX_BBPLL_DIV;
    while (PLL_mult > 1)
        {
            tmp = (uint64_t)rates[1] * PLL_mult;
            if (check(tmp, MIN_BBPLL_FREQ, MAX_BBPLL_FREQ))
                {
                    rates[0] = (uint64_t)rates[1] * PLL_mult;
                    return PLL_mult;
                }
            PLL_mult >>= 1;
        }
    return -1;
}


int determine_path_rates_with_fir(uint64_t sample_rate,
    uint64_t rate_gov,
    uint64_t *rx_path_clks,
    uint64_t *tx_path_clks,
    int FIR)
{
    uint64_t rates[6];
    int PLL_mult, k;

    max_rate_found = 0UL;

    const int HB_configs[][4] = {
        {3, 2, 2, FIR},  //12
        {2, 2, 2, FIR},  //8
        {3, 2, 1, FIR},  //6
        {2, 2, 1, FIR},  //4
        {2, 1, 1, FIR},  //2
        {3, 1, 1, FIR},  //3
        {1, 1, 1, FIR},  //1
    };

    // RX Path:
    // BBPLL -> /PLL_div -> /HB3 -> /HB2 -> /HB1 -> /FIR
    // TX Path:
    // BBPLL -> /(PLL_div*DAC_div) -> /HB3 -> /HB2 -> /HB1 -> /FIR

    // Cycle through possible decimations from highest to lowest
    for (k = 0; k < 7; k++)
        {
            // HB3 cannot be 3 if rate_gov enabled
            if ((rate_gov > 0) && HB_configs[k][0] == 3)
                continue;
            // Check if HB and FIR rates are valid
            if (check_rates(FIR, HB_configs[k], sample_rate, rates))
                {
                    // Calculate PLL divider for configuration
                    PLL_mult = determine_pll_div(rates);
                    if (PLL_mult > 0)
                        {
                            // Determine DAC divider setting and check ADC/DAC settings
                            int dac_div = check_dac_adc_config(rates[0], PLL_mult, k);
                            // printf("dac_div: %d\n",dac_div);
                            if (dac_div > 0)
                                set_rates(rx_path_clks, tx_path_clks, dac_div, rates, k);
                        }
                }
        }

    if (max_rate_found == 0UL)
        return -EINVAL;
    else
        return 0;
}


int ad9361_calculate_rf_clock_chain(uint64_t sample_rate,
    uint64_t rate_gov,
    uint64_t *rx_path_clks,
    uint64_t *tx_path_clks)
{
    int ret, k;
    int FIR[] = {4, 2, 1};

    // Check desired rate within bounds
    if (!check(sample_rate, MIN_DATA_RATE, MAX_DATA_RATE))
        return -EINVAL;

    // Rate selection will try to:
    // 1. Utilize the maximum decimation in the FIR
    // 2. Run the ADC/DAC as fast as possible
    // 3. Use the most decimation possible starting with HB3(closest to ADC)->HB1

    // Cycle through available FIR settings
    for (k = 0; k < 3; k++)
        {
            ret = determine_path_rates_with_fir(sample_rate, rate_gov, rx_path_clks,
                tx_path_clks, FIR[k]);
            if (ret == 0)
                break;
        }
    return ret;
}


int apply_custom_filter(struct iio_device *dev, unsigned dec_tx,
    unsigned dec_rx, short *tapsTx,
    short *tapsRx, unsigned taps,
    uint64_t rate,
    int gain_tx, int gain_rx,
    uint64_t wnom_tx, uint64_t wnom_rx)
{
    struct iio_channel *chanTX, *chanRX;
    long long current_rate;
    int ret, i, enable, len = 0;
    char *buf;

    chanTX = iio_device_find_channel(dev, "voltage0", true);
    if (chanTX == NULL)
        return -ENODEV;

    ret = iio_channel_attr_read_longlong(chanTX, "sampling_frequency", &current_rate);
    if (ret < 0)
        return ret;

    ret = ad9361_get_trx_fir_enable(dev, &enable);
    if (ret < 0)
        return ret;

    if (enable)
        {
            if (current_rate <= (25000000 / 12))
                iio_channel_attr_write_longlong(chanTX, "sampling_frequency", 3000000);

            ret = ad9361_set_trx_fir_enable(dev, false);
            if (ret < 0)
                return ret;
        }

    buf = (char *)malloc(FIR_BUF_SIZE);
    if (!buf)
        return -ENOMEM;


    len += snprintf(buf + len, FIR_BUF_SIZE - len, "RX 3 GAIN %d DEC %d\n", gain_rx,
        dec_rx);
    len += snprintf(buf + len, FIR_BUF_SIZE - len, "TX 3 GAIN %d INT %d\n", gain_tx,
        dec_tx);

    for (i = 0; i < (int)taps; i++)
        len += snprintf(buf + len, FIR_BUF_SIZE - len, "%d,%d\n", tapsRx[i], tapsTx[i]);

    len += snprintf(buf + len, FIR_BUF_SIZE - len, "\n");

    ret = iio_device_attr_write_raw(dev, "filter_fir_config", buf, len);
    free(buf);

    if (ret < 0)
        return ret;

    if (rate <= (25000000 / 12))
        {
            int dacrate, txrate, max;
            char readbuf[100];
            ret = iio_device_attr_read(dev, "tx_path_rates", readbuf, sizeof(readbuf));
            if (ret < 0)
                return ret;
            ret = sscanf(readbuf, "BBPLL:%*d DAC:%d T2:%*d T1:%*d TF:%*d TXSAMP:%d",
                &dacrate, &txrate);
            if (ret != 2)
                return -EFAULT;
            if (txrate == 0)
                return -EINVAL;
            max = (dacrate / txrate) * 16;
            if (max < taps)
                iio_channel_attr_write_longlong(chanTX, "sampling_frequency", 3000000);


            ret = ad9361_set_trx_fir_enable(dev, true);
            if (ret < 0)
                return ret;
            ret = iio_channel_attr_write_longlong(chanTX, "sampling_frequency", rate);
            if (ret < 0)
                return ret;
        }
    else
        {
            ret = iio_channel_attr_write_longlong(chanTX, "sampling_frequency", rate);
            if (ret < 0)
                return ret;
            ret = ad9361_set_trx_fir_enable(dev, true);
            if (ret < 0)
                return ret;
        }

    chanRX = iio_device_find_channel(dev, "voltage0", false);
    if (chanRX == NULL)
        return -ENODEV;
    ret = iio_channel_attr_write_longlong(chanTX, "rf_bandwidth", wnom_tx);
    if (ret < 0)
        return ret;
    ret = iio_channel_attr_write_longlong(chanRX, "rf_bandwidth", wnom_rx);
    if (ret < 0)
        return ret;

    return 0;
}


int ad9361_set_bb_rate_custom_filter_auto(struct iio_device *dev,
    uint64_t rate)
{
    struct filter_design_parameters fdpTX;
    struct filter_design_parameters fdpRX;
    short taps_tx[128];
    short taps_rx[128];
    int ret, num_taps_tx, num_taps_rx, gain_tx, gain_rx;
    unsigned dec_tx, dec_rx, num_taps;

    ret = ad9361_calculate_rf_clock_chain_fdp(&fdpTX, &fdpRX, rate);
    if (ret < 0)
        return ret;

    ret = ad9361_generate_fir_taps(&fdpRX, taps_rx, &num_taps_rx, &gain_rx);
    if (ret < 0)
        return ret;

    ret = ad9361_generate_fir_taps(&fdpTX, taps_tx, &num_taps_tx, &gain_tx);
    if (ret < 0)
        return ret;

    dec_tx = (unsigned)fdpTX.FIR;
    dec_rx = (unsigned)fdpRX.FIR;
    num_taps = (unsigned)fdpTX.maxTaps;
    ret = apply_custom_filter(dev, dec_tx, dec_rx, taps_tx, taps_rx, num_taps,
        rate, gain_tx, gain_rx, fdpTX.wnom, fdpRX.wnom);
    if (ret < 0)
        return ret;

    return 0;
}

int build_configuration(struct filter_design_parameters *fdpTX,
    struct filter_design_parameters *fdpRX,
    uint64_t sample_rate,
    uint64_t Fpass,
    uint64_t Fstop,
    uint64_t wnomTX,
    uint64_t wnomRX)
{
    double div, max;
    uint64_t rx_path_clk[6];
    uint64_t tx_path_clk[6];
    uint64_t *path_clk;
    struct filter_design_parameters *fdp;
    int ret, k;
    uint64_t rate_gov = 0;

    ret = ad9361_calculate_rf_clock_chain((uint64_t)sample_rate,
        rate_gov, rx_path_clk, tx_path_clk);
    if (ret < 0)
        return -EINVAL;

    for (k = 0; k < 2; k++)
        {
            if (k > 0)
                {
                    path_clk = tx_path_clk;
                    fdp = fdpTX;
                    fdp->RxTx = "Tx";
                    fdp->DAC_div = (double)rx_path_clk[1] / tx_path_clk[1];
                }
            else
                {
                    path_clk = rx_path_clk;
                    fdp = fdpRX;
                    fdp->RxTx = "Rx";
                    fdp->DAC_div = 1.0;
                }
            // Map rates and dividers
            fdp->PLL_rate = (double)path_clk[0];
            fdp->converter_rate = (double)path_clk[1];
            fdp->PLL_mult = (double)path_clk[0] / path_clk[1];
            fdp->HB3 = (double)path_clk[1] / path_clk[2];
            fdp->HB2 = (double)path_clk[2] / path_clk[3];
            fdp->HB1 = (double)path_clk[3] / path_clk[4];
            fdp->FIR = (double)path_clk[4] / path_clk[5];

            // Set default parameters
            fdp->Rdata = (double)path_clk[5];
            fdp->Type = "Lowpass";
            fdp->int_FIR = 1;
            fdp->Apass = 0.5;
            fdp->Astop = 80;
            fdp->phEQ = -1;
            fdp->FIRdBmin = 0;
            // Define filter design specifications
            fdp->Fpass = (double)Fpass;
            fdp->Fstop = (double)Fstop;
            fdp->Fcenter = 0.0;
            if (k > 0)
                fdp->wnom = (double)wnomTX;
            else
                fdp->wnom = (double)wnomRX;
            // Determine default analog bandwidth
            div = ceil((fdp->PLL_rate / fdp->wnom) * (log(2) / (2 * M_PI)));
            max = (div > 1) ? div : 1.0;
            fdp->caldiv = (max > 511) ? 511.0 : max;
            fdp->RFbw = calculate_rfbw(fdp->PLL_rate, fdp->caldiv, k > 0, &(fdp->caldiv));

            if (fdp->RFbw < 0)
                return -EINVAL;
        }
    set_max_taps(fdpTX, fdpRX);

    return 0;
}


int ad9361_set_bb_rate_custom_filter_manual(struct iio_device *dev,
    uint64_t rate, uint64_t Fpass,
    uint64_t Fstop, uint64_t wnom_tx, uint64_t wnom_rx)
{
    struct filter_design_parameters fdpTX;
    struct filter_design_parameters fdpRX;
    short taps_tx[128];
    short taps_rx[128];
    int ret, num_taps_tx, num_taps_rx, gain_tx, gain_rx;
    unsigned dec_tx, dec_rx, num_taps;

    if (Fpass >= Fstop)
        return -EINVAL;

    ret = build_configuration(&fdpTX, &fdpRX, rate, Fpass, Fstop, wnom_tx,
        wnom_rx);
    if (ret < 0)
        return ret;

    ret = ad9361_generate_fir_taps(&fdpRX, taps_rx, &num_taps_rx, &gain_rx);
    if (ret < 0)
        return ret;

    ret = ad9361_generate_fir_taps(&fdpTX, taps_tx, &num_taps_tx, &gain_tx);
    if (ret < 0)
        return ret;

    dec_tx = (unsigned)fdpTX.FIR;
    dec_rx = (unsigned)fdpRX.FIR;
    num_taps = (unsigned)fdpTX.maxTaps;

    ret = apply_custom_filter(dev, dec_tx, dec_rx, taps_tx, taps_rx, num_taps,
        rate, gain_tx, gain_rx, wnom_tx, wnom_rx);
    if (ret < 0)
        return ret;

    return 0;
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
    const std::string &gain_mode_rx1_,
    const std::string &gain_mode_rx2_,
    double rf_gain_rx1_,
    double rf_gain_rx2_,
    bool quadrature_,
    bool rfdc_,
    bool bbdc_)

{
    // RX stream config
    // Stream configurations
    struct stream_cfg rxcfg;
    rxcfg.bw_hz = bandwidth_;
    rxcfg.fs_hz = sample_rate_;
    rxcfg.lo_hz = freq_;
    rxcfg.rfport = rf_port_select_.c_str();

    std::cout << "AD9361 Acquiring IIO LOCAL context\n";
    struct iio_context *ctx;
    // Streaming devices
    struct iio_device *rx;
    struct iio_channel *rx_chan1;
    struct iio_channel *rx_chan2;

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

    std::cout << "* Enabling IIO streaming channels\n";
    iio_channel_enable(rx_chan1);
    iio_channel_enable(rx_chan2);

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
    const std::string &gain_mode_rx1_,
    const std::string &gain_mode_rx2_,
    double rf_gain_rx1_,
    double rf_gain_rx2_,
    bool quadrature_,
    bool rfdc_,
    bool bbdc_)
{
    std::string filter_source_("Off");
    float Fpass_ = 0.0, Fstop_ = 0.0;
    std::string filter_filename_;
    // RX stream config

    std::cout << "AD9361 Acquiring IIO REMOTE context in host " << remote_host << std::endl;
    struct iio_context *ctx;
    // Streaming devices
    struct iio_device *rx;
    struct iio_channel *rx_chan1;
    struct iio_channel *rx_chan2;

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

    struct iio_device *ad9361_phy;
    ad9361_phy = iio_context_find_device(ctx, "ad9361-phy");
    int ret;

    std::cout << "* Initializing AD9361 IIO streaming channels\n";
    if (!get_ad9361_stream_ch(ctx, RX, rx, 0, &rx_chan1))
        {
            std::cout << "RX chan i not found\n";
            throw std::runtime_error("RX chan i not found");
        }

    if (!get_ad9361_stream_ch(ctx, RX, rx, 1, &rx_chan2))
        {
            std::cout << "RX chan q not found\n";
            throw std::runtime_error("RX chan q not found");
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
                    //params.push_back("in_voltage_rf_bandwidth=" + boost::to_string(bandwidth));
                }
            //wr_ch_str(rx_chan1, "rf_port_select", rf_port_select_.c_str());
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
    iio_channel_enable(rx_chan1);
    iio_channel_enable(rx_chan2);

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
        };

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
        };

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

    /* Here, we verify that the filter file contains data for both RX+TX. */
    {
        char buf[256];
        do
            {
                ifs.getline(buf, sizeof(buf));
            }
        while (!(buf[0] == '-' || (buf[0] >= '0' && buf[0] <= '9')));

        std::string line(buf);
        if (line.find(',') == std::string::npos)
            throw std::runtime_error("Incompatible filter file");
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
