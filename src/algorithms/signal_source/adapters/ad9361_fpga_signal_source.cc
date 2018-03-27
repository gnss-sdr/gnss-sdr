/*!
 * \file ad9361_fpga_signal_source.cc
 * \brief signal source for Analog Devices front-end AD9361 connected directly to FPGA accelerators.
 * This source implements only the AD9361 control. It is NOT compatible with conventional SDR acquisition and tracking blocks.
 * Please use the fmcomms2 source if conventional SDR acquisition and tracking is selected in the configuration file.
 * \author Javier Arribas, jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#include "ad9361_fpga_signal_source.h"
#include "configuration_interface.h"
#include "GPS_L1_CA.h"
#include "GPS_L2C.h"
#include <signal.h>
#include <stdio.h>
#include <glog/logging.h>
#include <iostream>

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


using google::LogMessage;


/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;

/* check return value of attr_write function */
static void errchk(int v, const char* what) {
     if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); }
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
    printf("* Acquiring AD9361 phy channel %d\n", chid);
    if (!get_phy_chan(ctx, type, chid, &chn)) { return false; }
    wr_ch_str(chn, "rf_port_select",     cfg->rfport);
    wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
    wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

    // Configure LO channel
    printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
    if (!get_lo_chan(ctx, type, &chn)) { return false; }
    wr_ch_lli(chn, "frequency", cfg->lo_hz);
    return true;
}


Ad9361FpgaSignalSource::Ad9361FpgaSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream, unsigned int out_stream,
        boost::shared_ptr<gr::msg_queue> queue) :
                        role_(role), in_stream_(in_stream), out_stream_(out_stream),
                        queue_(queue)
{
    std::string default_item_type = "gr_complex";
    std::string default_dump_file = "./data/signal_source.dat";
    uri_ = configuration->property(role + ".device_address", std::string("192.168.2.1"));
    freq_ = configuration->property(role + ".freq", GPS_L1_FREQ_HZ);
    sample_rate_ = configuration->property(role + ".sampling_frequency", 2600000);
    bandwidth_ = configuration->property(role + ".bandwidth", 2000000);
    rx1_en_ = configuration->property(role + ".rx1_enable", true);
    rx2_en_ = configuration->property(role + ".rx2_enable", false);
    buffer_size_ = configuration->property(role + ".buffer_size", 0xA0000);
    quadrature_ = configuration->property(role + ".quadrature", true);
    rf_dc_ = configuration->property(role + ".rf_dc", true);
    bb_dc_ = configuration->property(role + ".bb_dc", true);
    gain_mode_rx1_ = configuration->property(role + ".gain_mode_rx1", std::string("manual"));
    gain_mode_rx2_ = configuration->property(role + ".gain_mode_rx2", std::string("manual"));
    rf_gain_rx1_ = configuration->property(role + ".gain_rx1", 64.0);
    rf_gain_rx2_ = configuration->property(role + ".gain_rx2", 64.0);
    rf_port_select_ = configuration->property(role + ".rf_port_select", std::string("A_BALANCED"));
    filter_file_ = configuration->property(role + ".filter_file", std::string(""));
    filter_auto_ = configuration->property(role + ".filter_auto", true);
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    samples_ = configuration->property(role + ".samples", 0);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    enable_dds_lo_=configuration->property(role + ".enable_dds_lo", false);
    freq_rf_tx_hz_=configuration->property(role + ".freq_rf_tx_hz", GPS_L1_FREQ_HZ-GPS_L2_FREQ_HZ-1000);
    freq_dds_tx_hz_=configuration->property(role + ".freq_dds_tx_hz", 1000);
    scale_dds_dbfs_=configuration->property(role + ".scale_dds_dbfs", -3.0);
    phase_dds_deg_=configuration->property(role + ".phase_dds_deg", 0.0);
    tx_attenuation_db_=configuration->property(role + ".tx_attenuation_db", 0.0);

    item_size_ = sizeof(gr_complex);

    std::cout << "device address: " << uri_ << std::endl;
    std::cout << "LO frequency : " << freq_ << " Hz" << std::endl;
    std::cout << "sample rate: " << sample_rate_ << " Hz" << std::endl;


    // AD9361 Frontend IC device operation
    int ret;


    // Streaming devices
    struct iio_device *rx;

    // RX stream config
    // Stream configurations
    struct stream_cfg rxcfg;
    rxcfg.bw_hz = bandwidth_;   // 2 MHz rf bandwidth
    rxcfg.fs_hz = sample_rate_;   // 2.5 MS/s rx sample rate
    rxcfg.lo_hz = freq_; // 2.5 GHz rf frequency
    rxcfg.rfport = rf_port_select_.c_str(); // port A (select for rf freq.)


    std::cout<<"AD9361 Acquiring IIO context\n";
    ctx = iio_create_default_context();
    if (!ctx)
        {
            std::cout<<"No context\n";
            throw std::runtime_error("AD9361 IIO No context");
        }

    if (iio_context_get_devices_count(ctx) <= 0)
        {
            std::cout<<"No devices\n";
            throw std::runtime_error("AD9361 IIO No devices");
        }

    std::cout<<"* Acquiring AD9361 streaming devices\n";

    if(!get_ad9361_stream_dev(ctx, RX, &rx))
        {
            std::cout<<"No rx dev found\n";
            throw std::runtime_error("AD9361 IIO No rx dev found");
        };

    std::cout<<"* Configuring AD9361 for streaming\n";
    if (!cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0))
        {
            std::cout<<"RX port 0 not found\n";
            throw std::runtime_error("AD9361 IIO RX port 0 not found");
        }

    std::cout<<"* Initializing AD9361 IIO streaming channels\n";
    if (!get_ad9361_stream_ch(ctx, RX, rx, 0, &rx0_i))
        {
            std::cout<<"RX chan i not found\n";
            throw std::runtime_error("RX chan i not found");
        }

    if (!get_ad9361_stream_ch(ctx, RX, rx, 1, &rx0_q))
        {
            std::cout<<"RX chan q not found\n";
            throw std::runtime_error("RX chan q not found");
        }

    std::cout<<"* Enabling IIO streaming channels\n";
    iio_channel_enable(rx0_i);
    iio_channel_enable(rx0_q);



    struct iio_device *ad9361_phy;
    ad9361_phy= iio_context_find_device(ctx, "ad9361-phy");
    ret=iio_device_attr_write(ad9361_phy,"trx_rate_governor","nominal");
    if (ret < 0) {
        std::cout<<"Failed to set trx_rate_governor: "<<ret<<std::endl;
    }
    ret=iio_device_attr_write(ad9361_phy,"ensm_mode","fdd");
    if (ret < 0) {
            std::cout<<"Failed to set ensm_mode: "<<ret<<std::endl;
        }
    ret=iio_device_attr_write(ad9361_phy,"calib_mode","auto");
    if (ret < 0) {
            std::cout<<"Failed to set calib_mode: "<<ret<<std::endl;
        }
    ret=iio_device_attr_write(ad9361_phy,"in_voltage0_gain_control_mode",gain_mode_rx1_.c_str());
    if (ret < 0) {
            std::cout<<"Failed to set in_voltage0_gain_control_mode: "<<ret<<std::endl;
        }
    ret=iio_device_attr_write(ad9361_phy,"in_voltage1_gain_control_mode",gain_mode_rx2_.c_str());
    if (ret < 0) {
            std::cout<<"Failed to set in_voltage1_gain_control_mode: "<<ret<<std::endl;
        }
    ret=iio_device_attr_write_double(ad9361_phy,"in_voltage0_hardwaregain",rf_gain_rx1_);
    if (ret < 0) {
            std::cout<<"Failed to set in_voltage0_hardwaregain: "<<ret<<std::endl;
        }
    ret=iio_device_attr_write_double(ad9361_phy,"in_voltage1_hardwaregain",rf_gain_rx2_);
    if (ret < 0) {
            std::cout<<"Failed to set in_voltage1_hardwaregain: "<<ret<<std::endl;
        }


    std::cout<<"End of AD9361 RX configuration.\n";

    //LOCAL OSCILLATOR DDS GENERATOR FOR DUAL FREQUENCY OPERATION
    if (enable_dds_lo_==true)
    {
        // TX stream config
        std::cout<<"Start of AD9361 TX Local Oscillator DDS configuration\n";
        struct stream_cfg txcfg;
        txcfg.bw_hz = bandwidth_;
        txcfg.fs_hz = sample_rate_;
        txcfg.lo_hz = freq_rf_tx_hz_;
        txcfg.rfport = "A";

        //find tx device
        struct iio_device *tx;

        std::cout<<"* Acquiring AD9361 TX streaming devices\n";

        if(!get_ad9361_stream_dev(ctx, TX, &tx))
            {
                std::cout<<"No tx dev found\n";
                throw std::runtime_error("AD9361 IIO No tx dev found");
            };

        std::cout<<"* Configuring AD9361 for streaming TX\n";
        if (!cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0))
            {
                std::cout<<"TX port 0 not found\n";
                throw std::runtime_error("AD9361 IIO TX port 0 not found");
            }

        //ENABLE DDS on TX1

        //set output amplifier attenuation
        ret = iio_device_attr_write_double(ad9361_phy, "out_voltage0_hardwaregain", -tx_attenuation_db_);
        if (ret < 0) {
            std::cout<<"Failed to set out_voltage0_hardwaregain value "<<-tx_attenuation_db_<<" error "<<ret<<std::endl;
        }

        struct iio_device *dds;
        dds= iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
        struct iio_channel *dds_channel0_I;
        dds_channel0_I = iio_device_find_channel(dds, "TX1_I_F1", true);

        struct iio_channel *dds_channel0_Q;
        dds_channel0_Q = iio_device_find_channel(dds, "TX1_Q_F1", true);

        ret = iio_channel_attr_write_bool(dds_channel0_I, "raw", true);
        if (ret < 0) {
            std::cout<<"Failed to toggle DDS: "<<ret<<std::endl;
        }

        //set frequency, scale and phase

        ret=iio_channel_attr_write_longlong(dds_channel0_I, "frequency",(long long) freq_dds_tx_hz_);
        if (ret < 0) {
            std::cout<<"Failed to set TX DDS frequency I: "<<ret<<std::endl;
        }

        ret=iio_channel_attr_write_longlong(dds_channel0_Q, "frequency",(long long) freq_dds_tx_hz_);
        if (ret < 0) {
            std::cout<<"Failed to set TX DDS frequency Q: "<<ret<<std::endl;
        }


        ret=iio_channel_attr_write_double(dds_channel0_I, "phase",0.0);
        if (ret < 0) {
            std::cout<<"Failed to set TX DDS phase I: "<<ret<<std::endl;
        }

        ret=iio_channel_attr_write_double(dds_channel0_Q, "phase",270000.0);
        if (ret < 0) {
            std::cout<<"Failed to set TX DDS phase Q: "<<ret<<std::endl;
        }

        ret=iio_channel_attr_write_double(dds_channel0_I, "scale",pow(10, scale_dds_dbfs_ / 20.0));
        if (ret < 0) {
            std::cout<<"Failed to set TX DDS scale I: "<<ret<<std::endl;
        }

        ret=iio_channel_attr_write_double(dds_channel0_Q, "scale",pow(10, scale_dds_dbfs_ / 20.0));
        if (ret < 0) {
            std::cout<<"Failed to set TX DDS scale Q: "<<ret<<std::endl;
        }

        //disable TX2


        ret = iio_device_attr_write_double(ad9361_phy, "out_voltage1_hardwaregain", -89.0);
        if (ret < 0) {
            std::cout<<"Failed to set out_voltage1_hardwaregain value "<<-89.0<<" error "<<ret<<std::endl;
        }

        struct iio_channel *dds_channel1_I;
        dds_channel1_I = iio_device_find_channel(dds, "TX2_I_F1", true);

        struct iio_channel *dds_channel1_Q;
        dds_channel1_Q = iio_device_find_channel(dds, "TX2_Q_F1", true);


        ret=iio_channel_attr_write_double(dds_channel1_I, "scale",0);
        if (ret < 0) {
            std::cout<<"Failed to set TX2 DDS scale I: "<<ret<<std::endl;
        }

        ret=iio_channel_attr_write_double(dds_channel1_Q, "scale",0);
        if (ret < 0) {
            std::cout<<"Failed to set TX2 DDS scale Q: "<<ret<<std::endl;
        }
    }

}


Ad9361FpgaSignalSource::~Ad9361FpgaSignalSource()
{
    /* cleanup and exit */
    std::cout<<"* AD9361 Disabling streaming channels\n";
    if (rx0_i) { iio_channel_disable(rx0_i); }
    if (rx0_q) { iio_channel_disable(rx0_q); }

    std::cout<<"* AD9361 Destroying context\n";
    if (ctx) { iio_context_destroy(ctx); }

    if (enable_dds_lo_)
    {
        struct iio_device *dds;
        dds= iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
        struct iio_channel *dds_channel0_I;
        dds_channel0_I = iio_device_find_channel(dds, "TX1_I_F1", true);

        struct iio_channel *dds_channel0_Q;
        dds_channel0_Q = iio_device_find_channel(dds, "TX1_Q_F1", true);
        int ret;
        ret = iio_channel_attr_write_bool(dds_channel0_I, "raw", false);
        if (ret < 0) {
            std::cout<<"Failed to toggle DDS: "<<ret<<std::endl;
        }

        ret=iio_channel_attr_write_double(dds_channel0_I, "scale",0.0);
        if (ret < 0) {
            std::cout<<"Failed to set TX DDS scale I: "<<ret<<std::endl;
        }

        ret=iio_channel_attr_write_double(dds_channel0_Q, "scale",0.0);
        if (ret < 0) {
            std::cout<<"Failed to set TX DDS scale Q: "<<ret<<std::endl;
        }

    }
}


void Ad9361FpgaSignalSource::connect(gr::top_block_sptr top_block)
{
    DLOG(INFO) << "AD9361 FPGA source nothing to connect";
}


void Ad9361FpgaSignalSource::disconnect(gr::top_block_sptr top_block)
{
    DLOG(INFO) << "AD9361 FPGA source nothing to disconnect";
}


gr::basic_block_sptr Ad9361FpgaSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return gr::basic_block_sptr();
}


gr::basic_block_sptr Ad9361FpgaSignalSource::get_right_block()
{
    LOG(WARNING) << "Trying to get AD9361 FPGA signal source right block.";
    return gr::basic_block_sptr();
}
