/*!
 * \file gr_complex_ip_packet_source.h
 *
 * \brief Receives ip frames containing samples in UDP frame encapsulation
 * using a high performance packet capture library (libpcap)
 * \author Javier Arribas jarribas (at) cttc.es
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


#ifndef GNSS_SDR_GR_COMPLEX_IP_PACKET_SOURCE_H
#define GNSS_SDR_GR_COMPLEX_IP_PACKET_SOURCE_H

#include <boost/thread.hpp>
#include <gnuradio/sync_block.h>
#include <arpa/inet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <netinet/if_ether.h>
#include <pcap.h>
#include <string>
#include <sys/ioctl.h>

class Gr_Complex_Ip_Packet_Source : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<Gr_Complex_Ip_Packet_Source> sptr;
    static sptr make(std::string src_device,
        const std::string &origin_address,
        int udp_port,
        int udp_packet_size,
        int n_baseband_channels,
        const std::string &wire_sample_type,
        size_t item_size,
        bool IQ_swap_);
    Gr_Complex_Ip_Packet_Source(std::string src_device,
        const std::string &origin_address,
        int udp_port,
        int udp_packet_size,
        int n_baseband_channels,
        const std::string &wire_sample_type,
        size_t item_size,
        bool IQ_swap_);
    ~Gr_Complex_Ip_Packet_Source();

    // Called by gnuradio to enable drivers, etc for i/o devices.
    bool start();

    // Called by gnuradio to disable drivers, etc for i/o devices.
    bool stop();

    // Where all the action really happens
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    boost::mutex d_mutex;
    pcap_t *descr;  // ethernet pcap device descriptor
    char *fifo_buff;
    int fifo_read_ptr;
    int fifo_write_ptr;
    int fifo_items;
    int d_sock_raw;
    int d_udp_port;
    // clang-format off
    struct sockaddr_in si_me{};
    // clang-format on
    std::string d_src_device;
    std::string d_origin_address;
    int d_udp_payload_size;
    bool d_fifo_full;
    int d_n_baseband_channels;
    int d_wire_sample_type;
    int d_bytes_per_sample;
    size_t d_item_size;
    bool d_IQ_swap;
    boost::thread *d_pcap_thread;
    void demux_samples(const gr_vector_void_star &output_items, int num_samples_readed);
    void my_pcap_loop_thread(pcap_t *pcap_handle);
    void pcap_callback(u_char *args, const struct pcap_pkthdr *pkthdr, const u_char *packet);
    static void static_pcap_callback(u_char *args, const struct pcap_pkthdr *pkthdr, const u_char *packet);
    /*
     * Opens the ethernet device using libpcap raw capture mode
     * If any of these fail, the function returns the error and exits.
     */
    bool open();
};

#endif  //  GNSS_SDR_GR_COMPLEX_IP_PACKET_SOURCE_H
