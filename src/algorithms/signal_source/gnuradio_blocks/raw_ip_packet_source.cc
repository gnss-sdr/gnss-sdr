/*!
 * \file raw_ip_packet_source.cc
 *
 * \brief Receives ip frames containing samples in UDP frame encapsulation
 * using a high performance packet capture library (libpcap)
 * \author Javier Arribas jarribas (at) cttc.es
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


#include <gnuradio/io_signature.h>
#include "raw_ip_packet_source.h"

#include <string.h>
#include <stdlib.h>


#define FIFO_SIZE 1000000


/* 4 bytes IP address */
typedef struct gr_ip_address{
    u_char byte1;
    u_char byte2;
    u_char byte3;
    u_char byte4;
}gr_ip_address;

/* IPv4 header */
typedef struct gr_ip_header{
    u_char  ver_ihl;        // Version (4 bits) + Internet header length (4 bits)
    u_char  tos;            // Type of service
    u_short tlen;           // Total length
    u_short identification; // Identification
    u_short flags_fo;       // Flags (3 bits) + Fragment offset (13 bits)
    u_char  ttl;            // Time to live
    u_char  proto;          // Protocol
    u_short crc;            // Header checksum
    gr_ip_address  saddr;      // Source address
    gr_ip_address  daddr;      // Destination address
    u_int   op_pad;         // Option + Padding
}gr_ip_header;

/* UDP header*/
typedef struct gr_udp_header{
    u_short sport;          // Source port
    u_short dport;          // Destination port
    u_short len;            // Datagram length
    u_short crc;            // Checksum
}gr_udp_header;

raw_ip_packet_source::sptr
raw_ip_packet_source::make(std::string src_device, std::string origin_address, int udp_port,  int udp_packet_size)
{
    return gnuradio::get_initial_sptr
            (new raw_ip_packet_source(src_device, origin_address,  udp_port, udp_packet_size));
}

/*
 * The private constructor
 */
raw_ip_packet_source::raw_ip_packet_source(std::string src_device, std::string origin_address, int udp_port,  int udp_packet_size)
: gr::sync_block("raw_ip_packet_source",
                 gr::io_signature::make(0, 0, 0),
                 gr::io_signature::make(1, 1, sizeof(char)))
{

    // constructor code here
    std::cout<<"Start Ethernet packet capture\n";

    d_src_device=src_device;
    d_udp_port=udp_port;
    d_udp_payload_size=udp_packet_size;
    d_fifo_full=false;
    d_last_frame_counter=0;
    d_num_rx_errors=0;

    //allocate signal samples buffer
    fifo_buff=new char[FIFO_SIZE];
    fifo_read_ptr=0;
    fifo_write_ptr=0;
    fifo_items=0;

    //open the ethernet device
    if (open()==true)
    {
        // start pcap capture thread
        d_pcap_thread=new boost::thread(boost::bind(&raw_ip_packet_source::my_pcap_loop_thread,this,descr));
    }else{
        exit(1); //ethernet error!
    }
}

bool raw_ip_packet_source::open()
{
    char errbuf[PCAP_ERRBUF_SIZE];
    boost::mutex::scoped_lock lock(d_mutex); 	// hold mutex for duration of this function
    /* open device for reading */
    descr = pcap_open_live(d_src_device.c_str(),1500,1,1000,errbuf);
    if(descr == NULL)
    {
        std::cout<<"Error openning Ethernet device "<<d_src_device<<std::endl;
        printf("Fatal Error in pcap_open_live(): %s\n",errbuf);
        return false;
    }
    //bind UDP port to avoid automatic reply with ICMP port ureacheable packets from kernel
    d_sock_raw = socket(AF_INET , SOCK_DGRAM , IPPROTO_UDP);
    if(d_sock_raw == -1)
    {
        std::cout<<"Error openning UDP socket"<<std::endl;
        return false;
    }

    // zero out the structure
    memset((char *) &si_me, 0, sizeof(si_me));

    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(d_udp_port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    //bind socket to port
    if( bind(d_sock_raw , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        std::cout<<"Error openning UDP socket"<<std::endl;
        return false;
    }
    return true;
}

/*
 * Our virtual destructor.
 */
raw_ip_packet_source::~raw_ip_packet_source()
{
    if(descr != NULL)
    {
        pcap_breakloop(descr);
        d_pcap_thread->join();
        pcap_close(descr);
    }

    delete fifo_buff;
    std::cout<<"Stop Ethernet packet capture\n";

}

void raw_ip_packet_source::static_pcap_callback(u_char *args, const struct pcap_pkthdr* pkthdr,
                                      const u_char* packet)
{
    raw_ip_packet_source *bridge=(raw_ip_packet_source*) args;
    bridge->pcap_callback(args, pkthdr, packet);
}

void raw_ip_packet_source::pcap_callback(u_char *args, const struct pcap_pkthdr* pkthdr,
                               const u_char* packet)
{
    boost::mutex::scoped_lock lock(d_mutex); 	// hold mutex for duration of this function

    gr_ip_header *ih;
    gr_udp_header *uh;

    // eth frame parameters
    // **** UDP RAW PACKET DECODER ****
    if ((packet[12]==0x08) & (packet[13]==0x00)) //IP FRAME
    {

        /* retireve the position of the ip header */
        ih = (gr_ip_header *) (packet +
                14); //length of ethernet header

        /* retireve the position of the udp header */
        u_int ip_len;
        ip_len = (ih->ver_ihl & 0xf) * 4;
        uh = (gr_udp_header *) ((u_char*)ih + ip_len);

        /* convert from network byte order to host byte order */
        u_short sport,dport;
        dport = ntohs( uh->dport );
        sport = ntohs( uh->sport );
        if (dport==d_udp_port)
        {
            // print ip addresses and udp ports
//            printf("%d.%d.%d.%d.%d -> %d.%d.%d.%d.%d\n",
//                   ih->saddr.byte1,
//                   ih->saddr.byte2,
//                   ih->saddr.byte3,
//                   ih->saddr.byte4,
//                   sport,
//                   ih->daddr.byte1,
//                   ih->daddr.byte2,
//                   ih->daddr.byte3,
//                   ih->daddr.byte4,
//                   dport);
//            std::cout<<"d_udp_port:"<<d_udp_port<<std::endl;
            //snapshots reading..
            u_char* udp_payload=((u_char*)uh+sizeof(gr_udp_header));
            if (fifo_items<=(FIFO_SIZE-d_udp_payload_size))
            {
                int aligned_write_items=FIFO_SIZE-fifo_write_ptr;
                if (aligned_write_items>=d_udp_payload_size)
                {
                    //write all in a single memcpy
                    memcpy(&fifo_buff[fifo_write_ptr],&udp_payload[0],d_udp_payload_size); //size in bytes
                    fifo_write_ptr+=d_udp_payload_size;
                    if (fifo_write_ptr==FIFO_SIZE) fifo_write_ptr=0;
                    fifo_items+=d_udp_payload_size;
                }else{
                    //two step wrap write
                    memcpy(&fifo_buff[fifo_write_ptr],&udp_payload[0],aligned_write_items); //size in bytes
                    fifo_write_ptr=d_udp_payload_size-aligned_write_items;
                    memcpy(&fifo_buff[0],&udp_payload[aligned_write_items],fifo_write_ptr); //size in bytes
                    fifo_items+=d_udp_payload_size;
                }
            }else{
                std::cout<<"Ou"<<std::flush;
            }
        }
    }

}

void raw_ip_packet_source::my_pcap_loop_thread(pcap_t *pcap_handle)

{

    pcap_loop(pcap_handle, -1, raw_ip_packet_source::static_pcap_callback, (u_char *)this);

}

int
raw_ip_packet_source::work(int noutput_items,
                 gr_vector_const_void_star &input_items,
                 gr_vector_void_star &output_items)
{

    // send samples to next GNU Radio block
    boost::mutex::scoped_lock lock(d_mutex); 	// hold mutex for duration of this function
    int num_samples_readed;

    if (noutput_items<fifo_items)
    {
        num_samples_readed=noutput_items;//read all
    }else{
        num_samples_readed=fifo_items;//read what we have
    }

    int aligned_read_items=FIFO_SIZE-fifo_read_ptr;
    if (aligned_read_items>=num_samples_readed)
    {
        //read all in a single memcpy
        memcpy(&((char*)output_items[0])[0],&fifo_buff[fifo_read_ptr],num_samples_readed);
        fifo_read_ptr=fifo_read_ptr+num_samples_readed; //increase the fifo pointer
        if (fifo_read_ptr==FIFO_SIZE) fifo_read_ptr=0;
    }else{
        //two step wrap read
        memcpy(&((char*)output_items[0])[0],&fifo_buff[fifo_read_ptr],aligned_read_items);
        fifo_read_ptr=num_samples_readed-aligned_read_items;//increase the fifo pointer considering the rollover
        memcpy(&((char*)output_items[0])[aligned_read_items],&fifo_buff[0],fifo_read_ptr);
    }

    fifo_items=fifo_items-num_samples_readed;

    // Tell runtime system how many output items we produced.
    //std::cout<<"fifo_items:"<<fifo_items<<"n:"<<num_samples_readed<<".";
    return num_samples_readed;
}

