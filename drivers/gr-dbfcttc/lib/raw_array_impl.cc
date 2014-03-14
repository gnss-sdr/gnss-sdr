/*!
 * \file raw_array_impl.cc
 * \brief GNU Radio source block to acces to experimental GNSS Array platform.
 * \author Javier Arribas, 2014. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "raw_array_impl.h"
#include <arpa/inet.h>
#include <net/if.h>
#include <net/ethernet.h>
#include <netinet/if_ether.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdlib.h>


#define FIFO_SIZE 1000000
#define DBFCTTC_NUM_CHANNELS 8

namespace gr {
  namespace dbfcttc {

    raw_array::sptr
    raw_array::make(const char *src_device,short number_of_channels, int snapshots_per_frame, int inter_frame_delay, int sampling_freq)
    {
      return gnuradio::get_initial_sptr
        (new raw_array_impl(src_device, number_of_channels, snapshots_per_frame, inter_frame_delay, sampling_freq));
    }

    /*
     * The private constructor
     */
    raw_array_impl::raw_array_impl(const char *src_device,short number_of_channels, int snapshots_per_frame, int inter_frame_delay, int sampling_freq)
      : gr::sync_block("raw_array",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(8, 8, sizeof(gr_complex)))
    {

    	// constructor code here
    	  fprintf(stdout,"DBFCTTC Start\n");

    	  d_src_device=src_device;
    	  d_number_of_channels=number_of_channels;
    	  d_snapshots_per_frame=snapshots_per_frame;
    	  d_inter_frame_delay=inter_frame_delay;
    	  d_sampling_freq=sampling_freq;

    	  d_flag_start_frame=true;
    	  d_fifo_full=false;
    	  d_last_frame_counter=0;
    	  d_num_rx_errors=0;
    	  flag_16_bits_sample=true;

    	   //allocate signal samples buffer
    	   //TODO: Check memory pointers
    	   fifo_buff_ch=new gr_complex*[DBFCTTC_NUM_CHANNELS];
    	   for (int i=0;i<DBFCTTC_NUM_CHANNELS;i++)
    	   {
    		   fifo_buff_ch[i]=new gr_complex[FIFO_SIZE];
    	   }

    	   fifo_read_ptr=0;
    	   fifo_write_ptr=0;
    	   fifo_items=0;

    	   //open the ethernet device
    	   if (open()==true)
    	   {
    		   // start pcap capture thread
    		   d_pcap_thread=new boost::thread(boost::bind(&raw_array_impl::my_pcap_loop_thread,this,descr));


    		   // send array configuration frame
    		   if (configure_array()==true)
    		   {
    			   if (start_array()==true)
    			   {
    				   printf("Array ready!\n");
    			   }else{
    				   exit(1); //ethernet error!
    			   }
    		   }else{
    			   exit(1); //ethernet error!
    		   }
    	   }else{
    		   exit(1); //ethernet error!
    	   }
    }

    bool raw_array_impl::open()
    {
      char errbuf[PCAP_ERRBUF_SIZE];
      boost::mutex::scoped_lock lock(d_mutex); 	// hold mutex for duration of this function
        char *dev;
        /* open device for reading */
        descr = pcap_open_live(d_src_device,1500,1,1000,errbuf);
        if(descr == NULL)
        {
        		printf("Error openning ethernet device: %s\n",d_src_device);
        		printf("Fatal Error in pcap_open_live(): %s\n",errbuf);
        	    return false;
        }
      return true;
    }

    bool raw_array_impl::configure_array()
    {
    	//prepare the config data for the ethernet frame
        // note: command=1 -> beamforming config
        //		 command=2 -> start
        //	     command=3 -> stop
        //	     command=4 -> raw array config

    	char data[20];

    	for(int i=0;i<20;i++)
    	{
    		data[i]=0x00;
    	}

    	data[0]=4; //command to activate RAW array
    	data[1]=d_number_of_channels;
    	data[2]=d_snapshots_per_frame>>8;
    	data[3]=d_snapshots_per_frame & 255;

    	printf("\n Total bytes in snapshots payload = %i\n",d_snapshots_per_frame*d_number_of_channels*2);
    	printf("\n Estimated eth RAW frame size [bytes] %i\n",12+2+3+d_snapshots_per_frame*d_number_of_channels*2+1);

    	data[4]=d_inter_frame_delay>>8;
    	data[5]=d_inter_frame_delay & 255;

    	data[6]=0xB;
    	data[7]=0xF;

    	//send the frame
    	struct ether_header myheader;
    	myheader.ether_type=0xbfcd; //this is the ethenet layer II protocol ID for the CTTC array hardware
    	memset(myheader.ether_dhost,0xff,sizeof(myheader.ether_dhost));
    	memset(myheader.ether_shost,0x11,sizeof(myheader.ether_shost));

    	unsigned char frame[sizeof(struct ether_header)+sizeof(data)];
        memcpy(frame,&myheader,sizeof(struct ether_header));
        memcpy(frame+sizeof(struct ether_header),&data,sizeof(data));

        if (pcap_inject(descr,frame,sizeof(frame))==-1) {
        	printf("Error sending configuration packet\n");
            pcap_perror(descr,0);
            return false;
        }else{
        	printf("Sent configuration packet OK\n");
        	return true;
        }
    }

    bool raw_array_impl::start_array()
    {
    	char data[20];

		for(int i=0;i<20;i++)
		{
			data[i]=0x00;
		}

		data[0]=2; //command to start the array operation (configured previously)

		//send the frame
		struct ether_header myheader;
		myheader.ether_type=0xbfcd; //this is the ethenet layer II protocol ID for the CTTC array hardware
		memset(myheader.ether_dhost,0xff,sizeof(myheader.ether_dhost));
		memset(myheader.ether_shost,0x11,sizeof(myheader.ether_shost));

		unsigned char frame[sizeof(struct ether_header)+sizeof(data)];
		memcpy(frame,&myheader,sizeof(struct ether_header));
		memcpy(frame+sizeof(struct ether_header),&data,sizeof(data));

		if (pcap_inject(descr,frame,sizeof(frame))==-1) {
			printf("Error sending start packet\n");
			pcap_perror(descr,0);
			return false;
		}else{
			printf("Sent start packet OK\n");
			return true;
		}
    }

    bool raw_array_impl::stop_array()
    {
    	char data[20];

		for(int i=0;i<20;i++)
		{
			data[i]=0x00;
		}

		data[0]=3; //command to stop the array operation (configured previously)

		//send the frame
		struct ether_header myheader;
		myheader.ether_type=0xbfcd; //this is the ethenet layer II protocol ID for the CTTC array hardware
		memset(myheader.ether_dhost,0xff,sizeof(myheader.ether_dhost));
		memset(myheader.ether_shost,0x11,sizeof(myheader.ether_shost));

		unsigned char frame[sizeof(struct ether_header)+sizeof(data)];
		memcpy(frame,&myheader,sizeof(struct ether_header));
		memcpy(frame+sizeof(struct ether_header),&data,sizeof(data));

		if (pcap_inject(descr,frame,sizeof(frame))==-1) {
			printf("Error sending stop packet\n");
			pcap_perror(descr,0);
			return false;
		}else{
			printf("Sent stop packet OK\n");
			return true;
		}
    }

    /*
     * Our virtual destructor.
     */
    raw_array_impl::~raw_array_impl()
    {
    	  // destructor code here
		   if (stop_array()==true)
		   {
			   printf("Array stopped!\n");
		   }else{
			   exit(1); //ethernet error!
		   }
    		if(descr != NULL)
    		{
    			pcap_breakloop(descr);
    			d_pcap_thread->join();
    			pcap_close(descr);
    		}

     	   for (int i=0;i<DBFCTTC_NUM_CHANNELS;i++)
     	   {
     		   delete[] fifo_buff_ch[i];
     	   }
     	  delete fifo_buff_ch;
    	  fprintf(stdout,"All stopped OK\n");

    }

    void raw_array_impl::static_pcap_callback(u_char *args, const struct pcap_pkthdr* pkthdr,
                                                    const u_char* packet)
       {
   //
       	raw_array_impl *bridge=(raw_array_impl*) args;
       	bridge->pcap_callback(args, pkthdr, packet);
       }

    void raw_array_impl::pcap_callback(u_char *args, const struct pcap_pkthdr* pkthdr,
    		const u_char* packet)
    {
    	boost::mutex::scoped_lock lock(d_mutex); 	// hold mutex for duration of this function
    	int numframebyte;
    	short int real,imag;
    	// eth frame parameters
    	int number_of_channels;
    	unsigned short int snapshots_per_frame;

    	// **** CTTC DBF PACKET DECODER ****
    	if ((packet[12]==0xCD) & (packet[13]==0xBF))
    	{
    		//printf(".");
    		// control parameters
    		number_of_channels=(int)packet[14];
    		//std::cout<<"number_of_channels="<<number_of_channels<<std::endl;
    		snapshots_per_frame=packet[15] << 8 | packet[16];
    		//std::cout<<"snapshots_per_frame="<<snapshots_per_frame<<std::endl;
    		//frame counter check for overflows!
    		numframebyte=(unsigned char)packet[16+snapshots_per_frame*2*number_of_channels+1];
    		//std::cout<<"numframebyte="<<numframebyte<<std::endl;
    		//Overflow detector and mitigator
    		if (d_flag_start_frame == true)
    		{
    			d_last_frame_counter=numframebyte;
    			d_flag_start_frame=false;
    		}else{

    			if ((d_last_frame_counter-numframebyte)>1)
    			{
    				int missing_frames=abs(d_last_frame_counter-numframebyte);
    				if (missing_frames!=255 )
    				{
    					//fake samples generation to help tracking loops
    					std::complex<float> last_sample[DBFCTTC_NUM_CHANNELS];
    					if (fifo_write_ptr == 0)
    					{
								for (int ch=0;ch<number_of_channels;ch++)
								{
									last_sample[ch]=fifo_buff_ch[ch][FIFO_SIZE];
								}
    					}else{
								for (int ch=0;ch<number_of_channels;ch++)
								{
									last_sample[ch]=fifo_buff_ch[ch][fifo_write_ptr-1];
								}
    					}
    					for(int i=0;i<(snapshots_per_frame*missing_frames);i++)
    					{
    						if (fifo_items <= FIFO_SIZE) {
    							for (int ch=0;ch<number_of_channels;ch++)
    							{
    								fifo_buff_ch[ch][fifo_write_ptr] = last_sample[ch];
    							}
    							fifo_write_ptr++;
    							if (fifo_write_ptr == FIFO_SIZE) fifo_write_ptr = 0;
    							fifo_items++;
    							if (d_fifo_full==true)
    							{
    								d_fifo_full=false;
    							}
    						}else{
    							if (d_fifo_full==false)
    							{
    								printf("FIFO full\n");
    								fflush(stdout);
    								d_fifo_full=true;
    							}
    						}
    					}
    					d_num_rx_errors=d_num_rx_errors + 1;
    					printf("RAW Array driver overflow RX %d\n",numframebyte);
    				}
    			}
    		}
    		d_last_frame_counter=numframebyte;

    	};


    	//snapshots reading..
    	for(int i=0;i<snapshots_per_frame;i++)
    	{
    		if (fifo_items <= FIFO_SIZE) {
    			for (int ch=0;ch<number_of_channels;ch++)
    			{
    				if (flag_16_bits_sample==true)
    				{
    					//(2i+2q)*8channels =32 bytes
    					real=(signed short int)(packet[17 + ch*4 + i * 32] << 8 | packet[17 + ch*4 + 1 + i * 32]);
    					imag=(signed short int)(packet[17 + ch*4 + 2 + i * 32] << 8 | packet[17 + ch*4 + 3 + i * 32]);
    				}else{
    					//(1i+1q)*8channels =16 bytes
						real = (signed char)packet[17 + ch*2 + i * 16];
						imag = (signed char)packet[17 + ch*2 + 1 + i * 16];
    				}
    				//todo: invert IQ in FPGA
    				//fifo_buff_ch[ch][fifo_write_ptr] = std::complex<float>(real, imag);
    				fifo_buff_ch[ch][fifo_write_ptr] = std::complex<float>(imag, real); //inverted due to inversion in front-end
    				//std::cout<<"["<<ch<<"]["<<fifo_write_ptr<<"]"<<fifo_buff_ch[ch][fifo_write_ptr]<<std::endl;
    			}
    			fifo_write_ptr++;
    			if (fifo_write_ptr == FIFO_SIZE) fifo_write_ptr = 0;
    			fifo_items++;
    			if (d_fifo_full==true)
    			{
    				d_fifo_full=false;
    			}
    		}else{
    			if (d_fifo_full==false)
    			{
    				printf("FIFO full\n");
    				fflush(stdout);
    				d_fifo_full=true;
    			}
    		}

    	}


    	//test RX

    	// **** CTTC DBF PACKET DECODER ***
    	//else{
    	//std::cout<<"RX PKT ID="<<(int)packet[12]<<","<<(int)packet[13]<<std::endl;
    	//}

    	//           	        // *** END CTTC DBF PACKET DECODER ***
    }


    void raw_array_impl::my_pcap_loop_thread(pcap_t *pcap_handle)

    {

        pcap_loop(pcap_handle, -1, raw_array_impl::static_pcap_callback, (u_char *)this);

    }



    int
    raw_array_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {

        //gr_complex *out = (gr_complex *) output_items[0];
    	  // channel output buffers
    	//  gr_complex *ch1 = (gr_complex *) output_items[0];
    	//  gr_complex *ch2 = (gr_complex *) output_items[1];
    	//  gr_complex *ch3 = (gr_complex *) output_items[2];
    	//  gr_complex *ch4 = (gr_complex *) output_items[3];
    	//  gr_complex *ch5 = (gr_complex *) output_items[4];
    	//  gr_complex *ch6 = (gr_complex *) output_items[5];
    	//  gr_complex *ch7 = (gr_complex *) output_items[6];
    	//  gr_complex *ch8 = (gr_complex *) output_items[7];

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
         	  for (int ch=0;ch<DBFCTTC_NUM_CHANNELS;ch++)
         	  {
         		  //((gr_complex*)output_items[ch])[i]=fifo_buff_ch[ch][fifo_read_ptr];
         		  memcpy(&((gr_complex*)output_items[ch])[0],&fifo_buff_ch[ch][fifo_read_ptr],sizeof(std::complex<float> )*num_samples_readed);
         	  }
         	  fifo_read_ptr=fifo_read_ptr+num_samples_readed; //increase the fifo pointer
         	  if (fifo_read_ptr==FIFO_SIZE) fifo_read_ptr=0;
     	  }else{
     		  //two step wrap read
         	  for (int ch=0;ch<DBFCTTC_NUM_CHANNELS;ch++)
         	  {
         		  //((gr_complex*)output_items[ch])[i]=fifo_buff_ch[ch][fifo_read_ptr];
         		  memcpy(&((gr_complex*)output_items[ch])[0],&fifo_buff_ch[ch][fifo_read_ptr],sizeof(std::complex<float> )*aligned_read_items);
         	  }
         	  fifo_read_ptr=fifo_read_ptr+aligned_read_items; //increase the fifo pointer

         	  if (fifo_read_ptr==FIFO_SIZE) fifo_read_ptr=0;

         	  for (int ch=0;ch<DBFCTTC_NUM_CHANNELS;ch++)
         	  {
         		  //((gr_complex*)output_items[ch])[i]=fifo_buff_ch[ch][fifo_read_ptr];
         		  memcpy(&((gr_complex*)output_items[ch])[aligned_read_items],&fifo_buff_ch[ch][fifo_read_ptr],sizeof(std::complex<float>)*(num_samples_readed-aligned_read_items));
         	  }
         	  fifo_read_ptr=fifo_read_ptr+(num_samples_readed-aligned_read_items); //increase the fifo pointer
     	  }

     	  fifo_items=fifo_items-num_samples_readed;


//          int num_samples_readed=0;
//    	      for(int i=0;i<noutput_items;i++)
//    	  {
//
//    	          if (fifo_items > 0) {
//    	        	  //TODO: optimize-me with memcpy!!
//    	        	  for (int ch=0;ch<DBFCTTC_NUM_CHANNELS;ch++)
//    	        	  {
//    	        		  ((gr_complex*)output_items[ch])[i]=fifo_buff_ch[ch][fifo_read_ptr];
//    	        	  }
//    					  fifo_read_ptr++;
//    					  if (fifo_read_ptr==FIFO_SIZE) fifo_read_ptr=0;
//    					  fifo_items--;
//    					  num_samples_readed++;
//    	          } else {
//    	              break;
//    	          }
//
//    	       }

        // Tell runtime system how many output items we produced.
        return num_samples_readed;
    }

  } /* namespace dbfcttc */
} /* namespace gr */

