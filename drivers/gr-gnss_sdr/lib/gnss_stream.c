/*!
 * \file gnss_sdr_source_b.h
 * \brief GNSS-SDR Hacker's Edition Board Driver.
 * \author Ajith Peter, Google Summer of Code 2014-15, ajith.peter(at)gmail.com
 *         Javier Arribas, 2014-15 jarribas(at)cttc.es
 *         Carles Fernandez Prades, 2014 carles.fernandez (at) cttc.cat
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ftdi.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <errno.h>

#include "gnss_stream.h"
#include "max2769.h"

void add_packet(struct gnss_device_context* dev, 
                signed char* data, 
                unsigned int len)
{
  data_packet* tmp;
  data_packet* new_packet;

  new_packet = (data_packet*)malloc(1 * sizeof(data_packet));
  new_packet->data = (signed char*)malloc(len * sizeof(signed char));
  memcpy(new_packet->data, data, len * sizeof(signed char));
  new_packet->len = len;
  new_packet->next = NULL;

  pthread_mutex_lock(&(dev->thread_lock));
  if (dev->packet_list_head == NULL)
  {
    dev->packet_list_head = new_packet;
  }
  else
  {
    tmp = dev->packet_list_head;
    while (tmp->next != NULL) tmp = tmp->next;
    tmp->next = new_packet;
  }
  pthread_mutex_unlock(&(dev->thread_lock));
}

int get_next_packet_data(struct gnss_device_context* dev, 
                      signed char** buffer, 
                      int nitems)
{
  data_packet* current_packet;
  unsigned int nitems_read;
  signed char* buf_ptr;

  nitems_read = 0;

  if (dev->packet_list_head == NULL) return 0;
  if (nitems <= 0) return 0;

  memset(*buffer, 0, nitems * sizeof(signed char));
  buf_ptr = *buffer;

  pthread_mutex_lock(&(dev->thread_lock));

  while ((nitems - (nitems_read)  > 0) && dev->packet_list_head != NULL)
  {
    if (dev->packet_list_head->len > (nitems - (nitems_read)))
    {
      current_packet = dev->packet_list_head;
      memcpy( buf_ptr, 
              current_packet->data, 
              (nitems - nitems_read) * sizeof(signed char));
      buf_ptr += (nitems - nitems_read);
      memmove(current_packet->data, 
              current_packet->data + (nitems - nitems_read), 
              (current_packet->len - (nitems - nitems_read)) * 
              sizeof(signed char));
      current_packet->len -= (nitems - nitems_read);
      nitems_read += (nitems - nitems_read);
      break;
    }
    else
    {
      current_packet = dev->packet_list_head;
      memcpy( buf_ptr, 
              current_packet->data, 
              current_packet->len * sizeof(signed char));
      buf_ptr += current_packet->len * sizeof(signed char);
      nitems_read += current_packet->len * sizeof(signed char);
      dev->packet_list_head = dev->packet_list_head->next;
      free(current_packet->data);
      free(current_packet);
    }
  }
  pthread_mutex_unlock(&(dev->thread_lock));

  return nitems_read;
}

/* Routines to access the FT2232H and Program the MAX2769 Registers */
void max2769_write_pin( struct gnss_device_context* dev,
                        unsigned char pin, 
                        unsigned char val)
{
  unsigned char buf[1];
  ftdi_read_pins(dev->ftdi_ctx, &(buf[0]));
  if (val) buf[0] |= (unsigned char)(1 << pin);
      else buf[0] &= ~(unsigned char)(1 << pin);

  ftdi_write_data(dev->ftdi_ctx, buf, 1);
}

void max2769_write_byte(struct gnss_device_context* dev, unsigned char b)
{
  int i;

  for (i = 7 ; i >= 0; i--)
  {
      if (b & (1 << i)) 
          max2769_write_pin(dev, MAX2769_TWI_DAT, 1); 
      else 
          max2769_write_pin(dev, MAX2769_TWI_DAT, 0);

      /* Toggle clock for a minimum of 1 usec */
      max2769_write_pin(dev, MAX2769_TWI_CLK, 1);
      usleep(MAX2769_CLK_DELAY_US);
      max2769_write_pin(dev, MAX2769_TWI_CLK, 0);
      usleep(MAX2769_CLK_DELAY_US);
  }
}

void program_register(struct gnss_device_context* dev, 
                      unsigned int register_value)
{
  fprintf(stdout, "Programming Register : 0x%.8X\n", register_value);
  fflush(stdout);
  max2769_write_pin(dev, MAX2769_TWI_CS, 0);        /* Pull Chip Select LOW */
  max2769_write_byte(dev, (register_value >> 24) & 0xFF);  /* MSB */
  max2769_write_byte(dev, (register_value >> 16) & 0xFF);
  max2769_write_byte(dev, (register_value >>  8) & 0xFF);
  max2769_write_byte(dev, (register_value      ) & 0xFF);  /* LSB */
  max2769_write_pin(dev, MAX2769_TWI_CS, 1);
}

/* Setter routines for CONF1 */
void set_CHIPEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.CHIPEN = value;
}

void set_IDLE(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.IDLE = value;
}

void set_ILNA1(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.ILNA1 = value;
}

void set_ILNA2(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.ILNA2 = value;
}

void set_ILO(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.ILO = value;
}

void set_IMIX(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.IMIX = value;
}

void set_MIXPOLE(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.MIXPOLE = value;
}

void set_LNAMODE(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.LNAMODE = value;
}

void set_MIXEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.MIXEN = value;
}

void set_ANTEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.ANTEN = value;
}

void set_FCEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.FCEN = value;
}

void set_FBW(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.FBW = value;
}

void set_F3OR5(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.F3OR5 = value;
}

void set_FCENX(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.FCENX = value;
}

void set_FGAIN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf1).parameters.FGAIN = value;
}

/* Setter routines for CONF2 */
void set_IQEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf2).parameters.IQEN = value;
}

void set_GAINREF(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf2).parameters.GAINREF = value;
}

void set_AGCMODE(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf2).parameters.AGCMODE = value;
}

void set_FORMAT(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf2).parameters.FORMAT = value;
}

void set_BITS(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf2).parameters.BITS = value;
}

void set_DRVCFG(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf2).parameters.DRVCFG = value;
}

void set_LOEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf2).parameters.LOEN = value;
}

void set_DIEID(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf2).parameters.DIEID = value;
}

/* Setter routines for CONF3 */
void set_GAININ(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.GAININ = value;
}

void set_FSLOWEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.FSLOWEN = value;
}

void set_HILOADEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.HILOADEN = value;
}

void set_ADCEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.ADCEN = value;
}

void set_DRVEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.DRVEN = value;
}

void set_FOFSTEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.FOFSTEN = value;
}

void set_FILTEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.FILTEN = value;
}

void set_FHIPEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.FHIPEN = value;
}

void set_PGAIEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.PGAIEN = value;
}

void set_PGAQEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.PGAQEN = value;
}

void set_STRMEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.STRMEN = value;
}

void set_STRMSTART(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.STRMSTART = value;
}

void set_STRMSTOP(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.STRMSTOP = value;
}

void set_STRMCOUNT(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.STRMCOUNT = value;
}

void set_STRMBITS(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.STRMBITS = value;
}

void set_STAMPEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.STAMPEN = value;
}

void set_TIMESYNCEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.TIMESYNCEN = value;
}

void set_DATSYNCEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.DATSYNCEN = value;
}

void set_STRMRST(struct gnss_device_context* dev, unsigned int value)
{
    (dev->conf3).parameters.STRMRST = value;
}

/* Setter routines for PLLCONF */
void set_VCOEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.VCOEN = value;
}

void set_IVCO(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.IVCO = value;
}

void set_REFOUTEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.REFOUTEN = value;
}

void set_REFDIV(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.REFDIV = value;
}

void set_IXTAL(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.IXTAL = value;
}

void set_XTALCAP(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.XTALCAP = value;
}

void set_LDMUX(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.LDMUX = value;
}

void set_ICP(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.ICP = value;
}

void set_PFDEN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.PFDEN = value;
}

void set_CPTEST(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.CPTEST = value;
}

void set_INT_PLL(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.INT_PLL = value;
}

void set_PWRSAV(struct gnss_device_context* dev, unsigned int value)
{
    (dev->pllconf).parameters.PWRSAV = value;
}

/* Setter routines for DIV */
void set_NDIV(struct gnss_device_context* dev, unsigned int value)
{
    (dev->divide).parameters.NDIV = value;
}

void set_RDIV(struct gnss_device_context* dev, unsigned int value)
{
    (dev->divide).parameters.RDIV = value;
}

/* Setter routines for FDIV */
void set_FDIV(struct gnss_device_context* dev, unsigned int value)
{
    (dev->fdiv).parameters.FDIV = value;
}

/* Setter routines for STRM */
void set_FRAMECOUNT(struct gnss_device_context* dev, unsigned int value)
{
    (dev->strm).parameters.FRAMECOUNT = value;
}

/* Setter routines for CLK */
void set_L_CNT(struct gnss_device_context* dev, unsigned int value)
{
    (dev->clk).parameters.L_CNT = value;
}

void set_M_CNT(struct gnss_device_context* dev, unsigned int value)
{
    (dev->clk).parameters.M_CNT = value;
}

void set_FCLKIN(struct gnss_device_context* dev, unsigned int value)
{
    (dev->clk).parameters.FCLKIN = value;
}

void set_ADCCLK(struct gnss_device_context* dev, unsigned int value)
{
    (dev->clk).parameters.ADCCLK = value;
}

void set_SERCLK(struct gnss_device_context* dev, unsigned int value)
{
    (dev->clk).parameters.SERCLK = value;
}

void set_MODE(struct gnss_device_context* dev, unsigned int value)
{
    (dev->clk).parameters.MODE = value;
}

int program_registers(struct gnss_device_context* dev)
{
  /* Open the FTDI interface for Two Wire Interface */
  dev->ftdi_ctx = ftdi_new();
  if (dev->ftdi_ctx == NULL)
  {
      fprintf(stderr, "Error initializing ftdi library. Exitting....\n");
      return -1;
  }
  
  /* Set the interface to read from */
  if (ftdi_set_interface(dev->ftdi_ctx, INTERFACE_B) < 0)
  {
      fprintf(stderr, "Error setting interface. Exitting...\n");
      ftdi_free(dev->ftdi_ctx);
      return -1;
  }

  /* Open the USB Descriptor  */
  if (ftdi_usb_open_desc(dev->ftdi_ctx, VENDOR_ID, PRODUCT_ID, NULL, NULL) < 0)
  {
      fprintf(stderr, "Error opening USB device : %s\n", 
              ftdi_get_error_string(dev->ftdi_ctx));
      ftdi_free(dev->ftdi_ctx);
      return -1;
  }

  /* Set the FTDI Interface to a bit bang mode */
  ftdi_set_bitmode(dev->ftdi_ctx, 0xFF, BITMODE_BITBANG);  

  max2769_write_pin(dev, MAX2769_TWI_CS, 1);       /* Pull Chip Select HIGH */
  max2769_write_pin(dev, MAX2769_TWI_CLK, 0);      /* Clock is default LOW */
  usleep(MAX2769_CLK_DELAY_US);       /* Hold onto that for a micro second */

  /* Program registers */
  program_register(dev, (dev->conf1).reg_value);
  program_register(dev, (dev->conf2).reg_value);
  program_register(dev, (dev->conf3).reg_value);
  program_register(dev, (dev->pllconf).reg_value);
  program_register(dev, (dev->divide).reg_value);
  program_register(dev, (dev->fdiv).reg_value);
  program_register(dev, (dev->strm).reg_value);
  program_register(dev, (dev->clk).reg_value);

  /* Close the two-wire interface */
  ftdi_usb_close(dev->ftdi_ctx);
  ftdi_free(dev->ftdi_ctx);

  return 0;
}

void init_gnss_device_context(struct gnss_device_context* dev)
{
  dev->packet_list_head = NULL;
  dev->exit_thread = 0;
  pthread_mutex_init(&(dev->thread_lock), NULL);

  /* Default values for configuration registers */
  (dev->conf1).reg_value   = 0xA2919A30;
  (dev->conf2).reg_value   = 0x05502881;
  (dev->conf3).reg_value   = 0xEAFF1DC2;
  (dev->pllconf).reg_value = 0x9EC00083;
  (dev->divide).reg_value  = 0x0C000804;
  (dev->fdiv).reg_value    = 0x80000705;
  (dev->strm).reg_value    = 0x80000006;
  (dev->clk).reg_value     = 0x10061B27;
  (dev->test1).reg_value   = 0x1E0F4018;
  (dev->test2).reg_value   = 0x14C04029;

}

int open_GNSS_SDR(struct gnss_device_context* dev)
{
  /* Initialize the FTDI Library */
  dev->ftdi_ctx = ftdi_new();
  if (dev->ftdi_ctx == NULL)
  {
    fprintf(stderr, "Error initializing ftdi library. Exitting....\n");
    return -1;
  }

  /* Set the interface to read from */
  if (ftdi_set_interface(dev->ftdi_ctx, INTERFACE_A) < 0)
  {
    fprintf(stderr, "Error setting interface. Exitting...\n");
    ftdi_free(dev->ftdi_ctx);
    return -1;
  }

  /* Open the USB Descriptor  */
  if (ftdi_usb_open_desc(dev->ftdi_ctx, VENDOR_ID, PRODUCT_ID, NULL, NULL) < 0)
  {
    fprintf(stderr, 
            "Error opening USB device : %s\n", 
            ftdi_get_error_string(dev->ftdi_ctx));
    ftdi_free(dev->ftdi_ctx);
    return -1;
  }

  /* Set the latency timer */
  if (ftdi_set_latency_timer(dev->ftdi_ctx, 2))
  {
    fprintf(stderr, 
            "Error setting latency : %s\n", 
            ftdi_get_error_string(dev->ftdi_ctx));
    ftdi_usb_close(dev->ftdi_ctx);
    ftdi_free(dev->ftdi_ctx);
    return -1;
  }

  return 0;
}

void close_GNSS_SDR(struct gnss_device_context* dev)
{
  ftdi_usb_close(dev->ftdi_ctx);
  ftdi_free(dev->ftdi_ctx);
}

static int GNSS_SDR_callback( uint8_t* buffer, 
                            int length, 
                            FTDIProgressInfo* progress, 
                            void* device_context)
{
  struct gnss_device_context* dev = (struct gnss_device_context*)device_context;

  add_packet(dev, (signed char*)buffer, length);

  return dev->exit_thread;
}

static void* capture_thread_function(void* device_context)
{
  data_packet* tmp;
  struct gnss_device_context* dev = (struct gnss_device_context*)device_context;
  dev->exit_thread = 0;
  struct sched_param priority_param;

  int min_real_pri = sched_get_priority_min(SCHED_FIFO);
  int max_real_pri = sched_get_priority_max(SCHED_FIFO);
  int thread_perm_retval = 0;

  memset (&priority_param, 0, sizeof (priority_param));
  priority_param.sched_priority = max_real_pri;

  fprintf(stdout, "Min thread priority: %d, Max thread priority: %d\n", min_real_pri, max_real_pri);
  fflush(stdout);
  /* Set thread priority (Experimental) */
  thread_perm_retval = pthread_setschedparam(dev->thread_id, SCHED_FIFO, &priority_param);
  if (thread_perm_retval == EINVAL)
  {
    fprintf(stderr, "EINVAL returned from pthread_setschedparam : System does not support SCHED_FIFO.");
    fflush(stderr);
  }
  else if (thread_perm_retval == EPERM)
  {
    fprintf(stderr, "EPERM returned from pthread_setschedparam : No sufficient privilleges to shift thread priorities.");
    fflush(stderr);
  }
  else if (thread_perm_retval == 0)
  {
    fprintf(stdout, "Thread Priority Changed.");
    fflush(stdout);
  }
  

  pthread_mutex_lock(&(dev->thread_lock));
  if (dev->packet_list_head != NULL)
  {
    while (dev->packet_list_head->next != NULL)
    {
      tmp = dev->packet_list_head;
      dev->packet_list_head = dev->packet_list_head->next;
      free(tmp->data);
      free(tmp);
    }
    free(dev->packet_list_head);
  }

  dev->packet_list_head = NULL;
  pthread_mutex_unlock(&(dev->thread_lock));

  ftdi_readstream(dev->ftdi_ctx, GNSS_SDR_callback, device_context, 8, 256);
  pthread_exit(NULL);
}

void start_capture(struct gnss_device_context* dev)
{
  if (pthread_create(&(dev->thread_id), NULL, capture_thread_function, (void*)dev))
  {
    fprintf(stderr, "Error spawning capture thread. exitting...\n");
    close_GNSS_SDR(dev);
  }
}

void stop_capture(struct gnss_device_context* dev)
{
  data_packet* tmp;

  fprintf(stdout, "Stopping GNSS Sample Capture....\n");
  fflush(stdout);

  pthread_mutex_lock(&(dev->thread_lock));
  if (dev->packet_list_head != NULL)
  {
    while (dev->packet_list_head->next != NULL)
    {
      tmp = dev->packet_list_head;
      dev->packet_list_head = dev->packet_list_head->next;
      free(tmp->data);
      free(tmp);
    }
    free(dev->packet_list_head);
  }

  dev->packet_list_head = NULL;
  pthread_mutex_unlock(&(dev->thread_lock));

  dev->exit_thread = 1;
}
