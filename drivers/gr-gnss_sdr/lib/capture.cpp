/*!
 * \file capture.cpp
 * \brief GNSS-SDR Hacker's Edition Board Driver Interface.
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
#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include "capture.hpp"
#include "max2769.h"

using namespace std;

Capture_GNSS_SDR_HE::Capture_GNSS_SDR_HE()
{
  /* Default values for configuration registers */
  (this->conf1).reg_value   = 0xA2919A30;
  (this->conf2).reg_value   = 0x05502881;
  (this->conf3).reg_value   = 0xEAFF1DC2;
  (this->pllconf).reg_value = 0x9EC00083;
  (this->divide).reg_value  = 0x0C000804;
  (this->fdiv).reg_value    = 0x80000705;
  (this->strm).reg_value    = 0x80000006;
  (this->clk).reg_value     = 0x10061B27;
  (this->test1).reg_value   = 0x1E0F4018;
  (this->test2).reg_value   = 0x14C04029;

}

Capture_GNSS_SDR_HE::~Capture_GNSS_SDR_HE()
{
  stop_capture();
}

int Capture_GNSS_SDR_HE::GNSS_SDR_callback( uint8_t* buffer, 
                            int length, 
                            FTDIProgressInfo* progress, 
                            void* cgsh_obj)
{
    Capture_GNSS_SDR_HE* me = (Capture_GNSS_SDR_HE*)cgsh_obj;
    SamplePacket* tmp;

    tmp = new SamplePacket;
    memset(tmp->samples, 0, 512 * sizeof(uint8_t));
    memcpy(tmp->samples, buffer, length * sizeof(uint8_t));
    tmp->length = length;
    
    while (!me->sample_fifo_queue.push(tmp));

    return me->exit_capture_thread;
}

void Capture_GNSS_SDR_HE::capture_thread_function()
{
  exit_capture_thread = 0;
  ftdi_readstream(ftdi_capture_context, 
                  GNSS_SDR_callback, 
                  this, 
                  8, 
                  256);
}

int Capture_GNSS_SDR_HE::start_capture()
{
  boost::thread capture_thread; 

 /* Initialize the FTDI Library */
  this->ftdi_capture_context = ftdi_new();
  if (this->ftdi_capture_context == NULL)
  {
    std::cerr << "Error initializing ftdi library. Exitting...." << std::endl;
    return -1;
  }

  /* Set the interface to read from */
  if (ftdi_set_interface(this->ftdi_capture_context, INTERFACE_A) < 0)
  {
    std::cerr << "Error setting interface. Exitting..." << std::endl;
    ftdi_free(this->ftdi_capture_context);
    return -1;
  }

  /* Open the USB Descriptor  */
  if (ftdi_usb_open_desc(this->ftdi_capture_context, VENDOR_ID, PRODUCT_ID, NULL, NULL) < 0)
  {
    std::cerr << "Error opening USB device : " 
              << ftdi_get_error_string(this->ftdi_capture_context)
              << endl;
    ftdi_free(this->ftdi_capture_context);
    return -1;
  }

  /* Set the latency timer */
  if (ftdi_set_latency_timer(this->ftdi_capture_context, 2))
  {
    std::cerr << "Error setting latency : " 
              << ftdi_get_error_string(this->ftdi_capture_context)
              << std::endl;

    ftdi_usb_close(this->ftdi_capture_context);
    ftdi_free(this->ftdi_capture_context);
    return -1;
  }

  /* Initialize Sample Packet to NULL */
  current_sample_packet = NULL;

  capture_thread = boost::thread(&Capture_GNSS_SDR_HE::capture_thread_function, this);

  return 0;
}

void Capture_GNSS_SDR_HE::stop_capture()
{
  exit_capture_thread = 1;
}

bool Capture_GNSS_SDR_HE::pop_samples(SamplePacket** pkt)
{
  while(!sample_fifo_queue.pop(*pkt));
  return true;
}

int Capture_GNSS_SDR_HE::pop_samples(signed char** buffer, int length)
{
  int num_bytes_written = 0;

  memset(*buffer, 0, length * sizeof(signed char));

  while (num_bytes_written < length)
  {
    if (current_sample_packet == NULL)
    {
        while(!sample_fifo_queue.pop(current_sample_packet));
    }

    if (current_sample_packet->length > 0)
    {
      if ((length - num_bytes_written) < current_sample_packet->length)
      {
        memcpy(&((*buffer)[num_bytes_written]), 
               current_sample_packet->samples, 
               (length - num_bytes_written) * sizeof(signed char));

        memmove(current_sample_packet->samples, 
                current_sample_packet->samples + (length - num_bytes_written), 
                current_sample_packet->length - (length - num_bytes_written));
        current_sample_packet->length -= (length - num_bytes_written);
        num_bytes_written += (length - num_bytes_written);
      }
      else
      {
        memcpy(&((*buffer)[num_bytes_written]), 
               current_sample_packet->samples, 
               current_sample_packet->length * sizeof(signed char));
        num_bytes_written += current_sample_packet->length;
        delete current_sample_packet;
        current_sample_packet = NULL;
      }
    }
    else
    {
      current_sample_packet = NULL;
    }
  }
  return num_bytes_written;
}


/* Setter routines for CONF1 */
void Capture_GNSS_SDR_HE::set_CHIPEN(unsigned int value)
{
    (this->conf1).parameters.CHIPEN = value;
}

void Capture_GNSS_SDR_HE::set_IDLE(unsigned int value)
{
    (this->conf1).parameters.IDLE = value;
}

void Capture_GNSS_SDR_HE::set_ILNA1(unsigned int value)
{
    (this->conf1).parameters.ILNA1 = value;
}

void Capture_GNSS_SDR_HE::set_ILNA2(unsigned int value)
{
    (this->conf1).parameters.ILNA2 = value;
}

void Capture_GNSS_SDR_HE::set_ILO(unsigned int value)
{
    (this->conf1).parameters.ILO = value;
}

void Capture_GNSS_SDR_HE::set_IMIX(unsigned int value)
{
    (this->conf1).parameters.IMIX = value;
}

void Capture_GNSS_SDR_HE::set_MIXPOLE(unsigned int value)
{
    (this->conf1).parameters.MIXPOLE = value;
}

void Capture_GNSS_SDR_HE::set_LNAMODE(unsigned int value)
{
    (this->conf1).parameters.LNAMODE = value;
}

void Capture_GNSS_SDR_HE::set_MIXEN(unsigned int value)
{
    (this->conf1).parameters.MIXEN = value;
}

void Capture_GNSS_SDR_HE::set_ANTEN(unsigned int value)
{
    (this->conf1).parameters.ANTEN = value;
}

void Capture_GNSS_SDR_HE::set_FCEN(unsigned int value)
{
    (this->conf1).parameters.FCEN = value;
}

void Capture_GNSS_SDR_HE::set_FBW(unsigned int value)
{
    (this->conf1).parameters.FBW = value;
}

void Capture_GNSS_SDR_HE::set_F3OR5(unsigned int value)
{
    (this->conf1).parameters.F3OR5 = value;
}

void Capture_GNSS_SDR_HE::set_FCENX(unsigned int value)
{
    (this->conf1).parameters.FCENX = value;
}

void Capture_GNSS_SDR_HE::set_FGAIN(unsigned int value)
{
    (this->conf1).parameters.FGAIN = value;
}

/* Setter routines for CONF2 */
void Capture_GNSS_SDR_HE::set_IQEN(unsigned int value)
{
    (this->conf2).parameters.IQEN = value;
}

void Capture_GNSS_SDR_HE::set_GAINREF(unsigned int value)
{
    (this->conf2).parameters.GAINREF = value;
}

void Capture_GNSS_SDR_HE::set_AGCMODE(unsigned int value)
{
    (this->conf2).parameters.AGCMODE = value;
}

void Capture_GNSS_SDR_HE::set_FORMAT(unsigned int value)
{
    (this->conf2).parameters.FORMAT = value;
}

void Capture_GNSS_SDR_HE::set_BITS(unsigned int value)
{
    (this->conf2).parameters.BITS = value;
}

void Capture_GNSS_SDR_HE::set_DRVCFG(unsigned int value)
{
    (this->conf2).parameters.DRVCFG = value;
}

void Capture_GNSS_SDR_HE::set_LOEN(unsigned int value)
{
    (this->conf2).parameters.LOEN = value;
}

void Capture_GNSS_SDR_HE::set_DIEID(unsigned int value)
{
    (this->conf2).parameters.DIEID = value;
}

/* Setter routines for CONF3 */
void Capture_GNSS_SDR_HE::set_GAININ(unsigned int value)
{
    (this->conf3).parameters.GAININ = value;
}

void Capture_GNSS_SDR_HE::set_FSLOWEN(unsigned int value)
{
    (this->conf3).parameters.FSLOWEN = value;
}

void Capture_GNSS_SDR_HE::set_HILOADEN(unsigned int value)
{
    (this->conf3).parameters.HILOADEN = value;
}

void Capture_GNSS_SDR_HE::set_ADCEN(unsigned int value)
{
    (this->conf3).parameters.ADCEN = value;
}

void Capture_GNSS_SDR_HE::set_DRVEN(unsigned int value)
{
    (this->conf3).parameters.DRVEN = value;
}

void Capture_GNSS_SDR_HE::set_FOFSTEN(unsigned int value)
{
    (this->conf3).parameters.FOFSTEN = value;
}

void Capture_GNSS_SDR_HE::set_FILTEN(unsigned int value)
{
    (this->conf3).parameters.FILTEN = value;
}

void Capture_GNSS_SDR_HE::set_FHIPEN(unsigned int value)
{
    (this->conf3).parameters.FHIPEN = value;
}

void Capture_GNSS_SDR_HE::set_PGAIEN(unsigned int value)
{
    (this->conf3).parameters.PGAIEN = value;
}

void Capture_GNSS_SDR_HE::set_PGAQEN(unsigned int value)
{
    (this->conf3).parameters.PGAQEN = value;
}

void Capture_GNSS_SDR_HE::set_STRMEN(unsigned int value)
{
    (this->conf3).parameters.STRMEN = value;
}

void Capture_GNSS_SDR_HE::set_STRMSTART(unsigned int value)
{
    (this->conf3).parameters.STRMSTART = value;
}

void Capture_GNSS_SDR_HE::set_STRMSTOP(unsigned int value)
{
    (this->conf3).parameters.STRMSTOP = value;
}

void Capture_GNSS_SDR_HE::set_STRMCOUNT(unsigned int value)
{
    (this->conf3).parameters.STRMCOUNT = value;
}

void Capture_GNSS_SDR_HE::set_STRMBITS(unsigned int value)
{
    (this->conf3).parameters.STRMBITS = value;
}

void Capture_GNSS_SDR_HE::set_STAMPEN(unsigned int value)
{
    (this->conf3).parameters.STAMPEN = value;
}

void Capture_GNSS_SDR_HE::set_TIMESYNCEN(unsigned int value)
{
    (this->conf3).parameters.TIMESYNCEN = value;
}

void Capture_GNSS_SDR_HE::set_DATSYNCEN(unsigned int value)
{
    (this->conf3).parameters.DATSYNCEN = value;
}

void Capture_GNSS_SDR_HE::set_STRMRST(unsigned int value)
{
    (this->conf3).parameters.STRMRST = value;
}

/* Setter routines for PLLCONF */
void Capture_GNSS_SDR_HE::set_VCOEN(unsigned int value)
{
    (this->pllconf).parameters.VCOEN = value;
}

void Capture_GNSS_SDR_HE::set_IVCO(unsigned int value)
{
    (this->pllconf).parameters.IVCO = value;
}

void Capture_GNSS_SDR_HE::set_REFOUTEN(unsigned int value)
{
    (this->pllconf).parameters.REFOUTEN = value;
}

void Capture_GNSS_SDR_HE::set_REFDIV(unsigned int value)
{
    (this->pllconf).parameters.REFDIV = value;
}

void Capture_GNSS_SDR_HE::set_IXTAL(unsigned int value)
{
    (this->pllconf).parameters.IXTAL = value;
}

void Capture_GNSS_SDR_HE::set_XTALCAP(unsigned int value)
{
    (this->pllconf).parameters.XTALCAP = value;
}

void Capture_GNSS_SDR_HE::set_LDMUX(unsigned int value)
{
    (this->pllconf).parameters.LDMUX = value;
}

void Capture_GNSS_SDR_HE::set_ICP(unsigned int value)
{
    (this->pllconf).parameters.ICP = value;
}

void Capture_GNSS_SDR_HE::set_PFDEN(unsigned int value)
{
    (this->pllconf).parameters.PFDEN = value;
}

void Capture_GNSS_SDR_HE::set_CPTEST(unsigned int value)
{
    (this->pllconf).parameters.CPTEST = value;
}

void Capture_GNSS_SDR_HE::set_INT_PLL(unsigned int value)
{
    (this->pllconf).parameters.INT_PLL = value;
}

void Capture_GNSS_SDR_HE::set_PWRSAV(unsigned int value)
{
    (this->pllconf).parameters.PWRSAV = value;
}

/* Setter routines for DIV */
void Capture_GNSS_SDR_HE::set_NDIV(unsigned int value)
{
    (this->divide).parameters.NDIV = value;
}

void Capture_GNSS_SDR_HE::set_RDIV(unsigned int value)
{
    (this->divide).parameters.RDIV = value;
}

/* Setter routines for FDIV */
void Capture_GNSS_SDR_HE::set_FDIV(unsigned int value)
{
    (this->fdiv).parameters.FDIV = value;
}

/* Setter routines for STRM */
void Capture_GNSS_SDR_HE::set_FRAMECOUNT(unsigned int value)
{
    (this->strm).parameters.FRAMECOUNT = value;
}

/* Setter routines for CLK */
void Capture_GNSS_SDR_HE::set_L_CNT(unsigned int value)
{
    (this->clk).parameters.L_CNT = value;
}

void Capture_GNSS_SDR_HE::set_M_CNT(unsigned int value)
{
    (this->clk).parameters.M_CNT = value;
}

void Capture_GNSS_SDR_HE::set_FCLKIN(unsigned int value)
{
    (this->clk).parameters.FCLKIN = value;
}

void Capture_GNSS_SDR_HE::set_ADCCLK(unsigned int value)
{
    (this->clk).parameters.ADCCLK = value;
}

void Capture_GNSS_SDR_HE::set_SERCLK(unsigned int value)
{
    (this->clk).parameters.SERCLK = value;
}

void Capture_GNSS_SDR_HE::set_MODE(unsigned int value)
{
    (this->clk).parameters.MODE = value;
}

/* Routines to access the FT2232H and Program the MAX2769 Registers */
void Capture_GNSS_SDR_HE::max2769_write_pin(unsigned char pin, 
                                            unsigned char val)
{
  unsigned char buf[1];
  ftdi_read_pins(this->ftdi_config_context, &(buf[0]));
  if (val) buf[0] |= (unsigned char)(1 << pin);
      else buf[0] &= ~(unsigned char)(1 << pin);

  ftdi_write_data(this->ftdi_config_context, buf, 1);
}

void Capture_GNSS_SDR_HE::max2769_write_byte(unsigned char b)
{
  int i;

  for (i = 7 ; i >= 0; i--)
  {
      if (b & (1 << i)) 
          max2769_write_pin(MAX2769_TWI_DAT, 1); 
      else 
          max2769_write_pin(MAX2769_TWI_DAT, 0);

      /* Toggle clock for a minimum of 1 usec */
      max2769_write_pin(MAX2769_TWI_CLK, 1);
      usleep(MAX2769_CLK_DELAY_US);
      max2769_write_pin(MAX2769_TWI_CLK, 0);
      usleep(MAX2769_CLK_DELAY_US);
  }
}

void Capture_GNSS_SDR_HE::program_register(unsigned int register_value)
{
  max2769_write_pin(MAX2769_TWI_CS, 0);        /* Pull Chip Select LOW */
  max2769_write_byte((register_value >> 24) & 0xFF);  /* MSB */
  max2769_write_byte((register_value >> 16) & 0xFF);
  max2769_write_byte((register_value >>  8) & 0xFF);
  max2769_write_byte((register_value      ) & 0xFF);  /* LSB */
  max2769_write_pin(MAX2769_TWI_CS, 1);
}


int Capture_GNSS_SDR_HE::program_registers()
{

  /* Open the FTDI interface for Two Wire Interface */
  this->ftdi_config_context = ftdi_new();
  if (this->ftdi_config_context == NULL)
  {
      std::cerr << "Error initializing ftdi library. Exitting...." << std::endl;
      return -1;
  }
  
  /* Set the interface to read from */
  if (ftdi_set_interface(this->ftdi_config_context, INTERFACE_B) < 0)
  {
      std::cerr << "Error setting interface. Exitting..." << std::endl;
      ftdi_free(this->ftdi_config_context);
      return -1;
  }

  /* Open the USB Descriptor  */
  if (ftdi_usb_open_desc(this->ftdi_config_context, VENDOR_ID, PRODUCT_ID, NULL, NULL) < 0)
  {
      std::cerr << "Error opening USB device : "
           << ftdi_get_error_string(this->ftdi_config_context)
           << std::endl;
      ftdi_free(this->ftdi_config_context);
      return -1;
  }

  /* Set the FTDI Interface to a bit bang mode */
  ftdi_set_bitmode(this->ftdi_config_context, 0xFF, BITMODE_BITBANG);  

  max2769_write_pin(MAX2769_TWI_CS, 1);       /* Pull Chip Select HIGH */
  max2769_write_pin(MAX2769_TWI_CLK, 0);      /* Clock is default LOW */
  usleep(MAX2769_CLK_DELAY_US);       /* Hold onto that for a micro second */

  /* Program registers */
  program_register((this->conf1).reg_value);
  program_register((this->conf2).reg_value);
  program_register((this->conf3).reg_value);
  program_register((this->pllconf).reg_value);
  program_register((this->divide).reg_value);
  program_register((this->fdiv).reg_value);
  program_register((this->strm).reg_value);
  program_register((this->clk).reg_value);

  /* Close the two-wire interface */
  ftdi_usb_close(this->ftdi_config_context);
  ftdi_free(this->ftdi_config_context);

  return 0;
}
