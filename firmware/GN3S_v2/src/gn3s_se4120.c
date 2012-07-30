/* 
 * Code from: USRP - Universal Software Radio Peripheral (GNU Radio) 
 * 
 * Initial modifications by: 
 * 
 * Stephan Esterhuizen, Aerospace Engineering Sciences 
 * University of Colorado at Boulder 
 * Boulder CO, USA 
 *  
 * Further modifications for use with the SiGe USB module to accompany 
 * the textbook: "A Software-Defined GPS and Galileo Receiver: A 
 * Single-Frequency Approach" by Kai Borre, Dennis Akos, et.al. by: 
 * 
 * Marcus Junered, GNSS Research Group 
 * Lulea University of Technology 
 * Lulea, Sweden  
 * junered@ltu.se 
 * 
 *Futher modified for usage with a new frontend PCB 
 * by Oscar Isoz GNSS Research Group 
 * Lulea University of Technology 
 * Lulea, Sweden 
 * --------------------------------------------------------------------- 
 * 
 * GN3S - GNSS IF Streamer for Windows 
 * Copyright (C) 2006 Marcus Junered 
 * 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 2 of the License, or 
 * (at your option) any later version. 
 * 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
 */ 
 
 
#include "gn3s_se4110.h"

static unsigned short idx = 0;


char init_se4110(void)
{

  /* D1,D5-D7 are inputs */
IOD = 0xff;
OED = 0xff;  
 // OED = 0x1D; 	// Set Port D as outputs, except ADC_DOUT 

  /* Set all "config" ports */
  A0 = 1; 	
  A1 = 0; 	
  A2 = 0; 	
  A3 = 1; 	
  A4 = 0;	
  A5 = 0;	
  A7 = 1; 	
  
  OEA = 0xBF; 	// Set Port A as output

  return 0;
}


short fifo_status() {
  return idx;
}



char program_3w(void) 
{
  char i;
  char by0, by1;
	by0 =0x18;
	by1 =0xA4;

  OEA = 0xFF;

  A3 = 0;

  A0 = 0;

    for (i=7; i>=0; i--) {
      A1 = (by1 >> i) & 0x1;
      A2 = 1;
      A2 = 0;
    }


  /* Latch Enable */
  A0 = 1;
  A0 = 0;

    for (i=7; i>=0; i--) {
      A1 = (by0 >> i) & 0x1;
      A2 = 1;
      A2 = 0;
    }
  
  /* Latch Enable */
  A0 = 1;

  OEA = 0xB9;

  return 0;
}
