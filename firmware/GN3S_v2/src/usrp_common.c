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
 * 
 * http://ccar.colorado.edu/gnss
 * 
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
 
 
#include "usrp_common.h"


/* If wordwide is defined, will grab 16 bits, otherwise only
 * bits 7-0 is used */
#undef wordwide

void init_usrp(void)
{
    CPUCS = bmCLKSPD1;          // CPU runs @ 48 MHz
    CKCON = 0;                  // MOVX takes 2 cycles

    IFCONFIG = bmIFGPIF;
    //IFCONFIG = bmIFGPIF|bmIFCLKSRC|bmIFCLKOE;//|bm3048MHZ;
    //IFCONFIG = bmIFGPIF|bmIFCLKSRC;
    SYNCDELAY;

	//REVCTL = bmDYN_OUT | bmENH_PKT; // highly recommended by docs
    //SYNCDELAY;

    // configure end points

    EP1OUTCFG = bmVALID | bmBULK;
    SYNCDELAY;
    EP1INCFG = bmVALID | bmBULK | bmIN;
    SYNCDELAY;

	// 512 quad bulk OUT
    //EP2CFG = bmVALID | bmBULK ;	//| bmQUADBUF;
	EP2CFG = 0;
    SYNCDELAY;          
	// disabled
    EP4CFG = 0;					
    SYNCDELAY; 
	// 512 quad bulk IN
    EP6CFG = bmVALID | bmBULK | bmQUADBUF | bmIN;
    SYNCDELAY;                  
	// disabled
    EP8CFG = 0;
    SYNCDELAY;                  

    // reset FIFOs
    FIFORESET = bmNAKALL;
    SYNCDELAY;
    //FIFORESET = 2;
    //SYNCDELAY;
    // FIFORESET = 4; SYNCDELAY;
    FIFORESET = 6;
    SYNCDELAY;
    // FIFORESET = 8; SYNCDELAY;
    FIFORESET = 0;
    SYNCDELAY;

    // configure end point FIFOs

    // let core see 0 to 1 transistion of autoout bit


#ifdef wordwide 
	EP2FIFOCFG = 0x00;
	EP6FIFOCFG = bmZEROLENIN | bmAUTOIN | bmWORDWIDE;
#else
	EP2FIFOCFG = 0x00;
   	EP4FIFOCFG = 0x00;
    EP6FIFOCFG = bmZEROLENIN | bmAUTOIN;
	EP8FIFOCFG = 0x00;
#endif

    SYNCDELAY;

    EP0BCH = 0;
    SYNCDELAY;

    // arm EP1OUT so we can receive "out" packets (TRM pg 8-8)

    EP1OUTBC = 0;
    SYNCDELAY;

    //EP2GPIFFLGSEL = 0x01;
    //SYNCDELAY;                  // For EP2OUT, GPIF uses EF flag
    EP6GPIFFLGSEL = 0x02;
    SYNCDELAY;                  // For EP6IN, GPIF uses FF flag
    
	/* waveform DONE when FF/EF flags get set */
    EP6GPIFPFSTOP=0x01; SYNCDELAY;
    //EP2GPIFPFSTOP=0x01; SYNCDELAY;

    EP6AUTOINLENH = (512) >> 8;
    SYNCDELAY;                  // this is the length for high speed
    EP6AUTOINLENL = (512) & 0xff;
    SYNCDELAY;
}
