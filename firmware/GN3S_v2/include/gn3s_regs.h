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
 
 
#ifndef _GN3S_REGS_H_
#define _GN3S_REGS_H_

/* Port A (bit addressable): */
#define	bmPORT_A_OUTPUTS  0	/* All inputs */
#define	bmPORT_A_INITIAL  0

/* Port B: GPIF	FD[7:0] */
// FIFO Input

/* Port C */
#define	bmPORT_C_OUTPUTS 0
#define	bmPORT_C_INITIAL 0

/* Port D: GPIF	FD[15:8] */
#define bmPORT_D_OUTPUTS 0
#define bmPORT_D_INITIAL 0

/* Port E: not bit addressible */
#define	bmPORT_E_OUTPUTS 0
#define	bmPORT_E_INITIAL 0

#endif 
