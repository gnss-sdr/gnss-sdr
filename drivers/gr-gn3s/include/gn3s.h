/*----------------------------------------------------------------------------------------------*/
/*! \file gn3s.h
//
// FILENAME: gn3s.h
//
// DESCRIPTION: Defines the GN3S class.
//
// DEVELOPERS: Gregory W. Heckler (2003-2009)
//
// LICENSE TERMS: Copyright (c) Gregory W. Heckler 2009
//
// This file is part of the GPS Software Defined Radio (GPS-SDR)
//
// The GPS-SDR is free software; you can redistribute it and/or modify it under the terms of the
// GNU General Public License as published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version. The GPS-SDR is distributed in the hope that
// it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// Note:  Comments within this file follow a syntax that is compatible with
//        DOXYGEN and are utilized for automated document extraction
//
// Reference:
*/
/*----------------------------------------------------------------------------------------------*/


#ifndef GN3S_H_
#define GN3S_H_


/* Includes */
/*--------------------------------------------------------------*/
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include "fusb.h"
#include "fusb_linux.h"
//#include "usrp_bytesex.h"
//#include "usrp_prims.h"
/*--------------------------------------------------------------*/


/* FX2 Configuration Structure */
/*--------------------------------------------------------------*/
struct fx2Config
{
	int interface;
	int altinterface;
	usb_dev_handle *udev;
	fusb_ephandle *d_ephandle;
	fusb_devhandle *d_devhandle;
};
/*--------------------------------------------------------------*/


/* FX2 Stuff */
/*--------------------------------------------------------------*/
#define RX_ENDPOINT		(0x86)
#define VRT_VENDOR_IN	(0xC0)
#define VRT_VENDOR_OUT	(0x40)
#define RX_INTERFACE	(2)
#define RX_ALTINTERFACE (0)
#define VRQ_GET_STATUS	(0x80)
#define GS_RX_OVERRUN	(1)  //!< Returns 1 byte
#define VRQ_XFER		(0x01)
/*--------------------------------------------------------------*/


/* GN3S Stuff */
/*--------------------------------------------------------------*/
#define GN3S_VID 	 		(0x16C0)
#define GN3S_PID 	 		(0x072F)
#define VID_OLD  	 		(0x1781)
#define PID_OLD  	 		(0x0B39)
#define PROG_SET_CMD 		(0xE600)
#define FUSB_BUFFER_SIZE 	(16 * (1L << 20)) 	//!< 8 MB
#define FUSB_BLOCK_SIZE  	(16 * (1L << 10)) 	//!< 16KB is hard limit
#define FUSB_NBLOCKS		(FUSB_BUFFER_SIZE / FUSB_BLOCK_SIZE)
/*--------------------------------------------------------------*/


/* The firmware is embedded into the executable */
/*--------------------------------------------------------------*/
extern char _binary_usrp_gn3s_firmware_ihx_start[];
/*--------------------------------------------------------------*/


/*--------------------------------------------------------------*/
/*! \ingroup CLASSES
 *
 */
class gn3s
{

	private:

		/* First or second board */
		int which;

		/* GN3S FX2 Stuff */
		struct fx2Config fx2_config;
		struct usb_device *fx2_device;
		struct usb_dev_handle *fx2_handle;

		/* USB IDs */
		unsigned int gn3s_vid, gn3s_pid;

		/* Pull in the binary firmware */
		int fstart;
		int fsize;
		//char *gn3s_firmware;

	public:

		gn3s(int _which);		//!< Constructor
		~gn3s();				//!< Destructor

		/* FX2 functions */
		struct usb_device* usb_fx2_find(int vid, int pid, char info, int ignore);
		bool usb_fx2_configure(struct usb_device *fx2, fx2Config *fx2c);
		fusb_devhandle* make_devhandle (usb_dev_handle *udh);
		int read(void *buff, int bytes);
		int write_cmd(int request, int value, int index, unsigned char *bytes, int len);
		bool _get_status(int which, bool *trouble);
		bool check_rx_overrun();
		bool usrp_xfer(char VRQ_TYPE, bool start);

		/* Used to flash the GN3S */
		int atoz(char *s);
		void upload_ram(char *buf, int start, int len);
		void program_fx2(char *filename, char mem);
		int prog_gn3s_board();

};
/*--------------------------------------------------------------*/


#endif /*GN3S_H_ */
