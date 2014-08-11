/*----------------------------------------------------------------------------------------------*/
/*! \file gn3s.cpp
//
// FILENAME: gn3s.cpp
//
// DESCRIPTION: Impelements the GN3S class.
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

#include "gn3s.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <usb.h>
#include "fusb_linux.h"

static char debug = 1; //!< 1 = Verbose

/*----------------------------------------------------------------------------------------------*/
gn3s::gn3s(int _which)
{

		//int fsize;
		bool ret;
		which = _which;

		fx2_device 	= NULL;
		fx2_handle 	= NULL;
		gn3s_vid 	= GN3S_VID;
		gn3s_pid 	= GN3S_PID;

		/* Get the firmware embedded in the executable */
		//fstart = (int) &_binary_usrp_gn3s_firmware_ihx_start;
		//fsize = strlen(_binary_usrp_gn3s_firmware_ihx_start);
		//gn3s_firmware = new char[fsize + 10];
		//memcpy(&gn3s_firmware[0], (void *)fstart, fsize);

		// Load the firmware from external file (Javier)


		//gn3s_firmware[fsize] = NULL;

		/* Search all USB busses for the device specified by VID/PID */
		fx2_device = usb_fx2_find(gn3s_vid, gn3s_pid, debug, 0);
		if (!fx2_device)
		{
			/* Program the board */
			ret = prog_gn3s_board();
			if(ret)
			{
				fprintf(stdout, "Could not flash GN3S device\n");
				throw(1);
			}

			/* Need to wait to catch change */
			sleep(2);

			/* Search all USB busses for the device specified by VID/PID */
			fx2_device = usb_fx2_find(gn3s_vid, gn3s_pid, debug, 0);
		}
		else
		{
			fprintf(stdout, "Found GN3S Device\n");
		}

		/* Open and configure FX2 device if found... */
		ret = usb_fx2_configure(fx2_device, &fx2_config);
		if(ret)
		{
			fprintf(stdout, "Could not obtain a handle to the GN3S device\n");
			throw(1);

	}

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
gn3s::~gn3s()
{

	usrp_xfer(VRQ_XFER, 0);

	//delete gn3s_firmware;
	delete fx2_config.d_ephandle;
	delete fx2_config.d_devhandle;

	usb_release_interface(fx2_config.udev, fx2_config.interface);
	usb_close(fx2_config.udev);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
int gn3s::prog_gn3s_board()
{

	char a;
	struct usb_bus *bus;
	struct usb_device *dev;
	//struct usb_dev_handle *han;
	int vid, pid;

	dev = NULL;

	usb_init();
	usb_find_busses();
	usb_find_devices();

	vid = (VID_OLD);
	pid = (PID_OLD);

	for(bus = usb_busses; bus; bus = bus->next)
	{
		for(dev = bus->devices; dev; dev = dev->next)
		{
			if((dev->descriptor.idVendor == vid) &&	(dev->descriptor.idProduct == pid))
			{
				fx2_device = dev;
				fprintf(stdout,"GN3S Device Found... awaiting firmware flash \n");
				break;
			}
		}
	}

	if(fx2_device == NULL)
	{
		fprintf(stderr,"Cannot find vid 0x%x pid 0x%x \n", vid, pid);
		return -1;
	}

	printf("Using device vendor id 0x%04x product id 0x%04x\n",
			fx2_device->descriptor.idVendor, fx2_device->descriptor.idProduct);

	fx2_handle = usb_open(fx2_device);

	/* Do the first set 0xE600 1 */
	char c[] = "1";
	char d[] = "0";

	a = atoz(c);

	fprintf(stdout,"GN3S flashing ... \n");

	upload_ram(&a, (PROG_SET_CMD),1);

	program_fx2(NULL, 1);

	a = atoz(d);

	upload_ram(&a, (PROG_SET_CMD),1);

	fprintf(stdout,"GN3S flash complete! \n");

	usb_close(fx2_handle);

	return(0);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
int gn3s::atoz(char *s)
{
    int a;
    if(!strncasecmp("0x", s, 2)){
        sscanf(s, "%x", &a);
        return a;
    }
    return atoi(s);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void gn3s::upload_ram(char *buf, int start, int len)
{
	int i;
	int tlen;
	int quanta = 16;
	int a;

	for (i = start; i < start + len; i += quanta) {
		tlen = len + start - i;

		if (tlen > quanta)
			tlen = quanta;

		if (debug >= 3)
			printf("i = %d, tlen = %d \n", i, tlen);
		a = usb_control_msg(fx2_handle, 0x40, 0xa0, i, 0,
				buf + (i - start), tlen, 1000);

		if (a < 0) {
			fprintf(stderr, "Request to upload ram contents failed: %s\n",
					usb_strerror());
			return;
		}
	}
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void gn3s::program_fx2(char *filename, char mem)
{
	FILE *f;
	char s[1024];
	char data[256];
	char checksum, a;
	int length, addr, type, i;
	unsigned int b;

	// *** mod javier: load firmware from external file ***

	//f = tmpfile();

	/* Dump firmware into temp file */
	//fputs(gn3s_firmware, f);
	//rewind(f);

	  f = fopen ("gn3s_firmware.ihx","r");
	  if (f!=NULL)
	  {
		printf("GN3S firmware file found!\n");
	  }else{
		  printf("Could not open GN3S firmware file!\n");
		  return;
	  }

	while (!feof(f)) {
		fgets(s, 1024, f); /* we should not use more than 263 bytes normally */

		if (s[0] != ':') {
			fprintf(stderr, "%s: invalid string: \"%s\"\n", filename, s);
			continue;
		}

		sscanf(s + 1, "%02x", &length);
		sscanf(s + 3, "%04x", &addr);
		sscanf(s + 7, "%02x", &type);

		if (type == 0) {
			// printf("Programming %3d byte%s starting at 0x%04x",
			//     length, length==1?" ":"s", addr);
			a = length + (addr & 0xff) + (addr >> 8) + type;

			for (i = 0; i < length; i++) {
				sscanf(s + 9 + i * 2, "%02x", &b);
				data[i] = b;
				a = a + data[i];
			}

			sscanf(s + 9 + length * 2, "%02x", &b);
			checksum = b;

			if (((a + checksum) & 0xff) != 0x00) {
				printf("  ** Checksum failed: got 0x%02x versus 0x%02x\n", (-a)
						& 0xff, checksum);
				continue;
			} else {
				//printf(", checksum ok\n");
			}

			upload_ram(data, addr, length);

		} else {
			if (type == 0x01) {
				printf("End of file\n");
				fclose(f);

				return;
			} else {
				if (type == 0x02) {
					printf("Extended address: whatever I do with it ?\n");
					continue;
				}
			}
		}
	}

	fclose(f);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
struct usb_device* gn3s::usb_fx2_find(int vid, int pid, char info, int ignore)
{
	struct usb_bus *bus;
	struct usb_device *dev;
	struct usb_device *fx2 = NULL;
	usb_dev_handle *udev;
	int count = 0;
	int ret;
	char str[256];

	usb_init();
	usb_find_busses();
	usb_find_devices();

	for(bus = usb_busses; bus; bus = bus->next)
	{
		for(dev = bus->devices; dev; dev = dev->next)
		{
			if((dev->descriptor.idVendor == vid) && (dev->descriptor.idProduct == pid))
			{
				fx2 = dev;
			}

			if(fx2 != NULL && info)
			{
				udev = usb_open(fx2);
				if(udev && dev->descriptor.idVendor == vid && dev->descriptor.idProduct == pid && count < ignore)
				{
					if(fx2->descriptor.iManufacturer)
					{
						ret = usb_get_string_simple(udev, fx2->descriptor.iManufacturer, str, sizeof(str));

						if(ret > 0)
							printf("- Manufacturer : %s\n", str);
						else
							printf("- Unable to fetch manufacturer string\n");
					}

					if(fx2->descriptor.iProduct)
					{
						ret = usb_get_string_simple(udev, fx2->descriptor.iProduct, str, sizeof(str));

						if(ret > 0)
							printf("- Product : %s\n", str);
						else
							printf("- Unable to fetch product string\n");
					}

					if(fx2->descriptor.iSerialNumber)
					{
						ret = usb_get_string_simple(udev, fx2->descriptor.iSerialNumber, str, sizeof(str));

						if(ret > 0)
							printf("- Serial Number: %s\n", str);
						else
							printf("- Unable to fetch serial number string\n");
					}

					usb_close (udev);
					return fx2;
				}
				else if(udev && dev->descriptor.idVendor == vid && dev->descriptor.idProduct == pid && count >= ignore)
				{
					count++;
				}

				if(!fx2->config)
				{
					printf(" Could not retrieve descriptors\n");
					continue;
				}

				for(int i = 0; i < fx2->descriptor.bNumConfigurations; i++)
				{
					//print_configuration(&fx2->config[i]);
				}
			}
		}
	}

	return fx2;
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
bool gn3s::usb_fx2_configure(struct usb_device *fx2, fx2Config *fx2c)
{

  char status = 0;
  int interface = RX_INTERFACE;
  int altinterface = RX_ALTINTERFACE;
  usb_dev_handle *udev;
  fusb_ephandle *d_ephandle;
  fusb_devhandle *d_devhandle;

  udev = usb_open(fx2);

  if(!udev)
  {
      fprintf(stdout, "Could not obtain a handle to GNSS Front-End device \n");
      return -1;
  }
  else
  {
	  if(debug)
		  printf("Received handle for GNSS Front-End device \n");

      if(usb_set_configuration (udev, 1) < 0)
      {
          fprintf (stdout,
                  "error in %s, \n%s \n",
                  __FUNCTION__,
                  usb_strerror());
          usb_close (udev);
          status = -1;
      }

      if(usb_claim_interface (udev, interface) < 0)
      {
          fprintf (stdout,
                  "error in %s, \n%s \n",
                  __FUNCTION__,
                  usb_strerror());
          usb_close (udev);
          fprintf (stdout, "\nDevice not programmed? \n");
          usb_close (udev);
          status = -1;
          throw(0);
      }

      if(usb_set_altinterface (udev, altinterface) < 0)
      {
          fprintf (stdout,
                  "error in %s, \n%s \n",
                  __FUNCTION__,
                  usb_strerror());
          usb_close (udev);
          usb_release_interface (udev, interface);
          usb_close (udev);
          status = -1;
      }

      d_devhandle = make_devhandle(udev);
      d_ephandle = d_devhandle->make_ephandle(RX_ENDPOINT, true, FUSB_BLOCK_SIZE, FUSB_NBLOCKS);

      if(!d_ephandle->start())
      {
          fprintf (stdout, "usrp0_rx: failed to start end point streaming");
          usb_strerror ();
          status = -1;
      }

      if(status == 0)
      {
          fx2c->interface = interface;
          fx2c->altinterface = altinterface;
          fx2c->udev = udev;
          fx2c->d_devhandle = d_devhandle;
          fx2c->d_ephandle = d_ephandle;
          return 0;
      }
      else
      {
          return -1;
      }
  }
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
fusb_devhandle* gn3s::make_devhandle(usb_dev_handle *udh)
{
  return new fusb_devhandle_linux(udh);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
int gn3s::read(void *buff, int bytes)
{
	return(fx2_config.d_ephandle->read(buff, bytes));
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
bool gn3s::check_rx_overrun()
{
	bool overrun;

	_get_status(GS_RX_OVERRUN, &overrun);

	return(overrun);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
bool gn3s::_get_status(int command, bool *trouble)
{
	unsigned char status;

	if(write_cmd(VRQ_GET_STATUS, 0, command, &status, sizeof(status)) != sizeof (status))
		return false;

	*trouble = status;
	return true;
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
bool gn3s::usrp_xfer(char VRQ_TYPE, bool start)
{
  int r;

  r = write_cmd(VRQ_TYPE, start, 0, 0, 0);

  return(r == 0);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
int gn3s::write_cmd(int request, int value, int index, unsigned char *bytes, int len)
{
	int requesttype;
	int r;

	requesttype = (request & 0x80) ? VRT_VENDOR_IN : VRT_VENDOR_OUT;
	r = usb_control_msg (fx2_config.udev, requesttype, request, value, index, (char *) bytes, len, 1000);
	if(r < 0)
	{
		/* We get EPIPE if the firmware stalls the endpoint. */
		if(errno != EPIPE)
			fprintf (stdout, "usb_control_msg failed: %s\n", usb_strerror());
	}
	return r;
}
/*----------------------------------------------------------------------------------------------*/
