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


#include "gn3s_main.h"


BYTE	DB_Addr;	// Dual Byte Address stat
BYTE	I2C_Addr;	// I2C address

volatile WORD guardTick = 0;
volatile WORD guardCnt = 0;

// Set the page size to 0, so we know to calculate it on an EEPROM write
BYTE EE_Page_Size = 0;	


void main(void) {
  init_usrp();
  init_gpif();
  init_se4110();
  
  TD_Init();	// Init fucntion for A9 vendor commands

  EA = 0;	// disable all interrupts

  setup_autovectors();
  usb_install_handlers();
    
  EIEX4 = 1;   	      	// disable INT4 FIXME
  EA = 1;           	// global interrupt enable

  fx2_renumerate();	// simulates disconnect / reconnect

 // enable_se4110();
  program_3w();
  //Javi: disable capture filesize limitation
  //hook_timer_tick((unsigned int)guardC);
 
  main_loop();
}



static 
void get_ep0_data(void)
{
    EP0BCL = 0;	// arm EP0 for OUT xfer.  This sets the
                // busy bit

    // wait for busy to clear
    while (EP0CS & bmEPBUSY);
}


/*
 * Handle our "Vendor Extension" commands on endpoint 0.
 * If we handle this one, return non-zero.
 */
unsigned char
app_vendor_cmd(void)
{
  WORD addr, len, bc;
  WORD i;

  if (bRequestType == VRT_VENDOR_IN) {

	// ///////////////////////////////
	// handle the IN requests
	// ///////////////////////////////

	switch (bRequest) {

	case VRQ_GET_STATUS:
	  switch (wIndexL) {

	  case GS_RX_OVERRUN:
		EP0BUF[0] = GPIFIDLECS>>7;
		EP0BCH = 0;
		EP0BCL = 1;
		break;
	  case GS_TX_UNDERRUN:
		EP0BUF[0] = GPIFIDLECS>>7;
		EP0BCH = 0;
		EP0BCL = 1;
		break;

	  default:
		return 0;
	  }
	  break;

	case VRQ_DB_FX:
	  DB_Addr = 0x01;	//TPM: need to assert double byte
	  I2C_Addr |= 0x01;	//TPM: need to assert double byte
	  // NOTE: This case falls through !
	case VRQ_RAM:
	case VRQ_EEPROM:
	  addr = SETUPDAT[2];	// Get address and length
	  addr |= SETUPDAT[3] << 8;
	  len = SETUPDAT[6];
	  len |= SETUPDAT[7] << 8;
	  
	  while(len)	// Move requested data through EP0IN 
	    {	      	// one packet at a time.

	      while(EP0CS & bmEPBUSY);

	      if(len < EP0BUFF_SIZE)
		bc = len;
	      else
		bc = EP0BUFF_SIZE;

	      // Is this a RAM upload ?
	      if(SETUPDAT[1] == VRQ_RAM)
		{
		  for(i=0; i<bc; i++)
		    *(EP0BUF+i) = *((BYTE xdata *)addr+i);
		}
	      else
		{
		  for(i=0; i<bc; i++)
		    *(EP0BUF+i) = 0xcd;
		  EEPROMRead(addr,(WORD)bc,(WORD)EP0BUF);
		}

	      EP0BCH = 0;
	      EP0BCL = (BYTE)bc; // Arm endpoint with # bytes to transfer

	      addr += bc;
	      len -= bc;

	    }
	  break;

	default:
	  return 0;
	}
  }

  else if (bRequestType == VRT_VENDOR_OUT) {

    // ///////////////////////////////
    // handle the OUT requests
    // ///////////////////////////////

    switch (bRequest) {

      /* Start/Stop transfer */
    case VRQ_XFER:

      /* start transfer */
      if (wValueL) {
        setup_flowstate_common();
        SYNCDELAY;
        GPIFABORT =0xff; SYNCDELAY;

        GPIFTRIG = 0; SYNCDELAY;

        setup_flowstate_read(); SYNCDELAY;
        /* Stop transfer */
        /* Zero out FIFO */
        FIFORESET = bmNAKALL;SYNCDELAY;
        FIFORESET = 2;SYNCDELAY;
        FIFORESET = 6;SYNCDELAY;
        FIFORESET = 0;SYNCDELAY;

	/* Start transfer */
        GPIFTRIG = bmGPIF_EP6_START | bmGPIF_READ; SYNCDELAY; 
        guardCnt = GUARD;
      }

      /* stop transfer */
      else {
	GPIFTRIG = 0; SYNCDELAY;
	GPIFABORT =0xff; SYNCDELAY;

	/* Clear fifo */
	FIFORESET = bmNAKALL;SYNCDELAY;
	FIFORESET = 2;SYNCDELAY;
	FIFORESET = 6;SYNCDELAY;
	FIFORESET = 0;SYNCDELAY;
    guardCnt = 0;
      }

      break;

    case VRQ_DB_FX:
      DB_Addr = 0x01;		//TPM: need to assert double byte
      I2C_Addr |= 0x01;		//TPM: need to assert double byte
      // NOTE: This case falls through !
    case VRQ_RAM:
    case VRQ_EEPROM:
      addr = SETUPDAT[2];	// Get address and length
      addr |= SETUPDAT[3] << 8;
      len = SETUPDAT[6];
      len |= SETUPDAT[7] << 8;
      
      // calculate the page size if we haven't already.
      // THIS IS A DESTRUCTIVE CALL - WILL ERASE THE EEPROM
      if (!EE_Page_Size)
	EE_Page_Size = EEPROMGetPageSize();	// Initialize EEPROM variables
      
      while(len)	// Move new data through EP0OUT 
	{		// one packet at a time.
	  // Arm endpoint - do it here to clear (after sud avail)
	  EP0BCH = 0;
	  EP0BCL = 0; // Clear bytecount to allow new data in; also stops NAKing
	  
	  while(EP0CS & bmEPBUSY);
	  
	  bc = EP0BCL; // Get the new bytecount
	      
	  // Is this a RAM download ?
	  if(SETUPDAT[1] == VRQ_RAM)
	    {
	      for(i=0; i<bc; i++)
		*((BYTE xdata *)addr+i) = *(EP0BUF+i);
	    }	
	  else
	    {
	      for (i = 0; i < bc; )
		{
		  // This write is normally one page long.  Two special cases:
		  // Starting from a non-page aligned address (addr & (EE_Page_Size - 1)) != 0
		  // Less than a page left
		  BYTE pageSize = EE_Page_Size;

		  if (EE_Page_Size != 1)
		    {
		      if (((addr+i) & (EE_Page_Size - 1)) != 0)
			pageSize = EE_Page_Size - ((addr+i) & (EE_Page_Size - 1));
		      if (bc-i < pageSize)
			pageSize = bc-i;
		    }

		  EEPROMWritePage(addr+i, EP0BUF+i, pageSize);
		  i+= pageSize;
		}
	    }

	  addr += bc;
	  len -= bc;
	}
      break;

    default:
      return 0;
    }
  }
  else
    return 0;	// invalid bRequestType

  return 1;
}


void guardC(void) interrupt {
	if(guardCnt)
		guardTick = 1;
    clear_timer_irq();
}

static void main_loop(void) 
{
  setup_flowstate_common();
  SYNCDELAY;	


  while (1) {
    // We don't do much, GPIF is running on autopilot

    if (_usb_got_SUDAV) {
      usb_handle_setup_packet();
      _usb_got_SUDAV = 0;
    }
    
   //Javi: Disable capture filesize limitation 
   /*
    if (guardTick && guardCnt) {
      guardTick = 0;
			
      if(!(--guardCnt)) {
        GPIFTRIG = 0; SYNCDELAY;
        GPIFABORT =0xff; SYNCDELAY;
        FIFORESET = bmNAKALL;SYNCDELAY;
        FIFORESET = 2;SYNCDELAY;
        FIFORESET = 6;SYNCDELAY;
        FIFORESET = 0;SYNCDELAY;
      }
    }
   */

  }
}

void TD_Init(void)	// Called once at startup
{
  //Rwuen = TRUE;	// Enable remote-wakeup
  // Set the page size to 0, so we know to calculate it on an EEPROM write
  EE_Page_Size = 0;           

  // Determine I2C boot eeprom device address; addr = 0x0 for 8 bit addr eeproms (24LC00)
  I2C_Addr = SERIAL_ADDR | ((I2CS & 0x10) >> 4); // addr=0x01 for 16 bit addr eeprom (LC65)
  
  // Indicate if it is a dual byte address part (BOOL)
  DB_Addr = (I2C_Addr & 0x01); // ID1 is 16 bit addr bit - set by rocker sw or jumper
}
