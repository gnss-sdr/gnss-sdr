//-----------------------------------------------------------------------------
//   File:      eeprom.c
//   Contents:   EEPROM update firmware source.  (Write only)
//
//   indent 3.  NO TABS!
//
//   Copyright (c) 2002 Cypress Semiconductor
//
// $Workfile: eeprom.c $
// $Date: 9/07/05 2:54p $
// $Revision: 1 $
//-----------------------------------------------------------------------------
//#include "fx2.h"
#include "fx2regs.h"
#include "eeprom.h"

///////////////////////////////////////////////////////////////////////////////////////

// Write up to one page of data to the EEPROM.
// Returns 0 on success, 1 on failure
// Normally called within a while() loop so that errors are retried:
// while (EEPROMWritePage(....))
//    ;
BYTE EEPROMWritePage(WORD addr, BYTE xdata * ptr, BYTE len)
{
    BYTE i;
    BYTE retval;

    //EEPROM_DISABLE_WRITE_PROTECT();

    // Make sure the i2c interface is idle
    EEWaitForStop();
    
    // write the START bit and i2c device address
    EEStartAndAddr();
    
    if(EEWaitForAck())
    {
        retval = 1;
        goto EXIT_WP;
    }

    // write the eeprom offset
    if (DB_Addr)
    {
        I2DAT = MSB(addr);
        if(EEWaitForAck())
        {
            retval = 1;
            goto EXIT_WP;
        }
    }
    I2DAT = LSB(addr);
    if(EEWaitForAck())
    {
        retval = 1;
        goto EXIT_WP;
    }

    // Write the data Page
    for (i = 0; i < len; i++)
    {
        I2DAT = *ptr++;
        if(EEWaitForDone())
        {
            retval = 1;
            goto EXIT_WP;
        }
    }	
    I2CS |= bmSTOP;
    WaitForEEPROMWrite();

    retval = 0;

EXIT_WP:            
    EEPROM_ENABLE_WRITE_PROTECT();
    return(retval);
}

void EEStartAndAddr()
{
      I2CS = bmSTART;
      I2DAT = I2C_Addr << 1;
}

// 0x2e in assembly, less than 0x20 with compiler optimization!!
void WaitForEEPROMWrite()
{
   EEWaitForStop();
waitForBusy:
	EEStartAndAddr();

   EEWaitForDone();
   I2CS |= bmSTOP;	//	; Set the STOP bit
   EEWaitForStop();

   if (!(I2CS & bmACK))  // If no ACK, try again.
      goto waitForBusy;
}

void EEWaitForStop()
{
   // Data should not be written to I2CS or I2DAT until the STOP bit returns low.
   while (I2CS & bmSTOP)
      ;
}

// Returns 0 on success, 1 on failure
BYTE EEPROMRead(WORD addr, BYTE length, BYTE xdata *buf)
{
   BYTE i;

   // Make sure the i2c interface is idle
   EEWaitForStop();
   
   // write the START bit and i2c device address
   EEStartAndAddr();

   if(EEWaitForAck())
      return(1);

   // write the eeprom offset
   if (DB_Addr)
      {
      I2DAT = MSB(addr);
      if(EEWaitForAck())
         return(1);
      }
   I2DAT = LSB(addr);
   if(EEWaitForAck())
      return(1);

   I2CS = bmSTART;

   // send the read command
   I2DAT = (I2C_Addr << 1) | 1;
   if(EEWaitForDone())
      return(1);

   // read dummy byte
   i = I2DAT;
   if(EEWaitForDone())
      return(1);

   for (i=0; i < (length - 1); i++)
   {
      *(buf+i) = I2DAT;
      if(EEWaitForDone())
         return(1);
   }
   
   I2CS = bmLASTRD;
   if(EEWaitForDone())
      return(1);

   *(buf+i) = I2DAT;
   if(EEWaitForDone())
      return(1);

   I2CS = bmSTOP;

   i = I2DAT;
   return(0);
}

// Return 0 for ok, 1 for error
BYTE EEWaitForDone()
{
   BYTE i;

   while (!((i = I2CS) & 1))  // Poll the done bit
      ;
   if (i & bmBERR)
      return 1;
   else
      return 0;
}

// Return 0 for ok, 1 for error
// Same as wait for done, but checks for ACK as well
BYTE EEWaitForAck()
{
   BYTE i;

   while (!((i = I2CS) & 1))  // Poll the done bit
      ;
   if (i & bmBERR)
      return 1;
   else if (!(i & bmACK))
      return 1;
   else
      return 0;
}


// Determine the page size supported by the EEPROM.  All of the EEPROMS we use
// (Atmel, Xicor and Microchip) will wrap their page address pointer if the page
// buffer is overrun.  We write 128 incrementing bytes to the EEPROM.  We then
// read the block back.  If location 0 is right, the page size is 128.
// If location 0 is wrong, it directly provides a mask for the page size.
// For example, if the page size is 4, location 0 will contain 0x7c
BYTE EEPROMGetPageSize()
{
      #define MAX_PAGE_SIZE 64
      BYTE xdata testData[MAX_PAGE_SIZE];
      BYTE xdata saveData[MAX_PAGE_SIZE];
      BYTE i;
      BYTE retval;

      if (!DB_Addr)
         return(1);

      EEPROMRead(0, MAX_PAGE_SIZE, saveData);
      for (i = 0; i < MAX_PAGE_SIZE; i++)
         {
         testData[i] = i;
         }
      EEPROMWritePage(0, testData, MAX_PAGE_SIZE);
      for (i = 0; i < MAX_PAGE_SIZE; i++)
         {
         testData[i] = ~i;
         }
      EEPROMRead(0, MAX_PAGE_SIZE, testData);
      i = testData[0];
      if (i & 1)
         retval = 1;       // Couldn't read back large EEPROM.  Assume page size 1.
      else
         {
         i = (MAX_PAGE_SIZE-1) ^ i;
         i++;
         retval = i;
         }

      for (i = 0; i < MAX_PAGE_SIZE; i+= retval)
         EEPROMWritePage(i, saveData, retval);         

      return(retval);
}

