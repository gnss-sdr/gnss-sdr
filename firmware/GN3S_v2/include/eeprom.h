#define EEPROM_ADDR 0x51

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------
// the 3684 DVK board uses port pin PA7 as an EEPROM write-protect enable/disable.
// If your design uses a different pin, modify the following macros accordingly.
#define EEPROM_ENABLE_WRITE_PROTECT()  OEA &= ~0x80             // float PA7
#define EEPROM_DISABLE_WRITE_PROTECT() PA7 = 0; OEA |= 0x80     // drive PA7 low
#define MSB(word)      (BYTE)(((WORD)(word) >> 8) & 0xff)
#define LSB(word)      (BYTE)((WORD)(word) & 0xff)

#define SWAP_ENDIAN(word)   ((BYTE*)&word)[0] ^= ((BYTE*)&word)[1];\
                     ((BYTE*)&word)[1] ^= ((BYTE*)&word)[0];\
                     ((BYTE*)&word)[0] ^= ((BYTE*)&word)[1]

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void EEWaitForStop();
BYTE EEWaitForAck();
static void EEStartAndAddr();
extern void WaitForEEPROMWrite();
BYTE EEPROMWritePage(WORD addr, BYTE xdata * ptr, BYTE len);
BYTE EEPROMRead(WORD addr, BYTE length, BYTE xdata *buf);
void WaitForEEPROMWrite2();
BYTE EEWaitForDone();
BYTE EEPROMGetPageSize();

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
extern BYTE			DB_Addr;					// Dual Byte Address stat
extern BYTE			I2C_Addr;				// I2C address
extern BYTE       EE_Page_Size;        // EEPROM page size

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
#define SERIAL_ADDR		0x50
