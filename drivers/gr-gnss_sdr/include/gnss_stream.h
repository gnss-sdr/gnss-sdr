/*!
 * \file gnss_stream.h
 * \brief GNSS-SDR Hacker's Edition Board Driver Interface Header.
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

#ifndef GNSS_STREAM_H
#define GNSS_STREAM_H

#include <pthread.h>
#include "max2769.h"

/* Vendor ID and Product ID for the FTDI FT2232H */
#define VENDOR_ID	  0x0403
#define PRODUCT_ID	0x6010

/* Pin configuration for the FT2232H <--> MAX2769 TWI 
 
 FT2232H Pin             MAX2769 Pin
    BD5         <----->       CS
    BD6         <----->       CLK
    BD7         <----->       DAT

*/
#define MAX2769_TWI_DAT         7
#define MAX2769_TWI_CLK         6
#define MAX2769_TWI_CS          5

/* Clock Interval */
#define MAX2769_CLK_DELAY_US  1

struct _data_packet
{
	signed char* data;
	unsigned int len;
	struct _data_packet* next;	
};

typedef struct _data_packet data_packet;

struct gnss_device_context
{
    struct ftdi_context* ftdi_ctx;
    data_packet* packet_list_head;
    pthread_t thread_id;
    pthread_mutex_t thread_lock;
    int exit_thread;
    reg_CONF1   conf1;
    reg_CONF2   conf2;
    reg_CONF3   conf3;
    reg_PLLCONF pllconf;
    reg_DIV     divide;
    reg_FDIV    fdiv;
    reg_STRM    strm;
    reg_CLK     clk;
    reg_TEST1   test1;
    reg_TEST2   test2;
};

/* Setter routines for CONF1 */

/*
  Register  : CHIPEN
  Bits      : 27 (1 bit)
  Function  : Chip enable. Set 1 to enable the device and 0 to disable the 
              entire device except the serial bus.
  Default   : 1
*/
extern void set_CHIPEN(struct gnss_device_context*, unsigned int);

/*
  Register  : IDLE
  Bits      : 26 (1 bit)
  Function  : Idle enable. Set 1 to put the chip in the idle mode and 0 for 
              operating mode.
  Default   : 0
*/
extern void set_IDLE(struct gnss_device_context*, unsigned int);

/*
  Register  : ILNA1
  Bits      : 25:22 (4 bits)
  Function  : LNA1 current programming.
  Default   : 8
*/
extern void set_ILNA1(struct gnss_device_context*, unsigned int);

/*
  Register  : ILNA2
  Bits      : 21:20 (2 bits)
  Function  : LNA2 current programming.
  Default   : 2
*/
extern void set_ILNA2(struct gnss_device_context*, unsigned int);

/*
  Register  : ILO
  Bits      : 19:18 (2 bits)
  Function  : LO Buffer current programming.
  Default   : 2
*/
extern void set_ILO(struct gnss_device_context*, unsigned int);

/*
  Register  : IMIX
  Bits      : 17:16 (2 bits)
  Function  : Mixer current programming.
  Default   : 1
*/
extern void set_IMIX(struct gnss_device_context*, unsigned int);

/*
  Register  : MIXPOLE
  Bits      : 15 (1 bit)
  Function  : Mixer pole selection. Set 1 to program the passive filter pole at 
              mixer output at 36MHz, or set 0 to program the pole at 13MHz.
  Default   : 0
*/
extern void set_MIXPOLE(struct gnss_device_context*, unsigned int);

/*
  Register  : LNAMODE
  Bits      : 14:13 (2 bits)
  Function  : LNA Mode Selection :
                0 - LNA Selection Gated by Antenna Bias Circuit.
                1 - LNA2 is active.
                2 - LNA1 is active.
                3 - Both LNA1 and LNA2 are off.
  Default   : 0
*/
extern void set_LNAMODE(struct gnss_device_context*, unsigned int);

/*
  Register  : MIXEN
  Bits      : 12 (1 bit)
  Function  : Mixer enable. Set 1 to enable the mixer and 0 to shut down the 
              mixer.
  Default   : 1
*/
extern void set_MIXEN(struct gnss_device_context*, unsigned int);

/*
  Register  : ANTEN
  Bits      : 11 (1 bit)
  Function  : Antenna bias enable. Set 1 to enable the antenna bias and 0 to 
              shut down the antenna bias.
  Default   : 1
*/
extern void set_ANTEN(struct gnss_device_context*, unsigned int);

/*
  Register  : FCEN
  Bits      : 10:5 (6 bits)
  Function  : IF center frequency programming. Default for fCENTER = 4MHz, 
              BW = 2.5MHz
  Default   : 13
*/
extern void set_FCEN(struct gnss_device_context*, unsigned int);

/*
  Register  : FBW
  Bits      : 4:3 (2 bits)
  Function  : IF filter center bandwidth selection : 
                0 - 2.5MHz
                1 - 8 MHz
                2 - 4.2MHz
                3 - 18MHz (only used as a low-pass filter)
  Default   : 0
*/
extern void set_FBW(struct gnss_device_context*, unsigned int);

/*
  Register  : F3OR5
  Bits      : 2 (1 bit)
  Function  : Filter order selection. Set 0 to select the 5th-order Butterworth 
              filter. Set 1 to select the 3rd-order Butterworth filter.
  Default   : 0
*/
extern void set_F3OR5(struct gnss_device_context*, unsigned int);

/*
  Register  : FCENX
  Bits      : 1 (1 bit)
  Function  : Polyphase filter selection. Set 1 to select complex bandpass 
              filter mode. Set 0 to select lowpass filter mode.
  Default   : 1
*/
extern void set_FCENX(struct gnss_device_context*, unsigned int);

/*
  Register  : FGAIN
  Bits      : 0 (1 bit)
  Function  : IF filter gain setting. Set 0 to reduce the filter gain by 6dB.
  Default   : 1
*/
extern void set_FGAIN(struct gnss_device_context*, unsigned int);

/* Setter routines for CONF2 */


/*
  Register  : IQEN
  Bits      : 27 (1 bit)
  Function  : I and Q channels enable. Set 1 to enable both I and Q channels 
              and 0 to enable I channel only.
  Default   : 0
*/
extern void set_IQEN(struct gnss_device_context*, unsigned int);

/*
  Register  : GAINREF
  Bits      : 26:15 (12 bits)
  Function  : AGC gain reference value expressed by the number of MSB counts 
              (magnitude bit density).
  Default   : 170
*/
extern void set_GAINREF(struct gnss_device_context*, unsigned int);

/*
  Register  : AGCMODE
  Bits      : 12:11 (2 bits)
  Function  : AGC mode control :
                0 - Independent I and Q.
                1 - I and Q gains are locked to each other.
                2 - Gain is set directly from the serial interface by GAININ.
                3 - Disallowed state.
  Default   : 0
*/
extern void set_AGCMODE(struct gnss_device_context*, unsigned int);


/*
  Register  : FORMAT
  Bits      : 10:9 (12 bits)
  Function  : Output data format : 
                0 - Unsigned binary. 
                1 - Sign and Magnitude.
                2 / 3 - Two’s complement binary.
  Default   : 1
*/
extern void set_FORMAT(struct gnss_device_context*, unsigned int);

/*
  Register  : BITS
  Bits      : 8:6  (3 bits)
  Function  : Number of bits in the ADC :
                0 - 1 bit.
                1 - 1.5 bits.
                2 - 2 bits.
                3 - 2.5 bits.
                4 - 3 bits.
  Default   : 2
*/
extern void set_BITS(struct gnss_device_context*, unsigned int);

/*
  Register  : DRVCFG
  Bits      : 5:4 (2 bits)
  Function  : Output driver configuration :
                0 - CMOS Logic.
                1 - Limited Differential Logic.
                2 / 3 - Analog Outputs.
  Default   : 0
*/
extern void set_DRVCFG(struct gnss_device_context*, unsigned int);

/*
  Register  : LOEN
  Bits      : 3 (1 bit)
  Function  : LO Buffer Enable. Set 1 to enable the buffer, 0 to disable the 
              buffer.
  Default   : 1
*/
extern void set_LOEN(struct gnss_device_context*, unsigned int);

/*
  Register  : DIEID
  Bits      : 1:0 (2 bits)
  Function  : Identifies a version of the IC.
  Default   : 0
*/
extern void set_DIEID(struct gnss_device_context*, unsigned int);

/* Setter routines for CONF3 */


/*
  Register  : GAININ
  Bits      : 27:22 (6 bits)
  Function  : PGA gain value programming from the serial interface in steps of 
              dB per LSB. 
  Default   : 58
*/
extern void set_GAININ(struct gnss_device_context*, unsigned int);

/*
  Register  : FSLOWEN
  Bits      : 21 (1 bit)
  Function  : Low value of the ADC full-scale enable. Set 1 to enable or 0 to 
              disable. 
  Default   : 1
*/
extern void set_FSLOWEN(struct gnss_device_context*, unsigned int);

/*
  Register  : HILOADEN
  Bits      : 20 (1 bit)
  Function  : Set 1 to enable the output driver to drive high loads. 
  Default   : 0
*/
extern void set_HILOADEN(struct gnss_device_context*, unsigned int);

/*
  Register  : ADCEN
  Bits      : 19 (1 bit)
  Function  : ADC enable. Set 1 to enable ADC or 0 to disable. 
  Default   : 1
*/
extern void set_ADCEN(struct gnss_device_context*, unsigned int);

/*
  Register  : DRVEN
  Bits      : 18 (1 bit)
  Function  : Output driver enable. Set 1 to enable the driver or 0 to disable.
  Default   : 1
*/
extern void set_DRVEN(struct gnss_device_context*, unsigned int);

/*
  Register  : FOFSTEN
  Bits      : 17 (1 bit)
  Function  : Filter DC offset cancellation circuitry enable. Set 1 to enable
              the circuitry or 0 to disable.
  Default   : 1
*/
extern void set_FOFSTEN(struct gnss_device_context*, unsigned int);

/*
  Register  : FILTEN
  Bits      : 16 (1 bit)
  Function  : IF filter enable. Set 1 to enable the filter or 0 to disable.
  Default   : 1
*/
extern void set_FILTEN(struct gnss_device_context*, unsigned int);

/*
  Register  : FHIPEN
  Bits      : 15 (1 bit)
  Function  : Highpass coupling enable. Set 1 to enable the highpass coupling 
              between the filter and PGA, or 0 to disable the coupling.
  Default   : 1
*/
extern void set_FHIPEN(struct gnss_device_context*, unsigned int);

/*
  Register  : PGAIEN
  Bits      : 13 (1 bit)
  Function  : I-channel PGA enable. Set 1 to enable PGA in the I channel or 0 
              to disable. 
  Default   : 1
*/
extern void set_PGAIEN(struct gnss_device_context*, unsigned int);

/*
  Register  : PGAQEN
  Bits      : 12 (1 bit)
  Function  : Q-channel PGA enable. Set 1 to enable PGA in the Q channel or 0 to disable. 
  Default   : 0
*/
extern void set_PGAQEN(struct gnss_device_context*, unsigned int);

/*
  Register  : STRMEN
  Bits      : 11 (1 bit)
  Function  : DSP interface for serial streaming of data enable. This bit 
              configures the IC such that the DSP interface is inserted in the 
              signal path. Set 1 to enable the interface or 0 to disable the 
              interface.
  Default   : 0
*/
extern void set_STRMEN(struct gnss_device_context*, unsigned int);

/*
  Register  : STRMSTART
  Bits      : 10 (1 bit)
  Function  : The positive edge of this command enables data streaming to the 
              output. It also enables clock, data sync, and frame sync outputs. 
  Default   : 0
*/
extern void set_STRMSTART(struct gnss_device_context*, unsigned int);

/*
  Register  : STRMSTOP
  Bits      : 9 (1 bit)
  Function  : The positive edge of this command disables data streaming to the 
              output. It also disables clock, data sync, and frame sync outputs. 
  Default   : 0
*/
extern void set_STRMSTOP(struct gnss_device_context*, unsigned int);

/*
  Register  : STRMCOUNT
  Bits      : 8:6 (3 bits)
  Function  : Sets the length of the data counter from 128 (000) to 16,394 (111)
              bits per frame. 
  Default   : 7
*/
extern void set_STRMCOUNT(struct gnss_device_context*, unsigned int);

/*
  Register  : STRMBITS
  Bits      : 5:4 (2 bits)
  Function  : Number of bits streamed: 
                0 - I MSB
                1 - I MSB, I LSB 
                2 - I MSB, Q MSB
                3 - I MSB, I LSB, Q MSB, Q LSB. 
  Default   : 1
*/
extern void set_STRMBITS(struct gnss_device_context*, unsigned int);

/*
  Register  : STAMPEN
  Bits      : 3 (1 bit)
  Function  : The signal enables the insertion of the frame number at the 
              beginning of each frame. If disabled, only the ADC data is 
              streamed to the output. 
  Default   : 1
*/
extern void set_STAMPEN(struct gnss_device_context*, unsigned int);

/*
  Register  : TIMESYNCEN
  Bits      : 2 (1 bit) 
  Function  : This signal enables the output of the time sync pulses at all 
              times when streaming is enabled by the STRMEN command. Otherwise, 
              the time sync pulses are available only when data streaming is 
              active at the output, for example, in the time intervals bound 
              by the STRMSTART and STRMSTOP commands. 
  Default   : 1
*/
extern void set_TIMESYNCEN(struct gnss_device_context*, unsigned int);

/*
  Register  : DATSYNCEN
  Bits      : 1 (1 bit)
  Function  : This control signal enables the sync pulses at the DATASYNC 
              output. Each pulse is coincident with the beginning of the 16-bit 
              data word that corresponds to a given output bit. 
  Default   : 0
*/
extern void set_DATSYNCEN(struct gnss_device_context*, unsigned int);

/*
  Register  : STRMRST
  Bits      : 0 (1 bit)
  Function  : This command resets all the counters irrespective of the timing 
              within the stream cycle. 
  Default   : 0
*/
extern void set_STRMRST(struct gnss_device_context*, unsigned int);

/* Setter routines for PLLCONF */

/*
  Register  : VCOEN
  Bits      : 27 (1 bit)
  Function  : VCO enable. Set 1 to enable the VCO or 0 to disable VCO.
  Default   : 1
*/
extern void set_VCOEN(struct gnss_device_context*, unsigned int);

/*
  Register  : IVCO
  Bits      : 26 (1 bit)
  Function  : VCO current-mode selection. Set 1 to program the VCO in the 
              low-current mode or 0 to program in the normal mode.
  Default   : 0
*/
extern void set_IVCO(struct gnss_device_context*, unsigned int);

/*
  Register  : REFOUTEN
  Bits      : 24 (1 bit)
  Function  : Clock buffer enable. Set to 1 to enable clock buffer and 0 to 
              disable it.
  Default   : 1
*/
extern void set_REFOUTEN(struct gnss_device_context*, unsigned int);

/*
  Register  : REFDIV
  Bits      : 22:21 (2 bits)
  Function  : Clock output divider ratio:
                0 - Clock frequency = XTAL Frequency x 2.
                1 - Clock frequency = XTAL Frequency / 4.
                2 - Clock frequency = XTAL Frequency / 2.
                3 - Clock frequency = XTAL Frequency.
  Default   : 3
*/
extern void set_REFDIV(struct gnss_device_context*, unsigned int);

/*
  Register  : IXTAL
  Bits      : 20:19 (2 bits)
  Function  : Current programming for XTAL oscillator / buffer:
                0 - Oscillator Normal Current.
                1 - Buffer Normal Current.
                2 - Oscillator Medium Current.
                3 - Oscillator High Current.
  Default   : 1
*/
extern void set_IXTAL(struct gnss_device_context*, unsigned int);

/*
  Register  : XTALCAP
  Bits      : 18:14 (5 bits)
  Function  : Digital XTAL load cap programming.
  Default   : 16
*/
extern void set_XTALCAP(struct gnss_device_context*, unsigned int);

/*
  Register  : LDMUX
  Bits      : 13:10 (4 bits)
  Function  : LD pin output selection.
                0 - Lock Detect Signal
  Default   : 0
*/
extern void set_LDMUX(struct gnss_device_context*, unsigned int);

/*
  Register  : ICP
  Bits      : 9 (1 bit)
  Function  : Charge-pump current selection. Set 1 for 1mA and 0 for 0.5mA.
  Default   : 0
*/
extern void set_ICP(struct gnss_device_context*, unsigned int);

/*
  Register  : PFDEN
  Bits      : 8 (1 bit)
  Function  : Set 0 for normal operation or 1 to disable the PLL phase frequency 
              detector.
  Default   : 0
*/
extern void set_PFDEN(struct gnss_device_context*, unsigned int);

/*
  Register  : CPTEST
  Bits      : 6:4 (3 bits)
  Function  : Charge-pump test :
                0 - Normal Operation
                2 / 6 - Pump Up
                1 / 5 - Pump Down
                4 - High Impedance
                7 - Both up and down on.
  Default   : 0
*/
extern void set_CPTEST(struct gnss_device_context*, unsigned int);

/*
  Register  : INT_PLL
  Bits      : 3 (1 bit)
  Function  : PLL mode control.  Set 1 to enable integer-N PLL or 0 to enable 
              fractional-N PLL.
  Default   : 1
*/
extern void set_INT_PLL(struct gnss_device_context*, unsigned int);

/*
  Register  : PWRSAV
  Bits      : 2 (1 bit)
  Function  : PLL power-save mode. Set 1 to enable the power-save mode or 0 to 
              disable.
  Default   : 0
*/
extern void set_PWRSAV(struct gnss_device_context*, unsigned int);

/* Setter routines for DIV */

/*
  Register  : NDIV
  Bits      : 27:13 (15 bits)
  Function  : PLL integer division ratio.
  Default   : 1536
*/
extern void set_NDIV(struct gnss_device_context*, unsigned int);

/*
  Register  : RDIV
  Bits      : 12:3 (10 bits)
  Function  : PLL reference division ratio.
  Default   : 16
*/
extern void set_RDIV(struct gnss_device_context*, unsigned int);

/* Setter routines for FDIV */


/*
  Register  : FDIV
  Bits      : 27:8 (20 bits)
  Function  : PLL fractional divider ratio.
  Default   : 524288
*/
extern void set_FDIV(struct gnss_device_context*, unsigned int);

/* Setter routines for STRM */

/*
  Register  : FRAMECOUNT
  Bits      : 27:0 (28 bits)
  Function  : This word defines the frame number at which to start streaming. 
              This mode is active when streaming mode is enabled by a command 
              STRMEN, but a command STRMSTART is not received. In this case, 
              the frame counter is reset upon the assertion of STRMEN, and it 
              begins its count. When the frame number reaches the value defined
              by FRMCOUNT, the streaming begins. 
  Default   : 134217728
*/
extern void set_FRAMECOUNT(struct gnss_device_context*, unsigned int);

/* Setter routines for CLK */

/*
  Register  : L_CNT
  Bits      : 27:16 (12 bits)
  Function  : Sets the value for the L counter.
  Default   : 256
*/
extern void set_L_CNT(struct gnss_device_context*, unsigned int);

/*
  Register  : M_CNT
  Bits      : 15:4 (12 bits)
  Function  : Sets the value for the M counter.
  Default   : 1563
*/
extern void set_M_CNT(struct gnss_device_context*, unsigned int);

/*
  Register  : FCLKIN
  Bits      : 3 (1 bit)
  Function  : Fractional clock divider. Set 1 to select the ADC clock to come 
              from the fractional clock divider, or 0 to bypass the ADC clock 
              from the fractional clock divider.
  Default   : 0
*/
extern void set_FCLKIN(struct gnss_device_context*, unsigned int);

/*
  Register  : ADCCLK
  Bits      : 2 (1 bit)
  Function  : ADC clock selection. Set 0 to select the ADC and fractional 
              divider clocks to come from the reference divider/multiplier.
  Default   : 0
*/
extern void set_ADCCLK(struct gnss_device_context*, unsigned int);

/*
  Register  : SERCLK
  Bits      : 1 (1 bit)
  Function  : Serializer clock selection. Set 0 to select the serializer clock 
              output to come from the reference divider/multiplier.
  Default   : 1
*/
extern void set_SERCLK(struct gnss_device_context*, unsigned int);

/*
  Register  : MODE
  Bits      : 0 (1 bit)
  Function  : DSP interface mode selection.
  Default   : 0
*/
extern void set_MODE(struct gnss_device_context*, unsigned int);

extern int program_registers(struct gnss_device_context*);
extern void add_packet(struct gnss_device_context*, signed char*, unsigned int);
extern int get_next_packet_data(struct gnss_device_context*, signed char**, int);
extern void init_gnss_device_context(struct gnss_device_context*);
extern int open_GNSS_SDR(struct gnss_device_context*);
extern void close_GNSS_SDR(struct gnss_device_context*);
extern void start_capture(struct gnss_device_context*);
extern void stop_capture(struct gnss_device_context*);

#endif

