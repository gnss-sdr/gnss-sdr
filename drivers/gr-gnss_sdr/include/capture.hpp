/*!
 * \file capture.hpp
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

#ifndef CAPTURE_GNSS_SDR_HE_H
#define CAPTURE_GNSS_SDR_HE_H

#include <ftdi.h>
#include "max2769.h"

#include <boost/atomic.hpp>
#include <boost/lockfree/spsc_queue.hpp>

/* Vendor ID and Product ID for the FTDI FT2232H */
#define VENDOR_ID   0x0403
#define PRODUCT_ID  0x6010

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

class Capture_GNSS_SDR_HE
{

  public:
    typedef struct _samplePacket
    {
      uint8_t samples[512];
      int length;
    } SamplePacket;

  private:
    struct ftdi_context* ftdi_capture_context;
    struct ftdi_context* ftdi_config_context;

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

    SamplePacket* current_sample_packet;

    boost::atomic<unsigned char> exit_capture_thread;
    boost::lockfree::spsc_queue<SamplePacket*, boost::lockfree::capacity<8192000> >
                    sample_fifo_queue;

    void max2769_write_pin(unsigned char, unsigned char);
    void max2769_write_byte(unsigned char);
    void program_register(unsigned int);
    static int GNSS_SDR_callback(uint8_t*,         
                                 int, 
                                 FTDIProgressInfo*, 
                                 void*);
    void capture_thread_function();
    
  public:

    Capture_GNSS_SDR_HE();
    ~Capture_GNSS_SDR_HE();

    bool pop_samples(SamplePacket** packet);
    int pop_samples(signed char** buffer, int num_samples);

    int program_registers();
    int start_capture();
    void stop_capture();

    /* Setter routines for CONF1 */

    /*
      Register  : CHIPEN
      Bits      : 27 (1 bit)
      Function  : Chip enable. Set 1 to enable the device and 0 to disable the 
                  entire device except the serial bus.
      Default   : 1
    */
    void set_CHIPEN(unsigned int);

    /*
      Register  : IDLE
      Bits      : 26 (1 bit)
      Function  : Idle enable. Set 1 to put the chip in the idle mode and 0 for 
                  operating mode.
      Default   : 0
    */
    void set_IDLE(unsigned int);

    /*
      Register  : ILNA1
      Bits      : 25:22 (4 bits)
      Function  : LNA1 current programming.
      Default   : 8
    */
    void set_ILNA1(unsigned int);

    /*
      Register  : ILNA2
      Bits      : 21:20 (2 bits)
      Function  : LNA2 current programming.
      Default   : 2
    */
    void set_ILNA2(unsigned int);

    /*
      Register  : ILO
      Bits      : 19:18 (2 bits)
      Function  : LO Buffer current programming.
      Default   : 2
    */
    void set_ILO(unsigned int);

    /*
      Register  : IMIX
      Bits      : 17:16 (2 bits)
      Function  : Mixer current programming.
      Default   : 1
    */
    void set_IMIX(unsigned int);

    /*
      Register  : MIXPOLE
      Bits      : 15 (1 bit)
      Function  : Mixer pole selection. Set 1 to program the passive filter pole at 
                  mixer output at 36MHz, or set 0 to program the pole at 13MHz.
      Default   : 0
    */
    void set_MIXPOLE(unsigned int);

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
    void set_LNAMODE(unsigned int);

    /*
      Register  : MIXEN
      Bits      : 12 (1 bit)
      Function  : Mixer enable. Set 1 to enable the mixer and 0 to shut down the 
                  mixer.
      Default   : 1
    */
    void set_MIXEN(unsigned int);

    /*
      Register  : ANTEN
      Bits      : 11 (1 bit)
      Function  : Antenna bias enable. Set 1 to enable the antenna bias and 0 to 
                  shut down the antenna bias.
      Default   : 1
    */
    void set_ANTEN(unsigned int);

    /*
      Register  : FCEN
      Bits      : 10:5 (6 bits)
      Function  : IF center frequency programming. Default for fCENTER = 4MHz, 
                  BW = 2.5MHz
      Default   : 13
    */
    void set_FCEN(unsigned int);

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
    void set_FBW(unsigned int);

    /*
      Register  : F3OR5
      Bits      : 2 (1 bit)
      Function  : Filter order selection. Set 0 to select the 5th-order Butterworth 
                  filter. Set 1 to select the 3rd-order Butterworth filter.
      Default   : 0
    */
    void set_F3OR5(unsigned int);

    /*
      Register  : FCENX
      Bits      : 1 (1 bit)
      Function  : Polyphase filter selection. Set 1 to select complex bandpass 
                  filter mode. Set 0 to select lowpass filter mode.
      Default   : 1
    */
    void set_FCENX(unsigned int);

    /*
      Register  : FGAIN
      Bits      : 0 (1 bit)
      Function  : IF filter gain setting. Set 0 to reduce the filter gain by 6dB.
      Default   : 1
    */
    void set_FGAIN(unsigned int);

    /* Setter routines for CONF2 */


    /*
      Register  : IQEN
      Bits      : 27 (1 bit)
      Function  : I and Q channels enable. Set 1 to enable both I and Q channels 
                  and 0 to enable I channel only.
      Default   : 0
    */
    void set_IQEN(unsigned int);

    /*
      Register  : GAINREF
      Bits      : 26:15 (12 bits)
      Function  : AGC gain reference value expressed by the number of MSB counts 
                  (magnitude bit density).
      Default   : 170
    */
    void set_GAINREF(unsigned int);

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
    void set_AGCMODE(unsigned int);


    /*
      Register  : FORMAT
      Bits      : 10:9 (12 bits)
      Function  : Output data format : 
                    0 - Unsigned binary. 
                    1 - Sign and Magnitude.
                    2 / 3 - Two’s complement binary.
      Default   : 1
    */
    void set_FORMAT(unsigned int);

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
    void set_BITS(unsigned int);

    /*
      Register  : DRVCFG
      Bits      : 5:4 (2 bits)
      Function  : Output driver configuration :
                    0 - CMOS Logic.
                    1 - Limited Differential Logic.
                    2 / 3 - Analog Outputs.
      Default   : 0
    */
    void set_DRVCFG(unsigned int);

    /*
      Register  : LOEN
      Bits      : 3 (1 bit)
      Function  : LO Buffer Enable. Set 1 to enable the buffer, 0 to disable the 
                  buffer.
      Default   : 1
    */
    void set_LOEN(unsigned int);

    /*
      Register  : DIEID
      Bits      : 1:0 (2 bits)
      Function  : Identifies a version of the IC.
      Default   : 0
    */
    void set_DIEID(unsigned int);

    /* Setter routines for CONF3 */


    /*
      Register  : GAININ
      Bits      : 27:22 (6 bits)
      Function  : PGA gain value programming from the serial interface in steps of 
                  dB per LSB. 
      Default   : 58
    */
    void set_GAININ(unsigned int);

    /*
      Register  : FSLOWEN
      Bits      : 21 (1 bit)
      Function  : Low value of the ADC full-scale enable. Set 1 to enable or 0 to 
                  disable. 
      Default   : 1
    */
    void set_FSLOWEN(unsigned int);

    /*
      Register  : HILOADEN
      Bits      : 20 (1 bit)
      Function  : Set 1 to enable the output driver to drive high loads. 
      Default   : 0
    */
    void set_HILOADEN(unsigned int);

    /*
      Register  : ADCEN
      Bits      : 19 (1 bit)
      Function  : ADC enable. Set 1 to enable ADC or 0 to disable. 
      Default   : 1
    */
    void set_ADCEN(unsigned int);

    /*
      Register  : DRVEN
      Bits      : 18 (1 bit)
      Function  : Output driver enable. Set 1 to enable the driver or 0 to disable.
      Default   : 1
    */
    void set_DRVEN(unsigned int);

    /*
      Register  : FOFSTEN
      Bits      : 17 (1 bit)
      Function  : Filter DC offset cancellation circuitry enable. Set 1 to enable
                  the circuitry or 0 to disable.
      Default   : 1
    */
    void set_FOFSTEN(unsigned int);

    /*
      Register  : FILTEN
      Bits      : 16 (1 bit)
      Function  : IF filter enable. Set 1 to enable the filter or 0 to disable.
      Default   : 1
    */
    void set_FILTEN(unsigned int);

    /*
      Register  : FHIPEN
      Bits      : 15 (1 bit)
      Function  : Highpass coupling enable. Set 1 to enable the highpass coupling 
                  between the filter and PGA, or 0 to disable the coupling.
      Default   : 1
    */
    void set_FHIPEN(unsigned int);

    /*
      Register  : PGAIEN
      Bits      : 13 (1 bit)
      Function  : I-channel PGA enable. Set 1 to enable PGA in the I channel or 0 
                  to disable. 
      Default   : 1
    */
    void set_PGAIEN(unsigned int);

    /*
      Register  : PGAQEN
      Bits      : 12 (1 bit)
      Function  : Q-channel PGA enable. Set 1 to enable PGA in the Q channel or 0 to disable. 
      Default   : 0
    */
    void set_PGAQEN(unsigned int);

    /*
      Register  : STRMEN
      Bits      : 11 (1 bit)
      Function  : DSP interface for serial streaming of data enable. This bit 
                  configures the IC such that the DSP interface is inserted in the 
                  signal path. Set 1 to enable the interface or 0 to disable the 
                  interface.
      Default   : 0
    */
    void set_STRMEN(unsigned int);

    /*
      Register  : STRMSTART
      Bits      : 10 (1 bit)
      Function  : The positive edge of this command enables data streaming to the 
                  output. It also enables clock, data sync, and frame sync outputs. 
      Default   : 0
    */
    void set_STRMSTART(unsigned int);

    /*
      Register  : STRMSTOP
      Bits      : 9 (1 bit)
      Function  : The positive edge of this command disables data streaming to the 
                  output. It also disables clock, data sync, and frame sync outputs. 
      Default   : 0
    */
    void set_STRMSTOP(unsigned int);

    /*
      Register  : STRMCOUNT
      Bits      : 8:6 (3 bits)
      Function  : Sets the length of the data counter from 128 (000) to 16,394 (111)
                  bits per frame. 
      Default   : 7
    */
    void set_STRMCOUNT(unsigned int);

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
    void set_STRMBITS(unsigned int);

    /*
      Register  : STAMPEN
      Bits      : 3 (1 bit)
      Function  : The signal enables the insertion of the frame number at the 
                  beginning of each frame. If disabled, only the ADC data is 
                  streamed to the output. 
      Default   : 1
    */
    void set_STAMPEN(unsigned int);

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
    void set_TIMESYNCEN(unsigned int);

    /*
      Register  : DATSYNCEN
      Bits      : 1 (1 bit)
      Function  : This control signal enables the sync pulses at the DATASYNC 
                  output. Each pulse is coincident with the beginning of the 16-bit 
                  data word that corresponds to a given output bit. 
      Default   : 0
    */
    void set_DATSYNCEN(unsigned int);

    /*
      Register  : STRMRST
      Bits      : 0 (1 bit)
      Function  : This command resets all the counters irrespective of the timing 
                  within the stream cycle. 
      Default   : 0
    */
    void set_STRMRST(unsigned int);

    /* Setter routines for PLLCONF */

    /*
      Register  : VCOEN
      Bits      : 27 (1 bit)
      Function  : VCO enable. Set 1 to enable the VCO or 0 to disable VCO.
      Default   : 1
    */
    void set_VCOEN(unsigned int);

    /*
      Register  : IVCO
      Bits      : 26 (1 bit)
      Function  : VCO current-mode selection. Set 1 to program the VCO in the 
                  low-current mode or 0 to program in the normal mode.
      Default   : 0
    */
    void set_IVCO(unsigned int);

    /*
      Register  : REFOUTEN
      Bits      : 24 (1 bit)
      Function  : Clock buffer enable. Set to 1 to enable clock buffer and 0 to 
                  disable it.
      Default   : 1
    */
    void set_REFOUTEN(unsigned int);

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
    void set_REFDIV(unsigned int);

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
    void set_IXTAL(unsigned int);

    /*
      Register  : XTALCAP
      Bits      : 18:14 (5 bits)
      Function  : Digital XTAL load cap programming.
      Default   : 16
    */
    void set_XTALCAP(unsigned int);

    /*
      Register  : LDMUX
      Bits      : 13:10 (4 bits)
      Function  : LD pin output selection.
                    0 - Lock Detect Signal
      Default   : 0
    */
    void set_LDMUX(unsigned int);

    /*
      Register  : ICP
      Bits      : 9 (1 bit)
      Function  : Charge-pump current selection. Set 1 for 1mA and 0 for 0.5mA.
      Default   : 0
    */
    void set_ICP(unsigned int);

    /*
      Register  : PFDEN
      Bits      : 8 (1 bit)
      Function  : Set 0 for normal operation or 1 to disable the PLL phase frequency 
                  detector.
      Default   : 0
    */
    void set_PFDEN(unsigned int);

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
    void set_CPTEST(unsigned int);

    /*
      Register  : INT_PLL
      Bits      : 3 (1 bit)
      Function  : PLL mode control.  Set 1 to enable integer-N PLL or 0 to enable 
                  fractional-N PLL.
      Default   : 1
    */
    void set_INT_PLL(unsigned int);

    /*
      Register  : PWRSAV
      Bits      : 2 (1 bit)
      Function  : PLL power-save mode. Set 1 to enable the power-save mode or 0 to 
                  disable.
      Default   : 0
    */
    void set_PWRSAV(unsigned int);

    /* Setter routines for DIV */

    /*
      Register  : NDIV
      Bits      : 27:13 (15 bits)
      Function  : PLL integer division ratio.
      Default   : 1536
    */
    void set_NDIV(unsigned int);

    /*
      Register  : RDIV
      Bits      : 12:3 (10 bits)
      Function  : PLL reference division ratio.
      Default   : 16
    */
    void set_RDIV(unsigned int);

    /* Setter routines for FDIV */


    /*
      Register  : FDIV
      Bits      : 27:8 (20 bits)
      Function  : PLL fractional divider ratio.
      Default   : 524288
    */
    void set_FDIV(unsigned int);

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
    void set_FRAMECOUNT(unsigned int);

    /* Setter routines for CLK */

    /*
      Register  : L_CNT
      Bits      : 27:16 (12 bits)
      Function  : Sets the value for the L counter.
      Default   : 256
    */
    void set_L_CNT(unsigned int);

    /*
      Register  : M_CNT
      Bits      : 15:4 (12 bits)
      Function  : Sets the value for the M counter.
      Default   : 1563
    */
    void set_M_CNT(unsigned int);

    /*
      Register  : FCLKIN
      Bits      : 3 (1 bit)
      Function  : Fractional clock divider. Set 1 to select the ADC clock to come 
                  from the fractional clock divider, or 0 to bypass the ADC clock 
                  from the fractional clock divider.
      Default   : 0
    */
    void set_FCLKIN(unsigned int);

    /*
      Register  : ADCCLK
      Bits      : 2 (1 bit)
      Function  : ADC clock selection. Set 0 to select the ADC and fractional 
                  divider clocks to come from the reference divider/multiplier.
      Default   : 0
    */
    void set_ADCCLK(unsigned int);

    /*
      Register  : SERCLK
      Bits      : 1 (1 bit)
      Function  : Serializer clock selection. Set 0 to select the serializer clock 
                  output to come from the reference divider/multiplier.
      Default   : 1
    */
    void set_SERCLK(unsigned int);

    /*
      Register  : MODE
      Bits      : 0 (1 bit)
      Function  : DSP interface mode selection.
      Default   : 0
    */
    void set_MODE(unsigned int);

};


#endif
