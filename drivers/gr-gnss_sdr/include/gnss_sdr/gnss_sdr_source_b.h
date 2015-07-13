/*!
 * \file gnss_sdr_source_b.h
 * \brief Source Block Driver Header for the GNSS-SDR Hacker's Edition
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


#ifndef INCLUDED_GNSS_SDR_GNSS_SDR_SOURCE_B_H
#define INCLUDED_GNSS_SDR_GNSS_SDR_SOURCE_B_H

#include <gnss_sdr/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace gnss_sdr {

    /*!
     * \brief A Source Driver for the GNSS-SDR Hacker's Edition.
     * \ingroup gnss_sdr
     *
     */
    class GNSS_SDR_API gnss_sdr_source_b : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<gnss_sdr_source_b> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of 
       * gnss_sdr::gnss_sdr_source_b.
       *
       * To avoid accidental use of raw pointers, gnss_sdr::gnss_sdr_source_b's
       * constructor is in a private implementation
       * class. gnss_sdr::gnss_sdr_source_b::make is the public interface for
       * creating new instances.
       */
      static sptr make();

      /* Routines to program configuration registers*/
      /* Setter routines for CONF1 */

      /*
       * \brief Chip Enable.
       * Register  : CHIPEN
       * Bits      : 27 (1 bit)
       * Function  : Chip enable. Set 1 to enable the device and 0 to disable
                     the entire device except the serial bus.
       * Default   : 1
      */
      virtual void set_frontend_register_CHIPEN(unsigned int) = 0;

      /* \brief Idle Enable.
       * Register  : IDLE
       * Bits      : 26 (1 bit)
       * Function  : Idle enable. Set 1 to put the chip in the idle mode and 0 
       *             for operating mode.
       * Default   : 0
      */
      virtual void set_frontend_register_IDLE(unsigned int) = 0;

      /* \brief LNA1 current Programming.
       * Register  : ILNA1
       * Bits      : 25:22 (4 bits)
       * Function  : LNA1 current programming.
       * Default   : 8
      */
      virtual void set_frontend_register_ILNA1(unsigned int) = 0;

      /* \brief LNA2 current programming.
       * Register  : ILNA2
       * Bits      : 21:20 (2 bits)
       * Function  : LNA2 current programming.
       * Default   : 2
      */
      virtual void set_frontend_register_ILNA2(unsigned int) = 0;

      /* \brief LO Buffer Current Programming.
       * Register  : ILO
       * Bits      : 19:18 (2 bits)
       * Function  : LO Buffer current programming.
       * Default   : 2
      */
      virtual void set_frontend_register_ILO(unsigned int) = 0;

      /* \brief Mixer current programming.
       * Register  : IMIX
       * Bits      : 17:16 (2 bits)
       * Function  : Mixer current programming.
       * Default   : 1
      */
      virtual void set_frontend_register_IMIX(unsigned int) = 0;

      /* \brief Mixer Pole Selection.
       * Register  : MIXPOLE
       * Bits      : 15 (1 bit)
       * Function  : Mixer pole selection. Set 1 to program the passive filter 
       *             pole at mixer output at 36MHz, or set 0 to program the 
       *             pole at 13MHz.
       * Default   : 0
      */
      virtual void set_frontend_register_MIXPOLE(unsigned int) = 0;

      /* \brief LNA Mode Selection.
       * Register  : LNAMODE
       * Bits      : 14:13 (2 bits)
       * Function  : LNA Mode Selection :
       *               0 - LNA Selection Gated by Antenna Bias Circuit.
       *               1 - LNA2 is active.
       *               2 - LNA1 is active.
       *               3 - Both LNA1 and LNA2 are off.
       * Default   : 0
      */
      virtual void set_frontend_register_LNAMODE(unsigned int) = 0;

      /* \brief Mixer Enable.
       * Register  : MIXEN
       * Bits      : 12 (1 bit)
       * Function  : Mixer enable. Set 1 to enable the mixer and 0 to shut down
       *             the mixer.
       * Default   : 1
      */
      virtual void set_frontend_register_MIXEN(unsigned int) = 0;

      /* \brief Antenna Bias Enable.
       * Register  : ANTEN
       * Bits      : 11 (1 bit)
       * Function  : Antenna bias enable. Set 1 to enable the antenna bias and 
       *             0 to shut down the antenna bias.
       * Default   : 1
      */
      virtual void set_frontend_register_ANTEN(unsigned int) = 0;

      /* \brief IF Center Frequency Programming.
       * Register  : FCEN
       * Bits      : 10:5 (6 bits)
       * Function  : IF center frequency programming. Default for fCENTER = 4MHz
       *             and BW = 2.5MHz
       * Default   : 13
      */
      virtual void set_frontend_register_FCEN(unsigned int) = 0;

      /* \brief IF Filter Bandwidth Selection.
       * Register  : FBW
       * Bits      : 4:3 (2 bits)
       * Function  : IF filter center bandwidth selection : 
       *               0 - 2.5MHz
       *               1 - 8 MHz
       *               2 - 4.2MHz
       *               3 - 18MHz (only used as a low-pass filter)
       * Default   : 0
      */
      virtual void set_frontend_register_FBW(unsigned int) = 0;

      /* \brief Filter order selection.
       * Register  : F3OR5
       * Bits      : 2 (1 bit)
       * Function  : Filter order selection. Set 0 to select the 5th-order
       *             Butterworth filter. Set 1 to select the 3rd-order 
       *             Butterworth filter.
       * Default   : 0
      */
      virtual void set_frontend_register_F3OR5(unsigned int) = 0;

      /* \brief Polyphase Filter Selection.
       * Register  : FCENX
       * Bits      : 1 (1 bit)
       * Function  : Polyphase filter selection. Set 1 to select complex 
                     bandpass filter mode. Set 0 to select lowpass filter mode.
       * Default   : 1
      */
      virtual void set_frontend_register_FCENX(unsigned int) = 0;

      /* \brief IF Filter Gain Setting.
       * Register  : FGAIN
       * Bits      : 0 (1 bit)
       * Function  : IF filter gain setting. Set 0 to reduce the filter gain by 
       *             6dB.
       * Default   : 1
      */
      virtual void set_frontend_register_FGAIN(unsigned int) = 0;

      /* Setter routines for CONF2 */


      /* \brief I and Q channels enable.
       * Register  : IQEN
       * Bits      : 27 (1 bit)
       * Function  : I and Q channels enable. Set 1 to enable both I and Q 
       *             channels 
       *             and 0 to enable I channel only.
       * Default   : 0
      */
      virtual void set_frontend_register_IQEN(unsigned int) = 0;

      /* \brief AGC Gain Reference Value.
       * Register  : GAINREF
       * Bits      : 26:15 (12 bits)
       * Function  : AGC gain reference value expressed by the number of MSB
       *             counts (magnitude bit density).
       * Default   : 170
      */
      virtual void set_frontend_register_GAINREF(unsigned int) = 0;

      /*  \brief AGC Mode Control.
       * Register  : AGCMODE
       * Bits      : 12:11 (2 bits)
       * Function  : AGC mode control :
       *           0 - Independent I and Q.
       *           1 - I and Q gains are locked to each other.
       *           2 - Gain is set directly from the serial interface by GAININ.
       *           3 - Disallowed state.
       * Default   : 0
      */
      virtual void set_frontend_register_AGCMODE(unsigned int) = 0;


      /* \brief Output data format.
       * Register  : FORMAT
       * Bits      : 10:9 (12 bits)
       * Function  : Output data format : 
       *           0 - Unsigned binary. 
       *           1 - Sign and Magnitude.
       *           2 / 3 - Two’s complement binary.
       * Default   : 1
      */
      virtual void set_frontend_register_FORMAT(unsigned int) = 0;

      /* \brief Number of bits in the ADC.
       * Register  : BITS
       * Bits      : 8:6  (3 bits)
       * Function  : Number of bits in the ADC :
       *           0 - 1 bit.
       *           1 - 1.5 bits.
       *           2 - 2 bits.
       *           3 - 2.5 bits.
       *           4 - 3 bits.
       * Default   : 2
      */
      virtual void set_frontend_register_BITS(unsigned int) = 0;

      /* \brief Output driver configuration.
       * Register  : DRVCFG
       * Bits      : 5:4 (2 bits)
       * Function  : Output driver configuration :
       *           0 - CMOS Logic.
       *           1 - Limited Differential Logic.
       *           2 / 3 - Analog Outputs.
       * Default   : 0
      */
      virtual void set_frontend_register_DRVCFG(unsigned int) = 0;

      /* \brief LO Buffer Enable.
       * Register  : LOEN
       * Bits      : 3 (1 bit)
       * Function  : LO Buffer Enable. Set 1 to enable the buffer, 0 to disable
       *             the buffer.
       * Default   : 1
      */
      virtual void set_frontend_register_LOEN(unsigned int) = 0;

      /* \brief IC Version Identifier.
       * Register  : DIEID
       * Bits      : 1:0 (2 bits)
       * Function  : Identifies a version of the IC.
       * Default   : 0
      */
      virtual void set_frontend_register_DIEID(unsigned int) = 0;

      /* Setter routines for CONF3 */


      /* \brief PGA gain value programming
       * Register  : GAININ
       * Bits      : 27:22 (6 bits)
       * Function  : PGA gain value programming from the serial interface in
       * steps of dB per LSB. 
       * Default   : 58
      */
      virtual void set_frontend_register_GAININ(unsigned int) = 0;

      /* \brief Low Value of the ADC full-scale enable.
       * Register  : FSLOWEN
       * Bits      : 21 (1 bit)
       * Function  : Low value of the ADC full-scale enable. Set 1 to enable 
       * or 0 to disable. 
       * Default   : 1
      */
      virtual void set_frontend_register_FSLOWEN(unsigned int) = 0;

      /* \brief Enabled the Output Driver to Drive High Loads.
       * Register  : HILOADEN
       * Bits      : 20 (1 bit)
       * Function  : Set 1 to enable the output driver to drive high loads. 
       * Default   : 0
      */
      virtual void set_frontend_register_HILOADEN(unsigned int) = 0;

      /* \brief ADC enable.
       * Register  : ADCEN
       * Bits      : 19 (1 bit)
       * Function  : ADC enable. Set 1 to enable ADC or 0 to disable. 
       * Default   : 1
      */
      virtual void set_frontend_register_ADCEN(unsigned int) = 0;

      /* \brief Output Driver Enable.
       * Register  : DRVEN
       * Bits      : 18 (1 bit)
       * Function  : Output driver enable. Set 1 to enable the driver or 0 to 
       *             disable.
       * Default   : 1
      */
      virtual void set_frontend_register_DRVEN(unsigned int) = 0;

      /* \brief Filter DC Offset Cancellation Circuitry Enable.
       * Register  : FOFSTEN
       * Bits      : 17 (1 bit)
       * Function  : Filter DC offset cancellation circuitry enable. Set 1 to
       *             enable the circuitry or 0 to disable.
       * Default   : 1
      */
      virtual void set_frontend_register_FOFSTEN(unsigned int) = 0;

      /* \brief IF Filter Enable.
       * Register  : FILTEN
       * Bits      : 16 (1 bit)
       * Function  : IF filter enable. Set 1 to enable the filter or 0 to 
       *             disable.
       * Default   : 1
      */
      virtual void set_frontend_register_FILTEN(unsigned int) = 0;

      /* \brief Highpass Coupling Enable.
       * Register  : FHIPEN
       * Bits      : 15 (1 bit)
       * Function  : Highpass coupling enable. Set 1 to enable the highpass 
       *             coupling between the filter and PGA, or 0 to disable the 
       *             coupling.
       * Default   : 1
      */
      virtual void set_frontend_register_FHIPEN(unsigned int) = 0;

      /* \brief I Channel PGA Enable.
       * Register  : PGAIEN
       * Bits      : 13 (1 bit)
       * Function  : I-channel PGA enable. Set 1 to enable PGA in the I channel
       *             or 0 to disable. 
       * Default   : 1
      */
      virtual void set_frontend_register_PGAIEN(unsigned int) = 0;

      /* \brief Q Channel PGA Enable.
       * Register  : PGAQEN
       * Bits      : 12 (1 bit)
       * Function  : Q-channel PGA enable. Set 1 to enable PGA in the Q channel
       *             or 0 to disable. 
       * Default   : 0
      */
      virtual void set_frontend_register_PGAQEN(unsigned int) = 0;

      /* \brief DSP Streaming Interface Enable.
       * Register  : STRMEN
       * Bits      : 11 (1 bit)
       * Function  : DSP interface for serial streaming of data enable. This bit 
       *             configures the IC such that the DSP interface is inserted
       *             in the signal path. Set 1 to enable the interface or 0 to 
       *             disable the interface.
       * Default   : 0
      */
      virtual void set_frontend_register_STRMEN(unsigned int) = 0;

      /* \brief Data Streaming Start.
       * Register  : STRMSTART
       * Bits      : 10 (1 bit)
       * Function  : The positive edge of this command enables data streaming to
       * the output. It also enables clock, data sync, and frame sync outputs. 
       * Default   : 0
      */
      virtual void set_frontend_register_STRMSTART(unsigned int) = 0;

      /* \brief Data Streaming Stop.
       * Register  : STRMSTOP
       * Bits      : 9 (1 bit)
       * Function  : The positive edge of this command disables data streaming 
       *             to the output. It also disables clock, data sync, and 
       *             frame sync outputs. 
       * Default   : 0
      */
      virtual void set_frontend_register_STRMSTOP(unsigned int) = 0;

      /* \brief Length of the Data Counter.
       * Register  : STRMCOUNT
       * Bits      : 8:6 (3 bits)
       * Function  : Sets the length of the data counter from 128 (000) to 
       *             16,394 (111) bits per frame. 
       * Default   : 7
      */
      virtual void set_frontend_register_STRMCOUNT(unsigned int) = 0;

      /* \brief Number of bits streamed.
       * Register  : STRMBITS
       * Bits      : 5:4 (2 bits)
       * Function  : Number of bits streamed: 
       *           0 - I MSB
       *           1 - I MSB, I LSB 
       *           2 - I MSB, Q MSB
       *           3 - I MSB, I LSB, Q MSB, Q LSB. 
       * Default   : 1
      */
      virtual void set_frontend_register_STRMBITS(unsigned int) = 0;

      /* \brief Enable the insertion of frame number at the beginning of frame.
       * Register  : STAMPEN
       * Bits      : 3 (1 bit)
       * Function  : The signal enables the insertion of the frame number at the 
       *             beginning of each frame. If disabled, only the ADC data is 
       *             streamed to the output. 
       * Default   : 1
      */
      virtual void set_frontend_register_STAMPEN(unsigned int) = 0;

      /* \brief Enable Time Sync Pulses.
       * Register  : TIMESYNCEN
       * Bits      : 2 (1 bit) 
       * Function  : This signal enables the output of the time sync pulses at 
       *             all times when streaming is enabled by the STRMEN command.
       *             Otherwise, the time sync pulses are available only when 
       *             data streaming is active at the output, for example, in 
       *             the time intervals bound by the STRMSTART and STRMSTOP 
       *             commands. 
       * Default   : 1
      */
      virtual void set_frontend_register_TIMESYNCEN(unsigned int) = 0;

      /* \brief Enable Sync Pulses at DATASYNC output.
       * Register  : DATSYNCEN
       * Bits      : 1 (1 bit)
       * Function  : This control signal enables the sync pulses at the DATASYNC 
       *             output. Each pulse is coincident with the beginning of the
       *             16-bit data word that corresponds to a given output bit. 
       * Default   : 0
      */
      virtual void set_frontend_register_DATSYNCEN(unsigned int) = 0;

      /* \brief Reset Stream Counters.
       * Register  : STRMRST
       * Bits      : 0 (1 bit)
       * Function  : This command resets all the counters irrespective of the 
       *             timing within the stream cycle. 
       * Default   : 0
      */
      virtual void set_frontend_register_STRMRST(unsigned int) = 0;

      /* Setter routines for PLLCONF */

      /* \brief VCO Enable.
       * Register  : VCOEN
       * Bits      : 27 (1 bit)
       * Function  : VCO enable. Set 1 to enable the VCO or 0 to disable VCO.
       * Default   : 1
      */
      virtual void set_frontend_register_VCOEN(unsigned int) = 0;

      /* \brief VCO Current Mode Selection.
       * Register  : IVCO
       * Bits      : 26 (1 bit)
       * Function  : VCO current-mode selection. Set 1 to program the VCO in the 
       *             low-current mode or 0 to program in the normal mode.
       * Default   : 0
      */
      virtual void set_frontend_register_IVCO(unsigned int) = 0;

      /* \brief Clock buffer enable.
       * Register  : REFOUTEN
       * Bits      : 24 (1 bit)
       * Function  : Clock buffer enable. Set to 1 to enable clock buffer and 0
       *             to disable it.
       * Default   : 1
      */
      virtual void set_frontend_register_REFOUTEN(unsigned int) = 0;

      /* \brief Clock output divider ratio.
       * Register  : REFDIV
       * Bits      : 22:21 (2 bits)
       * Function  : Clock output divider ratio:
       *           0 - Clock frequency = XTAL Frequency x 2.
       *           1 - Clock frequency = XTAL Frequency / 4.
       *           2 - Clock frequency = XTAL Frequency / 2.
       *           3 - Clock frequency = XTAL Frequency.
       * Default   : 3
      */
      virtual void set_frontend_register_REFDIV(unsigned int) = 0;

      /* \brief Current programming for XTAL oscillator / buffer.
       * Register  : IXTAL
       * Bits      : 20:19 (2 bits)
       * Function  : Current programming for XTAL oscillator / buffer:
       *           0 - Oscillator Normal Current.
       *           1 - Buffer Normal Current.
       *           2 - Oscillator Medium Current.
       *           3 - Oscillator High Current.
       * Default   : 1
      */
      virtual void set_frontend_register_IXTAL(unsigned int) = 0;

      /* \brief Digital XTAL load cap programming.
       * Register  : XTALCAP
       * Bits      : 18:14 (5 bits)
       * Function  : Digital XTAL load cap programming.
       * Default   : 16
      */
      virtual void set_frontend_register_XTALCAP(unsigned int) = 0;

      /* \brief LD pin output selection.
       * Register  : LDMUX
       * Bits      : 13:10 (4 bits)
       * Function  : LD pin output selection.
       *               0 - Lock Detect Signal
       * Default   : 0
      */
      virtual void set_frontend_register_LDMUX(unsigned int) = 0;

      /* \brief Charge Pump Current Selection.
       * Register  : ICP
       * Bits      : 9 (1 bit)
       * Function  : Charge-pump current selection. Set 1 for 1mA and 0 for 
       *             0.5mA.
       * Default   : 0
      */
      virtual void set_frontend_register_ICP(unsigned int) = 0;

      /* \brief PLL Phase Frequency Detector Enable.
       * Register  : PFDEN
       * Bits      : 8 (1 bit)
       * Function  : Set 0 for normal operation or 1 to disable the PLL phase
       *             frequency detector.
       * Default   : 0
      */
      virtual void set_frontend_register_PFDEN(unsigned int) = 0;

      /* \brief Charge Pump Test.
       * Register  : CPTEST
       * Bits      : 6:4 (3 bits)
       * Function  : Charge-pump test :
       *           0 - Normal Operation
       *           2 / 6 - Pump Up
       *           1 / 5 - Pump Down
       *           4 - High Impedance
       *           7 - Both up and down on.
       * Default   : 0
      */
      virtual void set_frontend_register_CPTEST(unsigned int) = 0;

      /* \brief PLL mode control - Integer / Fractional.
       * Register  : INT_PLL
       * Bits      : 3 (1 bit)
       * Function  : PLL mode control.  Set 1 to enable integer-N PLL or 0 to 
       *             enable fractional-N PLL.
       * Default   : 1
      */
      virtual void set_frontend_register_INT_PLL(unsigned int) = 0;

      /* \brief PLL power save mode.
       * Register  : PWRSAV
       * Bits      : 2 (1 bit)
       * Function  : PLL power-save mode. Set 1 to enable the power-save mode
       *             or 0 to disable.
       * Default   : 0
      */
      virtual void set_frontend_register_PWRSAV(unsigned int) = 0;

      /* Setter routines for DIV */

      /* \brief PLL integer division ratio.
       * Register  : NDIV
       * Bits      : 27:13 (15 bits)
       * Function  : PLL integer division ratio.
       * Default   : 1536
      */
      virtual void set_frontend_register_NDIV(unsigned int) = 0;

      /* \brief PLL reference division ratio.
       * Register  : RDIV
       * Bits      : 12:3 (10 bits)
       * Function  : PLL reference division ratio.
       * Default   : 16
      */
      virtual void set_frontend_register_RDIV(unsigned int) = 0;

      /* Setter routines for FDIV */


      /* \brief PLL fractional divider ratio.
       * Register  : FDIV
       * Bits      : 27:8 (20 bits)
       * Function  : PLL fractional divider ratio.
       * Default   : 524288
      */
      virtual void set_frontend_register_FDIV(unsigned int) = 0;

      /* Setter routines for STRM */

      /* \brief Frame Counter Start.
       * Register  : FRAMECOUNT
       * Bits      : 27:0 (28 bits)
       * Function  : This word defines the frame number at which to start 
       *             streaming.  This mode is active when streaming mode is 
       *             enabled by a command STRMEN, but a command STRMSTART is 
       *             not received. In this case, the frame counter is reset 
       *             upon the assertion of STRMEN, and it begins its count. 
       *             When the frame number reaches the value defined
       *             by FRMCOUNT, the streaming begins. 
       * Default   : 134217728
      */
      virtual void set_frontend_register_FRAMECOUNT(unsigned int) = 0;

      /* Setter routines for CLK */

      /* \brief Value for the L Counter.
       * Register  : L_CNT
       * Bits      : 27:16 (12 bits)
       * Function  : Sets the value for the L counter.
       * Default   : 256
      */
      virtual void set_frontend_register_L_CNT(unsigned int) = 0;

      /* \brief Value for the M Counter.
       * Register  : M_CNT
       * Bits      : 15:4 (12 bits)
       * Function  : Sets the value for the M counter.
       * Default   : 1563
      */
      virtual void set_frontend_register_M_CNT(unsigned int) = 0;

      /* \brief Value for the fractional clock divider.
       * Register  : FCLKIN
       * Bits      : 3 (1 bit)
       * Function  : Fractional clock divider. Set 1 to select the ADC clock 
       *             to come from the fractional clock divider, or 0 to bypass 
       *             the ADC clock from the fractional clock divider.
       * Default   : 0
      */
      virtual void set_frontend_register_FCLKIN(unsigned int) = 0;

      /* \brief ADC Clock Selection.
       * Register  : ADCCLK
       * Bits      : 2 (1 bit)
       * Function  : ADC clock selection. Set 0 to select the ADC and fractional 
       *         divider clocks to come from the reference divider/multiplier.
       * Default   : 0
      */
      virtual void set_frontend_register_ADCCLK(unsigned int) = 0;

      /* \brief Serializer Clock Selection.
       * Register  : SERCLK
       * Bits      : 1 (1 bit)
       * Function  : Serializer clock selection. Set 0 to select the serializer 
       *             clock output to come from the reference divider/multiplier.
       * Default   : 1
      */
      virtual void set_frontend_register_SERCLK(unsigned int) = 0;

      /* \brief DSP interface mode selection.
       * Register  : MODE
       * Bits      : 0 (1 bit)
       * Function  : DSP interface mode selection.
       * Default   : 0
      */
      virtual void set_frontend_register_MODE(unsigned int) = 0;
    };
  } // namespace gnss_sdr
} // namespace gr

#endif /* INCLUDED_GNSS_SDR_GNSS_SDR_SOURCE_B_H */

