/*!
 * \file max2769.h
 * \brief MAX2769 GNSS Frontend Register Map.
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

#ifndef GNSS_SDR_MAX_2769_H
#define GNSS_SDR_MAX_2769_H


/* 
    Configuration 1 (CONF1) 
    Address         : 0000 (Binary)
    Default Value   : A2919A3 (Hex)

    Configures RX and IF Sections, bias settings for individual blocks.
*/
typedef union 
{
    uint32_t reg_value;
    struct {
        unsigned address : 4;    /* Address of the CONF1 Register */
        unsigned FGAIN   : 1;    /* IF Filter Gain Setting */
        unsigned FCENX   : 1;    /* Polyphase Filter Selection */
        unsigned F3OR5   : 1;    /* Filter Order Selection */
        unsigned FBW     : 2;    /* IF Filter Center BW Selection */
        unsigned FCEN    : 6;    /* IF Center Frequency Programming */
        unsigned ANTEN   : 1;    /* Antenna Bias Enable */
        unsigned MIXEN   : 1;    /* Mixer Enable */
        unsigned LNAMODE : 2;    /* LNA Mode Selection */
        unsigned MIXPOLE : 1;    /* Mixer Pole Selection */
        unsigned IMIX    : 2;    /* Mixer Current Programming */
        unsigned ILO     : 2;    /* LO buffer current programming */
        unsigned ILNA2   : 2;    /* LNA2 Current Programming */
        unsigned ILNA1   : 4;    /* LNA1 Current Programming */
	unsigned IDLE    : 1;    /* Idle Enable */
        unsigned CHIPEN  : 1;    /* Chip Enable */
        } parameters;
} reg_CONF1;

/* 
    Configuration 2 (CONF2)
    Address         : 0001 (Binary)
    Default Value   : 0550288 (Hex)

    Configures AGC and output sections.
*/
typedef union 
{
    uint32_t reg_value;
    struct {
        unsigned address : 4;    /* Address of the CONF2 Register */
        unsigned DIEID   : 2;    /* Identifies a version of IC*/
        unsigned RESV_2  : 1;    /* Reserved Register */
        unsigned LOEN    : 1;    /* LO Buffer Enable */
        unsigned DRVCFG  : 2;    /* Output Driver Configuration */
        unsigned BITS    : 3;    /* Number of bits in ADC */
        unsigned FORMAT  : 2;    /* Output Data Format */
        unsigned AGCMODE : 2;    /* AGC Mode Control */
        unsigned RESV_1  : 2;    /* Reserved Registers */
        unsigned GAINREF : 12;   /* AGC Gain Reference Value */
        unsigned IQEN    : 1;    /* I and Q Channels Enable */        
        } parameters;
} reg_CONF2;

/* 
    Configuration 3 (CONF3)
    Address         : 0010 (Binary)
    Default Value   : EAFF1DC (Hex)

    Configures support and test functions for IF filter and AGC.
*/
typedef union 
{
    uint32_t reg_value;
    struct {
        unsigned address     : 4;    /* Address of the CONF3 Register */
        unsigned STRMRST     : 1;    /* Reset all counters */
        unsigned DATSYNCEN   : 1;    /* Enable sync pulses at DATASYC o/p */
        unsigned TIMESYNCEN  : 1;    /* Enable o/p of sync pulses  */
        unsigned STAMPEN     : 1;    /* Insert Frame Number */
        unsigned STRMBITS    : 2;    /* Number of bits streamed */
        unsigned STRMCOUNT   : 3;    /* Length of data counter */
        unsigned STRMSTOP    : 1;    /* Disable data streaming to o/p */
        unsigned STRMSTART   : 1;    /* Enable data streaming to o/p */
        unsigned STRMEN      : 1;    /* DSP i/f for serial streaming */
        unsigned PGAQEN      : 1;    /* Q Channel PGA Enable */
        unsigned PGAIEN      : 1;    /* I Channel PGA Enable */
        unsigned RESV        : 1;    /* Reserved Register*/
        unsigned FHIPEN      : 1;    /* Highpass coupling enable */
        unsigned FILTEN      : 1;    /* IF filter enable */
        unsigned FOFSTEN     : 1;    /* Filter and offset cancellation */
        unsigned DRVEN       : 1;    /* Output driver enable */
        unsigned ADCEN       : 1;    /* ADC Enable */
        unsigned HILOADEN    : 1;    /* Enable output driver */
        unsigned FSLOWEN     : 1;    /* Low value of ADC full-scale enable */
        unsigned GAININ      : 6;    /* PGA gain value programming */
        } parameters;
} reg_CONF3;

/* 
    PLL Configuration (PLLCONF)
    Address         : 0011 (Binary)
    Default Value   : 9EC0008 (Hex)

    PLL, VCO and CLK Settings.
*/
typedef union 
{
    uint32_t reg_value;
    struct {
        unsigned address  : 4;    /* Address of the PLLCONF Register */
        unsigned RESV_5   : 1;    /* Reserved Register */
        unsigned RESV_4   : 1;    /* Reserved Register */
        unsigned PWRSAV   : 1;    /* PLL Power Save Mode */
        unsigned INT_PLL  : 1;    /* PLL Mode Control */
        unsigned CPTEST   : 3;    /* Charge Pump Test */
        unsigned RESV_3   : 1;    /* Reserved Register */
        unsigned PFDEN    : 1;    /* Disable the PLL PFD */
        unsigned ICP      : 1;    /* Charge Pump Current Selection */
        unsigned LDMUX    : 4;    /* LD Pin output selection */
        unsigned XTALCAP  : 5;    /* Digital XTAL Load Cap Programming */
        unsigned IXTAL    : 2;    /* Current programming for XTAL osc */
        unsigned REFDIV   : 2;    /* Clock output divider ratio */
        unsigned RESV_2   : 1;    /* Reserved Register */
        unsigned REFOUTEN : 1;    /* Clock Buffer Enable */
        unsigned RESV_1   : 1;    /* Reserved Register */
        unsigned IVCO     : 1;    /* VCO Current Mode Selection */
        unsigned VCOEN    : 1;    /* VCO Enable */
        } parameters;
} reg_PLLCONF;

/* 
    PLL Integer Division Ratio (DIV)
    Address         : 0100 (Binary)
    Default Value   : 0C00080 (Hex)

    PLL main and reference division ratios, other controls. 
*/
typedef union 
{
    uint32_t reg_value;
    struct {
        unsigned address : 4;    /* Address of the DIV Register */
        unsigned RESV    : 3;    /* Reserved Register */
        unsigned RDIV    : 10;   /* PLL reference division ratio */
        unsigned NDIV    : 15;   /* PLL integer division ratio */
        } parameters;
} reg_DIV;

/* 
    PLL Division Ratio (FDIV)
    Address         : 0101 (Binary)
    Default Value   : 8000070 (Hex)

    PLL fractional division ratio, other controls. 
*/
typedef union 
{
    uint32_t reg_value;
    struct {
        unsigned address : 4;    /* Address of the FDIV Register */
        unsigned RESV    : 8;    /* Reserved Register */
        unsigned FDIV    : 20;   /* PLL reference division ratio */
        } parameters;
} reg_FDIV;

/* 
    DSP Interface (STRM)
    Address         : 0110 (Binary)
    Default Value   : 8000000 (Hex)

    DSP interface number of frames to stream. 
*/
typedef union 
{
    uint32_t reg_value;
    struct {
        unsigned address    : 4;    /* Address of the STRM Register */
        unsigned FRAMECOUNT : 28;   /* Frame number to start streaming */
        } parameters;
} reg_STRM;

/* 
    Clock Fractional Division Ratio (CLK)
    Address         : 0111 (Binary)
    Default Value   : 10061B2 (Hex)

    Fractional clock-divider values.
*/
typedef union 
{
    uint32_t reg_value;
    struct {
        unsigned address : 4;     /* Address of the CLK Register */
        unsigned MODE    : 1;     /* DSP Interface Mode Selection */
        unsigned SERCLK  : 1;     /* Serializer Clock Selection */
        unsigned ADCCLK  : 1;     /* ADC Clock Selection */
        unsigned FCLKIN  : 1;     /* Fractional Clock Divider */
        unsigned M_CNT   : 12;    /* Value for the M Counter */
        unsigned L_CNT   : 12;    /* Value for the L Counter */
        } parameters;
} reg_CLK;

/* 
    Test Mode 1 (TEST1)
    Address         : 1000 (Binary)
    Default Value   : 1E0F401 (Hex)

    Reserved for test mode.
*/
typedef union 
{
    uint32_t reg_value;
    struct {
        unsigned address : 4;    /* Address of the TEST1 Register */
        unsigned RESV    : 28;   /* Reserved Register */
        } parameters;
} reg_TEST1;

/* 
    Test Mode 2 (TEST2)
    Address         : 1001 (Binary)
    Default Value   : 14C0402 (Hex)

    Reserved for test mode.
*/
typedef union 
{
    uint32_t reg_value;
    struct {
        unsigned address : 4;    /* Address of the TEST2 Register */
        unsigned RESV    : 28;   /* Reserved Register */
        } parameters;
} reg_TEST2;

#endif
