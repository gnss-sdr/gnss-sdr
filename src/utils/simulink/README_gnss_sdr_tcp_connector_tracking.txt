 /*! 
  * \file README.txt
  * \brief How to add a block to the Simulink Library repository of Matlab
  * and how to use the "gnss_sdr_tcp_connector_tracking_start.m" script.
  *
  * \author David Pubill, 2012. dpubill(at)cttc.es
  *
  * -------------------------------------------------------------------------
  *
  * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
  *
  * GNSS-SDR is a software defined Global Navigation
  *          Satellite Systems receiver
  *
  * This file is part of GNSS-SDR.
  *
  * GNSS-SDR is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * at your option) any later version.
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


IMPORTANT: Please, to use this tracking check the configuration file called
'gnss-sdr_tcp_connector_tracking.conf'. There are two major changes:
	1.- Choose the [GPS_L1_CA_TCP_CONNECTOR_Tracking] tracking algorithm.
	2.- Choose a tcp port for channel 0 (e.g. Tracking.port_ch0=2060;) 
	

A) HOW TO add a block to the Simulink Library repository of your Matlab installation
   ---------------------------------------------------------------------------------
 (These steps should be followed only the first time)

1.- Copy the content of this folder to a folder accessible from Simulink.

2.- In the Matlab Command Window type:
	>> simulink;
    to open the Simulink Library Browser.

3.- Right-click on the Simulink/User-Defined Functions of the Simulink 
    Library menu, and click on "Open User-Defined Functions library" 
    (Window_1)

4.- Open the library model 'gnss_sdr_tcp_connector_tracking_lib.mdl'(Window_2)

5.- Drag and drop the gnss_sdr_tcp_connector_tracking block from Window_2 
    to Window_1. A new message should appear: "This library is locked. The 
    action performed requires it to be unlocked". Then, click on the "Unlock" 
    button (the block will be copied) and close Window_2.

6.- On Window_1 save the "simulink/User-Defined Functions" library. 
    To do that go to "File > Save". Then, close Window_1.

7.- From "Simulink Library Browser" window, press F5 to refresh and generate 
    the new Simulink Library repository. This may take a few seconds, and 
    this finish the installation of the custom Simulink block.


B) HOW TO use the "gnss_sdr_tcp_connector_tracking_start.m" script:
   ----------------------------------------------------------------

-----------------------     ------------------     -----------------------    
|                     |     |                |     |                     |    
| gnss_sdr_tcp_       |     |                |     | gnss_sdr_tcp_       |  
| connector_tracking_ | --> |      Core      | --> | connector_tracking_ |
| receive             |     |                |     | send                |     
|                     |     |                |     |                     |     
-----------------------     ------------------     -----------------------     

The 'gnss_sdr_tcp_connector_tracking_start.m' is the script that builds and
configures a simulink model for interacting with the GNSS-SDR platform 
through a TCP communication. 'User parameters' can be modified but, by 
default, these are the values assigned:
 
%User parameters 
    host = '84.88.61.86'; //Remote IP address (GNSS-SDR computer IP)
    port = 2060;          //Remote port (GNSS-SDR computer port for Ch0)
    datasize_RX = '28';   //Data size
    timeout = '10';       //Timeout in seconds

'host', 'port' and 'timeout' parameters configure both 'gnss_sdr_tcp_connector_tracking_receive'
and 'gnss_sdr_tcp_connector_tracking_send' blocks. The 'port' parameter 
sets the base port number for the first channel (ch0). Each of the 
subsequent channels increases their port by one unit (e.g. ch0_port=2060, 
ch1_port=2061,...)

'datasize_RX' is the size (in bytes) of the received TCP packet data field. 
For example, if the number of float (4 bytes) variables to be received from 
the the GNSS-SDR computer is 7, this parameter must be set to 7*4=28.

To run the script just type in the Matlab Command window the following:

>>gnss_sdr_tcp_connector_tracking_start(N);

where N must match the number of channels configured in the GNSS-SDR
platform.







