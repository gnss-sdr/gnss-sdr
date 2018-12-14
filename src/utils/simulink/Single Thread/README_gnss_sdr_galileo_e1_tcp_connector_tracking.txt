 /*! 
  * \file README.txt
  * \brief How to add a block to the Simulink Library repository of Matlab,
  * how to use the "gnss_sdr_galileo_e1_tcp_connector_tracking_start.m" script and how
  * to replace the tracking block of the library.
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
  * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
  *
  * -------------------------------------------------------------------------
  */


IMPORTANT: Please, to use this tracking check the configuration file called
'gnss-sdr_galileo_e1_tcp_connector_tracking.conf'. There are two major changes:
	1.- Choose the [Galileo_E1_TCP_CONNECTOR_Tracking] tracking algorithm.
	2.- Choose a tcp port for channel 0 (e.g. Tracking.port_ch0=2070;) 


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

4.- Open the library model 'gnss_sdr_galileo_e1_tcp_connector_tracking_lib.mdl'
    (Window_2)

5.- If this is not the first time there should be an existing 'gnss-sdr' 
    block in the 'User-Defined Functions' window that should be deleted 
    before drag and drop the new 'gnss_sdr' block (which includes 3 blocks: 
        - 'gnss_sdr_galileo_e1_tcp_connector_tracking_rx' block
        - 'gnss_sdr_galileo_e1_tcp_connector_tracking' block
        - 'gnss_sdr_galileo_e1_tcp_connector_tracking_tx' block) 
    from Window_2 to Window_1. A new message should appear: "This library 
    is locked. The action performed requires it to be unlocked". Then, 
    click on the "Unlock" button (the block will be copied) and close 
    Window_2.

6.- Right-click on the 'gnss-sdr' block and click on "Link Options --> 
    Disable link", repeat the action but now clicking on "Link Options --> 
    Break link". This action disables and breaks the link with the 
    original library model.

7.- On Window_1 save the "simulink/User-Defined Functions" library. 
    To do that go to "File > Save". Then, close Window_1.

8.- From "Simulink Library Browser" window, press F5 to refresh and generate 
    the new Simulink Library repository (it may take a few seconds). This 
    completes the installation of the custom Simulink block.


B) HOW TO use the "gnss_sdr_galileo_e1_tcp_connector_tracking_start.m" script:
   ----------------------------------------------------------------

 ----------------------       ----------------       ----------------------    
|                      |     | gnss_sdr_      |     |                      |    
| gnss_sdr_galileo_e1_ |     | galileo_e1_    |     | gnss_sdr_galileo_e1_ |  
| tcp_connector_       | --> | tcp_connector_ | --> | tcp_connector_       |
| tracking_rx          |     | tracking       |     | tracking_tx          |     
|                      |     |                |     |                      |     
 ----------------------       ----------------       ----------------------     

The 'gnss_sdr_galileo_e1_tcp_connector_tracking_start.m' is the script that builds
and configures a simulink model for interacting with the GNSS-SDR platform 
through a TCP communication. 'User parameters' can be modified but, by 
default, these are the values assigned:

%User parameters 
    host = '84.88.61.86'; %Remote IP address (GNSS-SDR computer IP)
    port = 2070;          %Remote port (GNSS-SDR computer port for Ch0)
    num_vars_rx = 13;     %Number of variables expected from GNSS-SDR
    num_vars_tx = 4;      %Number of variable to be transmitted to GNSS-SDR
    timeout = '10';       %Timeout in seconds

'host', 'port' and 'timeout' parameters configure both 
'gnss_sdr_galileo_e1_tcp_connector_tracking_rx' and 
'gnss_sdr_galileo_e1_tcp_connector_tracking_tx' blocks. The 'port' parameter 
sets the base port number for the first channel (ch0). Each of the subsequent
channels increases their port by one unit (e.g. ch0_port=2070, ch1_port=2071,...).

Also the name of the tracking block can be modified. It must match with 
the Simulink model name:

    %Name of the tracking block, it must match the Simulink model name
    tracking_block_name = 'gnss_sdr_galileo_e1_tcp_connector_tracking';

To run the script just type in the Matlab Command window the following:

>>gnss_sdr_galileo_e1_tcp_connector_tracking_start(N);

where N must match the number of channels configured in the GNSS-SDR
platform.


C) HOW TO replace the tracking block of the library
   ------------------------------------------------

1.- Open the library model 'gnss_sdr_galileo_e1_tcp_connector_tracking_lib.mdl'
2.- Unlock the library. Click on "Edit > Unlock Library".
3.- Open the "gnss-sdr" block and change the "gnss_sdr_galileo_e1_tcp_connector_tracking"
    block by another one. If the name is different it must be updated in
    the "gnss_sdr_galileo_e1_tcp_connector_parallel_tracking_start.m" code (see 
    section B)
4.- Save the new library.
5.- Go to section A and follow the instructions. 

