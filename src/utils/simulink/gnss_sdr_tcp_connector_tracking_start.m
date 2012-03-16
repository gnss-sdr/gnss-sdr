% /*! 
%  * \file gnss_sdr_tcp_connector_tracking_start.m
%  * \brief This MATLAB function builds and configures a simulink model 
%  * for interacting with the GNSS-SDR platform through a TCP communication.
%  * \author David Pubill, 2012. dpubill(at)cttc.es
%  *
%  * ----------------------------------------------------------------------
%  *
%  * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
%  *
%  * GNSS-SDR is a software defined Global Navigation
%  *          Satellite Systems receiver
%  *
%  * This file is part of GNSS-SDR.
%  *
%  * GNSS-SDR is free software: you can redistribute it and/or modify
%  * it under the terms of the GNU General Public License as published by
%  * the Free Software Foundation, either version 3 of the License, or
%  * at your option) any later version.
%  *
%  * GNSS-SDR is distributed in the hope that it will be useful,
%  * but WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  * GNU General Public License for more details.
%  *
%  * You should have received a copy of the GNU General Public License
%  * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
%  *
%  * ----------------------------------------------------------------------
%  */

function gnss_sdr_tcp_connector_tracking_start(num_channels)
    
    %User parameters 
    host = '84.88.61.86'; %Remote IP address (GNSS-SDR computer IP)
    port = 2060;          %Remote port (GNSS-SDR computer port for Ch0)
    datasize_RX = '28';   %Data size
    timeout = '10';       %Timeout in seconds
    
    % Layout coordinates for the first gnss_sdr_tcp_connector_tracking
    % block and offset definitions
    X0 = 20;
    X1 = 170;
    Y0 = 20;
    Y1 = 140;
    X_offset = 200;
    Y_offset = 160;
    
    %Create a Simulink model
    simulink('open');
    new_system('gnss_sdr_tcp_connector_tracking_aux');
    open_system('gnss_sdr_tcp_connector_tracking_aux');
    
    %Set parameters to configure the model Solver
    set_param('gnss_sdr_tcp_connector_tracking_aux',...
        'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete',...
        'FixedStep', '100', 'StopTime', 'inf');
    
    %Set parameters to avoid warnings in the Command Window
    set_param('gnss_sdr_tcp_connector_tracking_aux',...
        'InheritedTsInSrcMsg', 'none');
    warning('off', 'Simulink:Commands:SetParamLinkChangeWarn');
    
    %Block generation from the Simulink Library
    for i=0:num_channels-1;
        
        name_new_block=['gnss_sdr_tcp_connector_tracking_aux/gnss_sdr_tcp_connector_tracking_',...
            num2str(i)]; 
        
        add_block('simulink/User-Defined Functions/gnss_sdr_tcp_connector_tracking',...
            name_new_block);
                
        name_RX = ['gnss_sdr_tcp_connector_tracking_aux/gnss_sdr_tcp_connector_tracking_',...
            num2str(i),'/gnss_sdr_tcp_connector_tracking_receive'];
        
        set_param(name_RX, 'Port', num2str(port+i), 'Host', host,...
            'DataSize', datasize_RX, 'Timeout', timeout);
               
        name_TX = ['gnss_sdr_tcp_connector_tracking_aux/gnss_sdr_tcp_connector_tracking_',...
            num2str(i),'/gnss_sdr_tcp_connector_tracking_send'];
        
        set_param(name_TX, 'Port', num2str(port+i), 'Host', host,...
            'Timeout', timeout);

        %New layout coordinates for each block
        X2 = X0 + floor(i/4)*X_offset;
        X3 = X1 + floor(i/4)*X_offset;
        Y2 = Y0 + (i-4*floor(i/4))*Y_offset;
        Y3 = Y1 + (i-4*floor(i/4))*Y_offset;

        set_param(name_new_block, 'Position', [X2 Y2 X3 Y3]);
    end
    
    save_system('gnss_sdr_tcp_connector_tracking_aux', 'gnss_sdr_tcp_connector_tracking_ready');
    simulink('close');
    
    %Start Simulink simulation
    set_param('gnss_sdr_tcp_connector_tracking_ready','simulationcommand','start');
    
end
