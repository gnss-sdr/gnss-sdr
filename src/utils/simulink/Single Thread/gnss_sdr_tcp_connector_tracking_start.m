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
%  * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
%  *
%  * ----------------------------------------------------------------------
%  */

function gnss_sdr_tcp_connector_tracking_start(num_channels)
    
    %User parameters 
    host = '84.88.61.86'; %Remote IP address (GNSS-SDR computer IP)
    port = 2070;          %Remote port (GNSS-SDR computer port for Ch0)
    num_vars_rx = 9;      %Number of variables expected from GNSS-SDR
    num_vars_tx = 4;      %Number of variable to be transmitted to GNSS-SDR
    timeout = '40';       %Timeout [s]
    
    %name of the tracking block, it must match the name of the Simulink
    %model
    tracking_block_name = 'gnss_sdr_tcp_connector_tracking';
    
    % Layout coordinates for the first gnss_sdr_tcp_connector_tracking
    % block and offset definitions
    X0 = 20;
    X1 = 170;
    Y0 = 20;
    Y1 = 140;
    X_offset = 200;
    Y_offset = 160;
    
    %Calculate the size of the data received from GNSS-SDR 
    %(float = 4 bytes each variable)
    datasize_RX = num_vars_rx*4;
    
    %Create a Simulink model
    simulink('open');
    new_system('gnss_sdr_tcp_connector_tracking_aux');
    open_system('gnss_sdr_tcp_connector_tracking_aux');
    
    %Set parameters to avoid warnings in the Command Window
    set_param('gnss_sdr_tcp_connector_tracking_aux',...
        'InheritedTsInSrcMsg', 'none');
    warning('off', 'Simulink:Commands:SetParamLinkChangeWarn');
    
    %Assign values to the variables used by Simulink in the base workspace
    assignin('base', 'Ti', 1e-3);
    assignin('base', 'f0', 1.57542e9);
    assignin('base', 'SFunSlope', 3.5);
    assignin('base', 'Tc', 4e-3/4092);
    assignin('base', 'T', 1e-3);
    assignin('base', 'B_PLL', 50);
    assignin('base', 'B_DLL', 2);
    
    %Block generation from the Simulink Library
    for i = 0:num_channels-1;
           
        %Add and prepare an empty block to become the TCP connector block
        tcp_connector_block=['gnss_sdr_tcp_connector_tracking_aux/gnss_sdr_tcp_connector_tracking_', num2str(i)]; 
        
        add_block('simulink/Ports & Subsystems/Subsystem', tcp_connector_block);
        delete_line(tcp_connector_block,'In1/1', 'Out1/1')
        
        tcp_connector_tracking_i_In1 = ['gnss_sdr_tcp_connector_tracking_aux/gnss_sdr_tcp_connector_tracking_',num2str(i),'/In1'];
        tcp_connector_tracking_i_Out1 = ['gnss_sdr_tcp_connector_tracking_aux/gnss_sdr_tcp_connector_tracking_',num2str(i),'/Out1'];
        
        delete_block(tcp_connector_tracking_i_In1);
        delete_block(tcp_connector_tracking_i_Out1);
       
        %Add to the TCP connector block the receiver, the tracking and the
        %transmitter blocks
        tcp_connector_tracking_rx_block = ['gnss_sdr_tcp_connector_tracking_aux/gnss_sdr_tcp_connector_tracking_',num2str(i),'/gnss_sdr_tcp_connector_tracking_rx'];
        tcp_connector_tracking_block = ['gnss_sdr_tcp_connector_tracking_aux/gnss_sdr_tcp_connector_tracking_',num2str(i),'/',tracking_block_name];
        tcp_connector_tracking_tx_block = ['gnss_sdr_tcp_connector_tracking_aux/gnss_sdr_tcp_connector_tracking_',num2str(i),'/gnss_sdr_tcp_connector_tracking_tx'];
        
        add_block('simulink/User-Defined Functions/gnss_sdr/gnss_sdr_tcp_connector_tracking_rx',tcp_connector_tracking_rx_block);
        
        path_to_tracking_block = ['simulink/User-Defined Functions/gnss_sdr/', tracking_block_name];
        add_block(path_to_tracking_block, tcp_connector_tracking_block);
        
        add_block('simulink/User-Defined Functions/gnss_sdr/gnss_sdr_tcp_connector_tracking_tx',tcp_connector_tracking_tx_block);
                
        %Connect the receiver block to the tracking block
        for j=1:num_vars_rx;
            rx_out_ports =['gnss_sdr_tcp_connector_tracking_rx/',num2str(j)];
            tracking_in_ports =[tracking_block_name,'/',num2str(j)];
            
            add_line(tcp_connector_block, rx_out_ports, tracking_in_ports)
        end
        
        %Connect the tracking block to the transmitter block
        for k=1:num_vars_tx;
            tracking_out_ports =[tracking_block_name,'/',num2str(k)];
            tx_in_ports =['gnss_sdr_tcp_connector_tracking_tx/',num2str(k)];
            
            add_line(tcp_connector_block, tracking_out_ports, tx_in_ports)
        end
        
        %Add, place and connect two scopes in the TCP connector block
        name_scope_1 = [tcp_connector_block,'/Scope'];
        add_block('simulink/Sinks/Scope', name_scope_1, 'Position', [500 300 550 350]);
        set_param(name_scope_1, 'NumInputPorts', '4', 'LimitDataPoints', 'off');
        add_line(tcp_connector_block, 'gnss_sdr_tcp_connector_tracking_rx/9', 'Scope/1', 'autorouting','on')
        
        tracking_scope_port2 = [tracking_block_name,'/2'];
        add_line(tcp_connector_block, tracking_scope_port2, 'Scope/2', 'autorouting','on')
        tracking_scope_port3 = [tracking_block_name,'/3'];
        add_line(tcp_connector_block, tracking_scope_port3, 'Scope/3', 'autorouting','on')
        tracking_scope_port4 = [tracking_block_name,'/4'];
        add_line(tcp_connector_block, tracking_scope_port4, 'Scope/4', 'autorouting','on')
        
        name_scope_2 = [tcp_connector_block,'/EPL'];
        add_block('simulink/Sinks/Scope', name_scope_2, 'Position', [500 400 550 450]);
        set_param(name_scope_2, 'LimitDataPoints', 'off');
        tracking_scope2_port5 = [tracking_block_name,'/5'];
        add_line(tcp_connector_block, tracking_scope2_port5, 'EPL/1', 'autorouting','on')
        
        %Set the TCP receiver parameters
        tcp_receiver = ['gnss_sdr_tcp_connector_tracking_aux/gnss_sdr_tcp_connector_tracking_',num2str(i),'/gnss_sdr_tcp_connector_tracking_rx/RX'];
        set_param(tcp_receiver, 'Port', num2str(port+i), 'Host', host, 'DataSize', num2str(datasize_RX), 'Timeout', timeout);
        
        %Set the TCP transmitter parameters
        tcp_transmitter = ['gnss_sdr_tcp_connector_tracking_aux/gnss_sdr_tcp_connector_tracking_',num2str(i),'/gnss_sdr_tcp_connector_tracking_tx/TX'];
        set_param(tcp_transmitter, 'Port', num2str(port+i), 'Host', host,'Timeout', timeout);

        %New layout coordinates for each block
        X2 = X0 + floor(i/4)*X_offset;
        X3 = X1 + floor(i/4)*X_offset;
        Y2 = Y0 + (i-4*floor(i/4))*Y_offset;
        Y3 = Y1 + (i-4*floor(i/4))*Y_offset;

        %Place the block in the layout
        set_param(tcp_connector_block, 'Position', [X2 Y2 X3 Y3]);
    end
    
    %Set parameters to configure the model Solver
    set_param('gnss_sdr_tcp_connector_tracking_aux',...
        'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete',...
        'FixedStep', 'auto', 'StopTime', 'inf');
    
    %Save the model with a definitive name
    save_system('gnss_sdr_tcp_connector_tracking_aux', 'gnss_sdr_tcp_connector_tracking_ready');
    simulink('close');
    
    %Run the Simulink model
    set_param('gnss_sdr_tcp_connector_tracking_ready','simulationcommand','start');
    
end
