% Miguel Angel Gomez, 2024. gomezlma(at)inta.es
% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------
%
% SpirentSatData2struct Convert CSV file sat_data_V1A1.csv to a struct.
%   refSolution = SpirentSatData2struct(path_to_sat_V1_csv) parse the CSV sat_data_V1A1.CSV to a struct refSatData
%   refSatData has the following fields:
%
% refSatData.GALILEO.series(i).Sat_ID(j) = GAL(k,3);
%             refSatData.GALILEO.series(i).Sat_ID(j)     
%             refSatData.GALILEO.series(i).Sat_PRN(j)   
%             refSatData.GALILEO.series(i).sat_X(j)   
%             refSatData.GALILEO.series(i).sat_Y(j)    	
%             refSatData.GALILEO.series(i).sat_Z(j)   
%             refSatData.GALILEO.series(i).sat_vX(j)    
%             refSatData.GALILEO.series(i).sat_vY(j)    	
%             refSatData.GALILEO.series(i).sat_vZ(j)   	
%             refSatData.GALILEO.series(i).azimuth(j)   	
%             refSatData.GALILEO.series(i).elevation(j)    
%             refSatData.GALILEO.series(i).range(j)    
%             refSatData.GALILEO.series(i).pr_m(j)    
%             refSatData.GALILEO.series(i).pr_rate(j)   
%             refSatData.GALILEO.series(i).iono_delay(j)   
%             refSatData.GALILEO.series(i).tropo_delay(j)    
%             refSatData.GALILEO.series(i).pr_error(j)  	
%             refSatData.GALILEO.series(i).signal_dB(j)   
%             refSatData.GALILEO.series(i).ant_azimuth(j)   	
%             refSatData.GALILEO.series(i).ant_elevation(j)    
%             refSatData.GALILEO.series(i).range_rate(j)    	
%             refSatData.GALILEO.series(i).pr_Error_rate(j)    
%             refSatData.GALILEO.series(i).doppler_shift(j)   
%             refSatData.GALILEO.series(i).delta_range(j)    
%             refSatData.GALILEO.series(i).sat_Acc_X(j)   
%             refSatData.GALILEO.series(i).sat_Acc_Y(j)   	
%             refSatData.GALILEO.series(i).sat_Acc_Z(j)    
% %
% -------------------------------------------------------------------------
%  USE EXAMPLE: refSatData = SpirentSatData2struct('..\log_spirent\sat_data_V1A1_SPF_LD_05.csv')
% -------------------------------------------------------------------------




function [refSatData] = SpirentSatData2struct(path_to_sat_V1_csv)
    
    % if ~exist('gps_l1_ca_read_pvt_raw_dump.m', 'file')
    %     addpath('./libs')
    % end
    % 
    % if ~exist('cat2geo.m', 'file')
    %     addpath('./libs/geoFunctions')
    % end


    %% ===== Import data from text file motion_V1.csv 2 ARRAY ============================

% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 50);

% Specify range and delimiter
opts.DataLines = [6, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Time_ms", "Channel", "Sat_type", "Sat_ID", "Sat_PRN", "Echo_Num", "Sat_Pos_X", "Sat_Pos_Y", "Sat_Pos_Z", "Sat_Vel_X", "Sat_Vel_Y", "Sat_Vel_Z", "Azimuth", "Elevation", "Range", "PRangeGroupA", "PRangeGroupB", "PRangeGroupC", "PRangeGroupD", "PRangeGroupE", "PR_rate", "Iono_delayGroupA", "Iono_delayGroupB", "Iono_delayGroupC", "Iono_delayGroupD", "Iono_delayGroupE", "Tropo_delay", "PR_Error", "Signal_dBGroupA", "Signal_dBGroupB", "Signal_dBGroupC", "Signal_dBGroupD", "Signal_dBGroupE", "Ant_azimuth", "Ant_elevation", "Range_rate", "PR_Error_rate", "Doppler_shiftGroupA", "Doppler_shiftGroupB", "Doppler_shiftGroupC", "Doppler_shiftGroupD", "Doppler_shiftGroupE", "Delta_rangeGroupA", "Delta_rangeGroupB", "Delta_rangeGroupC", "Delta_rangeGroupD", "Delta_rangeGroupE", "Sat_Acc_X", "Sat_Acc_Y", "Sat_Acc_Z"];
opts.VariableTypes = ["double", "double", "char", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, "Sat_type", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "Sat_type", "EmptyFieldRule", "auto");

% Import the data
satdataV1A1SPFLD1 = readtable(path_to_sat_V1_csv, opts);

%% Convert to output type
satdataV1A1SPFLD1 = table2cell(satdataV1A1SPFLD1);
numIdx = cellfun(@(x) ~isnan(str2double(x)), satdataV1A1SPFLD1);
satdataV1A1SPFLD1(numIdx) = cellfun(@(x) {str2double(x)}, satdataV1A1SPFLD1(numIdx));

%% Clear temporary variables
clear opts
%% initialize

refSatData.GPS=[];
refSatData.GALILEO=[];
refSatData.GLONASS=[];
refSatData.BEIDOU=[];

%% split by constellations
[indGALILEO,~]= find(strcmp(satdataV1A1SPFLD1, 'GALILEO'));
[indGPS,~]= find(strcmp(satdataV1A1SPFLD1, 'GPS'));

if(~isempty(indGALILEO))
    GAL=satdataV1A1SPFLD1(indGALILEO,:);GAL(:,3)=[];
    GAL=cell2mat(GAL);
end

if(~isempty(indGPS))
    GPS=satdataV1A1SPFLD1(indGPS,:); GPS(:,3)=[];
    GPS=cell2mat(GPS);
end

if(~isempty(indGALILEO))
    refSatData.GALILEO.SIM_time=unique(GAL(:,1),'first');
    k=0;
    for i=1:length(refSatData.GALILEO.SIM_time)
        nsats=length(find(GAL(:,1)==refSatData.GALILEO.SIM_time(i)));        
        refSatData.GALILEO.series(i).nsats=nsats;
        
        for j=1:nsats
            k=k+1;
            refSatData.GALILEO.series(i).Sat_ID(j) = GAL(k,3);
            refSatData.GALILEO.series(i).Sat_ID(j)     = GAL(i,3);
            refSatData.GALILEO.series(i).Sat_PRN(j)    = GAL(i,4);
            refSatData.GALILEO.series(i).sat_X(j)    = GAL(k,6);	
            refSatData.GALILEO.series(i).sat_Y(j)    = GAL(k,7);	
            refSatData.GALILEO.series(i).sat_Z(j)    = GAL(k,8);	
            refSatData.GALILEO.series(i).sat_vX(j)    = GAL(k,9);	
            refSatData.GALILEO.series(i).sat_vY(j)    = GAL(k,10);	
            refSatData.GALILEO.series(i).sat_vZ(j)    = GAL(k,11);	
            refSatData.GALILEO.series(i).azimuth(j)    = GAL(k,12);	
            refSatData.GALILEO.series(i).elevation(j)    = GAL(k,13);
            refSatData.GALILEO.series(i).range(j)    = GAL(k,14);	
            refSatData.GALILEO.series(i).pr_m(j)    = GAL(k,15); 
            refSatData.GALILEO.series(i).pr_rate(j)    = GAL(k,20);	
            refSatData.GALILEO.series(i).iono_delay(j)    = GAL(k,21); 
            refSatData.GALILEO.series(i).tropo_delay(j)    = GAL(k,26);%		
            refSatData.GALILEO.series(i).pr_error(j)    = GAL(k,27);	
            refSatData.GALILEO.series(i).signal_dB(j)    = GAL(k,28);	
            refSatData.GALILEO.series(i).ant_azimuth(j)    = GAL(k,33);	
            refSatData.GALILEO.series(i).ant_elevation(j)    = GAL(k,34);	
            refSatData.GALILEO.series(i).range_rate(j)    = GAL(k,35);	
            refSatData.GALILEO.series(i).pr_Error_rate(j)    = GAL(k,36);	
            refSatData.GALILEO.series(i).doppler_shift(j)    = GAL(k,37); 
            refSatData.GALILEO.series(i).delta_range(j)    = GAL(k,42); 
            refSatData.GALILEO.series(i).sat_Acc_X(j)    = GAL(k,47);	
            refSatData.GALILEO.series(i).sat_Acc_Y(j)    = GAL(k,48);	
            refSatData.GALILEO.series(i).sat_Acc_Z(j)    = GAL(k,49);
        end

    end
end
    %% -------------------------------------
if(~isempty(indGPS))
    refSatData.GPS.SIM_time=unique(GPS(:,1),'first');
    k=0;
    for i=1:length(refSatData.GPS.SIM_time)
        nsats=length(find(GPS(:,1)==refSatData.GPS.SIM_time(i)));        
        refSatData.GPS.series(i).nsats=nsats;
        
        for j=1:nsats
            
            k=k+1;
            refSatData.GPS.series(i).Sat_ID(j)     = GPS(k,3);
            refSatData.GPS.series(i).Sat_PRN(j)    = GPS(k,4);
            refSatData.GPS.series(i).sat_X(j)    = GPS(k,6);	
            refSatData.GPS.series(i).sat_Y(j)    = GPS(k,7);	
            refSatData.GPS.series(i).sat_Z(j)    = GPS(k,8);	
            refSatData.GPS.series(i).sat_vX(j)    = GPS(k,9);	
            refSatData.GPS.series(i).sat_vY(j)    = GPS(k,10);	
            refSatData.GPS.series(i).sat_vZ(j)    = GPS(k,11);	
            refSatData.GPS.series(i).azimuth(j)    = GPS(k,12);	
            refSatData.GPS.series(i).elevation(j)    = GPS(k,13);
            refSatData.GPS.series(i).range(j)    = GPS(k,14);	
            refSatData.GPS.series(i).pr_m(j)    = GPS(k,15); 
            refSatData.GPS.series(i).pr_rate(j)    = GPS(k,20);	
            refSatData.GPS.series(i).iono_delay(j)    = GPS(k,21); 
            refSatData.GPS.series(i).tropo_delay(j)    = GPS(k,26);%		
            refSatData.GPS.series(i).pr_error(j)    = GPS(k,27);	
            refSatData.GPS.series(i).signal_dB(j)    = GPS(k,28);	
            refSatData.GPS.series(i).ant_azimuth(j)    = GPS(k,33);	
            refSatData.GPS.series(i).ant_elevation(j)    = GPS(k,34);	
            refSatData.GPS.series(i).range_rate(j)    = GPS(k,35);	
            refSatData.GPS.series(i).pr_Error_rate(j)    = GPS(k,36);	
            refSatData.GPS.series(i).doppler_shift(j)    = GPS(k,37); 
            refSatData.GPS.series(i).delta_range(j)    = GPS(k,42); 
            refSatData.GPS.series(i).sat_Acc_X(j)    = GPS(k,47);	
            refSatData.GPS.series(i).sat_Acc_Y(j)    = GPS(k,48);	
            refSatData.GPS.series(i).sat_Acc_Z(j)    = GPS(k,49);
        end
    end
end
    % Clear temporary variables
    clear satdataV1A1SPFLD05 GPS GAL
end