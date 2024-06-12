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
% SpirentMotion2struct Convert CSV file motionV1.csv to a struct.
%   refSolution = SpirentMotion2struct(path_to_motion_V1_csv) parse the CSV motionV1.CSV to a struct refSolution
%   refSolution has the following fields:
%
%                  - refSolution.SIM_time: simulation time from 0 (in ms)
%                  - refSolution.X: UTM referenced position X (in m)
%                  - refSolution.Y: UTM referenced position Y (in m)
%                  - refSolution.Z: UTM referenced position Z (in m)
%                  - refSolution.vX: referenced Velocity X (in m/s)
%                  - refSolution.vY: referenced Velocity Y (in m/s)
%                  - refSolution.vZ: referenced Velocity Z (in m/s)
%                  - refSolution.aX: referenced Aceleration X (in m/s2)
%                  - refSolution.aY: referenced Aceleration Y (in m/s2)
%                  - refSolution.aZ: referenced Aceleration Z (in m/s2)
%                  - refSolution.jX: referenced Jerk X (in m/s3)
%                  - refSolution.jY: referenced Jerk Y (in m/s3)
%                  - refSolution.jZ: referenced Jerk Z (in m/s3)
%                  - refSolution.latitude:reference Latitude(in degrees),
%                  ellipsoid WGS84?
%                  - refSolution.longitude:reference Longitude(in degrees),
%                  ellipsoid WGS84?
%                  - refSolution.height: referenced Heigh (in m),
%                  ellipsoid WGS84?
%                  - refSolution.dop: referenced antenna DOP 
%
% -------------------------------------------------------------------------
%  USE EXAMPLE: refSolution = SpirentMotion2struct('..\log_spirent\motion_V1_SPF_LD_05.csv')
% -------------------------------------------------------------------------




function [refSolution] = SpirentMotion2struct(path_to_motion_V1_csv)
    
    % if ~exist('gps_l1_ca_read_pvt_raw_dump.m', 'file')
    %     addpath('./libs')
    % end
    % 
    % if ~exist('cat2geo.m', 'file')
    %     addpath('./libs/geoFunctions')
    % end


    %% ===== Import data from text file motion_V1.csv 2 ARRAY ============================
    % Script for importing data from the following text file:
    %
    %    filename: D:\virtualBOX_VM\ubuntu20\ubuntu20\shareFolder\myWork\results\log_spirent\motion_V1.csv
    %
    % Auto-generated by MATLAB on 20-Nov-2022 12:31:17

    % Set up the Import Options and import the data
    opts = delimitedTextImportOptions("NumVariables", 38);

    % Specify range and delimiter
    opts.DataLines = [3, Inf];
    opts.Delimiter = ",";

    % Specify column names and types
    opts.VariableNames = ["Time_ms", "Pos_X", "Pos_Y", "Pos_Z", "Vel_X", "Vel_Y", "Vel_Z", "Acc_X", "Acc_Y", "Acc_Z", "Jerk_X", "Jerk_Y", "Jerk_Z", "Lat", "Long", "Height", "Heading", "Elevation", "Bank", "Angvel_X", "Angvel_Y", "Angvel_Z", "Angacc_X", "Angacc_Y", "Angacc_Z", "Ant1_Pos_X", "Ant1_Pos_Y", "Ant1_Pos_Z", "Ant1_Vel_X", "Ant1_Vel_Y", "Ant1_Vel_Z", "Ant1_Acc_X", "Ant1_Acc_Y", "Ant1_Acc_Z", "Ant1_Lat", "Ant1_Long", "Ant1_Height", "Ant1_DOP"];
    opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

    % Specify file level properties
    opts.ExtraColumnsRule = "ignore";
    opts.EmptyLineRule = "read";

    % Import the data
    motionV1 = readtable(path_to_motion_V1_csv, opts);

    % Convert to output type
    motionV1 = table2array(motionV1);

    % Clear temporary variables
    clear opts

    %% SPIRENT REFERENCE SOLUTION

    refSolution.RX_time=motionV1(:,1); % In Spirent SIM_time

    refSolution.pos_x=motionV1(:,2); % In Spirent X
    refSolution.pos_y=motionV1(:,3); % In Spirent Y
    refSolution.pos_z=motionV1(:,4); % In Spirent Z

    refSolution.vel_x=motionV1(:,5); % In Spirent vX
    refSolution.vel_y=motionV1(:,6); % In Spirent vY
    refSolution.vel_z=motionV1(:,7); % In Spirent vZ

    refSolution.aX=motionV1(:,8);
    refSolution.aY=motionV1(:,9);
    refSolution.aZ=motionV1(:,10);

    refSolution.jX=motionV1(:,11);
    refSolution.jY=motionV1(:,12);
    refSolution.jZ=motionV1(:,13);

    refSolution.latitude=rad2deg(motionV1(:,14));
    refSolution.longitude=rad2deg(motionV1(:,15));
    refSolution.height=motionV1(:,16);

    refSolution.dop=motionV1(:,38);
    
    % Dummy variables to match the same number of variables as SDR PVT
    % file
    refSolution.dummy1 = 0;
    refSolution.dummy2 = 0;
    refSolution.dummy3 = 0;
    refSolution.dummy4 = 0;
    refSolution.dummy5 = 0;
    refSolution.dummy6 = 0;
    refSolution.dummy7 = 0;
    refSolution.dummy8 = 0;
    refSolution.dummy9 = 0;
    refSolution.dummy10 = 0;
    refSolution.dummy11 = 0;

    % Clear temporary variables
    clear motionV1
end
