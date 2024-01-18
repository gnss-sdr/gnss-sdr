%% Import data from text file
% Script for importing data from the following text file:
%
%    filename: D:\virtualBOX_VM\ubuntu20\ubuntu20\shareFolder\myWork\results\spirent_usrp_airing\dump_vtl_file.csv
%
% %
% -------------------------------------------------------------------------
%  USE EXAMPLE: vtlSolution = Vtl2struct('dump_vtl_file.csv')
% -------------------------------------------------------------------------
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
function [vtlSolution] = Vtl2struct(path_to_vtl_csv)
%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 9);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["kf_state", "e06", "VarName3", "e06_1", "VarName5", "VarName6", "VarName7", "VarName8", "VarName9"];
opts.VariableTypes = ["char", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, "kf_state", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "kf_state", "EmptyFieldRule", "auto");

% Import the data
dumpvtlfile = readtable(path_to_vtl_csv, opts);

%% Convert to output type
dumpvtlfile = table2cell(dumpvtlfile);
numIdx = cellfun(@(x) ~isnan(str2double(x)), dumpvtlfile);
dumpvtlfile(numIdx) = cellfun(@(x) {str2double(x)}, dumpvtlfile(numIdx));

%% Clear temporary variables
clear opts
%%

vtlSolution.kfpvt=[];
vtlSolution.rtklibpvt=[];

%% split by solution type
[indKF,~]= find(strcmp(dumpvtlfile, 'kf_state'));
[indRTKlib,~]= find(strcmp(dumpvtlfile, 'rtklib_state'));
[indkf_err,~]= find(strcmp(dumpvtlfile, 'kf_xerr'));
[ind_filt_dop_sat,~]= find(strcmp(dumpvtlfile, 'filt_dopp_sat'));
[ind_CN0_sat,~]= find(strcmp(dumpvtlfile, 'CN0_sat'));


kfpvt=dumpvtlfile(indKF,:);kfpvt(:,1)=[];
rtklibpvt=dumpvtlfile(indRTKlib,:); rtklibpvt(:,1)=[];
kferr=dumpvtlfile(indkf_err,:); kferr(:,1)=[];
filt_dop_sat=dumpvtlfile(ind_filt_dop_sat,:); filt_dop_sat(:,1)=[];
CN0_sat=dumpvtlfile(ind_CN0_sat,:); CN0_sat(:,1)=[];

kfpvt=cell2mat(kfpvt);
rtklibpvt=cell2mat(rtklibpvt);
kferr=cell2mat(kferr);
filt_dop_sat=cell2mat(filt_dop_sat);
CN0_sat=cell2mat(CN0_sat);

vtlSolution.kfpvt.X=kfpvt(:,1);
vtlSolution.kfpvt.Y=kfpvt(:,2);
vtlSolution.kfpvt.Z=kfpvt(:,3);
vtlSolution.kfpvt.vX=kfpvt(:,4);
vtlSolution.kfpvt.vY=kfpvt(:,5);
vtlSolution.kfpvt.vZ=kfpvt(:,6);
vtlSolution.kfpvt.biasclock=kfpvt(:,7);
vtlSolution.kfpvt.rateblock=kfpvt(:,8);

vtlSolution.rtklibpvt.X=rtklibpvt(:,1);
vtlSolution.rtklibpvt.Y=rtklibpvt(:,2);
vtlSolution.rtklibpvt.Z=rtklibpvt(:,3);
vtlSolution.rtklibpvt.vX=rtklibpvt(:,4);
vtlSolution.rtklibpvt.vY=rtklibpvt(:,5);
vtlSolution.rtklibpvt.vZ=rtklibpvt(:,6);
vtlSolution.rtklibpvt.biasclock=rtklibpvt(:,7);
vtlSolution.rtklibpvt.rateblock=rtklibpvt(:,8);

vtlSolution.kferr.X=kferr(:,1);
vtlSolution.kferr.Y=kferr(:,2);
vtlSolution.kferr.Z=kferr(:,3);
vtlSolution.kferr.vX=kferr(:,4);
vtlSolution.kferr.vY=kferr(:,5);
vtlSolution.kferr.vZ=kferr(:,6);
vtlSolution.kferr.biasclock=kferr(:,7);
vtlSolution.kferr.rateblock=kferr(:,8);

vtlSolution.filt_dop_sat(1,:)=filt_dop_sat(:,1);
vtlSolution.filt_dop_sat(2,:)=filt_dop_sat(:,2);
vtlSolution.filt_dop_sat(3,:)=filt_dop_sat(:,3);
vtlSolution.filt_dop_sat(4,:)=filt_dop_sat(:,4);
vtlSolution.filt_dop_sat(5,:)=filt_dop_sat(:,5);

vtlSolution.CN0_sat(1,:)=CN0_sat(:,1);
vtlSolution.CN0_sat(2,:)=CN0_sat(:,2);
vtlSolution.CN0_sat(3,:)=CN0_sat(:,3);
vtlSolution.CN0_sat(4,:)=CN0_sat(:,4);
vtlSolution.CN0_sat(5,:)=CN0_sat(:,5);

end
