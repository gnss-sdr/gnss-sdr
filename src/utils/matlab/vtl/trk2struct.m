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

function [trkSolution] = trk2struct(path_to_trk_csv)
%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 5);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["doppler_corr", "VarName2", "VarName3", "VarName4","VarName5","VarName5"];
opts.VariableTypes = ["char", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, "doppler_corr", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "doppler_corr", "EmptyFieldRule", "auto");

% Import the data
dumptrackingfile = readtable(path_to_trk_csv, opts);

%% Convert to output type
dumptrackingfile = table2cell(dumptrackingfile);
numIdx = cellfun(@(x) ~isnan(str2double(x)), dumptrackingfile);
dumptrackingfile(numIdx) = cellfun(@(x) {str2double(x)}, dumptrackingfile(numIdx));

%% Clear temporary variables
clear numIdx opts

trkSolution.dopp=[];

%% split by solution type
[indDopp,~]= find(strcmp(dumptrackingfile, 'doppler_corr'));

trk_dopp=dumptrackingfile(indDopp,:);trk_dopp(:,1)=[];

trk_dopp=cell2mat(trk_dopp);

for i=1:length(trk_dopp)
    trkSolution.dopp.real=[trk_dopp(:,1) trk_dopp(:,3)];
    trkSolution.dopp.cmd=[trk_dopp(:,1) trk_dopp(:,2)];

    trkSolution.doppShift.real=[trk_dopp(:,1) trk_dopp(:,4)];
    trkSolution.tao.real=[trk_dopp(:,1) trk_dopp(:,5)];
end
end
