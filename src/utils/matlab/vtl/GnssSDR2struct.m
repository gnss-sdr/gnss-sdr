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
% GnssSDR2struct Convert GNSS-SDR output .mat file PVT.mat to a struct.
%   refSolution = SpirentMotion2struct(path_to_motion_V1_csv) Convert PVT.mat to a struct navSolution
%   refSolution has the following fields:

                    % navSolution.solution_status=solution_status;
                    % navSolution.solution_type=solution_type;
                    % navSolution.valid_sats=valid_sats;
                    % 
                    % navSolution.RX_time=RX_time;
                    % navSolution.TOW_at_current_symbol_ms=TOW_at_current_symbol_ms;
                    % 
                    % navSolution.X=pos_x;
                    % navSolution.Y=pos_y;
                    % navSolution.Z=pos_z;
                    % 
                    % navSolution.latitude=latitude;
                    % navSolution.longitude=longitude;
                    % navSolution.height=height;
                    % 
                    % navSolution.pdop=pdop;
                    % navSolution.gdop=gdop;
                    % navSolution.hdop=hdop;
                    % 
                    % navSolution.vX=vel_x;
                    % navSolution.vY=vel_y;
                    % navSolution.vZ=vel_z;
                    % 
                    % navSolution.vdop=vdop;
                    % 
                    % navSolution.week=week;
%
% -------------------------------------------------------------------------
%  USE EXAMPLE: navSolution = GnssSDR2struct('PVT_raw.mat')
% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------
% SPDX-FileCopyrightText: 2022 gomezlma(at)inta.es
% SPDX-License-Identifier: GPL-3.0-or-later



function [navSolution] = GnssSDR2struct(PVT_file_Path)

load(PVT_file_Path, ...
'RX_time' ,'TOW_at_current_symbol_ms' ,'pos_x','pos_y' ,'pos_z',...
'latitude' ,'longitude','height' ,'pdop' ,'gdop' ,'hdop' ,'vel_x' ,...
'vel_y' ,'vel_z' ,'vdop' ,'week','solution_status','solution_type',...
'valid_sats')

%% GNSS SDR SOLUTION

navSolution.solution_status=solution_status;
navSolution.solution_type=solution_type;
navSolution.valid_sats=valid_sats;

navSolution.RX_time=RX_time;
navSolution.TOW_at_current_symbol_ms=TOW_at_current_symbol_ms;

navSolution.X=pos_x;
navSolution.Y=pos_y;
navSolution.Z=pos_z;

navSolution.latitude=latitude;
navSolution.longitude=longitude;
navSolution.height=height;

navSolution.pdop=pdop;
navSolution.gdop=gdop;
navSolution.hdop=hdop;

navSolution.vX=vel_x;
navSolution.vY=vel_y;
navSolution.vZ=vel_z;

navSolution.vdop=vdop;

navSolution.week=week;

%% clear tmp variables
clearvars -except navSolution 
end
