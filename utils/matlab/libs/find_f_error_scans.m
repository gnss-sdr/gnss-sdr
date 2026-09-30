% find_f_error_scans.m
%
% Returns the indices into a scans struct array (as returned by
% load_f_error_dump) matching a given PRN, optionally restricted to a
% specific channel.
%
% Usage:
%   idxs = find_f_error_scans(scans, 13);        % all scans for PRN 13, any channel
%   idxs = find_f_error_scans(scans, 13, 2);      % PRN 13 on channel 2 only
%
% See also: load_f_error_dump, plot_f_error_scan

% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Joel Brenner, 2026. joel.brenner(at)saphyrion.ch
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------

function idxs = find_f_error_scans(scans, prn, channel)
    mask = [scans.prn] == prn;
    if nargin >= 3 && ~isempty(channel)
        mask = mask & ([scans.channel] == channel);
    end
    idxs = find(mask);
end
