% plot_all_f_error_scans.m
%
% Bar-plots every scan in a scans struct array (as returned by
% load_f_error_dump), one figure per scan -- a batch wrapper around
% plot_f_error_scan. Optionally restrict to a subset of scans (e.g. from
% find_f_error_scans) and/or save each figure to disk instead of leaving a
% window open per scan (recommended once there are more than a handful).
%
% Usage:
%   scans = load_f_error_dump();
%   plot_all_f_error_scans(scans);                          % every scan, one window each
%   plot_all_f_error_scans(scans, 0.004);                    % + theoretical sinc^2 overlay (Galileo E1 Td)
%   plot_all_f_error_scans(scans, 0.004, find_f_error_scans(scans, 13));  % only PRN 13's scans
%   plot_all_f_error_scans(scans, 0.004, [], 'plots');       % save PNGs into ./plots/ instead of opening windows
%
% See also: load_f_error_dump, find_f_error_scans, plot_f_error_scan

% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Joel Brenner, 2026. joel.brenner(at)saphyrion.ch
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------

function plot_all_f_error_scans(scans, Td, idxs, save_dir)
    if ~exist('load_f_error_dump.m', 'file')
        addpath('./libs')
    end
    if nargin < 2
        Td = [];
    end
    if nargin < 3 || isempty(idxs)
        idxs = 1:numel(scans);
    end
    if nargin < 4
        save_dir = '';
    end

    if ~isempty(save_dir)
        % note: do NOT set DefaultFigureVisible='off' here -- combined with an
        % invisible figure, saveas()/print() falls back to the fltk renderer,
        % which then fails without a real X display. Leaving figures nominally
        % visible (but headlessly rendered via gnuplot) is what actually works.
        % Use gnuplot only if it is available, and restore the caller's
        % toolkit when this function returns (also on error). If gnuplot is
        % not installed (or graphics_toolkit does not exist, as in MATLAB),
        % keep the current toolkit.
        try
            prev_toolkit = graphics_toolkit();
            graphics_toolkit('gnuplot');
            restore_toolkit = onCleanup(@() graphics_toolkit(prev_toolkit));
        catch
            % keep the current graphics toolkit
        end
        if ~exist(save_dir, 'dir')
            mkdir(save_dir);
        end
    end

    for idx = idxs
        s = scans(idx);
        plot_f_error_scan(scans, idx, Td);

        if ~isempty(save_dir)
            fname = fullfile(save_dir, sprintf('scan%04d_prn%d_ch%d.png', s.scan_id, s.prn, s.channel));
            saveas(gcf(), fname);
            close(gcf());
        end
    end

    if ~isempty(save_dir)
        fprintf('Saved %d plots to %s\n', numel(idxs), save_dir);
    end
end
