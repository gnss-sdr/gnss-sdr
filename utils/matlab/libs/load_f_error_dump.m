% load_f_error_dump.m
%
% Loads the CSV written by dll_pll_veml_tracking's state-5 (frequency error
% reduction) diagnostic -- see Dll_Pll_Conf::f_error_dump_filename -- into an
% Octave struct array, one element per scan (one satellite's pass through
% state 5). Each row of the CSV is one Doppler bin; this groups rows by
% scan_id.
%
% CSV columns (header row is skipped):
%   scan_id, prn, system_char, channel, cn0_dBHz, selected_doppler_hz,
%   selected_bin, bin_index, doppler_hz, power
%
% Usage:
%   scans = load_f_error_dump();                    % default: ./f_error_dump.csv
%   scans = load_f_error_dump('/path/to/dump.csv');
%   load_f_error_dump();                             % no output arg: also prints a summary table
%
% Each scans(i) has fields:
%   scan_id, prn, system_char, channel, cn0_dBHz, selected_doppler_hz, selected_bin,
%   doppler_hz (row vector, increasing Doppler order), power (row vector, same order)
%
% See also: find_f_error_scans, plot_f_error_scan

% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Joel Brenner, 2026. joel.brenner(at)saphyrion.ch
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------

function scans = load_f_error_dump(filename)
    if nargin < 1 || isempty(filename)
        filename = 'f_error_dump.csv';
    end

    raw = csvread(filename, 1, 0);  % skip header row
    if isempty(raw)
        error('load_f_error_dump: no data rows found in %s', filename);
    end

    scan_id_col = raw(:, 1);
    unique_ids = unique(scan_id_col);
    n = numel(unique_ids);

    scans = struct('scan_id', {}, 'prn', {}, 'system_char', {}, 'channel', {}, 'cn0_dBHz', {}, ...
                    'selected_doppler_hz', {}, 'selected_bin', {}, ...
                    'doppler_hz', {}, 'power', {});

    for i = 1:n
        rows = raw(scan_id_col == unique_ids(i), :);
        [~, order] = sort(rows(:, 8));  % bin_index, defensive (writer already emits increasing Doppler order)
        rows = rows(order, :);

        scans(i).scan_id              = rows(1, 1);
        scans(i).prn                  = rows(1, 2);
        scans(i).system_char          = rows(1, 3);
        scans(i).channel              = rows(1, 4);
        scans(i).cn0_dBHz             = rows(1, 5);
        scans(i).selected_doppler_hz  = rows(1, 6);
        scans(i).selected_bin         = rows(1, 7);
        scans(i).doppler_hz           = rows(:, 9)';
        scans(i).power                = rows(:, 10)';
    end

    if nargout == 0
        fprintf('%6s %5s %4s %9s %14s %8s %6s\n', 'scan', 'PRN', 'ch', 'CN0[dBHz]', 'selDoppler[Hz]', 'nBins', '');
        for i = 1:n
            fprintf('%6d %5d %4d %9.2f %14.2f %8d\n', scans(i).scan_id, scans(i).prn, scans(i).channel, ...
                scans(i).cn0_dBHz, scans(i).selected_doppler_hz, numel(scans(i).doppler_hz));
        end
        fprintf('\n%d scans loaded from %s\n', n, filename);
        fprintf('PRNs present: %s\n', mat2str(unique([scans.prn])));
        clear scans;  % avoid echoing the struct array to the console when called as a script
    end
end
