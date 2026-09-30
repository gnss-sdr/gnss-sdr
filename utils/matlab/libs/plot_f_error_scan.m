% plot_f_error_scan.m
%
% Bar-plots one state-5 frequency-error-reduction scan (as loaded by
% load_f_error_dump), with the selected/winning Doppler marked. Optionally
% overlays the theoretical sinc(pi*Delta_f*Td)^2 envelope (normalized to the
% scan's own peak) for comparison, if the coherent integration time Td [s] is
% given -- e.g. Td=0.004 for Galileo E1, Td=0.001 for GPS L1 C/A.
%
% Usage:
%   scans = load_f_error_dump();
%   plot_f_error_scan(scans, 5);              % plot scans(5)
%   plot_f_error_scan(scans, 5, 0.004);        % + theoretical Galileo E1 sinc^2 overlay
%
% See also: load_f_error_dump, find_f_error_scans

% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Joel Brenner, 2026. joel.brenner(at)saphyrion.ch
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------

function plot_f_error_scan(scans, idx, Td)
    if idx < 1 || idx > numel(scans)
        error('plot_f_error_scan: idx %d out of range (1..%d)', idx, numel(scans));
    end
    s = scans(idx);

    figure();
    bar(s.doppler_hz, s.power);
    hold on;

    if nargin >= 3 && ~isempty(Td)
        % theoretical envelope centered on the selected (winning) Doppler, since that's
        % this scan's best estimate of the true residual frequency
        theta = pi * (s.doppler_hz - s.selected_doppler_hz) * Td;
        sinc0 = @(x) (x == 0) * 1 + (x ~= 0) .* (sin(x + (x == 0)) ./ (x + (x == 0)));
        envelope = (sinc0(theta) .^ 2) * max(s.power);
        plot(s.doppler_hz, envelope, 'k--', 'LineWidth', 1.3);
    end

    yl = ylim();
    plot([s.selected_doppler_hz, s.selected_doppler_hz], yl, 'r--', 'LineWidth', 1.5);
    hold off;

    xlabel('Doppler tested [Hz]');
    ylabel('Accumulated correlation power');
    if nargin >= 3 && ~isempty(Td)
        legend('bin power', 'theoretical sinc^2 (centered on selected)', 'selected Doppler', 'Location', 'northeast');
    else
        legend('bin power', 'selected Doppler', 'Location', 'northeast');
    end
    title(sprintf('PRN%d ch%d scan %d: CN0=%.1f dB-Hz, selected=%.1f Hz (bin %d/%d)', ...
        s.prn, s.channel, s.scan_id, s.cn0_dBHz, s.selected_doppler_hz, s.selected_bin, numel(s.doppler_hz) - 1));
    grid on;
end
