% f_error_sim.m
%
% Octave Monte Carlo simulation of the tracking "frequency error reduction"
% state (state 5) implemented in dll_pll_veml_tracking.cc: right after
% pull-in, a passive scan tests f_error_step_num Doppler bins centered on the
% pull-in Doppler estimate, spaced f_error_doppler_step Hz apart
% (bin 0 -> +0, then alternating +step, -step, +2*step, -2*step, ...,
% matching dll_pll_veml_tracking::f_error_bin_multiplier()). Each bin is
% incoherently accumulated for f_error_accumulation code periods (|Prompt|^2
% summed), and the bin with the highest accumulated power is kept as the
% corrected Doppler.
%
% This script reproduces that exact bin geometry and accumulation logic on
% simulated noisy correlator samples, for a given C/N0 and initial (pull-in)
% frequency error, fixed to the Galileo E1 code period (Td = 4 ms). It also
% reproduces the CN0 M2M4 estimator (lock_detectors.cc::cn0_m2m4_estimator)
% applied to the winning bin's raw samples, matching the cn0_dBHz column of
% the receiver's f_error_dump CSV (Tracking_<Sig>.f_error_dump=true).
%
% Usage:
%   f_error_sim()                                     % all defaults
%   f_error_sim(CN0_dBHz, f_error_step_num, f_error_accumulation, ...
%               f_error_doppler_step, f_error_init_Hz, n_trials, seed)
%
%   f_error_sim(35, 9, 20, 62.5, 150)                 % 9 bins at 62.5 Hz, 150 Hz initial error
%   f_error_sim(30, 5, 10, 125, -300, 5000, 2)
%
% What it prints:
%   - the bin table (index, multiplier, Doppler tested, noiseless sinc loss)
%   - Monte Carlo statistics: residual Doppler error after the scan, bin
%     "capture" probability (did it pick the bin closest to the true
%     frequency?), and CN0 estimate bias/spread on the winning bin
%
% What it plots:
%   1) One example trial's per-bin accumulated power vs. Doppler tested,
%      with the true frequency marked -- the same data the receiver writes to
%      the doppler_hz / power columns of its f_error_dump CSV (there as
%      absolute Doppler values rather than offsets from the pull-in estimate).
%   2) Histogram of the residual Doppler error left after the scan, over all
%      Monte Carlo trials.

% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Joel Brenner, 2026. joel.brenner(at)saphyrion.ch
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------

function f_error_sim(CN0_dBHz, f_error_step_num, f_error_accumulation, f_error_doppler_step, f_error_init_Hz, n_trials, seed)

    %% ------------------------- PARAMETERS ----------------------------------
    if nargin < 1 || isempty(CN0_dBHz),            CN0_dBHz            = 35;    end  % [dB-Hz]
    if nargin < 2 || isempty(f_error_step_num),     f_error_step_num    = 9;     end  % number of Doppler bins
    if nargin < 3 || isempty(f_error_accumulation), f_error_accumulation = 20;   end  % incoherent accum. per bin [code periods]
    if nargin < 4 || isempty(f_error_doppler_step), f_error_doppler_step = 62.5; end  % [Hz] bin spacing
    if nargin < 5 || isempty(f_error_init_Hz),      f_error_init_Hz      = 150;  end  % [Hz] true residual Doppler at pull-in
    if nargin < 6 || isempty(n_trials),             n_trials             = 2000; end  % Monte Carlo trial count
    if nargin < 7 || isempty(seed),                 seed                 = 1;    end  % RNG seed, [] = random
    %% ------------------------------------------------------------------------

    if ~isempty(seed)
        if exist('rng') ~= 0
            % MATLAB, and Octave >= 7: seed the Mersenne Twister without
            % switching MATLAB to its legacy generators
            rng(seed, 'twister');
        else
            % Octave < 7 has no rng(); this is what rng(seed, 'twister') does there
            rand('state', seed);
            randn('state', seed);
        end
    end

    Td = 4e-3;  % Galileo E1 code period [s] -- fixed per this script's scope

    % mirror Dll_Pll_Conf::SetFromConfiguration(): f_error_step_num must be odd
    % (a center bin plus a symmetric number of +/- steps); round up if even.
    f_error_step_num = max(1, round(f_error_step_num));
    if mod(f_error_step_num, 2) == 0
        f_error_step_num = f_error_step_num + 1;
        fprintf('note: f_error_step_num rounded up to %d (must be odd)\n', f_error_step_num);
    end
    num_bins = f_error_step_num;

    % bin 0 -> 0; then alternating outward: +1, -1, +2, -2, +3, -3, ...
    % (matches dll_pll_veml_tracking::f_error_bin_multiplier(), 0-based bin_index)
    bin_mult = zeros(1, num_bins);
    for idx = 1:num_bins
        bin_index = idx - 1;
        if bin_index == 0
            bin_mult(idx) = 0;
        else
            half_steps = floor((bin_index + 1) / 2);
            if mod(bin_index, 2) == 1
                bin_mult(idx) = half_steps;
            else
                bin_mult(idx) = -half_steps;
            end
        end
    end
    bin_doppler_offset = bin_mult * f_error_doppler_step;               % [Hz], relative to pull-in estimate
    bin_residual_error  = f_error_init_Hz - bin_doppler_offset;         % [Hz], true - tested, per bin

    sinc0 = @(x) (x == 0) * 1 + (x ~= 0) .* (sin(x + (x == 0)) ./ (x + (x == 0)));
    A = sqrt(2 * 10^(CN0_dBHz / 10) * Td);  % correlator amplitude; SNR = 2*(C/N0)*Td, unit-variance complex noise per branch

    fprintf('--- f_error_sim: CN0=%.1f dB-Hz, f_error_step_num=%d, f_error_accumulation=%d, f_error_doppler_step=%.2f Hz, f_error_init=%.1f Hz, Td=%.1f ms (Galileo E1) ---\n', ...
           CN0_dBHz, num_bins, f_error_accumulation, f_error_doppler_step, f_error_init_Hz, Td * 1e3);

    % noiseless per-bin loss, for the printed table
    theta_bins = pi * bin_residual_error * Td;
    bin_loss_dB = 20 * log10(abs(sinc0(theta_bins)) + eps);

    fprintf('\n bin   mult   Doppler[Hz]   residual[Hz]   sinc loss[dB]\n');
    [~, ideal_bin] = min(abs(bin_residual_error));
    for idx = 1:num_bins
        marker = '';
        if idx == ideal_bin, marker = '  <- closest to truth'; end
        fprintf('  %2d   %+3d    %8.2f      %8.2f        %6.2f%s\n', ...
               idx - 1, bin_mult(idx), bin_doppler_offset(idx), bin_residual_error(idx), bin_loss_dB(idx), marker);
    end

    %% --------------------------- Monte Carlo ---------------------------------
    winning_bin      = zeros(1, n_trials);
    residual_after   = zeros(1, n_trials);
    cn0_est_dBHz     = zeros(1, n_trials);
    example_power    = [];  % saved from trial 1, for plotting

    for t = 1:n_trials
        bin_power = zeros(1, num_bins);
        bin_samples = cell(1, num_bins);
        for b = 1:num_bins
            theta = pi * bin_residual_error(b) * Td;
            amp   = A * sinc0(theta);
            k     = (1:f_error_accumulation).';
            phase = (2 * k - 1) * theta;  % midpoint-sampling phase ramp, one full 2*theta per epoch
            samples = amp * exp(1j * phase) + (randn(f_error_accumulation, 1) + 1j * randn(f_error_accumulation, 1));
            bin_power(b) = sum(abs(samples) .^ 2);
            bin_samples{b} = samples;
        end

        [~, best_bin] = max(bin_power);
        winning_bin(t)    = best_bin;
        residual_after(t) = f_error_init_Hz - bin_doppler_offset(best_bin);
        cn0_est_dBHz(t)   = cn0_m2m4(bin_samples{best_bin}, Td);

        if t == 1
            example_power = bin_power;
        end
    end

    capture_prob = mean(winning_bin == ideal_bin);

    fprintf('\nMonte Carlo results (%d trials):\n', n_trials);
    fprintf('  Residual Doppler error after scan: mean=%.2f Hz, std=%.2f Hz, RMSE=%.2f Hz (initial error was %.1f Hz)\n', ...
           mean(residual_after), std(residual_after), sqrt(mean(residual_after .^ 2)), f_error_init_Hz);
    fprintf('  Bin-capture probability (picked bin closest to truth, bin %d): %.1f%%\n', ideal_bin - 1, 100 * capture_prob);
    fprintf('  CN0 estimate on winning bin: mean=%.2f dB-Hz, std=%.2f dB-Hz (true CN0=%.1f dB-Hz)\n', ...
           mean(cn0_est_dBHz), std(cn0_est_dBHz), CN0_dBHz);

    %% ------------------------------ Plots -------------------------------------
    figure('Name', 'Frequency error reduction: example bin scan');
    bar(bin_doppler_offset, example_power);
    hold on;
    yl = ylim();
    plot([f_error_init_Hz, f_error_init_Hz], yl, 'r--', 'LineWidth', 1.5);
    hold off;
    xlabel('Doppler tested [Hz]');
    ylabel('Accumulated |Prompt|^2 power');
    title(sprintf('Example bin scan (trial 1) -- true error = %.1f Hz', f_error_init_Hz));
    legend('bin power', 'true frequency', 'Location', 'northeast');
    grid on;

    figure('Name', 'Frequency error reduction: residual error distribution');
    hist(residual_after, min(40, max(10, round(n_trials / 50))));
    xlabel('Residual Doppler error after scan [Hz]');
    ylabel('Trial count');
    title(sprintf('Residual error after %d-bin scan (initial error = %.1f Hz)', num_bins, f_error_init_Hz));
    grid on;
end


function cn0_dBHz = cn0_m2m4(prompt_samples, coh_integration_time_s)
    % Octave port of lock_detectors.cc::cn0_m2m4_estimator(), applied to a
    % column vector of complex Prompt samples, for exact parity with the
    % cn0_dBHz column in the receiver's f_error_dump CSV.
    n = numel(prompt_samples);
    if n == 0 || coh_integration_time_s == 0
        cn0_dBHz = -100;
        return;
    end
    Psig = mean(abs(real(prompt_samples)));
    Psig = Psig^2;
    p2 = abs(prompt_samples) .^ 2;
    m2 = mean(p2);
    m4 = mean(p2 .^ 2);
    % std::sqrt() of a negative argument returns NaN in C++, but a complex
    % value in Octave; map it to NaN explicitly so the fallback branch matches.
    aux = 2 * m2^2 - m4;
    if aux < 0
        aux = NaN;
    else
        aux = sqrt(aux);
    end
    if isnan(aux)
        denominator = m2 - Psig;
        if denominator == 0
            cn0_dBHz = -100;
            return;
        end
        SNR_aux = Psig / denominator;
    else
        denominator = m2 - aux;
        if denominator == 0
            cn0_dBHz = -100;
            return;
        end
        SNR_aux = aux / denominator;
    end
    if SNR_aux <= 0
        cn0_dBHz = -100;
        return;
    end
    cn0_dBHz = 10 * log10(SNR_aux) - 10 * log10(coh_integration_time_s);
end
