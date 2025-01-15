% Usage: read_true_sim_observables_dump (filename, [count])
%
% Opens gnss-sdr-sim observables dump and reads all chennels
%

% -------------------------------------------------------------------------
%
% GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
% This file is part of GNSS-SDR.
%
% SPDX-FileCopyrightText: Javier Arribas 2011
% SPDX-License-Identifier: GPL-3.0-or-later
%
% -------------------------------------------------------------------------


function [observables] = read_true_sim_observables_dump (filename, count)

m = nargchk (1,2,nargin);
channels=12; %Simulator always use 12 channels
num_double_vars=7;
double_size_bytes=8;
skip_bytes_each_read=double_size_bytes*num_double_vars*channels;
bytes_shift=0;

if (m)
    usage (m);
end

if (nargin < 2)
    count = Inf;
end
%loops_counter = fread (f, count, 'uint32',4*12);
f = fopen (filename, 'rb');
if (f < 0)
else
    for N=1:1:channels
        observables.RX_time(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.Carrier_Doppler_hz(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.Carrier_phase_hz(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.Pseudorange_m(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.True_range_m(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.Carrier_phase_hz_v2(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
        observables.PRN(N,:) = fread (f, count, 'float64',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
        fseek(f,bytes_shift,'bof'); % move to next interleaved
    end

    fclose (f);

    %     %%%%%%%% output vars %%%%%%%%
    %         for(int i=0;i<12;i++)
    %         {
    %             d_dump_file.read((char *) &gps_time_sec[i], sizeof(double));
    %             d_dump_file.read((char *) &doppler_l1_hz, sizeof(double));
    %             d_dump_file.read((char *) &acc_carrier_phase_l1_cycles[i], sizeof(double));
    %             d_dump_file.read((char *) &dist_m[i], sizeof(double));
    %             d_dump_file.read((char *) &true_dist_m[i], sizeof(double));
    %             d_dump_file.read((char *) &carrier_phase_l1_cycles[i], sizeof(double));
    %             d_dump_file.read((char *) &prn[i], sizeof(double));
    %         }
end
